/**
 * @file streamer.c
 * @brief See streamer.h for protocol/architecture.
 */
#include "streamer.h"

#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#if STREAMER_ENABLE_USB
#include "driver/usb_serial_jtag.h"
#endif

#if STREAMER_ENABLE_WIFI
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#endif

static const char *TAG = "streamer";

// ==================== Tunables ====================
#define WIFI_AP_SSID        "SHIELD_DAQ"
#define WIFI_AP_PASS        "shield1234"      // >= 8 chars for WPA2
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    2
#define TCP_PORT            3333

#define USB_QUEUE_LEN       4096              // ~80 KB with V2 packets
#define WIFI_QUEUE_LEN      1024              // ~20 KB with V2 packets
#define USB_TX_BUF          4096

// ==================== State ====================
static atomic_uint s_seq        = 0;
static atomic_uint s_drops      = 0;
static atomic_uint s_sent_usb   = 0;
static atomic_uint s_sent_wifi  = 0;

#if STREAMER_ENABLE_USB
static QueueHandle_t s_usb_queue  = NULL;
static volatile bool s_usb_ready  = false;
#endif

#if STREAMER_ENABLE_WIFI
static QueueHandle_t s_wifi_queue = NULL;
static volatile bool s_wifi_ready = false;
static volatile int  s_tcp_client_fd = -1;
#endif

// ==================== Publish helpers ====================
static inline void publish_pkt_(const stream_pkt_t *pkt)
{
#if STREAMER_ENABLE_USB || STREAMER_ENABLE_WIFI
    bool any_full = false;

#if STREAMER_ENABLE_USB
    if (s_usb_ready && s_usb_queue) {
        if (xQueueSend(s_usb_queue, pkt, 0) != pdTRUE) any_full = true;
    }
#endif

#if STREAMER_ENABLE_WIFI
    // Only enqueue Wi-Fi packets when a client is actually connected; otherwise
    // we waste RAM holding samples nobody will read.
    if (s_wifi_ready && s_wifi_queue && s_tcp_client_fd >= 0) {
        // Wi-Fi side is best-effort, don't count its drops as "drop".
        (void)xQueueSend(s_wifi_queue, pkt, 0);
    }
#endif

    if (any_full) atomic_fetch_add(&s_drops, 1);
#else
    (void)pkt;
#endif
}

void streamer_publish_record(const sensor_data_record_v2_t *rec)
{
    if (!rec) return;
    stream_pkt_t pkt = {
        .magic = STREAM_MAGIC_LE,
        .sensor_id = rec->sensor_id,
        .axis = 0,
        .kind = rec->kind,
        .flags = rec->flags,
        .reserved = 0,
        .seq = 0,
        .timestamp_ms = rec->timestamp_ms,
        .value = 0.0f,
    };

    uint8_t axis_count = rec->axis_count;
    if (axis_count == 0) axis_count = 1;
    if (axis_count > 3) axis_count = 3;

    if (axis_count == 1) {
        pkt.axis  = 0;
        pkt.value = rec->data[0];
        pkt.seq   = (uint32_t)atomic_fetch_add(&s_seq, 1);
        publish_pkt_(&pkt);
        return;
    }

    for (uint8_t a = 0; a < axis_count; ++a) {
        pkt.axis = (uint8_t)(a + 1);          // 1=x, 2=y, 3=z
        pkt.value = rec->data[a];
        pkt.seq = (uint32_t)atomic_fetch_add(&s_seq, 1);
        publish_pkt_(&pkt);
    }
}

void streamer_publish_fast(const fast_data_record_t *rec)
{
    streamer_publish_record(rec);
}

void streamer_publish_medium(const medium_data_record_t *rec)
{
    streamer_publish_record(rec);
}

void streamer_publish_slow(const slow_data_record_t *rec)
{
    streamer_publish_record(rec);
}

uint32_t streamer_get_drops(void)     { return atomic_load(&s_drops); }
uint32_t streamer_get_sent_usb(void)  { return atomic_load(&s_sent_usb); }
uint32_t streamer_get_sent_wifi(void) { return atomic_load(&s_sent_wifi); }

// ==================== USB Serial/JTAG backend ====================
#if STREAMER_ENABLE_USB
static void usb_streamer_task(void *arg)
{
    stream_pkt_t pkt;
    // Coalesce a few packets per write to reduce syscall overhead.
    static uint8_t tx_buf[sizeof(stream_pkt_t) * 32];

    while (1) {
        if (xQueueReceive(s_usb_queue, &pkt, portMAX_DELAY) != pdTRUE) continue;

        size_t n = 0;
        memcpy(tx_buf + n, &pkt, sizeof(pkt)); n += sizeof(pkt);
        // Drain up to 31 more without blocking
        while (n + sizeof(pkt) <= sizeof(tx_buf) &&
               xQueueReceive(s_usb_queue, &pkt, 0) == pdTRUE) {
            memcpy(tx_buf + n, &pkt, sizeof(pkt));
            n += sizeof(pkt);
        }

        int wrote = usb_serial_jtag_write_bytes(tx_buf, n, pdMS_TO_TICKS(20));
        if (wrote > 0) {
            atomic_fetch_add(&s_sent_usb, (unsigned)(wrote / sizeof(pkt)));
        }
    }
}

static bool usb_init_(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = USB_TX_BUF,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    s_usb_queue = xQueueCreate(USB_QUEUE_LEN, sizeof(stream_pkt_t));
    if (!s_usb_queue) {
        ESP_LOGE(TAG, "USB queue alloc failed");
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(usb_streamer_task, "usb_stream",
                                            4096, NULL, 4, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "USB task create failed");
        return false;
    }

    s_usb_ready = true;
    ESP_LOGI(TAG, "USB streamer ready (CDC over USB Serial/JTAG)");
    return true;
}
#endif // STREAMER_ENABLE_USB

// ==================== Wi-Fi SoftAP + TCP backend ====================
#if STREAMER_ENABLE_WIFI
static void wifi_event_handler_(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t*)data;
        ESP_LOGI(TAG, "STA join: " MACSTR " aid=%d", MAC2STR(e->mac), e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t*)data;
        ESP_LOGI(TAG, "STA leave: " MACSTR " aid=%d", MAC2STR(e->mac), e->aid);
    }
}

static bool wifi_softap_init_(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler_, NULL, NULL);

    wifi_config_t ap_cfg = { 0 };
    strncpy((char*)ap_cfg.ap.ssid, WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = (uint8_t)strlen(WIFI_AP_SSID);
    strncpy((char*)ap_cfg.ap.password, WIFI_AP_PASS, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.channel = WIFI_AP_CHANNEL;
    ap_cfg.ap.max_connection = WIFI_AP_MAX_CONN;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen(WIFI_AP_PASS) == 0) ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP up: SSID=\"%s\" pass=\"%s\" ch=%d ip=192.168.4.1 port=%d",
             WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, TCP_PORT);
    return true;
}

static int tcp_listen_(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return -1;

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = { 0 };
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(TCP_PORT);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    if (listen(fd, 1) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void wifi_streamer_task(void *arg)
{
    int listen_fd = -1;
    while (listen_fd < 0) {
        listen_fd = tcp_listen_();
        if (listen_fd < 0) {
            ESP_LOGE(TAG, "TCP listen failed, retry in 1s");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    while (1) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int fd = accept(listen_fd, (struct sockaddr*)&cli, &cli_len);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Disable Nagle for low-latency streaming.
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        // Drop any backlog accumulated before the client connected.
        xQueueReset(s_wifi_queue);

        ESP_LOGI(TAG, "TCP client %s:%d connected",
                 inet_ntoa(cli.sin_addr), ntohs(cli.sin_port));
        s_tcp_client_fd = fd;

        stream_pkt_t pkt;
        // Coalesce up to 32 packets per send().
        static uint8_t tx[sizeof(stream_pkt_t) * 32];

        while (1) {
            if (xQueueReceive(s_wifi_queue, &pkt, pdMS_TO_TICKS(2000)) != pdTRUE) {
                // Send a no-op heartbeat? Skip: TCP keepalive handles it.
                continue;
            }
            size_t n = 0;
            memcpy(tx + n, &pkt, sizeof(pkt)); n += sizeof(pkt);
            while (n + sizeof(pkt) <= sizeof(tx) &&
                   xQueueReceive(s_wifi_queue, &pkt, 0) == pdTRUE) {
                memcpy(tx + n, &pkt, sizeof(pkt));
                n += sizeof(pkt);
            }
            ssize_t w = send(fd, tx, n, 0);
            if (w <= 0) break;
            atomic_fetch_add(&s_sent_wifi, (unsigned)(w / sizeof(pkt)));
        }

        ESP_LOGW(TAG, "TCP client disconnected");
        s_tcp_client_fd = -1;
        close(fd);
    }
}

static bool wifi_init_(void)
{
    if (!wifi_softap_init_()) return false;

    s_wifi_queue = xQueueCreate(WIFI_QUEUE_LEN, sizeof(stream_pkt_t));
    if (!s_wifi_queue) {
        ESP_LOGE(TAG, "Wi-Fi queue alloc failed");
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(wifi_streamer_task, "wifi_stream",
                                            6144, NULL, 4, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Wi-Fi task create failed");
        return false;
    }

    s_wifi_ready = true;
    return true;
}
#endif // STREAMER_ENABLE_WIFI

// ==================== Public init ====================
void streamer_init(void)
{
#if STREAMER_ENABLE_USB
    const char *usb_status = usb_init_() ? "ok" : "FAIL";
#else
    const char *usb_status = "off";
#endif

#if STREAMER_ENABLE_WIFI
    const char *wifi_status = wifi_init_() ? "ok" : "FAIL";
#else
    const char *wifi_status = "off";
#endif

    ESP_LOGI(TAG, "streamer_init: usb=%s wifi=%s", usb_status, wifi_status);
}
