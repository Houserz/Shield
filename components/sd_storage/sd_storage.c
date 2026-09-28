/**
 * @file sd_storage.c
 * @brief SD card storage module implementation (SPI mode)
 *
 * REVIEW-1 CHANGES (C-11 item 1, manuscript §3.3 lines 299-302, §3.6 lines
 * 411-415, §3.7 lines 444-450). The released firmware flushed the libc
 * buffer (fflush) but never synchronised the file to the card (fsync ->
 * FATFS f_sync), never checked the result of fflush/fclose, and never looked
 * at the file sizes after closing. A session could therefore end with
 * meta.json reporting a full sample count while all three tier files had a
 * zero-length directory entry, and nothing on the board reported it. 32.4%
 * of the physical-stress sessions were lost that way, clustered on the first
 * session after a power cycle.
 *
 * What changed:
 *   1. flush_buffer() checks fwrite/fflush/ferror and calls fsync() at least
 *      every SD_SYNC_INTERVAL_MS, so the directory entry (size, cluster
 *      chain) is committed while the session runs, not only at close.
 *   2. sd_close_run_session() does fsync + fclose with return checks, then
 *      stat()s the three files and records their sizes and a close status,
 *      readable through sd_get_last_close() and written to events.log.
 *   3. sd_storage_init() runs a write/fsync/stat/read-back self-test on the
 *      freshly mounted card, with up to SD_SELFTEST_ATTEMPTS remount retries,
 *      so a card that is not ready after power-up is caught before a session
 *      is opened instead of after it is lost.
 *   4. max_files raised from 5 to 8: the three tier files, meta.json and its
 *      .tmp rewrite, and events.log can all be open at once, and the old
 *      limit could make sd_write_log()'s fopen fail silently.
 *   5. sd_write_log() retries once and counts failures; sd_write_marker()
 *      writes an operator marker with both the tick timestamp used in the
 *      data records and the microsecond timer.
 */

#include "sd_storage.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/stat.h>
#include <sys/unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SD_STORAGE";

// Global storage manager instance
static sd_storage_t storage = {0};
static sdmmc_card_t *card = NULL;
static sd_close_report_t last_close = {0};
static uint32_t log_failures = 0;
static bool fault_latched = false;

// ==================== Internal Helper Functions ====================

/**
 * @brief Get next available run number
 */
static int get_next_run_number(void) {
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) {
        return 1; // First run
    }

    int max_num = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "RUN_", 4) == 0) {
            int num = atoi(entry->d_name + 4);
            if (num > max_num) {
                max_num = num;
            }
        }
    }

    closedir(dir);
    return max_num + 1;
}

/**
 * @brief Flush single file buffer to the file, and sync to the card periodically
 */
static bool flush_buffer(file_buffer_t *buffer) {
    if (!buffer->is_open || buffer->file == NULL) {
        return true;
    }

    uint32_t now = get_timestamp_ms();

    if (buffer->buffer_pos > 0) {
        size_t written = fwrite(buffer->buffer, 1, buffer->buffer_pos, buffer->file);
        if (written != buffer->buffer_pos) {
            storage.write_error_count++;
            fault_latched = true;
            ESP_LOGE(TAG, "fwrite short: %u of %u (errno=%d)",
                     (unsigned)written, (unsigned)buffer->buffer_pos, errno);
            return false;
        }
        if (fflush(buffer->file) != 0 || ferror(buffer->file)) {
            storage.write_error_count++;
            fault_latched = true;
            ESP_LOGE(TAG, "fflush failed (errno=%d)", errno);
            clearerr(buffer->file);
            return false;
        }
        buffer->buffer_pos = 0;
        buffer->last_flush_time = now;
    }

    // Commit the directory entry (size, cluster chain) to the card at least
    // every SD_SYNC_INTERVAL_MS. This is the change that makes a session
    // survive a failed close or a power loss with its data intact.
    if (now - buffer->last_sync_time >= SD_SYNC_INTERVAL_MS) {
        if (fsync(fileno(buffer->file)) != 0) {
            storage.sync_error_count++;
            fault_latched = true;
            ESP_LOGE(TAG, "fsync failed (errno=%d)", errno);
            return false;
        }
        buffer->last_sync_time = now;
    }

    return true;
}

/**
 * @brief Write data to buffer
 */
static bool write_to_buffer(file_buffer_t *buffer, const void *data, size_t size) {
    if (!buffer->is_open) {
        return false;
    }

    // Check if flush is needed
    if (buffer->buffer_pos + size > WRITE_BUFFER_SIZE) {
        if (!flush_buffer(buffer)) {
            return false;
        }
    }

    // Write to buffer
    memcpy(buffer->buffer + buffer->buffer_pos, data, size);
    buffer->buffer_pos += size;
    storage.total_bytes_written += size;

    // Periodic flush (every 100ms)
    uint32_t current_time = get_timestamp_ms();
    if (current_time - buffer->last_flush_time > 100) {
        return flush_buffer(buffer);
    }

    return true;
}

/**
 * @brief Write / fsync / stat / read-back self-test on the mounted card
 */
static bool sd_self_test(void) {
    const char *path = SD_MOUNT_POINT "/selftest.bin";
    static uint8_t pattern[512];
    for (int i = 0; i < (int)sizeof(pattern); i++) pattern[i] = (uint8_t)(i * 7 + 3);

    unlink(path);
    FILE *f = fopen(path, "wb");
    if (!f) { ESP_LOGE(TAG, "selftest: fopen failed errno=%d", errno); return false; }
    size_t total = 0;
    for (int i = 0; i < SD_SELFTEST_BYTES / (int)sizeof(pattern); i++) {
        if (fwrite(pattern, 1, sizeof(pattern), f) != sizeof(pattern)) {
            ESP_LOGE(TAG, "selftest: fwrite failed errno=%d", errno);
            fclose(f); return false;
        }
        total += sizeof(pattern);
    }
    if (fflush(f) != 0 || fsync(fileno(f)) != 0) {
        ESP_LOGE(TAG, "selftest: fflush/fsync failed errno=%d", errno);
        fclose(f); return false;
    }
    if (fclose(f) != 0) { ESP_LOGE(TAG, "selftest: fclose failed errno=%d", errno); return false; }

    struct stat st;
    if (stat(path, &st) != 0 || (size_t)st.st_size != total) {
        ESP_LOGE(TAG, "selftest: size mismatch (stat=%ld expected=%u)",
                 (long)(stat(path, &st) == 0 ? st.st_size : -1), (unsigned)total);
        return false;
    }

    f = fopen(path, "rb");
    if (!f) { ESP_LOGE(TAG, "selftest: reopen failed errno=%d", errno); return false; }
    uint8_t rb[512];
    bool ok = true;
    // check first and last block
    if (fread(rb, 1, sizeof(rb), f) != sizeof(rb) || memcmp(rb, pattern, sizeof(rb)) != 0) ok = false;
    if (ok && fseek(f, (long)(total - sizeof(rb)), SEEK_SET) == 0) {
        if (fread(rb, 1, sizeof(rb), f) != sizeof(rb) || memcmp(rb, pattern, sizeof(rb)) != 0) ok = false;
    }
    fclose(f);
    unlink(path);
    if (!ok) ESP_LOGE(TAG, "selftest: read-back mismatch");
    return ok;
}

static bool sd_mount(void) {
    esp_err_t ret;

    // Enable internal pull-up on CS pin
    gpio_set_pull_mode(SD_PIN_CS, GPIO_PULLUP_ONLY);

    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_MOSI,
        .miso_io_num = SD_PIN_MISO,
        .sclk_io_num = SD_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // INVALID_STATE = already initialised (retry path)
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "SD CARD FAILED, OR NOT PRESENT! Error: %s", esp_err_to_name(ret));
        }
        return false;
    }
    return true;
}

static void sd_unmount(void) {
    if (card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        card = NULL;
    }
    spi_bus_free(SPI2_HOST);
}

// ==================== Public API Implementation ====================

/**
 * @brief Initialize SD card (SPI mode), with self-test and remount retries
 */
bool sd_storage_init(void) {
    if (storage.initialized) {
        return true;
    }

    ESP_LOGI(TAG, "Initializing SD card");

    /* Give SD card time to power up after boot (some adapters need this) */
    vTaskDelay(pdMS_TO_TICKS(2000));

    bool ok = false;
    for (int attempt = 1; attempt <= SD_SELFTEST_ATTEMPTS && !ok; attempt++) {
        if (attempt > 1) {
            ESP_LOGW(TAG, "SD init attempt %d of %d", attempt, SD_SELFTEST_ATTEMPTS);
            sd_unmount();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (!sd_mount()) {
            continue;
        }
        ESP_LOGI(TAG, "SD CARD MOUNTED. Running self-test (%d bytes)...", SD_SELFTEST_BYTES);
        ok = sd_self_test();
        storage.selftest_attempts = attempt;
    }

    if (!ok) {
        ESP_LOGE(TAG, "SD self-test FAILED after %d attempts", SD_SELFTEST_ATTEMPTS);
        sd_unmount();
        return false;
    }

    ESP_LOGI(TAG, "SD CARD INITIALIZED (self-test passed on attempt %d).", storage.selftest_attempts);
    sdmmc_card_print_info(stdout, card);

    storage.initialized = true;
    storage.mounted = true;
    storage.total_bytes_written = 0;
    storage.write_error_count = 0;
    storage.sync_error_count = 0;
    fault_latched = false;

    return true;
}

/**
 * @brief Deinitialize SD card
 */
void sd_storage_deinit(void) {
    if (!storage.initialized) {
        return;
    }

    // Close current session
    if (storage.current_session.is_active) {
        sd_close_run_session();
    }

    sd_unmount();

    storage.initialized = false;
    storage.mounted = false;
}

/**
 * @brief Create new run session
 */
bool sd_create_run_session(void) {
    if (!storage.initialized) {
        return false;
    }

    if (storage.current_session.is_active) {
        sd_close_run_session();
    }

    // Generate run ID
    int run_num = get_next_run_number();
    snprintf(storage.current_session.run_id, sizeof(storage.current_session.run_id),
             "RUN_%03d", run_num);

    // Create run directory
    snprintf(storage.current_session.run_path, MAX_DIR_PATH_LEN,
             "%s/%s", SD_MOUNT_POINT, storage.current_session.run_id);

    if (mkdir(storage.current_session.run_path, 0777) != 0) {
        ESP_LOGE(TAG, "mkdir failed: path=%s errno=%d (%s)",
                 storage.current_session.run_path, errno, strerror(errno));
        return false;
    }

    // Build file paths
    snprintf(storage.current_session.fast_file, MAX_FILE_PATH_LEN,
             "%s/fast_data.bin", storage.current_session.run_path);
    snprintf(storage.current_session.medium_file, MAX_FILE_PATH_LEN,
             "%s/medium_data.bin", storage.current_session.run_path);
    snprintf(storage.current_session.slow_file, MAX_FILE_PATH_LEN,
             "%s/slow_data.bin", storage.current_session.run_path);
    snprintf(storage.current_session.meta_file, MAX_FILE_PATH_LEN,
             "%s/meta.json", storage.current_session.run_path);
    snprintf(storage.current_session.log_file, MAX_FILE_PATH_LEN,
             "%s/events.log", storage.current_session.run_path);

    // Open data files
    storage.fast_buffer.file = fopen(storage.current_session.fast_file, "wb");
    storage.medium_buffer.file = fopen(storage.current_session.medium_file, "wb");
    storage.slow_buffer.file = fopen(storage.current_session.slow_file, "wb");

    if (!storage.fast_buffer.file) {
        ESP_LOGE(TAG, "fopen failed (fast_data): path=%s errno=%d (%s)",
                 storage.current_session.fast_file, errno, strerror(errno));
        return false;
    }
    if (!storage.medium_buffer.file) {
        ESP_LOGE(TAG, "fopen failed (medium_data): path=%s errno=%d (%s)",
                 storage.current_session.medium_file, errno, strerror(errno));
        fclose(storage.fast_buffer.file);
        storage.fast_buffer.file = NULL;
        return false;
    }
    if (!storage.slow_buffer.file) {
        ESP_LOGE(TAG, "fopen failed (slow_data): path=%s errno=%d (%s)",
                 storage.current_session.slow_file, errno, strerror(errno));
        fclose(storage.fast_buffer.file);
        fclose(storage.medium_buffer.file);
        storage.fast_buffer.file = NULL;
        storage.medium_buffer.file = NULL;
        return false;
    }

    // Initialize buffers
    uint32_t now = get_timestamp_ms();
    file_buffer_t *bufs[3] = {&storage.fast_buffer, &storage.medium_buffer, &storage.slow_buffer};
    for (int i = 0; i < 3; i++) {
        bufs[i]->is_open = true;
        bufs[i]->buffer_pos = 0;
        bufs[i]->last_flush_time = now;
        bufs[i]->last_sync_time = now;
    }

    storage.current_session.is_active = true;
    memset(&last_close, 0, sizeof(last_close));

    sd_write_log("INFO", "Session started");

    return true;
}

/**
 * @brief Close a tier file: final flush, fsync, fclose, then stat its size
 */
static bool close_tier_file(file_buffer_t *buffer, const char *path, uint32_t *bytes_out) {
    bool ok = true;
    if (buffer->is_open && buffer->file) {
        if (!flush_buffer(buffer)) ok = false;
        if (fsync(fileno(buffer->file)) != 0) {
            storage.sync_error_count++;
            ESP_LOGE(TAG, "close: fsync failed on %s errno=%d", path, errno);
            ok = false;
        }
        if (fclose(buffer->file) != 0) {
            storage.write_error_count++;
            ESP_LOGE(TAG, "close: fclose failed on %s errno=%d", path, errno);
            ok = false;
        }
        buffer->file = NULL;
        buffer->is_open = false;
    }
    struct stat st;
    if (stat(path, &st) == 0) {
        *bytes_out = (uint32_t)st.st_size;
    } else {
        *bytes_out = 0;
        ESP_LOGE(TAG, "close: stat failed on %s errno=%d", path, errno);
        ok = false;
    }
    return ok;
}

/**
 * @brief Close current run session, verify the files on the card
 */
bool sd_close_run_session(void) {
    if (!storage.current_session.is_active) {
        return true;
    }

    bool ok = true;
    if (!close_tier_file(&storage.fast_buffer,   storage.current_session.fast_file,   &last_close.fast_bytes))   ok = false;
    if (!close_tier_file(&storage.medium_buffer, storage.current_session.medium_file, &last_close.medium_bytes)) ok = false;
    if (!close_tier_file(&storage.slow_buffer,   storage.current_session.slow_file,   &last_close.slow_bytes))   ok = false;

    // A tier file of zero length after a session that wrote anything is the
    // exact failure the released corpus suffered. Treat it as a close failure.
    if (storage.total_bytes_written > 0 &&
        (last_close.fast_bytes == 0 || last_close.medium_bytes == 0 || last_close.slow_bytes == 0)) {
        ok = false;
    }
    last_close.close_ok = ok && !fault_latched;
    last_close.write_errors = storage.write_error_count;
    last_close.sync_errors = storage.sync_error_count;
    last_close.log_failures = log_failures;
    last_close.valid = true;

    char msg[160];
    snprintf(msg, sizeof(msg),
             "CLOSE fast=%" PRIu32 " medium=%" PRIu32 " slow=%" PRIu32
             " write_err=%" PRIu32 " sync_err=%" PRIu32 " log_fail=%" PRIu32 " status=%s",
             last_close.fast_bytes, last_close.medium_bytes, last_close.slow_bytes,
             last_close.write_errors, last_close.sync_errors, last_close.log_failures,
             last_close.close_ok ? "OK" : "FAIL");
    sd_write_log(last_close.close_ok ? "INFO" : "ERROR", msg);
    ESP_LOGI(TAG, "%s", msg);

    sd_write_log("INFO", "Session closed");
    storage.current_session.is_active = false;

    return last_close.close_ok;
}

/**
 * @brief Write fast data record
 */
bool sd_write_fast_data(const fast_data_record_t *record) {
    return write_to_buffer(&storage.fast_buffer, record, sizeof(fast_data_record_t));
}

/**
 * @brief Write medium data record
 */
bool sd_write_medium_data(const medium_data_record_t *record) {
    return write_to_buffer(&storage.medium_buffer, record, sizeof(medium_data_record_t));
}

/**
 * @brief Write slow data record
 */
bool sd_write_slow_data(const slow_data_record_t *record) {
    return write_to_buffer(&storage.slow_buffer, record, sizeof(slow_data_record_t));
}

/**
 * @brief Write event log (retries once; failures counted)
 */
void sd_write_log(const char *level, const char *message) {
    if (!storage.current_session.is_active) {
        return;
    }

    FILE *log_file = fopen(storage.current_session.log_file, "a");
    if (!log_file) {
        vTaskDelay(pdMS_TO_TICKS(10));
        log_file = fopen(storage.current_session.log_file, "a");
    }
    if (log_file) {
        uint32_t timestamp = get_timestamp_ms();
        fprintf(log_file, "[%010"PRIu32"] %s: %s\n", timestamp, level, message);
        fflush(log_file);
        fsync(fileno(log_file));
        fclose(log_file);
    } else {
        log_failures++;
        ESP_LOGE(TAG, "events.log open failed errno=%d (%" PRIu32 " failures)", errno, log_failures);
    }
}

/**
 * @brief Write an operator marker: tick timestamp (same clock as the data
 *        records), microsecond timer, and a free-text label.
 */
void sd_write_marker(const char *label) {
    char msg[128];
    snprintf(msg, sizeof(msg), "MARK t_ms=%" PRIu32 " t_us=%lld label=%s",
             get_timestamp_ms(), (long long)esp_timer_get_time(),
             label ? label : "");
    sd_write_log("MARK", msg);
}

/**
 * @brief Flush all buffers to SD card
 */
bool sd_flush_all_buffers(void) {
    bool success = true;

    if (!flush_buffer(&storage.fast_buffer)) success = false;
    if (!flush_buffer(&storage.medium_buffer)) success = false;
    if (!flush_buffer(&storage.slow_buffer)) success = false;

    return success;
}

/**
 * @brief Sync all open tier files to the card now (directory entries committed)
 */
bool sd_sync_all(void) {
    bool success = true;
    file_buffer_t *bufs[3] = {&storage.fast_buffer, &storage.medium_buffer, &storage.slow_buffer};
    for (int i = 0; i < 3; i++) {
        if (bufs[i]->is_open && bufs[i]->file) {
            if (!flush_buffer(bufs[i])) success = false;
            if (fsync(fileno(bufs[i]->file)) != 0) { storage.sync_error_count++; success = false; }
            bufs[i]->last_sync_time = get_timestamp_ms();
        }
    }
    return success;
}

/**
 * @brief Get SD card information
 */
bool sd_get_card_info(uint64_t *total_mb, uint64_t *free_mb) {
    if (!storage.mounted) {
        return false;
    }

    FATFS *fs;
    DWORD fre_clust;

    if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
        uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * fs->ssize;
        uint64_t free = (uint64_t)fre_clust * fs->csize * fs->ssize;

        if (total_mb) *total_mb = total / (1024 * 1024);
        if (free_mb) *free_mb = free / (1024 * 1024);

        return true;
    }

    return false;
}

/**
 * @brief Get current run session information
 */
const run_session_t* sd_get_current_session(void) {
    return &storage.current_session;
}

/**
 * @brief Get storage statistics
 */
void sd_get_statistics(uint64_t *bytes_written, uint32_t *error_count) {
    if (bytes_written) *bytes_written = storage.total_bytes_written;
    if (error_count) *error_count = storage.write_error_count + storage.sync_error_count;
}

/**
 * @brief Report of the last session close
 */
bool sd_get_last_close(sd_close_report_t *out) {
    if (!out || !last_close.valid) return false;
    *out = last_close;
    return true;
}

/**
 * @brief True if any write/flush/fsync error was recorded since init
 */
bool sd_storage_fault(void) {
    return fault_latched;
}

int sd_get_selftest_attempts(void) {
    return storage.selftest_attempts;
}
