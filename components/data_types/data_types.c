/**
 * @file data_types.c
 * @brief Data types and metadata management implementation
 *
 * REVIEW-1 CHANGES (C-11 items 4 and 7): provenance block in meta.json
 * (board MAC, firmware git hash, tick rate, BNO085 report configuration and
 * calibration-config read-back, RTC state, SD self-test attempts); finalize
 * writes the tier-file sizes and the close status. Record layouts unchanged.
 */

#include "data_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>   /* fsync */

static uint32_t system_start_time_ms = 0;
static meta_provenance_t s_prov = {
    .board_mac = "not-read",
    .firmware_git = FW_GIT_HASH,
    .tick_hz = CONFIG_FREERTOS_HZ,
    .accel_interval_us = 0, .gyro_interval_us = 0, .mag_interval_us = 0,
    .accel_report = "not-set", .gyro_report = "not-set", .mag_report = "not-set",
    .bno085_cal_config = -1,
    .sd_selftest_attempts = 0,
    .rtc_set = false,
};

// ==================== Data Types Helper Functions ====================

void data_types_init(void) {
    system_start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * @brief Milliseconds relative to system start, on the FreeRTOS tick clock.
 *        UNCHANGED on purpose: the data records, events.log and markers all
 *        share this clock, and the released decoder derives achieved rates
 *        from it. Resolution is 1000/CONFIG_FREERTOS_HZ ms (10 ms on the
 *        recording build).
 */
uint32_t get_timestamp_ms(void) {
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return current_ms - system_start_time_ms;
}

void metadata_set_provenance(const meta_provenance_t *prov) {
    if (prov) s_prov = *prov;
    if (s_prov.firmware_git[0] == '\0') strncpy(s_prov.firmware_git, FW_GIT_HASH, sizeof(s_prov.firmware_git) - 1);
    if (s_prov.tick_hz == 0) s_prov.tick_hz = CONFIG_FREERTOS_HZ;
}

void metadata_read_board_mac(char *out, size_t out_len) {
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        snprintf(out, out_len, "read-failed");
    }
}

// ==================== Metadata Management Functions ====================

bool metadata_create(const char *filepath, const char *run_id) {
    FILE *file = fopen(filepath, "w");
    if (!file) {
        return false;
    }

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    time_t now = time(NULL);
    bool rtc_set = (now > 1600000000);   // any real date after 2020 means SETTIME was applied

    fprintf(file, "{\n");
    fprintf(file, "  \"run_id\": \"%s\",\n", run_id);
    fprintf(file, "  \"start_time\": \"%u\",\n", (unsigned int)now);
    fprintf(file, "  \"rtc_set\": %s,\n", rtc_set ? "true" : "false");
    fprintf(file, "  \"end_time\": null,\n");
    fprintf(file, "  \"device_info\": {\n");
    fprintf(file, "    \"chip\": \"ESP32-S3\",\n");
    fprintf(file, "    \"cores\": %d,\n", chip_info.cores);
    fprintf(file, "    \"revision\": %d,\n", chip_info.revision);
    fprintf(file, "    \"board_mac\": \"%s\",\n", s_prov.board_mac);
    fprintf(file, "    \"firmware_version\": \"1.1.0-review1\",\n");
    fprintf(file, "    \"firmware_git\": \"%s\",\n", s_prov.firmware_git);
    fprintf(file, "    \"idf_version\": \"%s\",\n", IDF_VER);
    fprintf(file, "    \"freertos_tick_hz\": %" PRIu32 ",\n", s_prov.tick_hz);
    fprintf(file, "    \"sd_selftest_attempts\": %d\n", s_prov.sd_selftest_attempts);
    fprintf(file, "  },\n");
    fprintf(file, "  \"bno085\": {\n");
    fprintf(file, "    \"accelerometer_report\": \"%s\",\n", s_prov.accel_report);
    fprintf(file, "    \"accelerometer_interval_us\": %" PRIu32 ",\n", s_prov.accel_interval_us);
    fprintf(file, "    \"gyroscope_report\": \"%s\",\n", s_prov.gyro_report);
    fprintf(file, "    \"gyroscope_interval_us\": %" PRIu32 ",\n", s_prov.gyro_interval_us);
    fprintf(file, "    \"magnetometer_report\": \"%s\",\n", s_prov.mag_report);
    fprintf(file, "    \"magnetometer_interval_us\": %" PRIu32 ",\n", s_prov.mag_interval_us);
    if (s_prov.bno085_cal_config >= 0) {
        fprintf(file, "    \"dynamic_calibration_config\": %d,\n", s_prov.bno085_cal_config);
        fprintf(file, "    \"dynamic_calibration_note\": \"sh2_getCalConfig bitmask: 1=accel 2=gyro 4=mag 8=planar; hub default, never changed by this firmware\"\n");
    } else {
        fprintf(file, "    \"dynamic_calibration_config\": null,\n");
        fprintf(file, "    \"dynamic_calibration_note\": \"hub default, not read back\"\n");
    }
    fprintf(file, "  },\n");
    fprintf(file, "  \"sensors\": {\n");
    fprintf(file, "    \"fast\": [\n");
    fprintf(file, "      {\"id\": 1, \"name\": \"SW420_Vibration\", \"type\": \"VIBRATION\", \"rate\": 1000, \"unit\": \"binary\"},\n");
    fprintf(file, "      {\"id\": 5, \"name\": \"INMP441_Microphone\", \"type\": \"MICROPHONE\", \"rate\": 1000, \"unit\": \"dBFS\"},\n");
    fprintf(file, "      {\"id\": 7, \"name\": \"BNO085_Magnetometer\", \"type\": \"MAGNETOMETER\", \"rate\": 1000, \"unit\": \"uT\"},\n");
    fprintf(file, "      {\"id\": 8, \"name\": \"BNO085_Gyroscope\", \"type\": \"GYROSCOPE\", \"rate\": 1000, \"unit\": \"rad/s\"},\n");
    fprintf(file, "      {\"id\": 9, \"name\": \"BNO085_Accelerometer\", \"type\": \"ACCELEROMETER\", \"rate\": 1000, \"unit\": \"m/s^2\"}\n");
    fprintf(file, "    ],\n");
    fprintf(file, "    \"medium\": [\n");
    fprintf(file, "      {\"id\": 2, \"name\": \"ACS723_Current\", \"type\": \"CURRENT\", \"rate\": 200, \"unit\": \"A\"},\n");
    fprintf(file, "      {\"id\": 6, \"name\": \"751-1015-ND_Photodiode\", \"type\": \"PHOTODIODE\", \"rate\": 200, \"unit\": \"V\"}\n");
    fprintf(file, "    ],\n");
    fprintf(file, "    \"slow\": [\n");
    fprintf(file, "      {\"id\": 3, \"name\": \"MPL3115_Pressure\", \"type\": \"PRESSURE\", \"rate\": 50, \"unit\": \"kPa\"},\n");
    fprintf(file, "      {\"id\": 4, \"name\": \"MCP9808_Temp\", \"type\": \"TEMPERATURE\", \"rate\": 50, \"unit\": \"C\"}\n");
    fprintf(file, "    ]\n");
    fprintf(file, "  },\n");
    fprintf(file, "  \"data_files\": {\n");
    fprintf(file, "    \"fast\": \"fast_data.bin\",\n");
    fprintf(file, "    \"medium\": \"medium_data.bin\",\n");
    fprintf(file, "    \"slow\": \"slow_data.bin\"\n");
    fprintf(file, "  },\n");
    fprintf(file, "  \"close_report\": {\n");
    fprintf(file, "    \"fast_bytes\": null,\n");
    fprintf(file, "    \"medium_bytes\": null,\n");
    fprintf(file, "    \"slow_bytes\": null,\n");
    fprintf(file, "    \"marker_count\": null,\n");
    fprintf(file, "    \"close_ok\": null\n");
    fprintf(file, "  },\n");
    fprintf(file, "  \"statistics\": {\n");
    fprintf(file, "    \"total_samples\": {\n");
    fprintf(file, "      \"fast\": 0,\n");
    fprintf(file, "      \"medium\": 0,\n");
    fprintf(file, "      \"slow\": 0\n");
    fprintf(file, "    },\n");
    fprintf(file, "    \"duration_ms\": 0,\n");
    fprintf(file, "    \"queue_overruns\": 0,\n");
    fprintf(file, "    \"sd_write_errors\": 0\n");
    fprintf(file, "  }\n");
    fprintf(file, "}\n");

    fflush(file);
    fsync(fileno(file));
    fclose(file);
    return true;
}

/**
 * @brief Rewrite meta.json line by line with a replacement callback pattern.
 *        (Kept as the original two-file rewrite; both files are fsynced.)
 */
bool metadata_update_statistics(const char *filepath, const daq_statistics_t *stats) {
    char temp_path[256];
    snprintf(temp_path, sizeof(temp_path), "%s.tmp", filepath);

    FILE *file_in = fopen(filepath, "r");
    FILE *file_out = fopen(temp_path, "w");

    if (!file_in || !file_out) {
        if (file_in) fclose(file_in);
        if (file_out) fclose(file_out);
        return false;
    }

    char line[256];
    bool in_stats = false;

    while (fgets(line, sizeof(line), file_in)) {
        if (strstr(line, "\"statistics\"")) {
            in_stats = true;
        }

        if (in_stats) {
            if (strstr(line, "\"fast\":")) {
                fprintf(file_out, "      \"fast\": %"PRIu32",\n", stats->fast_samples);
                continue;
            } else if (strstr(line, "\"medium\":")) {
                fprintf(file_out, "      \"medium\": %"PRIu32",\n", stats->medium_samples);
                continue;
            } else if (strstr(line, "\"slow\":")) {
                fprintf(file_out, "      \"slow\": %"PRIu32"\n", stats->slow_samples);
                continue;
            } else if (strstr(line, "\"duration_ms\":")) {
                fprintf(file_out, "    \"duration_ms\": %"PRIu32",\n", stats->duration_ms);
                continue;
            } else if (strstr(line, "\"queue_overruns\":")) {
                fprintf(file_out, "    \"queue_overruns\": %"PRIu32",\n", stats->queue_overruns);
                continue;
            } else if (strstr(line, "\"sd_write_errors\":")) {
                fprintf(file_out, "    \"sd_write_errors\": %"PRIu32"\n", stats->sd_errors);
                in_stats = false;
                continue;
            }
        }

        fputs(line, file_out);
    }

    fclose(file_in);
    fflush(file_out);
    fsync(fileno(file_out));
    fclose(file_out);

    remove(filepath);
    rename(temp_path, filepath);

    return true;
}

bool metadata_finalize(const char *filepath, uint32_t fast_bytes, uint32_t medium_bytes,
                       uint32_t slow_bytes, bool close_ok, uint32_t marker_count) {
    char temp_path[256];
    snprintf(temp_path, sizeof(temp_path), "%s.tmp", filepath);

    FILE *file_in = fopen(filepath, "r");
    FILE *file_out = fopen(temp_path, "w");

    if (!file_in || !file_out) {
        if (file_in) fclose(file_in);
        if (file_out) fclose(file_out);
        return false;
    }

    char line[256];
    while (fgets(line, sizeof(line), file_in)) {
        if (strstr(line, "\"end_time\": null")) {
            fprintf(file_out, "  \"end_time\": \"%u\",\n", (unsigned int)time(NULL));
        } else if (strstr(line, "\"fast_bytes\": null")) {
            fprintf(file_out, "    \"fast_bytes\": %" PRIu32 ",\n", fast_bytes);
        } else if (strstr(line, "\"medium_bytes\": null")) {
            fprintf(file_out, "    \"medium_bytes\": %" PRIu32 ",\n", medium_bytes);
        } else if (strstr(line, "\"slow_bytes\": null")) {
            fprintf(file_out, "    \"slow_bytes\": %" PRIu32 ",\n", slow_bytes);
        } else if (strstr(line, "\"marker_count\": null")) {
            fprintf(file_out, "    \"marker_count\": %" PRIu32 ",\n", marker_count);
        } else if (strstr(line, "\"close_ok\": null")) {
            fprintf(file_out, "    \"close_ok\": %s\n", close_ok ? "true" : "false");
        } else {
            fputs(line, file_out);
        }
    }

    fclose(file_in);
    fflush(file_out);
    fsync(fileno(file_out));
    fclose(file_out);

    remove(filepath);
    rename(temp_path, filepath);

    return true;
}
