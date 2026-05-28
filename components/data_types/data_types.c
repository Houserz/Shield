/**
 * @file data_types.c
 * @brief Data types and metadata management implementation
 */

#include "data_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>

static uint32_t system_start_time_ms = 0;

typedef struct {
    uint8_t id;
    const char *name;
    const char *type;
    const char *tier;
    uint16_t poll_rate_hz;
    uint16_t expected_rate_hz;
    uint8_t axis_count;
    const char *unit;
    const char *processing;
} sensor_meta_t;

static const sensor_meta_t SENSOR_META[] = {
    {1, "SW420_Vibration", "VIBRATION", "fast", 1000, 1000, 1, "binary", "passthrough"},
    {2, "ACS723_Current", "CURRENT", "medium", 200, 200, 1, "A", "gaussian_noise_lowpass_8hz"},
    {3, "MPL3115_Pressure", "PRESSURE", "slow", 50, 50, 1, "Pa", "gaussian_noise_lowpass_2hz"},
    {4, "MCP9808_Temp", "TEMPERATURE", "slow", 50, 50, 1, "C", "gaussian_noise_lowpass_2hz"},
    {5, "INMP441_Microphone", "MICROPHONE", "fast", 1000, 1000, 1, "rms", "gaussian_noise_lowpass_8hz"},
    {6, "751-1015-ND_Photodiode", "PHOTODIODE", "medium", 200, 200, 1, "V", "gaussian_noise_lowpass_8hz"},
    {7, "BNO085_Magnetometer", "MAGNETOMETER", "fast", 1000, 100, 3, "uT", "gaussian_noise_lowpass_8hz"},
    {8, "BNO085_Gyroscope", "GYROSCOPE", "fast", 1000, 100, 3, "rad/s", "gaussian_noise_lowpass_8hz"},
    {9, "BNO085_Accelerometer", "ACCELEROMETER", "fast", 1000, 250, 3, "m/s^2", "gaussian_noise_lowpass_8hz"},
};

static char s_meta_run_id[16] = {0};
static unsigned int s_meta_start_time = 0;
static unsigned int s_meta_end_time = 0;
static daq_statistics_t s_last_stats = {0};

// ==================== Data Types Helper Functions ====================

/**
 * @brief Initialize data types module
 */
void data_types_init(void) {
    system_start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * @brief Get current system timestamp (milliseconds)
 * 
 * @return Milliseconds relative to system start
 */
uint32_t get_timestamp_ms(void) {
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return current_ms - system_start_time_ms;
}

// ==================== Metadata Management Functions ====================

static void metadata_write_sensor_list(FILE *file, const daq_statistics_t *stats) {
    float duration_s = 0.0f;
    if (stats && stats->duration_ms > 0) {
        duration_s = (float)stats->duration_ms / 1000.0f;
    }

    fprintf(file, "  \"sensors\": [\n");
    for (size_t i = 0; i < sizeof(SENSOR_META) / sizeof(SENSOR_META[0]); i++) {
        const sensor_meta_t *s = &SENSOR_META[i];
        uint32_t samples = stats ? stats->sensor_samples[s->id] : 0;
        float observed_hz = (duration_s > 0.0f) ? ((float)samples / duration_s) : 0.0f;
        fprintf(file,
                "    {\"id\": %u, \"name\": \"%s\", \"type\": \"%s\", \"tier\": \"%s\", "
                "\"poll_rate_hz\": %u, \"expected_rate_hz\": %u, "
                "\"observed_rate_hz\": %.3f, \"axis_count\": %u, "
                "\"unit\": \"%s\", \"processing\": \"%s\"}%s\n",
                (unsigned)s->id, s->name, s->type, s->tier,
                (unsigned)s->poll_rate_hz, (unsigned)s->expected_rate_hz,
                observed_hz, (unsigned)s->axis_count, s->unit, s->processing,
                (i + 1 == sizeof(SENSOR_META) / sizeof(SENSOR_META[0])) ? "" : ",");
    }
    fprintf(file, "  ],\n");
}

static bool metadata_write_full(const char *filepath, const char *run_id,
                                unsigned int start_time, unsigned int end_time,
                                const daq_statistics_t *stats) {
    char temp_path[384];
    snprintf(temp_path, sizeof(temp_path), "%s.tmp", filepath);

    FILE *file = fopen(temp_path, "w");
    if (!file) {
        return false;
    }

    daq_statistics_t zero_stats = {0};
    if (!stats) {
        stats = &zero_stats;
    }

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    fprintf(file, "{\n");
    fprintf(file, "  \"run_id\": \"%s\",\n", run_id && run_id[0] ? run_id : "UNKNOWN");
    fprintf(file, "  \"start_time\": \"%u\",\n", start_time);
    if (end_time > 0) {
        fprintf(file, "  \"end_time\": \"%u\",\n", end_time);
    } else {
        fprintf(file, "  \"end_time\": null,\n");
    }
    fprintf(file, "  \"device_info\": {\n");
    fprintf(file, "    \"chip\": \"ESP32-S3\",\n");
    fprintf(file, "    \"cores\": %d,\n", chip_info.cores);
    fprintf(file, "    \"revision\": %d,\n", chip_info.revision);
    fprintf(file, "    \"firmware_version\": \"2.0.0\",\n");
    fprintf(file, "    \"idf_version\": \"%s\"\n", IDF_VER);
    fprintf(file, "  },\n");
    fprintf(file, "  \"record_format\": {\n");
    fprintf(file, "    \"version\": 2,\n");
    fprintf(file, "    \"record_size_bytes\": %u,\n", (unsigned)sizeof(sensor_data_record_v2_t));
    fprintf(file, "    \"clean_definition\": \"physical_units_directly_read_from_sensor\",\n");
    fprintf(file, "    \"noisy_definition\": \"clean * (1 + gaussian_noise), sigma=0.4, clipped to [-1, 1] before multiplication\",\n");
    fprintf(file, "    \"denoised_definition\": \"single-pole realtime low-pass applied to noisy data\",\n");
    fprintf(file, "    \"kind\": {\"0\": \"clean\", \"1\": \"noisy\", \"2\": \"denoised\"},\n");
    fprintf(file, "    \"flags\": {\"0x01\": \"same_as_clean\", \"0x02\": \"noise_injected\", \"0x04\": \"denoise_active\", \"0x08\": \"filter_spike\"}\n");
    fprintf(file, "  },\n");
    fprintf(file, "  \"data_files\": {\n");
    fprintf(file, "    \"fast\": \"fast_data.bin\",\n");
    fprintf(file, "    \"medium\": \"medium_data.bin\",\n");
    fprintf(file, "    \"slow\": \"slow_data.bin\"\n");
    fprintf(file, "  },\n");
    metadata_write_sensor_list(file, stats);
    fprintf(file, "  \"statistics\": {\n");
    fprintf(file, "    \"tier_samples\": {\"fast\": %"PRIu32", \"medium\": %"PRIu32", \"slow\": %"PRIu32"},\n",
            stats->fast_samples, stats->medium_samples, stats->slow_samples);
    fprintf(file, "    \"tier_records\": {\"fast\": %"PRIu32", \"medium\": %"PRIu32", \"slow\": %"PRIu32"},\n",
            stats->fast_records, stats->medium_records, stats->slow_records);
    fprintf(file, "    \"kind_records\": {\"clean\": %"PRIu32", \"noisy\": %"PRIu32", \"denoised\": %"PRIu32"},\n",
            stats->clean_records, stats->noisy_records, stats->denoised_records);
    fprintf(file, "    \"sensor_samples\": {");
    for (uint8_t id = 1; id <= MAX_SENSOR_ID; id++) {
        fprintf(file, "\"%u\": %"PRIu32"%s", (unsigned)id, stats->sensor_samples[id],
                (id == MAX_SENSOR_ID) ? "" : ", ");
    }
    fprintf(file, "},\n");
    fprintf(file, "    \"sensor_records\": {");
    for (uint8_t id = 1; id <= MAX_SENSOR_ID; id++) {
        fprintf(file, "\"%u\": %"PRIu32"%s", (unsigned)id, stats->sensor_records[id],
                (id == MAX_SENSOR_ID) ? "" : ", ");
    }
    fprintf(file, "},\n");
    fprintf(file, "    \"duration_ms\": %"PRIu32",\n", stats->duration_ms);
    fprintf(file, "    \"queue_overruns\": %"PRIu32",\n", stats->queue_overruns);
    fprintf(file, "    \"sd_write_errors\": %"PRIu32"\n", stats->sd_errors);
    fprintf(file, "  }\n");
    fprintf(file, "}\n");

    fclose(file);

    remove(filepath);
    if (rename(temp_path, filepath) != 0) {
        remove(temp_path);
        return false;
    }

    return true;
}

/**
 * @brief Create metadata file
 */
bool metadata_create(const char *filepath, const char *run_id) {
    memset(&s_last_stats, 0, sizeof(s_last_stats));
    s_meta_start_time = (unsigned int)time(NULL);
    s_meta_end_time = 0;
    snprintf(s_meta_run_id, sizeof(s_meta_run_id), "%s", run_id ? run_id : "UNKNOWN");

    return metadata_write_full(filepath, s_meta_run_id, s_meta_start_time,
                               s_meta_end_time, &s_last_stats);
}

/**
 * @brief Update metadata statistics
 */
bool metadata_update_statistics(const char *filepath, const daq_statistics_t *stats) {
    if (stats) {
        s_last_stats = *stats;
    }

    return metadata_write_full(filepath, s_meta_run_id, s_meta_start_time,
                               s_meta_end_time, &s_last_stats);
}

/**
 * @brief Mark run as finished
 */
bool metadata_finalize(const char *filepath) {
    if (s_meta_end_time == 0) {
        s_meta_end_time = (unsigned int)time(NULL);
    }

    return metadata_write_full(filepath, s_meta_run_id, s_meta_start_time,
                               s_meta_end_time, &s_last_stats);
}
