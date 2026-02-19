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

/**
 * @brief Create metadata file
 */
bool metadata_create(const char *filepath, const char *run_id) {
    FILE *file = fopen(filepath, "w");
    if (!file) {
        return false;
    }
    
    // Get system information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    // Write JSON-formatted metadata
    fprintf(file, "{\n");
    fprintf(file, "  \"run_id\": \"%s\",\n", run_id);
    fprintf(file, "  \"start_time\": \"%u\",\n", (unsigned int)time(NULL));
    fprintf(file, "  \"end_time\": null,\n");
    fprintf(file, "  \"device_info\": {\n");
    fprintf(file, "    \"chip\": \"ESP32-S3\",\n");
    fprintf(file, "    \"cores\": %d,\n", chip_info.cores);
    fprintf(file, "    \"revision\": %d,\n", chip_info.revision);
    fprintf(file, "    \"firmware_version\": \"1.0.0\",\n");
    fprintf(file, "    \"idf_version\": \"%s\"\n", IDF_VER);
    fprintf(file, "  },\n");
    fprintf(file, "  \"sensors\": {\n");
    fprintf(file, "    \"fast\": [\n");
    fprintf(file, "      {\"id\": 0, \"name\": \"BNO085_IMU\", \"type\": \"IMU\", \"rate\": 1000, \"unit\": \"m/s^2\"},\n");
    fprintf(file, "      {\"id\": 1, \"name\": \"SW420_Vibration\", \"type\": \"VIBRATION\", \"rate\": 1000, \"unit\": \"binary\"},\n");
    fprintf(file, "      {\"id\": 5, \"name\": \"INMP441_Microphone\", \"type\": \"MICROPHONE\", \"rate\": 1000, \"unit\": \"dBFS\"}\n");
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
    
    fclose(file);
    return true;
}

/**
 * @brief Update metadata statistics
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
    fclose(file_out);
    
    remove(filepath);
    rename(temp_path, filepath);
    
    return true;
}

/**
 * @brief Mark run as finished
 */
bool metadata_finalize(const char *filepath) {
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
        } else {
            fputs(line, file_out);
        }
    }
    
    fclose(file_in);
    fclose(file_out);
    
    remove(filepath);
    rename(temp_path, filepath);
    
    return true;
}
