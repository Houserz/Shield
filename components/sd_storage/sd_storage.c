/**
 * @file sd_storage.c
 * @brief SD card storage module implementation (SPI mode)
 */

#include "sd_storage.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/stat.h>
#include <sys/unistd.h>
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
 * @brief Flush single file buffer
 */
static bool flush_buffer(file_buffer_t *buffer) {
    if (!buffer->is_open || buffer->buffer_pos == 0) {
        return true;
    }
    
    size_t written = fwrite(buffer->buffer, 1, buffer->buffer_pos, buffer->file);
    if (written != buffer->buffer_pos) {
        storage.write_error_count++;
        return false;
    }
    
    fflush(buffer->file);
    buffer->buffer_pos = 0;
    buffer->last_flush_time = get_timestamp_ms();
    
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

// ==================== Public API Implementation ====================

/**
 * @brief Initialize SD card (SPI mode)
 */
bool sd_storage_init(void) {
    if (storage.initialized) {
        return true;
    }

    ESP_LOGI(TAG, "Initializing SD card");

    esp_err_t ret;

    // Enable internal pull-up on CS pin
    gpio_set_pull_mode(SD_PIN_CS, GPIO_PULLUP_ONLY);

    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_MOSI,
        .miso_io_num = SD_PIN_MISO,
        .sclk_io_num = SD_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "SPI Initialized");

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "SD CARD FAILED, OR NOT PRESENT! Error: %s", esp_err_to_name(ret));
        }
        spi_bus_free(host.slot);
        return false;
    }

    ESP_LOGI(TAG, "SD CARD INITIALIZED.");
    sdmmc_card_print_info(stdout, card);

    storage.initialized = true;
    storage.mounted = true;
    storage.total_bytes_written = 0;
    storage.write_error_count = 0;

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
    
    // Unmount file system
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);

    // Free the SPI bus
    spi_bus_free(SPI2_HOST);

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
    
    if (!storage.fast_buffer.file || !storage.medium_buffer.file || !storage.slow_buffer.file) {
        return false;
    }
    
    // Initialize buffers
    storage.fast_buffer.is_open = true;
    storage.fast_buffer.buffer_pos = 0;
    storage.fast_buffer.last_flush_time = get_timestamp_ms();
    
    storage.medium_buffer.is_open = true;
    storage.medium_buffer.buffer_pos = 0;
    storage.medium_buffer.last_flush_time = get_timestamp_ms();
    
    storage.slow_buffer.is_open = true;
    storage.slow_buffer.buffer_pos = 0;
    storage.slow_buffer.last_flush_time = get_timestamp_ms();
    
    storage.current_session.is_active = true;
    
    sd_write_log("INFO", "Session started");
    
    return true;
}

/**
 * @brief Close current run session
 */
bool sd_close_run_session(void) {
    if (!storage.current_session.is_active) {
        return true;
    }
    
    // Flush all buffers
    sd_flush_all_buffers();
    
    // Close files
    if (storage.fast_buffer.is_open && storage.fast_buffer.file) {
        fclose(storage.fast_buffer.file);
        storage.fast_buffer.is_open = false;
    }
    
    if (storage.medium_buffer.is_open && storage.medium_buffer.file) {
        fclose(storage.medium_buffer.file);
        storage.medium_buffer.is_open = false;
    }
    
    if (storage.slow_buffer.is_open && storage.slow_buffer.file) {
        fclose(storage.slow_buffer.file);
        storage.slow_buffer.is_open = false;
    }
    
    sd_write_log("INFO", "Session closed");
    
    storage.current_session.is_active = false;
    
    return true;
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
 * @brief Write event log
 */
void sd_write_log(const char *level, const char *message) {
    if (!storage.current_session.is_active) {
        return;
    }
    
    FILE *log_file = fopen(storage.current_session.log_file, "a");
    if (log_file) {
        uint32_t timestamp = get_timestamp_ms();
        fprintf(log_file, "[%010"PRIu32"] %s: %s\n", timestamp, level, message);
        fclose(log_file);
    }
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
    if (error_count) *error_count = storage.write_error_count;
}

