/**
 * @file sd_storage.h
 * @brief SD card storage module (SPI mode)
 *
 * Provides SD card initialization, file system mounting, run directory management, etc.
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "data_types.h"

// ==================== Configuration Parameters ====================

// SD card SPI pin configuration
#define SD_PIN_MISO     14
#define SD_PIN_MOSI     13
#define SD_PIN_SCK      12
#define SD_PIN_CS       15

// SD card mount point
#define SD_MOUNT_POINT  "/sdcard"

// Write buffer size
#define WRITE_BUFFER_SIZE   4096    // 4KB

// Path length limits
#define MAX_FILENAME_LEN    64
#define MAX_DIR_PATH_LEN    256  // For directory paths
#define MAX_FILE_PATH_LEN   320  // For full file paths (directory + filename)

// ==================== Data Structures ====================

/**
 * @brief Run session information
 */
typedef struct {
    char run_id[16];                      // Run ID (e.g., "RUN_001")
    char run_path[MAX_DIR_PATH_LEN];      // Full path to run directory
    char fast_file[MAX_FILE_PATH_LEN];    // Fast data file path
    char medium_file[MAX_FILE_PATH_LEN];  // Medium data file path
    char slow_file[MAX_FILE_PATH_LEN];    // Slow data file path
    char meta_file[MAX_FILE_PATH_LEN];    // Metadata file path
    char log_file[MAX_FILE_PATH_LEN];     // Log file path
    bool is_active;                       // Whether session is active
} run_session_t;

/**
 * @brief File write buffer
 */
typedef struct {
    FILE *file;                       // File handle
    uint8_t buffer[WRITE_BUFFER_SIZE]; // Buffer
    size_t buffer_pos;                 // Current buffer position
    uint32_t last_flush_time;          // Last flush time
    bool is_open;                      // Whether file is open
} file_buffer_t;

/**
 * @brief SD card storage manager
 */
typedef struct {
    bool initialized;                  // Whether initialized
    bool mounted;                      // Whether SD card is mounted
    run_session_t current_session;     // Current run session
    file_buffer_t fast_buffer;         // Fast data buffer
    file_buffer_t medium_buffer;       // Medium data buffer
    file_buffer_t slow_buffer;         // Slow data buffer
    uint64_t total_bytes_written;      // Total bytes written
    uint32_t write_error_count;        // Write error count
} sd_storage_t;

// ==================== Function Declarations ====================

/**
 * @brief Initialize SD card (SPI mode)
 *
 * @return true=success, false=failure
 */
bool sd_storage_init(void);

/**
 * @brief Deinitialize SD card
 */
void sd_storage_deinit(void);

/**
 * @brief Create new run session
 * 
 * Automatically generates run ID (RUN_001, RUN_002...), creates directories and files
 * 
 * @return true=success, false=failure
 */
bool sd_create_run_session(void);

/**
 * @brief Close current run session
 * 
 * Flushes all buffers, closes files, updates metadata
 * 
 * @return true=success, false=failure
 */
bool sd_close_run_session(void);

/**
 * @brief Write fast data record
 * 
 * @param record Data record pointer
 * @return true=success, false=failure
 */
bool sd_write_fast_data(const fast_data_record_t *record);

/**
 * @brief Write medium data record
 * 
 * @param record Data record pointer
 * @return true=success, false=failure
 */
bool sd_write_medium_data(const medium_data_record_t *record);

/**
 * @brief Write slow data record
 * 
 * @param record Data record pointer
 * @return true=success, false=failure
 */
bool sd_write_slow_data(const slow_data_record_t *record);

/**
 * @brief Write event log
 * 
 * @param level Log level ("INFO", "WARN", "ERROR")
 * @param message Log message
 */
void sd_write_log(const char *level, const char *message);

/**
 * @brief Flush all buffers to SD card
 * 
 * @return true=success, false=failure
 */
bool sd_flush_all_buffers(void);

/**
 * @brief Get SD card information
 * 
 * @param total_mb Total capacity (MB)
 * @param free_mb Free capacity (MB)
 * @return true=success, false=failure
 */
bool sd_get_card_info(uint64_t *total_mb, uint64_t *free_mb);

/**
 * @brief Get current run session information
 * 
 * @return Run session pointer (read-only)
 */
const run_session_t* sd_get_current_session(void);

/**
 * @brief Get storage statistics
 * 
 * @param bytes_written Bytes written
 * @param error_count Error count
 */
void sd_get_statistics(uint64_t *bytes_written, uint32_t *error_count);

#endif // SD_STORAGE_H