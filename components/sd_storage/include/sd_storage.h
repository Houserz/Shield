/**
 * @file sd_storage.h
 * @brief SD card storage module (SPI mode)
 *
 * Provides SD card initialization, file system mounting, run directory management, etc.
 *
 * REVIEW-1 CHANGES (C-11 item 1): periodic fsync, checked close with a
 * per-file size report, boot self-test with remount retries, max_files 8,
 * operator markers. See sd_storage.c for the rationale.
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

// Review-1 additions
#define SD_SYNC_INTERVAL_MS   5000   // fsync each tier file at least this often
#define SD_SELFTEST_BYTES     65536  // boot self-test file size
#define SD_SELFTEST_ATTEMPTS  3      // mount + self-test attempts before giving up

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
    uint32_t last_flush_time;          // Last flush time (libc -> VFS)
    uint32_t last_sync_time;           // Last fsync time (VFS -> card)
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
    uint32_t write_error_count;        // fwrite/fflush/fclose error count
    uint32_t sync_error_count;         // fsync error count
    int      selftest_attempts;        // mount attempts used at init
} sd_storage_t;

/**
 * @brief Report of the last closed session (sizes read back from the card)
 */
typedef struct {
    bool     valid;
    bool     close_ok;        // all files closed, synced and non-empty
    uint32_t fast_bytes;
    uint32_t medium_bytes;
    uint32_t slow_bytes;
    uint32_t write_errors;
    uint32_t sync_errors;
    uint32_t log_failures;
} sd_close_report_t;

// ==================== Function Declarations ====================

bool sd_storage_init(void);
void sd_storage_deinit(void);
bool sd_create_run_session(void);

/**
 * @brief Close current run session: flush, fsync, fclose, stat.
 * @return true only if every tier file closed cleanly and is non-empty
 */
bool sd_close_run_session(void);

bool sd_write_fast_data(const fast_data_record_t *record);
bool sd_write_medium_data(const medium_data_record_t *record);
bool sd_write_slow_data(const slow_data_record_t *record);

/**
 * @brief Write event log ("INFO", "WARN", "ERROR", "MARK")
 */
void sd_write_log(const char *level, const char *message);

/**
 * @brief Write an operator marker line to events.log with the record clock
 *        (t_ms, same clock as timestamp_ms in the data files) and esp_timer µs.
 */
void sd_write_marker(const char *label);

bool sd_flush_all_buffers(void);

/**
 * @brief Flush and fsync every open tier file now
 */
bool sd_sync_all(void);

bool sd_get_card_info(uint64_t *total_mb, uint64_t *free_mb);
const run_session_t* sd_get_current_session(void);
void sd_get_statistics(uint64_t *bytes_written, uint32_t *error_count);

/**
 * @brief Copy of the last close report (valid after sd_close_run_session)
 */
bool sd_get_last_close(sd_close_report_t *out);

/**
 * @brief True if any storage error was latched since init
 */
bool sd_storage_fault(void);

int sd_get_selftest_attempts(void);

#endif // SD_STORAGE_H
