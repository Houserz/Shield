/**
 * @file data_types.h
 * @brief Data type definitions for the data acquisition system
 * 
 * Defines core data structures including sensor data packets, queue messages, etc.
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ==================== Queue Configuration ====================
#define FAST_QUEUE_SIZE     200     // 1kHz × 2 sensors × 100ms buffer
#define MEDIUM_QUEUE_SIZE   20      // 200Hz × 1 sensor × 100ms buffer
#define SLOW_QUEUE_SIZE     10      // 50Hz × 2 sensors × 100ms buffer

// ==================== Binary Data Packet Formats ====================

/**
 * @brief Fast data record (IMU + Vibration)
 * 
 * File: fast_data.bin
 * Sample rate: 1kHz
 * Size: 16 bytes/record
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID (0=IMU, 1=Vibration)
    uint8_t reserved[3];      // Padding for alignment
    float data;               // Sensor data
} fast_data_record_t;

/**
 * @brief Medium data record (Current)
 * 
 * File: medium_data.bin
 * Sample rate: 200Hz
 * Size: 8 bytes/record
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    float current;            // Current value (amperes)
} medium_data_record_t;

/**
 * @brief Slow data record (Pressure + Temperature)
 * 
 * File: slow_data.bin
 * Sample rate: 50Hz
 * Size: 12 bytes/record
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID (3=Pressure, 4=Temperature)
    uint8_t reserved[3];      // Padding for alignment
    float data;               // Sensor data
} slow_data_record_t;

// ==================== Queue Message Types ====================

/**
 * @brief Queue message type enumeration
 */
typedef enum {
    QUEUE_MSG_DATA,           // Data message
    QUEUE_MSG_FLUSH,          // Flush buffer
    QUEUE_MSG_STOP            // Stop acquisition
} queue_msg_type_t;

/**
 * @brief Fast queue message
 */
typedef struct {
    queue_msg_type_t type;
    fast_data_record_t data;
} fast_queue_msg_t;

/**
 * @brief Medium queue message
 */
typedef struct {
    queue_msg_type_t type;
    medium_data_record_t data;
} medium_queue_msg_t;

/**
 * @brief Slow queue message
 */
typedef struct {
    queue_msg_type_t type;
    slow_data_record_t data;
} slow_queue_msg_t;

// ==================== System State ====================

/**
 * @brief Data acquisition system state
 */
typedef enum {
    DAQ_STATE_IDLE,           // Idle
    DAQ_STATE_INITIALIZING,   // Initializing
    DAQ_STATE_RUNNING,        // Running
    DAQ_STATE_STOPPING,       // Stopping
    DAQ_STATE_ERROR           // Error
} daq_state_t;

/**
 * @brief Data acquisition statistics
 */
typedef struct {
    uint32_t fast_samples;    // Number of fast samples
    uint32_t medium_samples;  // Number of medium samples
    uint32_t slow_samples;    // Number of slow samples
    uint32_t queue_overruns;  // Queue overflow count
    uint32_t sd_errors;       // SD card error count
    uint32_t duration_ms;     // Running duration (milliseconds)
} daq_statistics_t;

// ==================== Helper Functions ====================

/**
 * @brief Get current system timestamp (milliseconds)
 */
uint32_t get_timestamp_ms(void);

/**
 * @brief Initialize data types module
 */
void data_types_init(void);

// ==================== Metadata Management ====================

/**
 * @brief Create metadata file
 * 
 * Creates meta.json file at the beginning of a run session
 * 
 * @param filepath Metadata file path
 * @param run_id Run ID
 * @return true=success, false=failure
 */
bool metadata_create(const char *filepath, const char *run_id);

/**
 * @brief Update metadata statistics
 * 
 * Updates sampling statistics, running duration, etc. during or at the end of run
 * 
 * @param filepath Metadata file path
 * @param stats Statistics information
 * @return true=success, false=failure
 */
bool metadata_update_statistics(const char *filepath, const daq_statistics_t *stats);

/**
 * @brief Mark run as finished
 * 
 * Updates end time and final statistics
 * 
 * @param filepath Metadata file path
 * @return true=success, false=failure
 */
bool metadata_finalize(const char *filepath);

#endif // DATA_TYPES_H
