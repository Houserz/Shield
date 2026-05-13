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
#define FAST_QUEUE_SIZE     600     // Fast-tier queue (raw+processed records)
#define MEDIUM_QUEUE_SIZE   80      // Medium-tier queue (raw+processed records)
#define SLOW_QUEUE_SIZE     20      // Slow-tier queue (raw+processed records)

#define MAX_SENSOR_ID       9

// ==================== Data Kind / Flags ====================

typedef enum {
    DATA_KIND_RAW = 0,
    DATA_KIND_PROCESSED = 1
} data_kind_t;

#define DATA_FLAG_NONE                  0x00u
#define DATA_FLAG_PROCESSED_SAME_AS_RAW 0x01u
#define DATA_FLAG_FILTER_ACTIVE         0x02u
#define DATA_FLAG_FILTER_SPIKE          0x04u

// ==================== Binary Data Packet Formats ====================

/**
 * @brief Unified V2 sensor data record
 *
 * File: fast_data.bin / medium_data.bin / slow_data.bin
 * Size: 20 bytes/record
 * raw = physical-unit sample before processing
 * processed = raw after the active per-sensor processing stage
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID (1..9)
    uint8_t kind;             // data_kind_t: raw or processed
    uint8_t axis_count;       // scalar=1, vector=3
    uint8_t flags;            // DATA_FLAG_* bits
    float data[3];            // scalar: data[0]; vector: x,y,z
} sensor_data_record_v2_t;

typedef sensor_data_record_v2_t fast_data_record_t;
typedef sensor_data_record_v2_t medium_data_record_t;
typedef sensor_data_record_v2_t slow_data_record_t;

#ifdef __cplusplus
static_assert(sizeof(sensor_data_record_v2_t) == 20, "sensor_data_record_v2_t must stay 20 bytes");
#else
_Static_assert(sizeof(sensor_data_record_v2_t) == 20, "sensor_data_record_v2_t must stay 20 bytes");
#endif

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
    uint32_t fast_records;    // Number of fast-tier records stored
    uint32_t medium_records;  // Number of medium-tier records stored
    uint32_t slow_records;    // Number of slow-tier records stored
    uint32_t raw_records;     // Number of raw records stored
    uint32_t processed_records; // Number of processed records stored
    uint32_t sensor_samples[MAX_SENSOR_ID + 1]; // Logical samples by sensor ID
    uint32_t sensor_records[MAX_SENSOR_ID + 1]; // Stored records by sensor ID
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
