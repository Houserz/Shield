/**
 * @file streamer_config.h
 * @brief Select which live-stream transports are compiled into the firmware.
 *
 * Edit these two switches, then run a normal:
 *
 *     idf.py build flash monitor
 *
 * Common modes:
 *   Wi-Fi only:      USB=0, WIFI=1
 *   USB only:        USB=1, WIFI=0
 *   USB + Wi-Fi:     USB=1, WIFI=1
 *   SD only:         USB=0, WIFI=0
 */
#ifndef STREAMER_CONFIG_H
#define STREAMER_CONFIG_H

#define STREAMER_ENABLE_USB   1
#define STREAMER_ENABLE_WIFI  0

#endif // STREAMER_CONFIG_H
