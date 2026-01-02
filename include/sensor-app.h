/**
 * @file sensor-app.h
 * @author Peter Murman
 * @brief Application configuration
 * @date 30 Dec 2021, 6 Jun 2022, 30 Oct 2022
 */
#if !defined _SENSOR_APP_H_
#define _SENSOR_APP_H_

#include <Arduino.h>

/*
 * Error codes
 */
typedef enum {
  E_UNKNOWN     = 0,                      // (error) Unspecified error
  E_WDT_RST     = 0x1,                    // (error) Watchdog initiated reset
  E_PROT_RST    = 0x2,                    // (error) Protection fault reset
  W_LOW_BATT    = 0x4,                    // (warning) Board supply voltage running low
  W_TEST_MOD    = 0x8,                    // (warning) Test Mode active
  E_NO_DATA     = 0x10,                   // (error) Sensor error - replaced by faulty sensor pattern at tx-time
} nodeError_t;

/**
 * Config Settings - use recommended settings '[...]' for regular battery-powered operation
 */
#define _TO_ 0                            // if true: Terminal monitoring enabled [0]
#define _LED_ 0                           // if true: Use onboard LED(s) [0]
const bool por = 1;                       // if false: run TMP117 Power-Up Reset configuration initialization; set to true when TMP117 POR configured [1]
const uint8_t startupMode = 0;            // Error Test vs. Normal mode - set to 0 for normal operation [0]
                                          // set to E_NO_DATA | W_LOW_BATT | E_WDT_RST for full error test
                                          // E_NO_DATA set: simulate 'no sensor' fault once after startup for all sensors
                                          // E_LOW_BATT set: simulate 'low battery' once after startup
                                          // E_WDT_RST set: provoke WDT reset once after startup
const uint32_t cycle = 900;               // Sensing and transmission cycle time in s [900]
const uint32_t battCheckInterval = 8;     // Battery Voltage check interval in h (0 = every tx cycle) [24]
const bool sendAlways = 0;                // if true: Always send sensor data, otherwise only when changed [0]
const bool sendErrors = 1;                // if true: Transmit errors/warnings [1] 
const bool storeMinMax = 0;               // if true: Update Min/Max temperatures in EEPROM when they change [1]

/*
 * Payload sensor data parameters (between First and End - max. 12 parameters)
 * Position in enum list == position in packet -> put rarely sent parameters just before End
 */
#if defined SFF_NODE
enum par { First = 0, T_NOW = First, T_MIN, T_MAX, End };
#elif defined AB01_NODE
enum par { First = 0, T_NOW = First, PKT_NR, V_BATT, T_MIN, T_MAX, CYCLE_T, End };
#endif
#define MAX_PAYLOAD (1 + 3 + 4 + 12*3)    // error hdr[1] + error_data[3] + data hdr[4] + 12 x data[3]
#define Sensor_serviced(s) (1u << s)
#define All_sensors(s) ((1u << s) - 1)

/*
 * Sensor data template
 */
typedef struct {                               
  int32_t raw;                            // raw sensor value
  par idx;                                // position index in transmission packet (0 = 1st parameter)
  uint8_t sz;                             // sensor data size field [1:0] in bytes (0-3)
} sensor_t;

#endif // \!defined _SENSOR_APP_H_