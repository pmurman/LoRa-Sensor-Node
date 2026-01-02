/**
 * @file sensor_node.h
 */

#if !defined SENSOR_NODE_H
#define SENSOR_NODE_H

#include <Arduino.h>
#include "payload.h"
#include "TMP117.h"

/* Trigger a data acquisition cycle */
void StartSensors(void);

/* TMP117 sensor #1 data acquisition done */
void TempSensor1Ready(void);

/* Marks beginning of new data acquisition cycle */
void NextCycle(void);

/* Upload data to server */
void Upload(uint8_t *binary_data_packet, const uint8_t packet_bytes);

/* Upload via radio completed */
void UploadDone(void);

/* Include data packet number in payload data */
void AddPktNr(void);

/* Data acquisition time-out handler */
void Timeout(void);

/* Get battery voltage */
int8_t BatteryVoltage(void);

/* Upload error/warning */
void Error(nodeError_t error_warning_code);

/* Control onboard R-G-B leds */
void Led(uint32_t rgb_0xHHH);

#endif // \!defined SENSOR_NODE_H