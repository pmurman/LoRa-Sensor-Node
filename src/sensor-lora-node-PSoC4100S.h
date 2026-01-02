/**
 * @file sensor_lora_node_PSoC4100S.h
 * @date 2022-01-21
 */
#if !defined SENSOR_LORA_NODE_PSOC4100S_H
#define SENSOR_LORA_NODE_PSOC4100S_H

#include "sensor-node.h"

#define TMP117_ALERT GPIO0

#define RF_FREQUENCY                868000000 // Hz
#define TX_OUTPUT_POWER             14        // dBm
#define LORA_BANDWIDTH              0         // [0: 125 kHz,
                                              //  1: 250 kHz,
                                              //  2: 500 kHz,
                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR       7         // [SF7..SF12]
#define LORA_CODINGRATE             1         // [1: 4/5,
                                              //  2: 4/6,
                                              //  3: 4/7,
                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

#endif // \!defined SENSOR_LORA_NODE_PSOC4100S_H