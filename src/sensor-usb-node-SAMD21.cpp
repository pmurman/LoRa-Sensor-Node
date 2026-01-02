/*!
 * @author  Peter Murman
 * @date    2 Jan 2021, 14 Jan 2022, 29 Jun 2022, 2 Nov 2022
 * 
 * @brief   Temperature sensor node for remote temperature sensing.
 *
 * @license Licensed under see license.txt
 *
 * v1.1.0   - Dev Version -- Sensor Node with USB and TI TMP117 sensor
 *
 * Collect temperature data using a TMP117 sensor breakout PCB every 15 minutes and transmit via Serial USB port.
 * To reduce data, a new value is sent only if it has changed since the previous value sent.
 *
 * Development done with a Sodaq SFF dev board (Microchip SAMD21 MCU) - courtesy Sodaq B.V. Netherlands.
 *
 * Features:
 * - Low power: only wakes up from deep sleep† to:
 *   a. Trigger a sensor conversion cycle every 15mins, and
 *   b. Transmit sensor data after a sensor Data Ready event
 * - Basic error messaging (e.g. sensor not working)
 * - Support for additional sensor modules
 * - Data sent via Serial port (Base64)
 * - Configurable
 *
 *  † not implemented (Sensor Node is USB powered)
 *
 * Signal wiring between BlueDot TMP117 sensor board‡ and SODAQ SFF dev board
 * - BlueDot TMP117 I2C <==> SODAQ SDA/SCL
 * - BlueDot TMP117 ALERT ---> SODAQ A11
 *
 *  ‡ has on-board 4.7k pullups on all I/Os
 */

#include "sensor-app.h"
#include "sensor-usb-node-SAMD21.h"
#include "interval-timer.h"

const char *devEui = "00000000FDFF0400";
uint8_t errorTest = startupMode; // see Config Settings (sensor-app.h)
static uint8_t sensorCount = 0; // keep track of available sensors
static uint32_t sensorsServiced = 0; // sensor n sets bit[n] when serviced after issuing sensor ready interrupt
static u_int8_t emptyCycles = 0; // # cycles since previous tranmission
static sensor_t nodeData[] {
                      { raw : 0, idx : T_NOW, sz : 2 },
                      { raw : 0, idx : T_MIN, sz : 2 },
                      { raw : 0, idx : T_MAX, sz : 2 },
                    };

TMP117 TempSensor1(ADD0_TO_VCC, TMP117_ALERT, TempSensor1Ready, Error);
Payload Packet(Upload, AddPktNr);

/**
 * @brief Initialization of sensors, LoRa
 */
void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB) ;

#if _LED_
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH); // off
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
#else
#define Led(c)
#endif

  // re-order nodeData so that nodeData[PARx] contains PARx data
  for (uint32_t i = 0; i < sizeof nodeData / sizeof nodeData[0]; ) {
    sensor_t data = nodeData[i];

    if (data.idx != i) { // when data not in its reserved seat
      nodeData[i] = nodeData[data.idx]; // swap seats
      nodeData[data.idx] = data; // data can sit back & relax
      i = 0;
    }
    else
      i++; // data can stay here
  }

  TempSensor1.init(0, sensorCount++);
  nodeData[T_MIN].raw = TempSensor1.getTemperature(T_MIN);
  nodeData[T_MAX].raw = TempSensor1.getTemperature(T_MAX);
  Packet.addSensorPar(nodeData[T_MIN]);
  Packet.addSensorPar(nodeData[T_MAX]);

  if (errorTest)
    Error(W_TEST_MOD);

  if (!por) {
    ; // - sensor Power-Up-Reset programming goes here -
  }

  InitTimer();
  SetTimer(cycle * 32, StartSensors); // timer tick = 1/32 sec
  
  // comment out next line to start after timer delay for sensor acclimatization
  StartSensors(); // start data acquisition
}

/**
 * @brief Main loop
 * When all sensors ready and read-out, node data is encoded/formatted and handed over 
 * to the tranmit/upload handler.
 */
void loop() {
  if (sensorsServiced == All_sensors(sensorCount)) {
    Led(0x00f);
    Packet.sendPacket(nodeData);
    Led(0);
    sensorsServiced = 0;
  }
}

/**
 * @brief Start of sampling cycle - trigger conversion
 */
void StartSensors(void) {
  emptyCycles++;
  if (sensorsServiced & All_sensors(sensorCount) || errorTest & E_NO_DATA) {
    // not all sensor data was successfully read in previous cycle
    Error(E_NO_DATA);
    // send what we've got
    Packet.sendPacket(nodeData);
    TempSensor1.softReset();
  }
  sensorsServiced = 0U;
  TempSensor1.startConversion();
  Led(0x0f0);
}

/**
 * @brief Service TMP117 conversion done interrupt by reading its data
 */
void TempSensor1Ready(void) {
  static uint16_t t_last;
  int16_t t_now = TempSensor1.readSensor(&sensorsServiced);

  if (t_now - t_last > 2 || t_last - t_now > 2 || sendAlways) {
    // only send when changed at least 0.0234°C
    Packet.addSensorPar(nodeData[T_NOW]);
    nodeData[T_NOW].raw = t_now;
    t_last = t_now;
#if _TO_
    SerialUSB.print("temp.: ");
    SerialUSB.print(t_now * TMP117_RES, 2);
    SerialUSB.print("°C, ");
#endif
  }
  Led(0);
}

/**
 * @brief Catch sensor data and upload
 * 
 * @param packet Payload data packet using Payload Data Encoding scheme
 * @param packetSize Payload in bytes
 */
void Upload(uint8_t *packet, uint8_t packetSize) {
  SerialUSB.print("DevEUI=0x");
  SerialUSB.print(devEui);
  SerialUSB.print("&packet=");
  SerialUSB.println(Packet.convert2B64(packet, packet + packetSize));
  emptyCycles = 0;
}

/**
 * @brief Add packet sequence number to payload when App supports this, otherwise leave empty
 */
void AddPktNr(void) {
}

/**
 * @brief Broadcast error/warning code
 * 
 * @param e Error/warning code
 */
void Error(nodeError_t e) {
  if (!sendErrors)
    return;

#if _TO_
  SerialUSB.print("error ");
  SerialUSB.print(e & 0xf);
  SerialUSB.print(", ");
#endif
  Led(0xf00);

  switch (e) {
    case E_NO_DATA:
      if (errorTest & E_NO_DATA)
        sensorsServiced = 0U; // emulate no data from any sensor
      errorTest &= ~e; // turn off error test
      Packet.addError((sensorsServiced ^ All_sensors(sensorCount)) << 8);
      break;

    case W_TEST_MOD:
      Packet.addError(e);
      break;

    default:
      Packet.sendError((uint32_t)E_UNKNOWN);
      break;
  }
}

/**
 * @brief Control R-G-B leds
 * 
 * @param rgb Led control value as 0xH0xH0xH, 0 = no color
 */
#if _LED_
void Led(uint32_t rgb) {
  digitalWrite(LED_RED, !(rgb & 0xf00));
  digitalWrite(LED_GREEN, !(rgb & 0x0f0));
  digitalWrite(LED_BLUE, !(rgb & 0x00f));

  // set min. 'on'-time
  static uint32_t lastHere;
  while (rgb && millis() - lastHere < 100) ;
  lastHere = millis();
}
#endif