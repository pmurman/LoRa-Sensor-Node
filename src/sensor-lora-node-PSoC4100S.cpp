/*!
 * @author  Peter Murman
 * @date    2 Jan 2021, 14 Jan 2022, 9 Apr 2022, 6 Jun 2022, 27 Oct 2022
 * 
 * @brief   Temperature sensor node for remote temperature sensing.
 *
 *
 * @license Licensed under see license.txt
 *
 * v1.1.0   - Dev Version -- Heltec HTCC-AB01 (V1) with LoRa radio + TI TMP117 sensor
 *
 * Collect temperature data using a BlueDot TMP117 sensor breakout at regular intervals
 * (typically every 15 minutes), and broadcast data via a LoRa radio.
 * LoRa module: Heltec CubeCell AB01 Dev-Board v1.2.
 * Temperature sensor module: BlueDot TMP117 sensor breakout.
 * To reduce power consumption, data is only sent if it has changed since the previous
 * sample sent.
 * This version supports only LoRa - not LoRaWAN.
 *
 * Features:
 * - Low power: only wakes up from deep sleep to:
 *   a. Trigger a sensor conversion cycle every 15 mins, and
 *   b. Transmit sensor data after a sensor Data Ready event or sensor time-out
 * - Error reporting (e.g. sensor not working)
 * - Watchdog support
 * - Auto-sensing of lost packages and cycle time by receiver
 * - Multi-sensor support
 * - Data and errors sent via LoRa radio (binary) and USB Serial port (Base64)
 * - Configurable settings for error condition testing, cycle-time, data via
 *   USB Serial port, send on change, battery check, and more.
 *
 * Signal wiring between TMP117 sensor board and Heltec CubeCell-AB01
 * - BlueDot TMP117 I2C <==> Heltec HTCC-AB01 SDA/SCL
 * - BlueDot TMP117 ALERT ---> Heltec HTCC-AB01 GPIO0
 *
 * ASR6501 contains a PSoC4100S Plus (128KB Flash + 16KB RAM) and a SemTech SX1262 LoRa TransCeiver
 * (source: SemiMedia, http://www.semimedia.cc/?p=2955)
 * Reference docs:
 *  "Infineon-PSoC_4100S_and_PSoC_4100S_Plus_PSoC_4_Architecture_TRM-AdditionalTechnicalInformation-v12_00-EN.pdf"
 *  "Infineon-PSoC_4100S_Plus_Registers_Technical_Reference_Manual-AdditionalTechnicalInformation-v04_00-EN.pdf"
 */

#include "sensor-app.h"
#include "sensor-lora-node-PSoC4100S.h"
#include <CubeCell_NeoPixel.h>
#include <LoRaWan_APP.h>

uint8_t errorTest = startupMode; // see Config Settings (sensor-app.h)
static uint8_t resetReason;
uint8_t devEui[8];
static char hexEui[sizeof "1122334455667788"];

static bool checkBattReq = true;
TimerEvent_t cycleTimer;
TimerEvent_t sensorTimeout;

static uint8_t sensorCount = 0; // # of registered sensors
static uint32_t sensorsServiced; // sensor n sets bit[n] when 'data ready' IRQ serviced
static u_int8_t emptyCycles = 0; // # cycles since previous tranmission
static sensor_t nodeData[] {
                      { raw : 0, idx : T_NOW, sz : 2 },
                      { raw : 0, idx : T_MIN, sz : 2 },
                      { raw : 0, idx : T_MAX, sz : 2 },
                      { raw : 0, idx : PKT_NR, sz : 1 },
                      { raw : 0, idx : V_BATT, sz : 1 },
                      { raw : cycle, idx : CYCLE_T, sz : 2 },
                    };
static RadioEvents_t RadioEvents;

TMP117 TempSensor1(ADD0_TO_VCC, TMP117_ALERT, TempSensor1Ready, Error);
Payload Packet(Upload, AddPktNr);
#if _LED_
CubeCell_NeoPixel Pixels(1, RGB, NEO_GRB + NEO_KHZ800);
#endif

/**
 * @brief Initialization of sensors, LoRa
 */
void setup() {
  CySysWdtEnable();
#if _TO_
  Serial.begin(115200);
#endif

#if _LED_
  // SK6812 RGB Leds
  pinMode(Vext, OUTPUT);
  digitalWrite(VBAT_ADC_CTL, HIGH);
  digitalWrite(Vext, LOW); // set power
  Pixels.begin(); // init NeoPixel strip
  Pixels.clear(); // all pixels off
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

  LoRaWAN.generateDeveuiByChipID(); // puts it in devEui[]
#if _TO_
  // also needed in ASCII when uploading via Serial
  char *p = hexEui;
  for (auto hh : devEui) {
    *p++ = "0123456789abcdef"[hh >> 4U];
    *p++ = "0123456789abcdef"[hh & 0x0f];
  }
  *p = '\0';
#endif

  RadioEvents.TxDone = UploadDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(
    MODEM_LORA,
    TX_OUTPUT_POWER,
    0,
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    true,
    0,
    0,
    LORA_IQ_INVERSION_ON,
    3000
  );

  if (errorTest)
    Error(W_TEST_MOD);

  if (!por) {
    Led(0x005); // blue = initializing TMP117
    TempSensor1.initSetup(TMP117::shutdown, TMP117::avg8, 0, sensorCount++);
    if (TempSensor1.initPowerUpSettings())
      Led(0x500); // red = initialization failed (EEPROM write error)
    else {
      Led(0x050); // green = TMP117 initialized: set `por` to 1 and rebuild
      CySysWdtDisable();
      while (true == true) ;
    }
  }
  else
    TempSensor1.init(storeMinMax, sensorCount++);
  // include sensor Min/Max values and cycle time in 1st data transmission  
  nodeData[T_MIN].raw = TempSensor1.getTemperature(T_MIN);
  nodeData[T_MAX].raw = TempSensor1.getTemperature(T_MAX);
  Packet.addSensorPar(nodeData[T_MIN]);
  Packet.addSensorPar(nodeData[T_MAX]);
  Packet.addSensorPar(nodeData[CYCLE_T]);

  resetReason = CySysGetResetReason(CY_SYS_RESET_WDT | CY_SYS_RESET_PROTFAULT);
  if (resetReason & CY_SYS_RESET_PROTFAULT)
    Error(E_PROT_RST);
  else if (resetReason & CY_SYS_RESET_WDT)
    Error(E_WDT_RST);

  TimerInit(&cycleTimer, NextCycle);
  TimerInit(&sensorTimeout, Timeout);

  // in testmode, provoke 'no data' error by setting non-existing sensor to 'serviced'
  sensorsServiced = errorTest & E_NO_DATA ? 1 << sensorCount : 0;
  StartSensors();
}

/**
 * @brief Main loop - transmit data when all sensors reported ready - go to sleep
 */
void loop() {
  if (sensorsServiced == All_sensors(sensorCount)) {
    sensorsServiced = 0;
    TimerStop(&sensorTimeout);

    if (checkBattReq) {
      nodeData[V_BATT].raw = BatteryVoltage();
      Packet.addSensorPar(nodeData[V_BATT]);
      checkBattReq = false;
    }
    if (nodeData[V_BATT].raw < 89 || errorTest & W_LOW_BATT) // < 3.0 V
      Error(W_LOW_BATT);

    if (emptyCycles >= 24 * 3600 / cycle) // send heartbeat when 24h since last tx
      Packet.addSensorPar(nodeData[PKT_NR]);
    Led(0x00f);
    Packet.sendPacket(nodeData);
    Led(0);
    
    if (!(errorTest & E_WDT_RST))
      CySysWdtDisable();
  }

#if _TO_
  Serial.flush();
  unsigned long w = millis();
  while (millis() - w < 2) ;
#endif

  lowPowerHandler(); // back to deep sleep
}

/**
 * @brief Cycle lapsed
 */
void NextCycle(void) {
  static int32_t nextBattCheck;
  nextBattCheck += cycle;
  if (nextBattCheck >= battCheckInterval * 3600) {
    nextBattCheck = 0;
    checkBattReq = true;
  }

  StartSensors();
}

/**
 * @brief Start collecting data and transmission cycle
 */
void StartSensors(void) {
  CySysWdtClearInterrupt(); // feed dog before releasing it
  CySysWdtEnable();

  emptyCycles++;

  TimerSetValue(&cycleTimer, cycle * 1000);
  TimerStart(&cycleTimer);
  TimerSetValue(&sensorTimeout, 2000); // timeout must be less than 3 WDT cycles (≈6s)
  TimerStart(&sensorTimeout);

  TempSensor1.startConversion();
  Led(0x050);
}

/**
 * @brief Service TMP117 conversion done interrupt by reading its data
 */
void TempSensor1Ready(void) {
  static int16_t t_last;
  int16_t t_now = TempSensor1.readSensor(&sensorsServiced);

  if (t_now - t_last > 2 || t_last - t_now > 2 || sendAlways) {
    // only pass when changed at least 3 * 7.8125m°C = .0234°C
    Packet.addSensorPar(nodeData[T_NOW]);
    nodeData[T_NOW].raw = t_now;
    t_last = t_now;
#if _TO_
    Serial.print("temp. ");
    Serial.print(t_now * TMP117_RES, 2);
    Serial.print("°C, ");
#endif
  }
}

/**
 * @brief Enable transmission of available data when not all sensors finished in time
 * (Attempts to use WDT IRQs instead of TimerEvent_t didn't succeed - unexpected behavior)
 */
void Timeout(void) {
  // try to 'fix' sensors that did not respond
  TempSensor1.softReset();

  Error(E_NO_DATA);
  sensorsServiced = All_sensors(sensorCount); // set 'ready to transmit' (+ avoid WDT reset)
}

/**
 * @brief Payload callback to transmit/upload payload data
 * 
 * @param packet Payload data packet
 * @param packetSize Payload in bytes
 */
void Upload(uint8_t *packet, const uint8_t packetSize) {
#if _TO_
  Serial.printf("DevEUI=0x%s&packet=%s\n", hexEui, Packet.convert2B64(packet, packet + packetSize));
#endif
  // TODO: (lorawan) in case of an FPort encoded PDE header, set appPort to packet[0]
  // Only activate when the receiver supports it also and uses correct FPort coding scheme
  Radio.Send((uint8_t *)packet, packetSize);
  emptyCycles = 0;
}

/**
 * @brief Put radio in sleep mode when transmission done (pwr goes down from 2.2mA to 7uA)
 */
void UploadDone(void) {
  Radio.Sleep();
}

/**
 * @brief Payload callback to include packet sequence number in the packet 
 * (leave function empty when packet sequence number not used)
 */
void AddPktNr(void) {
  Packet.addSensorPar(nodeData[PKT_NR]);
  nodeData[PKT_NR].raw++;
#if _TO_
  Serial.print("pkt# ");
  Serial.print(nodeData[PKT_NR].raw);
  Serial.print(", ");
#endif 
}

/**
 * @brief Measure battery voltage and scale down to 7-bit
 * For ADC in single-ended, unsigned mode:
 *  Vplus     value
 *  0         0
 *  Vref      0x800 (2048)
 *  2 x Vref  0xfff (4095)
 * 
 * Vref = 1.2 V -> max. ADC input = 2.4 V -> Vbat max. = 4.8 V
 * Resolution: 4800 / 4095 = 1,17 mV/increment
 * (Measured: ADC value 3551 / Vbat 4.13V = 1,16 mV/increment)
 * 
 * Expected max. Vbat ≈ 4.3 V, or 34 mV/increment using 7-bit
 * (representable voltage range: 0 - 4.318 V)
 *
 * @return voltage as 7-bit value
 */
int8_t BatteryVoltage(void) {
  pinMode(VBAT_ADC_CTL, OUTPUT);
  digitalWrite(VBAT_ADC_CTL, LOW);

  // ADC value to 7-bit scaling: 3684 (≈ ADC value @ 4.318 V) / 127 = 29
  int8_t v = (2 * analogRead(ADC) + 29) / 58; // rounding & scaling
  digitalWrite(VBAT_ADC_CTL, HIGH);
  pinMode(VBAT_ADC_CTL, INPUT);

  return v;
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
  Serial.printf("error %d, ", e & 0xf);
#endif
  Led(0x500);

  switch (e) {
    case E_NO_DATA:
      if (errorTest & E_NO_DATA)
        sensorsServiced = 0U; // emulate no data from any sensor
      errorTest &= ~e; // turn off error test
      Packet.addError((sensorsServiced ^ All_sensors(sensorCount)) << 8);
      break;

    case W_LOW_BATT:
      errorTest &= ~e; // turn off error test
    case W_TEST_MOD:
      Packet.addError(e);
      break;

    case E_WDT_RST:
      errorTest &= ~e; // turn off error test
    case E_PROT_RST: 
      Packet.sendError(e);
      break;

    default:
      Packet.sendError((uint32)E_UNKNOWN);
      break;
  }
}

/**
 * @brief Prep for TX
 * 
 * github.com/angelnu/LorawanSensor
 * 
 * general LoRaWAN (lw) call-flow:
 * deviceState -- probably defined in LoRaWAN...
 * deviceState = INIT;
 * lw.ifskipjoin(); ?? is this to skip join - likely so
 * INIT : lw.init(A, EU868); -> JOIN
 * 
 * JOIN : lw.join(); state changed by lorawan/mac handler to SEND
 * 
 * SEND : lw.send(); -> SLEEP
*/

/**
 * @brief Set R-G-B leds - wait until 200ms since last setting
 * (also helps to see short flashes)
 * 
 * @param rgb Led control value as 0xhhh, no color when h = 0
 */
#if _LED_
void Led(uint32_t rgb) {
  static uint32_t lastHere;

  while (millis() - lastHere < 200) ;
  lastHere = millis();

  if (rgb == 0)
    Pixels.clear();
  else
    Pixels.setPixelColor(0, Pixels.Color(rgb >> 8 & 0xf, rgb >> 4 & 0xf, rgb & 0xf));
  Pixels.show();
}
#endif
