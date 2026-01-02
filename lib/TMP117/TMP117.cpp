/*!
 * @author  Peter Murman
 * @date    2 Jan 2021
 *          30 Dec 2021
 * 
 * @brief   Driver for TI's TMP117 temperature sensor.
 * 
 * @license MIT License (see license.txt)
 * 
 * v1.0.0   - Initial version 
 * 
 * TMP117 driver 'Lite', inspired by the implementation by Nils Minor.
 *
 * 
 * TMP117 Lite provides basic functionality for temperature measurements at minimum power.
 * Features:
 * - Provides actual, min and max temperatures
 * - Keeps track of `lowest ever` and `highest ever` min/max temperatures in EEPROM
 * - Temperature data available after a One-Shot conversion:
 *    . Actual temperature
 *    . Min / Max temperature(s)
 * - Data Ready callback
 * - Uses shutdown mode to minimize power consumption (150nA)
 * - Power-On Reset (POR) setting for production use reducing software initialization overhead
 * - Error feedback after EEPROM write failure
 */

#include <Arduino.h>
#include <Wire.h>
#include "sensor-app.h"
#include "TMP117.h"

/**
 * @brief Constructor - setup I2C address, Alert signal pin assignment and interrupt & error callbacks
 *
 * @param a Device I2C address [0x48 - 0x4B]
 * @param p MCU pin to capture TMP117 'Data Ready' at the Alert pin
 * @param f ISR function handling 'Conversion Ready' Alert event
 * @param e Error handler (optional)
 */
TMP117::TMP117(const uint8_t a, uint8_t p, void (*f)(void), void (*e)(nodeError_t)) : address_(a), alertPin_(p), isr_(f), error_(e = nullptr) {
}

/**
 * @brief Entry point when device POR configuration has been setup using initSetup()
 *
 * @param saveMinMax When set, write Min/Max temperatures to EEPROM
 * @param sensorId Sensor # (0-31) assigned to this sensor
 */
void TMP117::init(bool saveMinMax, uint8_t sensorId) {
  saveTemp_ = saveMinMax;
  thisSensor_ = sensorId;
  pinMode(alertPin_, INPUT);
  attachInterrupt(alertPin_, isr_, FALLING);

  Wire.begin();
  while (i2cRead2B(eep_ul_r) & eep_busy) ; // POR sequence

  minTemp_ = i2cRead2B(thl_r);
  maxTemp_ = i2cRead2B(tll_r);
}

/**
 * @brief Modify device POR initialization settings in EEPROM.
 *
 * @param mode Sensor conversion mode (only [SHUTDOWN, ONE_SHOT] are used)
 * @param averaging Number of conversion results to be averaged [NO_AVG, AVG8, AVG32, AVG64]
 * @param saveMinMax When set, write Min/Max temperatures to EEPROM
 * @param sensorId Sensor # (0-31) assigned to this sensor
 */
void TMP117::initSetup(TMP117_mod mode, TMP117_avg averaging, bool saveMinMax, uint8_t sensorId) {
  init(saveMinMax, sensorId);

  // program device confiuration
  config_ = i2cRead2B(conf_r) & TMP117_MOD_CLR_MASK & TMP117_AVG_CLR_MASK;
  config_ |= mode | averaging | drdy; // (+ set Alert pin to data ready flag)
  i2cWrite2B(conf_r, config_);
}

/**
 * @brief Issue sensor reset command (device reloads POR settings)
 */
void TMP117::softReset(void) {
  i2cWrite2B(conf_r, TMP117_SOFT_RST);
}

/**
 * @brief Store current config as Power-Up Reset setting and set TLow/THigh Limit locations to factory values
 * 
 * @returns Error flag
 */
bool TMP117::initPowerUpSettings(void) {
  bool err = false;
  minTemp_ = 0x6000; // +192°C
  maxTemp_ = 0x8000; // -256°C

  if (i2cRead2B(tll_r) != maxTemp_)
    err |= progEeprom(tll_r, maxTemp_);
  if (i2cRead2B(thl_r) != minTemp_)
    err |= progEeprom(thl_r, minTemp_);
  if ((i2cRead2B(conf_r) & TMP117_CONF_RD) != (config_ & TMP117_CONF_RD))
    err |= progEeprom(conf_r, config_);

  return err;
}

#if 0 // not used
/**
 * @brief Set averaging mode (not used)
 *
 * @param avgs Number of averages [NO_AVG, AVG8, AVG32, AVG64]
 */
void TMP117::setAveraging(TMP117_avg avgs) {
  int16_t config = i2cRead2B(conf_r) & TMP117_AVG_CLR_MASK;
 
  i2cWrite2B(conf_r, conf | avgs);
}

/**
 * @brief Set offset temperature (not used)
 *
 * @param target Offset temperature in the range of ±256°C
 */
void TMP117::setOffsetTemperature(double offset) {
  i2cWrite2B(t_offset_r, offset / TMP117_RES);
}
#endif // \0

/**
 * @brief Trigger single temperature conversion cycle
 */
void TMP117::startConversion(void) {
  int16_t config = i2cRead2B(conf_r) & TMP117_MOD_CLR_MASK;

  i2cWrite2B(conf_r, config | one_shot);
}

/**
 * @brief Pass cached temperature value
 *
 * @param p Temperature parameter [T_NOW, T_MIN, T_MAX]
 * @returns Temperature in 0.0078125°C per increment
 */
int16_t TMP117::getTemperature(par p = T_NOW) {
  return p == T_MIN ? minTemp_ : p == T_MAX ? maxTemp_ : actualTemp_;
}

/**
 * @brief Read sensor temperature, update actual and historic min/max values if needed
 *
 * @param sensorsServiced Global status of all sensors - sensor must set 'its' bit when serviced
 * @returns Most recent temperature
 */
int16_t TMP117::readSensor(uint32_t * const sensorsServiced) {
  actualTemp_ = i2cRead2B(temp_r);

  // only update Min / Max when changed at least 6 * 7.8125m°C = .047°C, keeping # EEPROM writes low* and save little energy
  // *) note: storing every .047°C change over a range of 100°C takes 2128 EEPROM writes
  if (actualTemp_ <= minTemp_ - 6) {
    minTemp_ = actualTemp_;
    if (saveTemp_) 
      if (progEeprom(thl_r, minTemp_) && error_ != nullptr)
        error_(nodeError_t(thisSensor_));
  }

  if (actualTemp_ >= maxTemp_ + 6) {
    maxTemp_ = actualTemp_;
    if (saveTemp_)
      if (progEeprom(tll_r, maxTemp_) && error_ != nullptr)
        error_(nodeError_t(thisSensor_));
  }

  *sensorsServiced |= Sensor_serviced(thisSensor_);
  return actualTemp_;
}

/**
 * @brief Write two bytes (16 bits) to TMP117 register
 *
 * @param reg Target register
 * @param data Data to be written into reg
 */
void TMP117::i2cWrite2B(TMP117_reg reg, int16_t data) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(data >> 8);
  Wire.write(data & 0xff);
  Wire.endTransmission();
}

/**
 * @brief Read two bytes (16 bits) from TMP117 register
 *
 * @param reg Target register to read from
 * @returns Register readback
 */
uint16_t TMP117::i2cRead2B(TMP117_reg reg) {
  int16_t data = 0;

  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address_, 2);
  while (Wire.available())
    data = data << 8U | Wire.read();
  return data;
}

/**
 * @brief Program single EEPROM location
 *
 * @param reg Target register to program
 * @param val Register value to program
 * @param retries Number of retries left if programming failed
 * @returns Error flag when programming failed
 */
bool TMP117::progEeprom(TMP117_reg reg, int16_t val, int8_t retries) {
  static bool busy = false;

  if (retries < 0 || busy)
    return true;

  busy = true;
  i2cWrite2B(eep_ul_r, eep_unlock);
  i2cWrite2B(reg, val); // start programming operation
  while (i2cRead2B(eep_ul_r) & eep_busy) ;
  // ≈7ms later

  // issue I2C general-call reset to lock EEPROM and reload R/W registers from EEPROM
  Wire.beginTransmission(0x00); // general call address
  Wire.write(0x06); // reset command
  Wire.endTransmission();

  while (i2cRead2B(eep_ul_r) & eep_busy) ;
  // ≈1.5ms later
 
  int16_t check = i2cRead2B(reg);
  if (reg == conf_r) {
    check &= TMP117_CONF_RD;
    val &= TMP117_CONF_RD;
  }
  busy = false;

  return !(check == val || !progEeprom(reg, val, retries - 1));
}