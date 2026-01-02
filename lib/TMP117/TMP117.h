/**
 * @file TMP117.h
 */
#ifndef _TMP117_H_
#define _TMP117_H_

#if !defined Sensor_serviced
#define Sensor_serviced(s) (1u << s)
#endif

// Address Pin to Slave Address mapping
#define ADD0_TO_GND             0x48
#define ADD0_TO_VCC             0x49 // (<-- BlueDot 'as is')
#define ADD0_TO_SDA             0x4A
#define ADD0_TO_SCL             0x4B

#define TMP117_RES (double)0.0078125

#define TMP117_MOD_CLR_MASK     0xF3FF 
#define TMP117_AVG_CLR_MASK     0xFF9F
#define TMP117_SOFT_RST         0x0002

#define TMP117_CONF_RD          0x0464 // conf reg readback mask

class TMP117 {

  public:
              TMP117(const uint8_t, const uint8_t, void (*)(void), void (*)(nodeError_t));
    // Register Map
    enum TMP117_reg   { temp_r, conf_r, thl_r, tll_r, eep_ul_r, eep1_r, eep2_r, t_offset_r, eep3_r };
    // Supported Config Register Fields
    enum TMP117_mod   { shutdown = 0x0400, one_shot = 0x0C00 };
    enum TMP117_avg   { no_avg = 0x0000, avg8 = 0x0020, avg32 = 0x0040, avg64 = 0x0060 };
    enum TMP117_alert { drdy = 0x0004 };

    void      initSetup(TMP117_mod mode, TMP117_avg averaging, const bool save_min_max_in_eeprom, uint8_t sensor_id);
    void      init(const bool save_min_max_in_eeprom, uint8_t sensor_id);
    bool      initPowerUpSettings(void);
    void      softReset(void);
    void      setAveraging(TMP117_avg averaging);
    void      startConversion(void);
    void      setOffsetTemperature(double cal_offset);
    int16_t   getTemperature(par temp_par);
    int16_t   readSensor(uint32_t * const sensors_serviced);
 
  private:
    // EEPROM Unlock Register Fields
    enum TMP117_eeprom_ul { eep_unlock = 0x8000, eep_busy = 0x4000 };
  
    const uint8_t address_;
    const uint8_t alertPin_;
    uint8_t   thisSensor_;
    int16_t   actualTemp_;
    int16_t   minTemp_;
    int16_t   maxTemp_;
    bool      saveTemp_;
    int16_t   config_;
    void      (*isr_)(void);
    void      (*error_)(nodeError_t);

    uint16_t  i2cRead2B(TMP117_reg sensor_reg);    
    void      i2cWrite2B(TMP117_reg sensor_reg, int16_t new_val);   
    bool      progEeprom(TMP117_reg eeprom_reg, int16_t new_val, int8_t retries = 2);
};
#endif