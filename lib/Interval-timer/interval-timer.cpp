/*!
 * @author  Peter Murman
 * @date    13 Jan 2021 (SAMD21) / 14 Jan 2022 (ASR6501 / PSoc 4100 Plus)
 * 
 * @brief   15 minute periodic timer
 * 
 * @license Licensed under see license.txt
 *
 *
 * - Sensor Sample Interval Timer: a free-running timer with an interval period of 15 minutes which
 * continues running in deep sleep mode (PSoc 4100S Plus) and wakes up the system at the end of each interval.
 * 
 * -- SAMD21 (Microchip Technology Inc.) --
 * TC4 (16-bit timer/counter) is used as interval timer.
 * The timer must be clocked with less than 72.8 ticks/s (65,536/900) to cover a 900s interval.
 * By setting the clock prescaler to 1,024 the timer is clocked with 32Hz ticks, using the watch osc.
 * The sample timer period must be 900s x 32ticks/s = 28,800 ticks (counter max = 28,800 - 1).
 * 
 * -- PSoc 4100S Plus (Infineon Technologies AG) --
 * ===> Only use this Timer() implementation when Heltec Asr_Timer() not available
 * - put timers WDT0 and WDT1 in cascade to serve as interval timer;
 * - WDT0,1 timer clocked by the WCO at 32,768Hz.
 * - for 15min: set to 900s x 32,768ticks/s = 294,912,000 ticks (counter max = 294,912,000 - 1).
 */
#include "sensor-app.h"
#include "interval-timer.h"

static void (*wakeup_)(void);


#if defined SAMD21

/**
 * @brief Initialize SAMD21 timer/counter TC4 at startup as interval timer
 * 
 * @returns false
 */
bool InitTimer(void) {
  // (GENDIV reset value is ok)

  // select external 32.768k clock, run in standby, enable clockgen
  GCLK->GENCTRL.reg = SYSCTRL_XOSC32K_RUNSTDBY |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_XOSC32K |
                      GCLK_CLKCTRL_ID(GCLK_SRC);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) ;

  // use clkgen to clock TC4/TC5 pair
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(GCLK_SRC) |
                      TC4_GCLK_ID;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) ;

  // enable APBC clock for TC4
  PM->APBCMASK.reg |= PM_APBCMASK_TC4;

  // @reset already set as 16-bit up-counter
  // counter clock = 32768 / 1024 = 32 Hz (31.25 ms), MFRQ = wrap timer around at CC0
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_RUNSTDBY |
                           TC_CTRLA_PRESCALER_DIV1024 |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY) ;

  NVIC_SetPriority(TC4_IRQn, 0x01);
  NVIC_EnableIRQ(TC4_IRQn);

  return false;
}

/**
 * @brief Set & start periodic wakeup timer
 * 
 * @param interval Timer ticks between two wakeups
 * @param action Callback on wakeup after interval
 */
void SetTimer(uint32_t interval, void (*action)(void)) {
  TC4->COUNT16.CC[0].reg = (uint16_t)interval - 1;
  while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY) ;
  TC4->COUNT16.INTENSET.reg = TC_INTFLAG_MC0; // go

  wakeup_ = action;
}

/**
 * @brief Handle TC4 interrupt
 */
void TimerISR(void) {
  if (TC4->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0) {
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0; // clr irq flag
    wakeup_();
  }
  else
    TC4->COUNT16.INTFLAG.reg = 0x3B; // clr all
}

/**
 * @brief (development/testing only) Read timer COUNT register
 * 
 * @returns Timer COUNT value
 */
uint16_t CountReg(void) {
  TC4->COUNT16.READREQ.reg = TC_READREQ_RREQ | // read sync request
                             TC_READREQ_ADDR(0x10); // for COUNT register
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY) ;

  return TC4->COUNT16.COUNT.reg;
}

#elif defined CY_PSOC4_4100 // \SAMD21

/**
 * @brief Setup interval timer using WDT0,1 timers - assuming WCO clock already done
 * 
 * @param interval Timer ticks between two wakeups
 * @param isr Timer interrupt handler
 * @param action Callback on wakeup after interval
 * @returns Error flag if setup failed
 */
bool InitTimer(void) {
  __disable_irq();
  vectorTable_t v = *(vectorTable_t *)0xE000ED08; // Vector Table Offset Reg
  relayCall_ = v[WDT_IRQn]; // use 29 if WDT_IRQn not included in CMSIS
  v[WDT_IRQn] = TimerISR;
  __enable_irq();

  uint32_t timeLeft = 2000; // actual timeLeft ticks required: 2 x 300-500 (@fcpu = 48MHz)
  *(reg32 *)CYREG_WCO_WDT_CONTROL &= 0xFFFEFEFE; // disable T0,1,2 before writing to CONFIG reg
  while (timeLeft && (*(reg32 *)CYREG_WCO_WDT_CONTROL & WCO_WDT_ENABLED_MASK) != 0ul) // takes a while
    timeLeft--;

  // set T0 mode to 'NOTHING' and free running - put T0,1 in cascade - set T1 mode to 'INT' and clear on match
  *(reg32 *)CYREG_WCO_WDT_CONFIG = *(reg32 *)CYREG_WCO_WDT_CONFIG & 0xFFFF0000 | 0x0508;
  *(reg32 *)CYREG_WCO_WDT_CONTROL = 0x00010909; // reset T0,1 and enable T0,1,2
  while (timeLeft && (*(reg32 *)CYREG_WCO_WDT_CONTROL & (WCO_WDT_ENABLED_MASK | WCO_WDT_RESET_MASK)) != WCO_WDT_ENABLED_MASK)
    timeLeft--;

  return timeLeft == 0;
}

/**
 * @brief Set timeout interval and timeout callback
 */
void SetTimer(uint32_t interval, void (*action)(void)) {
  *(reg32 *)CYREG_WCO_WDT_MATCH = interval - 1;
  wakeup_ = action;
}

/**
 * @brief Handle WDTx interrupt
 */
void TimerISR(void) {
  const uint32_t source = *(reg32 *)CYREG_WCO_WDT_CONTROL & (CY_SYS_TIMER0_INT | CY_SYS_TIMER1_INT | CY_SYS_TIMER2_INT);
  if (source == CY_SYS_TIMER2_INT) {
    // not our call
    relayCall_();
    return;
  }

  // clear interrupt(s) and mandatory readback until verified
  // ("...WDT_CONTROL must be read for the hardware to internally remove the clear flag")
  while (*(reg32 *)CYREG_WCO_WDT_CONTROL & (CY_SYS_TIMER1_INT | CY_SYS_TIMER0_INT))
    *(reg32 *)CYREG_WCO_WDT_CONTROL |= CY_SYS_TIMER1_INT | CY_SYS_TIMER0_INT;

  wakeup_();
}

#endif // \CY_PSOC4_4100