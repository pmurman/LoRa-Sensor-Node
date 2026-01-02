/*!
 * @file interval-timer.h
 */

#ifndef _INT_TIMER_H_
#define _INT_TIMER_H_

#if defined CY_PSOC4_4100
#define WCO_WDT_ENABLED_MASK  0x00020202 // T0,1,2 enabled status readback mask
#define WCO_WDT_RESET_MASK    0x00080808 // T0,1,2 reset status readback mask
typedef void (**vectorTable_t)();
static void (*relayCall_)(void);

#elif defined SAMD21
#define GCLK_SRC 4 // use clockgen 4 for Timer (0, 1, 3 already used by SODAQ SFF firmware)
#define TimerISR TC4_Handler // Microchip ISR naming convention
uint16_t countReg(void); // dev use only
#endif

bool InitTimer(void);
void SetTimer(uint32_t _t_x_2exp_min15_sec, void (* callback)(void));
void TimerISR(void);

#endif // \_INT_TIMER_H