/*! @addtogroup Timer
@{ */

// RFbeam Microwave GmbH
////////////////////////////////////////////////////////////////////////////////
//  Project      RSP1 Radar Signal Processor
//  Target CPU   Atmel ATxmega32A4U
/*! @file        Timer.h
//  @brief       Software Timer Functions
//  @author      Klaus Mezger
//  @version     18-07-05   Mez     inital
//               10-08-13	mez		cosmetics
//
*///////////////////////////////////////////////////////////////////////////////
#ifndef TIMER_H_
#define TIMER_H_

// Timers used:
//TCC0 : Sampling clock (initialised in AnalogInit.c) 
//TCE0 : SW Tick 


//== standard libraries ==
#include <stdlib.h>
#include <memory.h>
#include "main.h"

//== public defines
#define MAX_SW_TIMERS			4
#define DELAY_TIMER				0
#define KEY_TIMER				1
#define MESSAGE_TIMER			2
#define WD_TIMER                3
#define SW_TIMER_PERIOD			1 //ms

// SW Timer Defines
#define TICK_TIMER				TIM3	//V2.0
//#define TIMER_BASE_FREQ			10		// Hz -> T=100ms

//== public functions
void ecSWTimerInit(void);
void ecSWTimerClk(void);
void ecSWTimerReset(uint8_t TimerNr);
void ecSWTimerStart(uint8_t TimerNr, uint16_t Time);
uint16_t ecSWTimerRead(uint8_t TimerNr);

void ecBlinkHandler(void); //to call from timer interrupt
uint8_t ecSetLED(eLED_t ledNbr, uint16_t blinkPeriod);
void ecLEDinit(void);

#endif /* TIMER_H_ */
/*! @} */ //end of doxygen module group
