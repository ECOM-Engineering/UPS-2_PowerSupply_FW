/*! @addtogroup Timer
@{ */

////////////////////////////////////////////////////////////////////////////////
//  Project      RSP1 Radar Signal Processor
//  Target CPU   STM32xxxx
/*! @file        Timer.c
//  @brief       Software Timer Functions & Frequency Generation
//  @author      Klaus Mezger
//  @version     05-18-07   Mez     inital
//
//  Timebase is interrupt driven by CPU internal timer.
//  Software timers are derived from HW timer interrupt and are only
//  accessible by public access functions.
//  - Number of timers: defined in Timer.h by @ref C_MAX_TIMER
//  - Mutual exclusion: Timers may also be used in interrupt functions
*///////////////////////////////////////////////////////////////////////////////
//== standard libraries ==


//== module headers ==
#include <main.h>
#include "Timer.h"

//== local defines

//== local function prototypes

//== local typedefs

//== module global variables
unsigned short	gSWTimer[MAX_SW_TIMERS]; 

GPIO_t      gLED[LED_COUNT];
uint16_t    gBlinkChangeTime[LED_COUNT];    // [ms]
uint16_t    gBlinkPeriod[LED_COUNT];
uint16_t    gBlinkCount[LED_COUNT];


//== public functions

//////////////////////////////////////////////////////////////////////////////
/*! @brief      Resets all timer to zero (=inactive)
//  @param[in]  TimerNr   Timer identifier from Timer.h
//  @param[out] --
//  @return     --
//  @Version	1.0  2013-02-01 mez
//
*///////////////////////////////////////////////////////////////////////////////
void ecSWTimerInit(void)
{
	uint8_t i;
	for (i = 0; i < MAX_SW_TIMERS; i++)
	{
		gSWTimer[i] = 0;
	}
}


//////////////////////////////////////////////////////////////////////////////
/*! @brief      Resets selected timer to zero (=inactive)
//  @param[in]  TimerNr   Timer identifier from Timer.h
//  @param[out] --
//  @return     --
//  @Version	1.0  2013-02-01 mez
//
*///////////////////////////////////////////////////////////////////////////////
void ecSWTimerReset(uint8_t TimerNr)
{

	uint32_t controlReg; //prevent irq during SW timer operation

	if(TimerNr < MAX_SW_TIMERS){
		// disallow interrupts
		controlReg = __get_PRIMASK(); //save current irq situation
		__disable_irq();

		// Reset Timer		
		gSWTimer[TimerNr] = 0;
		__set_PRIMASK(controlReg); //restore irq situation;
	}
}

//////////////////////////////////////////////////////////////////////////////
/*! @brief      Sets timer to Value
//  @param[in]  TimerNr		Timer identifier from Timer.h
//  @param[in]  Value		Start value in [100ms]
//  @param[out] --
//  @return     --
//  @Version	1.0  2013-02-01 mez
//
*///////////////////////////////////////////////////////////////////////////////
void ecSWTimerStart(uint8_t TimerNr, uint16_t Value)
{
	uint32_t controlReg; //prevent irq during SW timer operation
	
	if(TimerNr < MAX_SW_TIMERS)
	{
		// disallow interrupts
		controlReg = __get_PRIMASK(); //save current irq situation
		__disable_irq();
		
		// Set the Timer
		gSWTimer[TimerNr] = Value;
		
		__set_PRIMASK(controlReg); //restore irq situation;
	}
}

//////////////////////////////////////////////////////////////////////////////
/*! @brief      returns timer value. 0 if period has expired
//  @param[in]  TimerNr   Timer identifier from Timer.h
//  @param[out] --
//  @return     remaining value in ms
//  @Version	1.0  2013-02-01 mez
//
*///////////////////////////////////////////////////////////////////////////////
uint16_t ecSWTimerRead(uint8_t TimerNr)
{
	uint32_t controlReg; //prevent irq during SW timer operation
	uint16_t TimerValue;

	if(TimerNr < MAX_SW_TIMERS)
	{
		// disallow interrupts
		controlReg = __get_PRIMASK(); //save current irq situation
		__disable_irq();

		// get the Timer				
		TimerValue = gSWTimer[TimerNr];
		__set_PRIMASK(controlReg); //restore irq situation;

		return (TimerValue);
	}
	else{
		return(0xFFFF);
	}
}

////////////////////////////////////////////////////////////////////////////////
/*! @brief      Software Timer Overflow from ISR
//  @param[in]  --
//  @param[out]
//  @Version      1.0  2013-02-01 mez
//
*///////////////////////////////////////////////////////////////////////////////
void ecSWTimerClk(void)
{
	uint8_t TimerNr;
	// decrement software timers
	for(TimerNr = 0; TimerNr < MAX_SW_TIMERS; TimerNr++)
	{
		if(gSWTimer[TimerNr])
		{
			gSWTimer[TimerNr]--;
		}
	}
}


/**
  * @brief  		Public:
  * @param[in]		none
  * param[out]		None
  * @retval 		None
  */
void ecLEDinit(void)
{
    gLED[LED_Pi].port =  LED_Pi_GPIO_Port;
	gLED[LED_Pi].pin = LED_Pi_Pin;
    gLED[LED_Batt].port =  LED_Batt_GPIO_Port;
    gLED[LED_Batt].pin = LED_Batt_Pin;
    gLED[LED_Main].port =  LED_Main_GPIO_Port;
    gLED[LED_Main].pin = LED_Main_Pin;
//    gLED[LED_NUCLEO].port = LED_GPIO_Port;
//    gLED[LED_NUCLEO].pin = LED_Pin;
    /* ... let compiler set all globals to null*/
}

/**
  * @brief  		Public:
  * @param[in]		ledNbr:         LED number (from enum eLED)
  * @param[in]      blinkPeriod:    max 32s in ms. 0: LED_OFF, 0xFFFF: LED_ON
  * param[out]		None
  * @retval 		state of the requested LED
  */
uint8_t ecSetLED(eLED_t ledNbr, uint16_t blinkPeriod)
{
  if(ledNbr < LED_COUNT)
  {
      if(blinkPeriod == LED_ON)
      {
           gBlinkChangeTime[ledNbr] = LED_ON; //prevents blinking
           gBlinkPeriod[ledNbr] = 0;
           HAL_GPIO_WritePin(gLED[ledNbr].port, gLED[ledNbr].pin,  GPIO_PIN_SET);
      } else if(blinkPeriod == LED_OFF)
      {
          gBlinkChangeTime[ledNbr] = LED_OFF; //prevents blinking
          gBlinkPeriod[ledNbr] = 0;
          HAL_GPIO_WritePin(gLED[ledNbr].port, gLED[ledNbr].pin,  GPIO_PIN_RESET);
      }else
      {
          if(gBlinkPeriod[ledNbr] !=  blinkPeriod ) //do not overwrite if already set
          {
              gBlinkPeriod[ledNbr] = blinkPeriod;
              gBlinkChangeTime[ledNbr] = blinkPeriod / 2;
          }
      }
  }
  return HAL_GPIO_ReadPin(gLED[ledNbr].port, gLED[ledNbr].pin);
}

/**
  * @brief  		called by timer interrupt [1ms]
  * @param[in]		*htim
  * param[out]		None
  * @retval 		None
  */
void ecBlinkHandler(void)
{
    eLED_t ledNbr;
    for(ledNbr = 0; ledNbr < LED_COUNT; ledNbr++)
    {
        if((gBlinkChangeTime[ledNbr] > 0) && (gBlinkChangeTime[ledNbr] < 0xFFFF))
        {
            gBlinkChangeTime[ledNbr] --;
            if(gBlinkChangeTime[ledNbr] == 1)
            {
                HAL_GPIO_TogglePin(gLED[ledNbr].port, gLED[ledNbr].pin);
                gBlinkChangeTime[ledNbr] = gBlinkPeriod[ledNbr] / 2;
            }
        }
    }
}

/*! @} */ //end of doxygen module group
