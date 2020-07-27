////////////////////////////////////////////////////////////////////////////////
//  Project      UPS-2 Raspberry pi power supply
/*! @file        targetComm.c
//  @brief       Software Timer Functions & Frequency Generation
//  @author      Klaus Mezger
//  @version     1
//
*///////////////////////////////////////////////////////////////////////////////
//== standard libraries ==


//== module headers ==
#include <main.h>
#include <stdio.h>


#include "UPS-2.h"
#include "interface.h"
#include "Timer.h"



//== local defines
#define PORT_SHORT_PULSE 100



//== local function prototypes

//== local typedefs

//== module global variables
char strMessage[81];
uint8_t g_PI_watchdog_Ok;



//== public functions
/**
  * @brief          get STM32g0x0 CPU temperature using fixpoint
  * @param[in]      ADC value, 12bit
  * @param[out]     none
  * @retval         temperature in 1/10°Celsius
  * @Author         Klaus Mezger
*/

/* Tested with STM32G070 and g030.
 * This routine is fast because of using fixoint arithmetic.
 * Works without TS_CAL2, because this is not mentioned in the datasheets of STM32G030 and g070.
 * Even with using TS_CAL2 from STM32g031 / g071 you do possibly not get better results
 *
*/
uint16_t ecGetCPUTemp(uint16_t ADCvalue)
{
    /* 130° and 30°C value from STM32G31 */
    #define TEMP30_CAL  *(uint16_t*) 0x1FFF75A8
    #define TEMP130_CAL *(uint16_t*) 0x1FFF75CA //Not existent in STM32G0x0 DS, used here as test reference
    #define REF_TEMP 30         //calibration from datasheet

    /* decimal fixpoint d.1 */
    #define SLOPE_Y         25  //10*mV /°C from datasheet
    #define SLOPE_Y_INV     (int32_t)(100000/SLOPE_Y) //10 * inverse of slope
    #define VDDA_10d1       33  //VDDA 3.3V
    #define VREF_CAL_10d1   30  //VDDA 3.0V reference used in factory TS_CALx values

    uint32_t cal30;
    int16_t temp_10d1;  //resulting temperature
    int32_t adc_0;

    /* calc 30°C reference using VDDA 3.3V */
    cal30 = (TEMP30_CAL * VREF_CAL_10d1 /  VDDA_10d1);                  //ADC at 30° @ VDDA = 3.3V

    /* calc ADC value @ 0°C using typical TS slope from datasheet */
    adc_0 =  cal30 - ((REF_TEMP * SLOPE_Y << 12) / VDDA_10d1) / 1000;   //calculate offset @ 0°C

    /* calculate temperature based on TS offset @ 0°C */
    temp_10d1 = (ADCvalue - adc_0) * ((SLOPE_Y_INV * VDDA_10d1) >> 12) / 10;

    // original formula needs float temp = ((130-30)/(TS_CAL2 - TS_CAL1)) * (TS_DATA * TS_CAL1) + 30
    return temp_10d1;
}


/**
  * @brief      Evaluate Pi state, feedback state and transitions by LED
  * @param[in]  keyPress
  * @param[in]  actual Pi state
  * @param[out] new Pi state
  * @retval     None
*/
ePiState_t ecPortHandleFeedback(eKeyPress_t keyPress, ePiState_t piState)
{
    static uint8_t preparePiPower_off;
    uint8_t piACK_Port, piCMD_Port;
    ePiState_t old_piState;
    ePiPortState_t piPortState;
    static ePiPortState_t old_piPortState;

    old_piState = piState;
    if(piState == PI_STATE_UNKNOWN)
        old_piPortState = 0xff;

    /* read both ports in order to evaluate the general state of pi */
    piACK_Port = HAL_GPIO_ReadPin(ACK_IN_GPIO_Port , ACK_IN_Pin);
    piCMD_Port = HAL_GPIO_ReadPin(CMD_OUT_GPIO_Port, CMD_OUT_Pin);
    if (piCMD_Port) //check if no spike on this line during startup
    {
        ecSWTimerStart(DELAY_TIMER,10);
        while(ecSWTimerRead(DELAY_TIMER)); // wait
        piCMD_Port = HAL_GPIO_ReadPin(CMD_OUT_GPIO_Port, CMD_OUT_Pin);
    }

    piPortState = piACK_Port + (piCMD_Port << 1);

    if(keyPress == KEY_NO_PRESS) //supress port changes caused by command
    {
        if(piPortState != old_piPortState)
        {
            sprintf(strMessage, "CMD % d ACK %d \r\n",piCMD_Port, piACK_Port );
            ecTxString(strMessage, strlen(strMessage));
            old_piPortState = piPortState;
        }
    }
    switch(piPortState)
    {
        case CMD0_ACK0:
            if(piState != PI_PWR_OFF)
            {
                piState = PI_STBY;
                sprintf(strMessage, "Stand by\r\n");
            }
             ////ecSetLED(LED_NUCLEO, LED_OFF);
             ecSetLED(LED_Pi, LED_OFF);

             if(keyPress == KEY_SHORT_PRESS) //toggle Pi power to get it restarted
             {
                 HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_RESET); //5V enable = low: power OFF
                 ecSWTimerStart(DELAY_TIMER,200);
                 while(ecSWTimerRead(DELAY_TIMER)); // wait
                 HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_SET); //5V enable = high: power ON
             }

            if(preparePiPower_off)
             {
                 ecSWTimerStart(DELAY_TIMER,2000);
                 while(ecSWTimerRead(DELAY_TIMER)); //safety  wait
                 sprintf(strMessage, "Pi Power OFF\r\n");
                 HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_RESET); //5V enable = low = OFF
                 preparePiPower_off = 0;
                 piState = PI_PWR_OFF;
//                 HAL_PWR_EnterSTANDBYMode();
             }
            break;

        case CMD0_ACK1:

            if(keyPress == KEY_PENDING)
            {
                ecSetLED(LED_Pi, 1000);            //signal key press
            }

            break;
        case CMD1_ACK0:
            if(piState == PI_RUNNING)
             {
                  piState = PI_SHUTTING_DOWN;
                  sprintf(strMessage, "Shutting down\r\n");
                  ////ecSetLED(LED_NUCLEO, 250);
                  ecSetLED(LED_Pi, 250);

             }
             if(piState == PI_STBY)
             {
                 piState = PI_STARTING_UP;
                 sprintf(strMessage, "Starting Up\r\n");
                 ////ecSetLED(LED_NUCLEO, 150);
                 ecSetLED(LED_Pi, 150);
             }
             if(piState == PI_PWR_OFF)
              {
                  piState = PI_STARTING_UP;
                  sprintf(strMessage, "PWR_ON and starting up\r\n");
                  ////ecSetLED(LED_NUCLEO, 150);
                  ecSetLED(LED_Pi, 150);

              }
            break;
        case CMD1_ACK1:
            sprintf(strMessage, "RUNNING\r\n");
            piState = PI_RUNNING;
            ////ecSetLED(LED_NUCLEO, LED_ON);
            ecSetLED(LED_Pi, LED_ON);       //signal Pi ready

            if(keyPress == KEY_SUPER_LONG_PRESS)
                preparePiPower_off = 1; //prepare power OFF after standby

            break;
    }

    if(piState != old_piState)
    {
        ecTxString(strMessage, strlen(strMessage));

    }
    return piState;
}


/**
  * @brief          Execute Button commands and forward on parallel port
  * @param[in]      keyPress comes from  ecButtonHandler()
  * @param[out]
  * @retval     None

Long and Superlong button is directly passed via GPIO and serial port.
Short and double click will be compressed and sent as pulses to port.
*/
int ecPortExecCommand(eKeyPress_t keyPress)
{

    static eKeyPress_t lastCmd;
    static uint8_t doubleCounter;
    switch(keyPress)
      {
          case KEY_PENDING:
//              if(ecSWTimerRead(MESSAGE_TIMER) == 0)
//              {
//                  HAL_UART_Transmit(&g_huart2, (uint8_t *)"KEY pending\r\n", 13, 20);
//                  ecSWTimerStart(MESSAGE_TIMER, 500);
//              }
              ecSWTimerStart(KEY_TIMER, PORT_SHORT_PULSE); //set minimum pulse time
              HAL_GPIO_WritePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin, GPIO_PIN_RESET);
              break;

          case KEY_SHORT_PRESS:  //UPS internal command: switch power on
              sprintf(strMessage, "> Pi power ON\r\n");
              ecTxString(strMessage, strlen(strMessage));
              HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_SET); //5V enable = high = ON
              lastCmd = KEY_SHORT_PRESS; //used later for CMD_OUT_Pin pulse
              break;

          case KEY_DOUBLE_PRESS: //shutdown and restart
              sprintf(strMessage, "u!shutdown -r now\n");
              ecTxString(strMessage, strlen(strMessage));
              HAL_GPIO_WritePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin, GPIO_PIN_RESET);
              doubleCounter = 0;
              lastCmd = KEY_DOUBLE_PRESS;
              break;

          case KEY_LONG_PRESS:  //shutdown and goto standby
              sprintf(strMessage, "u!shutdown now\r\n");
              ecTxString(strMessage, strlen(strMessage));
              HAL_GPIO_WritePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin, GPIO_PIN_SET);
              break;

          case KEY_SUPER_LONG_PRESS: //shutdown and switch power down
              sprintf(strMessage, "u!shutdown -P now\r\n");
              ecTxString(strMessage, strlen(strMessage));
              HAL_GPIO_WritePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin, GPIO_PIN_SET);
              break;
          default:
              break;
      }
      if(lastCmd != keyPress)
      {
      }


      if(lastCmd == KEY_SHORT_PRESS)
      {
          if(ecSWTimerRead(KEY_TIMER) == 0) //
          {
              HAL_GPIO_WritePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin, GPIO_PIN_SET);
              lastCmd = 0;
          }
      }
      if(lastCmd == KEY_DOUBLE_PRESS)
      {
          if (doubleCounter < 3)
          {
              if(ecSWTimerRead(KEY_TIMER) == 0)
              {
                  HAL_GPIO_TogglePin(CMD_OUT_GPIO_Port,  CMD_OUT_Pin);
                  ecSWTimerStart(KEY_TIMER, PORT_SHORT_PULSE); //set minimum hold time
                  doubleCounter ++;
              }
          }
          else
          {
              lastCmd = 0;
              doubleCounter = 0;
          }
      }

      if(ecSWTimerRead(KEY_TIMER) == 0)
          return 1;
      else
          return 0;
}


/**
  * @brief          process raspi response on query
  * @param[in]      *RxStr  response string from raspi
  * @param[in]      strlen
  * @param[out]     true, if ack response
  * @retval         None

*/
uint8_t ecDecodePiAnswer(uint8_t *RxStr, const char *ackStr)
{
    uint8_t ackChars;
    uint8_t ackMatch = 1;
    for (ackChars = 0; ackChars < strlen(ackStr); ackChars ++)
    {
            if(RxStr[ackChars] != ackStr[ackChars])
                ackMatch = 0;
    }
    return ackMatch;
}

/**
  * @brief          process raspi response on query
  * @param[in]      *RxStr  response string from raspi
  * @param[in]      strlen
  * @param[out]     true, if ack response
  * @retval         None

*/
uint8_t ecDecodePiRequest(uint8_t *RxStr, const char *requestStr)
{
    uint8_t reqChars;
    uint8_t reqMatch = 1;
    for (reqChars = 0; reqChars < strlen(requestStr); reqChars ++)
    {
            if(RxStr[reqChars] != requestStr[reqChars])
                reqMatch = 0;
    }
    return reqMatch;
}


/**
  * @brief          Execute Button commands and send command by UART
  * @param[in]      keyPress comes from  ecButtonHandler()
  * @param[out]
  * @retval     None

Long and Superlong button is directly passed via GPIO and serial port.
Short and double click will be compressed and sent as pulses to port.
*/
int ecSerialExecCommand(eKeyPress_t keyPress)
{

     switch(keyPress)
      {
          case KEY_PENDING:
              break;

          case KEY_SHORT_PRESS:  //UPS internal command: switch power on
              sprintf(strMessage, "> Pi power ON\r\n");
              ecTxString(strMessage, strlen(strMessage));
              break;

          case KEY_DOUBLE_PRESS: //shutdown and restart
              sprintf(strMessage, "u!shutdown -r now\r\n");
              ecTxString(strMessage, strlen(strMessage));
              break;

          case KEY_LONG_PRESS:  //shutdown and goto standby
              sprintf(strMessage, "u!shutdown now\r\n");
              ecTxString(strMessage, strlen(strMessage));
              break;

          case KEY_SUPER_LONG_PRESS: //shutdown and switch power down
              //power down is handled by UPS
              sprintf(strMessage, "u!shutdown -P now\r\n");
              ecTxString(strMessage, strlen(strMessage));
              break;
          default:
              break;
      }

      if(ecSWTimerRead(KEY_TIMER) == 0)
          return 1;
      else
          return 0;
}

/**
  * @brief      Evaluate Pi state, feedback state and transitions by LED
  * @param[in]  keyPress
  * @param[in]  actual Pi state
  * @param[out] new Pi state
  * @retval     None
*/
ePiState_t ecSerialHandleFeedback(eKeyPress_t keyPress, ePiState_t piState)
{
    static uint8_t preparePiPower_off;
    uint8_t rxPort;
    uint8_t rxStrBuf[RX_BUFFER_SIZE];


    /* evaluate state on Rx line (USART2) */
    rxPort = (GPIOA->IDR >> 3) & 0x01; //PA3

    /* generate feedback */
    switch(piState)
    {
        case PI_STATE_UNKNOWN:
            if(rxPort == 1)
            {
                ecSWTimerStart(WD_TIMER, 5000); //trigger watchdog timer
                sprintf(strMessage, "u?ready\r\n");
                ecTxString(strMessage, strlen(strMessage));
                g_PI_watchdog_Ok = 0;
                piState = PI_RUNNING;
            }
            else
                piState = PI_STBY;
            break;


        case PI_RUNNING:

            /* Watchdog for PI */
            if(ecSWTimerRead(WD_TIMER) == 0)
            {
                if(g_PI_watchdog_Ok) //check PI all 2000 sec
                {
                    sprintf(strMessage, "u?ready\r\n");
                    ecTxString(strMessage, strlen(strMessage));
                    ecSWTimerStart(WD_TIMER, 2000);
                    g_PI_watchdog_Ok = 0; //will be set after serial response of PI
                }
                else
                {
                    piState = PI_SHUTTING_DOWN; //this happens, if shutdown is locally initiated by Pi
                }
            }

            /* user feedback during key press */
            if(keyPress == KEY_PENDING)
                ecSetLED(LED_Pi, 1000);         //signal key press
            else
                ecSetLED(LED_Pi, LED_ON);       //signal Pi ready

            /* prepare power off after Pi is down */
            if (keyPress == KEY_SUPER_LONG_PRESS)
                preparePiPower_off = 1;

            /* process Pi responses */
            if(ecGetRxString(rxStrBuf, RX_BUFFER_SIZE))
            {
                if(ecDecodePiRequest(rxStrBuf, PI_REQ_STATUS))
                {
                    sprintf(strMessage, "0x%04X  %s", g_powerState, C_HDR_STR);
                    ecTxString(strMessage, strlen(strMessage));
                }
                if(ecDecodePiRequest(rxStrBuf, PI_REQ_ANALOG))
                {
                    ecTxString(g_analogStr, strlen(g_analogStr));
                }
                if(ecDecodePiRequest(rxStrBuf, PI_REQ_PWR_OFF))
                {
                    sprintf(strMessage,"u> %s\n", PI_REQ_PWR_OFF);
                    ecTxString(strMessage, strlen(strMessage));
                    preparePiPower_off = 1;
                    piState = PI_SHUTTING_DOWN;
                }


                if(ecDecodePiAnswer(rxStrBuf, PI_ACK_SHUTDOWN))
                    piState = PI_SHUTTING_DOWN;

                if(ecDecodePiAnswer(rxStrBuf, PI_ACK_READY))
                    g_PI_watchdog_Ok = 1;

            }
            break;

        case PI_SHUTTING_DOWN:
            ecSetLED(LED_Pi, 250);
             if (rxPort == 0)
            {
                piState = PI_STBY;
            }
            break;

        case  PI_STBY:
            ecSetLED(LED_Pi, LED_OFF);
            if(rxPort == 1)
            {
                piState = PI_STARTING_UP;
            }
            else
            {
                if( preparePiPower_off)
                {
                    ecSWTimerStart(DELAY_TIMER,2000); //allow pi a last breath
                    piState = PI_PWR_OFF;
                }
                /* check for wakup */
                if(keyPress == KEY_SHORT_PRESS)
                {
                    HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_RESET); //5V enable = low: OFF
                    ecSWTimerStart(DELAY_TIMER,200);
                    while(ecSWTimerRead(DELAY_TIMER)); //allow power down
                    HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_SET); //5V enable = high: ON
                    //after power ON pi starts up
                }
            }
            break;

        case PI_STARTING_UP:
            ecSetLED(LED_Pi, 150);

            if (rxPort == 1)
            {
                 if (ecSWTimerRead(DELAY_TIMER) == 0)
                 {
                     if(ecGetRxString(rxStrBuf, RX_BUFFER_SIZE))
                     {
                         g_PI_watchdog_Ok = 1;
                         piState = PI_RUNNING;
                     }
                 }
             }
            break;


        case PI_PWR_OFF:
            if (ecSWTimerRead(DELAY_TIMER) == 0)
            {
                HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_RESET); //5V enable = low:OFF
                preparePiPower_off = 0;
                piState = PI_STBY;
            }
            break;

    }

    g_debugPiState = piState;
     return piState;
}


/**
  * @brief      Analog signal processing and error handler
  * @param[in]  adcValues       ADC values in 0.1V or 0.1°C
  * @param[in]  analog_limits
  * @param[out] global g_powerState
  * @retval     error state
*/
uint16_t ecAnalogHandler(adc_buf_t adcVoltages)
{
    uint16_t retValue = 0;
    uint8_t nBatt;

    /* active supply */
    nBatt = HAL_GPIO_ReadPin( nBatt_GPIO_Port , nBatt_Pin); //0 = battery input active, else main input
    if(adcVoltages.Pi_voltage > PI_5V_DIODE_THRESHOLD)
        retValue |= PI_USB_POWERED;
    else
    {
        if(nBatt == 1)  retValue |= PI_MAIN_POWERED;
        else            retValue |= PI_BATT_POWERED;
    }

    /* Supply conditions */
    if(adcVoltages.main_voltage < MAIN_LOW_VOLTAGE_LIMIT) // || (adcVoltages.main_voltage < adcVoltages.batt_voltage))
        retValue |= ERR_LOW_MAIN;

    if(adcVoltages.batt_voltage < BAT_LOW_VOLTAGE_LIMIT)
        retValue |= ERR_LOW_BATT;

    if(adcVoltages.CPU_temp > CPU_HIGH_TEMP_LIMIT)
        retValue |= ERR_HIGH_CPU_TEMP;

    if(adcVoltages.main_voltage == 0)
    {
        ecSetLED(LED_Main, LED_OFF);
    }
    else if(retValue & ERR_LOW_MAIN)
    {
        ecSetLED(LED_Main, 1000);
        retValue |=MAIN_PRESENT;
    }
    else
    {
        ecSetLED(LED_Main, LED_ON);
        retValue |=MAIN_PRESENT;
    }

    if(adcVoltages.batt_voltage == 0)
    {
        ecSetLED(LED_Batt, LED_OFF);
    }
    else if(retValue & ERR_LOW_BATT)
    {
        ecSetLED(LED_Batt, 1000);
        retValue |= BATT_PRESENT;
    }
    else
    {
       ecSetLED(LED_Batt, LED_ON);
       retValue |= BATT_PRESENT;
    }


    return retValue;
}


