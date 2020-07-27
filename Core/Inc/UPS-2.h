/**
  ******************************************************************************
  * @file           : UPS-2.h
  * @brief          : Header for project specific defines and global variables
  * @author         : Klaus Mezger
  *                   This file contains common defines of the application.
  ******************************************************************************
*/
#include "main.h"

#ifndef INC_UPS_2_H_
#define INC_UPS_2_H_

//#define SERIAL_DEBUG

#define C_HDR_STR "ECOM UPS-2 build " __DATE__"\r\n"

#define PI_ACK_SHUTDOWN     ">OK sudo shutdown"
#define PI_ACK_READY        ">OK ready"

#define PI_REQ_STATUS       "r?status"
#define PI_REQ_ANALOG       "r?analog"
#define PI_REQ_PWR_OFF      "r?shutdown -P"



//

#define LED_COUNT  4
#define LED_ON      0xFFFF
#define LED_OFF     0x0000

#define SYS_MODE_SERIAL     0   //Mode jumper set
#define SYS_MODE_PARALLEL   1   //Mode jumper open



/* Voltage levels * 10 */
#define MAIN_LOW_VOLTAGE_LIMIT  70  //7.0V
#define BAT_LOW_VOLTAGE_LIMIT   68 // --> 6.5V under load
#define CPU_HIGH_TEMP_LIMIT     800  //80°C
#define PI_5V_DIODE_THRESHOLD   19  //threshold UISB powered: > 1.9V


/* power supply states */
#define ERR_MASK            0xFF00
#define ERR_LOW_MAIN        0x0100
#define ERR_HIGH_MAIN       0x0200
#define ERR_LOW_BATT        0x0400
#define ERR_HIGH_BATT       0x0800
#define ERR_HIGH_CPU_TEMP   0x1000
#define NO_ERROR            0x0000

#define SUPPLY_MASK         0x00F0
#define MAIN_PRESENT        0x0010
#define BATT_PRESENT        0x0020
#define USB_PRESENT         0x0040

#define PI_POWER_MASK       0x000F
#define PI_POWER_OFF        0x0000
#define PI_MAIN_POWERED     0x0001
#define PI_BATT_POWERED     0x0002
#define PI_USB_POWERED      0x0004




typedef enum
{
    LED_Pi,
    LED_Batt,
    LED_Main,
    LED_NUCLEO //for development on NUCLEO G070 purposes only
}eLED_t;


typedef enum
{
    PI_STATE_UNKNOWN,
    PI_RUNNING,
    PI_SHUTTING_DOWN,
    PI_STARTING_UP,
    PI_STBY,
    PI_PWR_OFF
}ePiState_t;

typedef enum
{
    CMD0_ACK0 = 00,
    CMD0_ACK1 = 01,
    CMD1_ACK0 = 02,
    CMD1_ACK1 = 03
}ePiPortState_t; //bitcoded port states: bit0: ACK, bit1: command


typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_t;

typedef enum
{
    KEY_NO_PRESS,
    KEY_PENDING,
    KEY_SHORT_PRESS,
    KEY_LONG_PRESS,
    KEY_SUPER_LONG_PRESS,
    KEY_DOUBLE_PRESS
}eKeyPress_t;


typedef struct
{
    uint16_t main_voltage;
    uint16_t batt_voltage;
    uint16_t Pi_voltage;
    uint16_t CPU_temp;
}adc_buf_t;


//== global variables
int g_debugInt;
ePiState_t g_debugPiState;
uint8_t g_sys_Mode;
uint8_t g_piAck;

ADC_HandleTypeDef g_hadc1;
adc_buf_t g_adc_buf;
adc_buf_t g_adc_voltages;
uint16_t g_powerState;
char g_analogStr[48];


/* simplify access code to IO. For port defs see main.h */
/*
#define LED_Pi_Pin GPIO_PIN_7
#define LED_Pi_GPIO_Port GPIOB
#define LED_Batt_Pin GPIO_PIN_9
#define LED_Batt_GPIO_Port GPIOB
#define LED_Main_Pin GPIO_PIN_10
#define LED_Main_GPIO_Port GPIOC
*/

#endif /* INC_UPS_2_H_ */
