/*! @addtogroup Interface
@{ */
////////////////////////////////////////////////////////////////////////////////
//  Project      UPS-2 Raspberry pi power supply
/*! @file        lowlevel.c
//  @brief       interrupt callbacks and access API
//  @author      Klaus Mezger
//  @version     1
//
*///////////////////////////////////////////////////////////////////////////////

//== standard libraries ==
#include <stdio.h>
#include <string.h>


//== module headers ==
#include "UPS-2.h"
#include "lowlevel.h"
//== module headers ==
#include <main.h>
#include <stdio.h>

//== local defines

//== local function prototypes

//== local typedefs

//== module global variables
uint8_t rxBuffer1[RX_BUFFER_SIZE];
uint8_t rxBuffer2[RX_BUFFER_SIZE];
uint8_t *rxActiveBuffer;
uint8_t *rxUserBuffer;

uint8_t     g_rx_chars;
int16_t     g_rxRdy;


//== public functions
/**
  * @brief  initiates RX transfer
  * @param[out]  global
  * @retval None
  */
void ecRx_Init(void)
{
  g_rxRdy = 0;
  g_rx_chars = 0;
  rxActiveBuffer = rxBuffer1;
  rxUserBuffer = rxBuffer1;

//  USART2->CR2 = USART_CR2_LINEN | USART_CR2_LBDIE; //enable LIN (break detection)
  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART2);
  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);

}



/**
  * @brief      Send string in blocking mode
  * @param[in]  *string
  * @param[in]  string kength
  * @retval     None
  */
void ecTxString(char *string, uint8_t Size)
{

    uint8_t enableTx;
#ifdef SERIAL_DEBUG
    enableTx = 1;
#else
    enableTx = !g_sys_Mode; //serial mode: g_sys_Mode = 0
#endif
    if(enableTx)
    {
      uint32_t index = 0;
      char *pchar = string;

      /* Send characters one per one, until last char to be sent */
      for (index = 0; index < Size; index++)
      {
        /* Wait for TXE flag to be raised */
        while (!LL_USART_IsActiveFlag_TXE(USART2))
        {
        }
        /* Write character in Transmit Data register.
           TXE flag is cleared by writing data in TDR register */
        LL_USART_TransmitData8(USART2, *pchar++);
      }

      /* Wait for TC flag to be raised for last char */
      while (!LL_USART_IsActiveFlag_TC(USART2))
      {
      }
    }
}

/*todo explanation ecGetRxString */
/**
  * @brief
  * @param[out] *databuf pointer to external buffer
  * @param[in]  maxSize databuf lenght
  * @retval     None
  */
int16_t ecGetRxString(uint8_t *dataBuf, uint8_t maxSize)
{
    int16_t retVal = 0;

    if(g_rxRdy > 0)
    {
       strncpy((char*)dataBuf, (char*)rxUserBuffer, maxSize);
       rxUserBuffer = 0;
       retVal = g_rxRdy;
       g_rxRdy = 0;
//       g_rx_chars = 0;
       NVIC_EnableIRQ(USART2_IRQn);
    }
    return retVal;
}

//== private functions

/**
  * @brief  Function called from USART IRQ Handler from file stm32g0xx_it.c
  * @param  None
  * @retval None
  */
void ecUSART_Rx_Callback(void)
{

    uint8_t rxData;
    /* Read Received character. RXNE flag is cleared by reading of RDR register */
    if(g_rx_chars < RX_BUFFER_SIZE)
    {
        rxData = LL_USART_ReceiveData8(USART2);
        if((rxData == '\r') || (rxData == '\n'))
        {
            if(g_rx_chars > 0) //else ignore EOL
            {
                rxActiveBuffer[g_rx_chars] = 0;
                g_rxRdy = g_rx_chars;;
                g_rx_chars = 0;
                rxUserBuffer = rxActiveBuffer;
                if(rxActiveBuffer == rxBuffer1)
                    rxActiveBuffer = rxBuffer2;
                else
                    rxActiveBuffer = rxBuffer1;
            }
        }
        else
        {
            rxActiveBuffer[g_rx_chars] = rxData;
            g_rx_chars ++;
        }
    }
    else
    {
        g_rx_chars = 0;
        g_rxRdy = -1;
    }

}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void ecUSART_Rx_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
   NVIC_DisableIRQ(USART2_IRQn);

  /* Error handling :
    in this UPS application, all Rx errors must be ignored because of
     undefined states of the Rx line
  */
  isr_reg = LL_USART_ReadReg(USART2, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : Clear NF Flag */
    LL_USART_ClearFlag_NE(USART2);
  }
  else
  {

      if(LL_USART_IsActiveFlag_ORE(USART2))
      {
          LL_USART_ClearFlag_ORE(USART2);

      }
      else if(LL_USART_IsActiveFlag_FE(USART2))
      {
          LL_USART_ClearFlag_FE(USART2);
      }
      else if(LL_USART_IsActiveFlag_NE(USART2))
      {
          LL_USART_ClearFlag_NE(USART2);
      }
      g_rx_chars = 0;
      g_debugInt ++;
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
//    LED_Blinking(LED_BLINK_ERROR);
  }
  NVIC_EnableIRQ(USART2_IRQn);
}

/*! @} */ //end of doxygen module group
