/*! @addtogroup Interface
@{ */
////////////////////////////////////////////////////////////////////////////////
//  Project      UPS-2 Raspberry pi power supply
/*! @file        lowlevel.h
//  @brief       interrupt callbacks and access API
//  @author      Klaus Mezger
//  @version     1
//
*///////////////////////////////////////////////////////////////////////////////
#ifndef __LOWLEVEL_H
#define __LOWLEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "UPS-2.h"

//==external defined

//== public defines
#define RX_BUFFER_SIZE     48


//== public functions
void ecUSART_Rx_Callback(void);
void ecUSART_Rx_Error_Callback(void);
void ecRx_Init(void);
int16_t  ecGetRxString(uint8_t *dataBuf, uint8_t maxSize);
void     ecTxString(char *string, uint8_t Size);

//== public typedefs


#ifdef __cplusplus
}
#endif
#endif //__LOWLEVEL_H

/*! @} */ //end of doxygen module group
