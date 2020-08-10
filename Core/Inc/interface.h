/*! @addtogroup Interface
@{ */
////////////////////////////////////////////////////////////////////////////////
//  Project      UPS-2 Raspberry pi power supply
/*! @file        interface.h
//  @brief       Software Timer Functions & Frequency Generation
//  @author      Klaus Mezger
//  @version     1
//
*///////////////////////////////////////////////////////////////////////////////
#ifndef __INTERFACE_H
#define __INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "UPS-2.h"

//==external defined

//== public defines


//== public functions
int ecPortExecCommand(eKeyPress_t keyPress);
ePiState_t ecPortHandleFeedback(eKeyPress_t keyPress, ePiState_t piState);
uint16_t ecGetCPUTemp(uint16_t ADCvalue);
uint16_t ecAnalogHandler(adc_buf_t adcValues);
int ecSerialExecCommand(eKeyPress_t keyPress);
ePiState_t ecSerialHandleFeedback(eKeyPress_t keyPress, ePiState_t piState);
uint8_t ecDecodePiAnswer(uint8_t *RxStr, const char *ackStr);
uint8_t ecDecodePiRequest(uint8_t *RxStr, const char *requestStr);
//uint8_t ecDecodePiAnswer(uint8_t *RxStr, uint8_t strLen);

//== public typedefs


#ifdef __cplusplus
}
#endif
#endif //__INTERFACE_H

/*! @} */ //end of doxygen module group

