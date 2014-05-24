/**************************************************************************************************
  Filename:       znp_app.h
  Revised:        $Date: 2010-01-17 08:58:03 -0800 (Sun, 17 Jan 2010) $
  Revision:       $Revision: 21533 $

  Description:    This file is the Application declaration for the ZNP.


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/
#ifndef NP_APP_H
#define NP_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

//efine SYS_EVENT_MSG             0x8000
#define ZNP_SPI_RX_AREQ_EVENT     0x4000
#define ZNP_SPI_RX_SREQ_EVENT     0x2000
#define ZNP_UART_TX_READY_EVENT   0x1000
/* See MT_SYS_OSAL_EVENT_MASK in MT_SYS.h
#define ZNP_OSAL_TIMER_0_EVENT    0x0800
#define ZNP_OSAL_TIMER_1_EVENT    0x0400
#define ZNP_OSAL_TIMER_2_EVENT    0x0200
#define ZNP_OSAL_TIMER_3_EVENT    0x0100
 */
#define ZNP_PWRMGR_CONSERVE_EVENT 0x0080
#define ZNP_SPI_TIMER_EVENT       0x0040

#define ZNP_RDY(RDY)      (P2_1 = !RDY)

#if !defined ZNP_PWRMGR_CONSERVE_DELAY
#define ZNP_PWRMGR_CONSERVE_DELAY          10
#endif

#if !defined ZNP_SPI_TIMER_DELAY
#define ZNP_SPI_TIMER_DELAY                1000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern uint8 znpTaskId;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

void znpInit(uint8 taskId);
uint16 znpEventLoop(uint8 taskId, uint16 events);
void znpTestRF(void);

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif
