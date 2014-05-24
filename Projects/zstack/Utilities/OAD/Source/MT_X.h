/**************************************************************************************************
  Filename:       MT_X.h
  Revised:        $Date: 2009-08-21 13:10:13 -0700 (Fri, 21 Aug 2009) $
  Revision:       $Revision: 20630 $

  Description:    This file contains the interface to the Flash Service.


  Copyright 2008-2009 Texas Instruments Incorporated. All rights reserved.

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

#ifndef MT_X_H
#define MT_X_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"
#include "OSAL_Nv.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Macros  
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Globals  
 * ------------------------------------------------------------------------------------------------
 */

extern uint8 mtxMode;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          MT_X_UartProcessZToolData
 *
 * @brief       This function is called by Hal_UART_SendCallBack to read incoming Rx data.
 *
 *              Attempt to sync on and parse the old MT 1.0 format:
 *              | SOP | Data Length  |   CMD   |   Data   |  FCS  |
 *              |  1  |     1        |    2    |  0-Len   |   1   |
 *
 *              And then pass on all data to the MT 2.0 by invoking 
 *              void MT_UartProcessZToolData ( uint8 port, uint8 event )
 *
 * input parameters
 *
 * @param       port - UART port.
 * @param       event - Event that causes the callback.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MT_X_UartProcessZToolData(uint8 port, uint8 event);

/**************************************************************************************************
 * @fn          MT_X_TransportSend
 *
 * @brief       Prepare message to send by old MT format if it is pertinent to ZOAD.
 *              Copy message header to re-use to also send in new MT format and then
 *              fill in SOP and FCS then send out the msg.
 *
 * input parameters
 *
 * @param       msg - pointer to the message that contains spare byte, CMD, length, data and FCS.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MT_X_TransportSend(uint8 *msg);

/**************************************************************************************************
 * @fn          MT_X_FakeNwkJoinCnf
 *
 * @brief       Fake a SPI_CB_NLME_JOIN_CNF to the old ZOAD.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MT_X_FakeNwkJoinCnf(void);

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif
