/**************************************************************************************************
    Filename:       TransmitApp.h
    Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
    Revision:       $Revision: 15795 $

    Description:    This file contains the Transmit Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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

#ifndef TRANSMITAPP_H
#define TRANSMITAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define TRANSMITAPP_ENDPOINT           1

#define TRANSMITAPP_PROFID             0x0F05
#define TRANSMITAPP_DEVICEID           0x0001
#define TRANSMITAPP_DEVICE_VERSION     0
#define TRANSMITAPP_FLAGS              0

#define TRANSMITAPP_MAX_CLUSTERS       1
#define TRANSMITAPP_CLUSTERID_TESTMSG  1

// Application Events (OSAL) - These are bit weighted definitions.
#define TRANSMITAPP_SEND_MSG_EVT       0x0001
#define TRANSMITAPP_RCVTIMER_EVT       0x0002
#define TRANSMITAPP_SEND_ERR_EVT       0x0004

/*********************************************************************
 * MACROS
 */

// Define to enable fragmentation transmit test
//#define TRANSMITAPP_FRAGMENTED

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Transmit Application
 */
extern void TransmitApp_Init( byte task_id );

/*
 * Task Event Processor for the Transmit Application
 */
extern UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events );

extern void TransmitApp_ChangeState( void );
extern void TransmitApp_SetSendEvt( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITAPP_H */
