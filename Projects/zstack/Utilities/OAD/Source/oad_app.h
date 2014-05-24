/**************************************************************************************************
  Filename:       oad_app.h
  Revised:        $Date: 2007-01-15 10:33:29 -0700 (Mon, 15 Jan 2007) $
  Revision:       $Revision: 13298 $

  Description:    This file contains the declaration of an Over Air Download application.

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

#ifndef OAD_APP_H
#define OAD_APP_H

#ifdef __cplusplus
extern "C"
{
#endif
 
/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_oad.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Macros  
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define OAD_PROFILE_ID       0xC003

#define OAD_CLUSTERID_CS     0x0001  // Used by the original Client-Server managed-session method.
#define OAD_CLUSTER_CNT      2
#define OAD_DEVICEID         0x0001
#define OAD_DEVICE_VERSION   0
#define OAD_FLAGS            0
 
#define OAD_ENDPOINT         200

#define OAD_EVT_TIMER        0x0001

// Message ID's.
#define  ZLMSGID_STATUSQ        ((uint8) 0x01)
#define  ZLMSGID_SESSION_START  ((uint8) 0x02)
#define  ZLMSGID_SESSION_TERM   ((uint8) 0x03)
#define  ZLMSGID_CLIENT_CMD     ((uint8) 0x04)
#define  ZLMSGID_CODE_ENABLE    ((uint8) 0x05)
#define  ZLMSGID_SEND_DATA      ((uint8) 0x06)
#define  ZLMSGID_RESET          ((uint8) 0x07)
// Bit OR'ed into message ID to signify a reply to that message.
#define  ZLMSGID_REPLY_BIT      ((uint8) 0x80)

// for now, the block size and number of blocks per data transfer tansaction is fixed.
#define  ZL_DATA_BLK_SIZE       8
#define  ZL_NUM_DATA_BLKS       4

// capabilties bits. none for now.
#define ZLOAD_CAPABILTIES       (0)
#define ZLOAD_PROTOCOL_VERSION  (1)

// Event message bits.
#define  ZLOAD_CODE_ENABLE_EVT  0x0001
#define  ZLOAD_IS_CLIENT_EVT    0x0002
#define  ZLOAD_XFER_DONE_EVT    0x0004
#define  ZLOAD_RESET_EVT        0x0008
#define  ZLOAD_SDRTIMER_EVT     0x0010
#define  ZLOAD_RESET_BOARD_EVT  0x0020

// STATE definitions for state machine
#define ZLSTATE_IDLE               (1)
#define ZLSTATE_CLIENT             (2)
#define ZLSTATE_SERVER             (3)
#define ZLSTATE_PASS_THROUGH       (4)
#define ZLSTATE_SUBSTATE_NONE      (1)
#define ZLSTATE_SUBSTATE_XFER_DONE (2)

// Error Codes for replies to commands.
// Status Query and Reset always return No Error

#define EC_NO_ERROR        ((uint8) 0x00)

// Begin Session
#define EC_BS_NOT_IDLE     ((uint8) 0x21)
#define EC_BS_NO_MATCHES   ((uint8) 0x22)
#define EC_BS_NO_MEM       ((uint8) 0x23)
#define EC_BS_NOT_SERVER   ((uint8) 0x24)
// End Session
#define EC_ES_BAD_SESS_ID  ((uint8) 0x31)
#define EC_ES_NOT_SERVER   ((uint8) 0x32)
#define EC_ES_NO_MEM       ((uint8) 0x33)
// Client
#define EC_CL_NOT_IDLE     ((uint8) 0x41)
#define EC_CL_NO_MEM       ((uint8) 0x42)
#define EC_CL_NOT_CLIENT   ((uint8) 0x43)
// Code enable
#define EC_CE_NO_IMAGE     ((uint8) 0x51)
#define EC_CE_NO_MATCH     ((uint8) 0x52)
#define EC_CE_NOT_IDLE     ((uint8) 0x53)
#define EC_CE_IMAGE_INSANE ((uint8) 0x54)
#define EC_CE_NOT_CLIENT   ((uint8) 0x55)
// Send data
#define EC_SD_NOT_SERVER   ((uint8) 0x61)
#define EC_SD_BAD_SESS_ID  ((uint8) 0x62)
#define EC_SD_BAD_PKT_NUM  ((uint8) 0x63)
#define EC_SD_NO_BEG_SESS  ((uint8) 0x64)
#define EC_SD_NOT_CLIENT   ((uint8) 0x65)
#define EC_SD_NO_MEM       ((uint8) 0x66)

// other support
#define  PREAMBLE_DL           ((uint8) 1)
#define  PREAMBLE_OP           ((uint8) 2)
#define  ZL_IMAGE_ID_LENGTH    (6)
#define  SERIAL_SERVER_ADDRESS (0xFFFE)

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

// Message header.
PACK_1
typedef struct  {
  uint8  zlhdr_msgid;
  uint8  zlhdr_seqnum;
  uint8  zlhdr_msglen;
} zlmhdr_t;

//   STATUS QUERY
//       Command:  no payload with status query

//       Reply
PACK_1
typedef struct  {
  uint8  zlsqR_state;
  uint8  zlsqR_errorCode;
  uint8  zlsqR_ProtocolVersion;
  uint8  zlsqR_capabilties;
  uint16 zlsqR_opVer;
  uint16 zlsqR_opManu;
  uint16 zlsqR_opProd;
  uint16 zlsqR_dlVer;
  uint16 zlsqR_dlManu;
  uint16 zlsqR_dlProd;
  uint16 zlsqR_curPkt;
  uint16 zlsqR_totPkt;
} zlstatusR_t;

//   SESSION START
//       Command
PACK_1
typedef struct  {
  uint16 zlbsC_ver;
  uint16 zlbsC_manu;
  uint16 zlbsC_prod;
  uint8  zlbsC_sessionID;
} zlbegsessC_t;

//       Reply
PACK_1
typedef struct  {
  uint8  zlbsR_state;
  uint8  zlbsR_errorCode;
  uint32 zlbsR_imgLen;
  uint8  zlbsR_blkSize;
  uint8  zlbsR_numBlks;
  uint8  zlbsR_preambleOffset;
} zlbegsessR_t;

//   SESSION TERMINATE
//       Command
PACK_1
typedef struct  {
  uint8  zlesC_sessionID;
} zlendsessC_t;

//       Reply
PACK_1
typedef struct  {
  uint8  zlesR_state;
  uint8  zlesR_errorCode;
} zlendsessR_t;

//   CLIENT COMMAND
//       Command
PACK_1
typedef struct  {
  uint16 zlclC_ver;
  uint16 zlclC_manu;
  uint16 zlclC_prod;
  uint8  zlclC_IEEE[Z_EXTADDR_LEN];
  uint16 zlclC_nwk;
  uint8  zlclC_endp;
} zlclientC_t;

//       Reply
PACK_1
typedef struct  {
  uint8  zlclR_state;
  uint8  zlclR_errorCode;
} zlclientR_t;

//   CODE ENABLE COMMAND
//       Command
PACK_1
typedef struct  {
  uint16 zlceC_ver;
  uint16 zlceC_manu;
  uint16 zlceC_prod;
} zlceC_t;

//       Reply
PACK_1
typedef struct  {
  uint8  zlceR_state;
  uint8  zlceR_errorCode;
} zlceR_t;

//   SEND DATA COMMAND
//       Command
PACK_1
typedef struct  {
  uint16 zlsdC_pktNum;
  uint8  zlsdC_sessionID;
} zlsdC_t;

//       Reply
PACK_1
typedef struct  {
  uint8  zlsdR_state;
  uint8  zlsdR_errorCode;
  uint16 zlsdR_pktNum;
  uint8  zlsdR_data[ZL_DATA_BLK_SIZE * ZL_NUM_DATA_BLKS];
} zlsdR_t;

//   RESET COMMAND
//       Command:  no payload with reset command

//       Reply
PACK_1
typedef struct  {
  uint8  zlrstR_state;
  uint8  zlrstR_errorCode;
} zlrstR_t;


// the replies (except for send data) are all small. use a union so we can have a single
// malloc() and save code space.
typedef union  {
  zlstatusR_t  statusq;
  zlbegsessR_t begSess;
  zlendsessR_t endSess;
  zlclientR_t  client;
  zlceR_t      enable;
  zlrstR_t     reset;
} zlreply_t;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern uint8 oad_app_taskId;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          oadAppInit
 *
 * @brief       This function is called by OSAL system startup.
 *
 * input parameters
 *
 * @param       id - The Task ID assigned by OSAL.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void oadAppInit(uint8 id);

/**************************************************************************************************
 * @fn          oadAppEvt
 *
 * @brief       This function is called to process the OSAL events for the task.
 *
 * input parameters
 *
 * @param       id - The Task ID assigned by OSAL.
 * @param       evts - A bit mask representing the OSAL events that are pending for this task.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint16 oadAppEvt(uint8 id, uint16 evts);

/**************************************************************************************
 * @fn      oadAppRegisterCB
 *
 * @brief   Register a callback to be referenced when OAD events occur.
 *
 * @param   input
 *            pCBFunction  pointer to void function with an unsigned short argument.
 *            eventMask    bit mask of events for which the callback should be invoked.
 *
 *    When the callback is invoked its argument will indicate via the bit mask argument
 *    which event occurred. A null pointer or a null bit mask will have the effect of
 *    deregistration.
 *
 * @return  none.
 */
void oadAppRegisterCB(void (*pCBFunction)(uint16), uint16 eventMask);

/**************************************************************************************************
*/

#ifdef __cplusplus
}
#endif

#endif
