/**************************************************************************************************
  Filename:       oad_app.c
  Revised:        $Date: 2009-08-21 13:10:13 -0700 (Fri, 21 Aug 2009) $
  Revision:       $Revision: 20630 $

  Description:    This file contains the implementation of an Over Air Download application.


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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include <string.h>

#include "AF.h"
#include "hal_board_cfg.h"
#include "hal_flash.h"
#include "hal_oad.h"
#include "oad_app.h"
#include "oad_preamble.h"
#include "OnBoard.h"
#include "OSAL_Nv.h"

#if defined ZPORT
#include "hal_key.h"
#include "MT.h"
#include "MT_APP.h"
#include "MT_X.h"
#include "ZDApp.h"
#else
#if (HAL_OAD_XNV_IS_INT && ((HAL_OAD_DL_OSET % HAL_FLASH_PAGE_SIZE) != 0))
#include "hal_xnv.h"
#endif
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros  
 * ------------------------------------------------------------------------------------------------
 */

#define  DO_EVENT_CALLBACK(e) st ( if (s_pCallback && ((e) & s_eventMask))  s_pCallback((e)); )

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined OAD_NV_ID
// Arbitrarily pick the last Id available to the user Application.
#define OAD_NV_ID  0x0FFF
#endif
#define PREAMBLE_NV_ID                    OAD_NV_ID

// support to select callback event for which subscription is desired
// this information is transmitted as a bit map.
#define  ZLCB_EVENT_OADBEGIN_CLIENT     ((uint16)0x0001)
#define  ZLCB_EVENT_OADEND_CLIENT       ((uint16)0x0002)
#define  ZLCB_EVENT_OADBEGIN_SERVER     ((uint16)0x0004)
#define  ZLCB_EVENT_OADEND_SERVER       ((uint16)0x0008)
#define  ZLCB_EVENT_CODE_ENABLE_RESET   ((uint16)0x0010)
#define  ZLCB_EVENT_ALL                 (ZLCB_EVENT_OADBEGIN_CLIENT   | \
                                         ZLCB_EVENT_OADEND_CLIENT     | \
                                         ZLCB_EVENT_OADBEGIN_SERVER   | \
                                         ZLCB_EVENT_OADEND_SERVER     | \
                                         ZLCB_EVENT_CODE_ENABLE_RESET   \
                                        )

#define  SDC_RETRY_COUNT         (10)
#define  SDR_WAIT_TO             (1000)
// Some reasonable time to get the code enable response out.
#define  SDC_WAIT_TO_ENABLE      (10000)

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

// Used by the optional OAD en masse mode - TODO.
/*
typedef enum {
  emBeg = 0x0010,
  emDat,
  emRst,
  emEnd
} en_masse_t;

#define OAD_CLUSTERID_EM  0x0010
*/

#if defined ZPORT
#define SIZEOF_ZAIN_HDR   (sizeof(uint16) + sizeof(uint8) + sizeof(uint16) + sizeof(uint8))

// Z-Architect headers
// inbound to host (from dongle directly or external platform)
PACK_1
typedef struct  {
    uint16 zaproxy_nwkAddr;
    uint8  zaproxy_endp;
    uint16 zaproxy_ClusterID;
    uint8  zaproxy_msglen;
    uint8  zaproxy_payload[1];
} zahdrin_t;

// outbound from host (to dongle directly or external platform)
PACK_1
typedef struct  {
    uint16 zaproxy_nwkAddr;
    uint8  zaproxy_endp;
    uint16 zaproxy_ClusterID;
    uint8  zaproxy_msglen;
    uint8  zaproxy_payload[1];
} zahdrout_t;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 oad_app_taskId;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

#if defined ZPORT
#define HalOADChkDL(V)  1
#define HalOADInvRC(V)
#define HalOADAvail(V)  HAL_OAD_DL_SIZE
#define HalOADRead(A, B, C, D)
#define HalOADWrite(A, B, C, D)
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 transId;
static afAddrType_t dstAddr;

static zlclientC_t *s_clientInfo;
static uint8 s_State, s_SessionID, s_blkSize;
static uint16 s_NextPacket, s_NumPktGet, s_SDCRetryCount;
static zlmhdr_t    *s_sdcmd;
static zlsdC_t     *s_sdcpayload;
static zlmhdr_t    *s_sdreply;
#if !defined ZPORT
static zlsdR_t     *s_sdrpayload;
static image_t      s_itype;
static uint8       s_lastSeen, s_firstTx = 1;
#endif
static void        (*s_pCallback)(uint16);
static uint16      s_eventMask;
static uint8       s_lastTxSeqNum;

#if defined ZPORT
static uint8       s_serialMsg;
static uint16      s_lastSeqNum;
static uint16      s_myNwkAddr = 0xFFFE;

// pass through stuff
static uint8                  s_PTSeqNum;
static afIncomingMSGPacket_t  s_PTClientInfo;
#else

#pragma location="CRC_SHDW"
const CODE uint16 _crcShdw = 0xFFFF;
#pragma required=_crcShdw

#pragma location="PREAMBLE"
const CODE preamble_t _preamble = {
 {PREAMBLE_MAGIC1, PREAMBLE_MAGIC2},
  HAL_OAD_DL_SIZE,
  HAL_OAD_VERS,
  HAL_OAD_MANU,
  HAL_OAD_PROD
};
#pragma required=_preamble
#endif

// This list should be filled with Application specific Cluster IDs.
static const cId_t OAD_ClusterList[OAD_CLUSTER_CNT] =
{
  OAD_CLUSTERID_CS
  //,OAD_CLUSTERID_EM
};

static const SimpleDescriptionFormat_t OAD_SimpleDesc =
{
  OAD_ENDPOINT,
  OAD_PROFILE_ID,
  OAD_DEVICEID,
  OAD_DEVICE_VERSION,
  OAD_FLAGS,
  OAD_CLUSTER_CNT,
  (cId_t *)OAD_ClusterList,
  0,
  NULL
};

static const endPointDesc_t OAD_epDesc=
{
  OAD_ENDPOINT,
  &oad_app_taskId,
  (SimpleDescriptionFormat_t *)&OAD_SimpleDesc,
  noLatencyReqs,
};

#if defined HAL_OAD_BL21
// this is the mailbox value that tells the boot code to flash the downloaded image
// even though the operational image may be sane.
#define MBOX_OAD_ENABLE      0x454E424C           // 'ENBL' enable downloaded code
PACK_1
typedef struct mbox_s {
  volatile unsigned long BootRead;
  volatile unsigned long AppRead;
} mboxMsg_t;
#pragma location="MBOXMSG_ADDR"
__no_init mboxMsg_t mboxMsg;
PACK_1
typedef struct  {
     uint8      (*ReadFlash)(image_t, uint32 addr, uint8 *pBuf, uint16 len);
     uint8      (*WriteFlash)(image_t, uint32 addr, uint8 *pBuf, uint16 len);
     uint8      (*CheckCodeSanity)(image_t, uint32 addr1, uint32 addr2);
     uint8      (*GetPreamble)(image_t, uint32 addr, preamble_t *pBuf);
} mbox_t;
#pragma location="MBOX_ADDR"
__no_init mbox_t mbox;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void procSysEvtMsg(void);
static void zlResetState(void);

static void   ZLOADApp_MessageMSGCB( afIncomingMSGPacket_t * );
static void   ZLOADApp_handleCommand(afIncomingMSGPacket_t *, zlmhdr_t *);
static void   ZLOADApp_handleReply(afIncomingMSGPacket_t *, zlmhdr_t *);

static void   zlCleanupOnReset(void);
static void   zlStartClientSession(void);
static void   zlProcessSDR(zlsdR_t *);
static void   zlRequestNextDataPacket(void);
static void   zlCleanupOnXferDone(void);
static void   zlResendSDC(void);

#if defined ZPORT
static void   zlSendSerial(uint8 *, uint8);
static void   ZLOADApp_SerialMessageMSGCB(zahdrout_t *);
static void   zlZArchitectProxyMsg(zahdrout_t *);

static uint8      zlPassOnStartSessionOK(uint8 *);
static zahdrin_t *zlBuildExternalInboundSerialMSG(afIncomingMSGPacket_t *, uint8 *, uint8);
static zahdrin_t *zlBuildInternalInboundSerialMSG(uint8 *, uint8);
static void zlHandleKeys(uint8 shift, uint8 keys);
#else
static uint8  zlSendCommand(uint8, uint8 *);
static void   zlProcessSDC(zlsdC_t *);
static uint8  zlIsReqPacketNumOK(uint16);
#if (HAL_OAD_XNV_IS_INT && ((HAL_OAD_DL_OSET % HAL_FLASH_PAGE_SIZE) != 0))
static Status_t zlEraseHalfPage(void);
#endif
#endif

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
void oadAppInit(uint8 id)
{
  oad_app_taskId = id;
  afRegister((endPointDesc_t *)&OAD_epDesc);
  s_State = ZLSTATE_IDLE;
  id = PREAMBLE_OFFSET;
  osal_nv_item_init(PREAMBLE_NV_ID, 1, &id);
#if defined HAL_OAD_BL21
  mboxMsg.BootRead = 0;
#endif
#if defined ZPORT
  // Register for all key events - This app will handle all key events
  RegisterForKeys(oad_app_taskId);
  mtxMode = TRUE;
#endif
}

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
uint16 oadAppEvt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;
  
  if (evts & SYS_EVENT_MSG)
  {
    mask = SYS_EVENT_MSG;
    procSysEvtMsg();
  }
  else if (evts & ZLOAD_CODE_ENABLE_EVT)
  {
    // let the user shut down the environment if requested
    DO_EVENT_CALLBACK(ZLCB_EVENT_CODE_ENABLE_RESET);

#if defined HAL_OAD_BL21
    // Set up mail box to tell the pre-2.2 boot code to flash the downloaded image.
    mboxMsg.BootRead = MBOX_OAD_ENABLE;
#else
    HalOADInvRC();
#endif
    SystemReset();
  }
  else if (evts & ZLOAD_IS_CLIENT_EVT)
  {
    mask = ZLOAD_IS_CLIENT_EVT;
    zlStartClientSession();
  }
  else if (evts & ZLOAD_XFER_DONE_EVT)
  {
    mask = ZLOAD_XFER_DONE_EVT;
    zlCleanupOnXferDone();
  }
  else if (evts & ZLOAD_RESET_EVT)
  {
    mask = ZLOAD_RESET_EVT;
    zlCleanupOnReset();
  }
  else if (evts & ZLOAD_SDRTIMER_EVT)
  {
    mask = ZLOAD_SDRTIMER_EVT;
    zlResendSDC();
  }
  else if (evts & ZLOAD_RESET_BOARD_EVT)
  {
    SystemReset();
  }
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}

/**************************************************************************************************
 * @fn          procSysEvtMsg
 *
 * @brief       This function is called by oadAppEvt() to process all of the pending OSAL messages.
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
static void procSysEvtMsg(void)
{
  uint8 *msgPtr;

  while ((msgPtr = osal_msg_receive(oad_app_taskId)))
  {
#if defined ZPORT
    s_serialMsg = 0;
#endif

    switch ( *msgPtr )
    {
    case MT_SYS_APP_MSG:
    case MT_SYS_APP_RSP_MSG:
#if defined ZPORT
      // This is how we get messages from a Host application (ZOAD.exe).
      s_serialMsg = 1;
      ZLOADApp_SerialMessageMSGCB((zahdrout_t *)(((mtSysAppMsg_t *)msgPtr)->appData));
#endif
      break;

    case AF_DATA_CONFIRM_CMD:
      break;

    case AF_INCOMING_MSG_CMD:
      ZLOADApp_MessageMSGCB((afIncomingMSGPacket_t *)msgPtr);
      break;

#if defined ZPORT
    case KEY_CHANGE:
      zlHandleKeys(((keyChange_t *)msgPtr)->state, ((keyChange_t *)msgPtr)->keys);
      break;
#endif

    case ZDO_NEW_DSTADDR:
      /*
      dstEP = msgPtr[1];
      dstAddr = (zAddrType_t *)&msgPtr[2];

      dstAddr.addrMode = dstAddr->addrMode;
      dstAddr.endPoint = dstEP;
      if (dstAddr->addrMode == afAddr16Bit)
        dstAddr.addr.shortAddr = dstAddr->addr.shortAddr;
      else
        osal_memcpy(dstAddr.addr.extAddr, dstAddr->addr.extAddr, Z_EXTADDR_LEN);
       */
      break;

    case ZDO_STATE_CHANGE:
#if defined ZPORT
      if (((devStates_t)(((osal_event_hdr_t *)msgPtr)->status) == DEV_END_DEVICE_UNAUTH) ||
          ((devStates_t)(((osal_event_hdr_t *)msgPtr)->status) == DEV_END_DEVICE))
      {
        MT_X_FakeNwkJoinCnf();
      }
#endif
      break;

    default:
      break;
    }

    osal_msg_deallocate(msgPtr);  // Receiving task is responsible for releasing the memory.
  }
}

/**********************************************************************************
 * @fn      ZLOADApp_MessageMSGCB
 *
 * @brief   Handle a normal ZLOAD command or reply. Forward it to proper handler
 *
 * @param   MSGpkt -  pointer to incoming AF packet
 *
 * @return  none
 */
static void ZLOADApp_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt)
{
  zlmhdr_t *inMsg = (zlmhdr_t *)MSGpkt->cmd.Data;

  if (inMsg->zlhdr_msgid & ZLMSGID_REPLY_BIT)
  {
    ZLOADApp_handleReply(MSGpkt, inMsg);
  }
  else
  {
    ZLOADApp_handleCommand(MSGpkt, inMsg);
  }

  return;
}

/**********************************************************************************
 * @fn      ZLOADApp_handleCommand
 *
 * @brief   Handle a normal ZLOAD command
 *
 * @param   input
 *            MSGpkt  pointer to incoming AF packet. needed to get
 *                    reply address info
 *            msg     pointer to ZLOAD message
 *
 * @return  none
 */
static void ZLOADApp_handleCommand(afIncomingMSGPacket_t *MSGpkt, zlmhdr_t *msg)
{
  uint8 *cpc, *cpr, paylSize;
  preamble_t preamble;

#if defined ZPORT
  static uint8 *buf;
  if (!buf)
  {
    if (!(buf = osal_mem_alloc(sizeof(zlmhdr_t) + sizeof(zlreply_t))))
    {
      return;
    }
  }
#else
  uint8 *buf = NULL;
  // allocate the reply buffer. use a union of the replies -- they're all pretty
  // small -- except for Send Data. that one is preallocated because we want
  // to really have it and not spend time doing malloc/free for so many transactions
  // besides, we don't want a malloc() to fail during the transfer.
  if (msg->zlhdr_msgid != ZLMSGID_SEND_DATA)
  {
    if (!(buf = osal_mem_alloc(sizeof(zlmhdr_t) + sizeof(zlreply_t))))
    {
      return;
    }
  }
  else
  {
    // make sure we don't have a premature send-data command. this is awkward because
    // in the normal case we've done a "static" allocation (for the durtion of the session)
    // to prevent thousands of alloc/free calls or a failed alloc. but now, we dont' have
    // that "static" memory yet.
    if (s_State != ZLSTATE_SERVER)
    {
      // uh oh. we never got a start session. now we need memory to
      // send the reply
      if (!(buf = osal_mem_alloc(sizeof(zlmhdr_t) + sizeof(zlsdR_t))))
      {
        return;
      }
      // make the rest of the code work
      s_sdreply                     = (zlmhdr_t *)buf;
      s_sdreply->zlhdr_msgid        = ZLMSGID_SEND_DATA | ZLMSGID_REPLY_BIT;
      s_sdreply->zlhdr_msglen       = sizeof(zlsdR_t);
      s_sdrpayload                  = (zlsdR_t *)(buf+sizeof(zlmhdr_t));
      s_sdrpayload->zlsdR_state     = s_State;
      s_sdrpayload->zlsdR_errorCode = EC_SD_NOT_SERVER;
    }
  }
#endif

  cpr = buf + sizeof(zlmhdr_t);            // offset of reply payload
  cpc = (uint8 *)msg + sizeof(zlmhdr_t);   // offset of command payload

  switch (msg->zlhdr_msgid)  {
  case ZLMSGID_STATUSQ:
    {
      uint8 dlImagePreambleOffset;
      zlstatusR_t *reply = (zlstatusR_t *)cpr;
      paylSize = sizeof(zlstatusR_t);

      reply->zlsqR_state     = s_State;
      reply->zlsqR_errorCode = EC_NO_ERROR;
      // report capabiltities and version
      reply->zlsqR_ProtocolVersion = ZLOAD_PROTOCOL_VERSION;
      reply->zlsqR_capabilties     = ZLOAD_CAPABILTIES;
      cpr +=4;

      // populate the operational version values
      HalOADRead(PREAMBLE_OFFSET, (uint8 *)&preamble, sizeof(preamble_t), HAL_OAD_RC);
      cpr = osal_memcpy(cpr, &preamble.vers, ZL_IMAGE_ID_LENGTH);

      // do downloaded image if it's there
      osal_nv_read(PREAMBLE_NV_ID, 0, 1, &dlImagePreambleOffset);
      HalOADRead(dlImagePreambleOffset, (uint8 *)&preamble, sizeof(preamble_t), HAL_OAD_DL);
      if (preamble.vers != 0xFF)
      {
        cpr = osal_memcpy(cpr, &preamble.vers, ZL_IMAGE_ID_LENGTH);
      }
      else
      {
        osal_memset(cpr, 0, ZL_IMAGE_ID_LENGTH);
        cpr += ZL_IMAGE_ID_LENGTH;
      }

      cpr = osal_memcpy(cpr, &s_NextPacket, sizeof(s_NextPacket));
      osal_memcpy(cpr, &s_NumPktGet, sizeof(s_NumPktGet));
    }
    break;

  case ZLMSGID_SESSION_START:
#if defined ZPORT
    // the pass through condition starts here, when a client tries
    // to begin a session with the dongle.
    if (s_State != ZLSTATE_IDLE)  {
      ((zlbegsessR_t *)cpr)->zlbsR_errorCode = EC_BS_NOT_IDLE;
    }
    else if (s_serialMsg)  {
      // somehow this came over the serial port and it isn't legal
      ((zlbegsessR_t *)cpr)->zlbsR_errorCode = EC_BS_NOT_SERVER;
    }
    else  {
      // set mode. delay reply until we hear back from the Host
      s_PTSeqNum     = msg->zlhdr_seqnum;
      s_PTClientInfo = *MSGpkt;
      s_SessionID    = ((zlbegsessC_t *)cpc)->zlbsC_sessionID;
      if (zlPassOnStartSessionOK((uint8 *)msg))  {
        s_State = ZLSTATE_PASS_THROUGH;
        return;
      }
      else  {
        ((zlbegsessR_t *)cpr)->zlbsR_errorCode = EC_BS_NO_MEM;
      }
    }
#else
    do {
      uint8 dlImagePreambleOffset;
      zlbegsessR_t *reply = (zlbegsessR_t *)cpr;

      // assume we're OK and set up fixed part of reply here
      reply->zlbsR_blkSize   = ZL_DATA_BLK_SIZE;
      reply->zlbsR_numBlks   = ZL_NUM_DATA_BLKS;
      reply->zlbsR_state     = s_State;
      reply->zlbsR_errorCode = EC_NO_ERROR;

      paylSize = sizeof(zlbegsessR_t);

      if (s_State != ZLSTATE_IDLE)  {
        if (s_SessionID == ((zlbegsessC_t *)cpc)->zlbsC_sessionID)
        {
          // Ok - this must be a no ack retransmit or a retry.
        }
        else
        {
          reply->zlbsR_errorCode = EC_BS_NOT_IDLE;
          continue;  // done
        }
      }

      // see if there is a DL image to send
      osal_nv_read(PREAMBLE_NV_ID, 0, 1, &dlImagePreambleOffset);
      HalOADRead(dlImagePreambleOffset, (uint8 *)&preamble, sizeof(preamble_t), HAL_OAD_DL);
      if (preamble.vers != 0xFFFF)
      {
        if (!memcmp(cpc, (uint8 *)&preamble.vers, ZL_IMAGE_ID_LENGTH))  {
          // image matches request.
          osal_memcpy(&reply->zlbsR_imgLen, &preamble.len, sizeof(uint32));
          s_State             = ZLSTATE_SERVER;
          s_itype             = HAL_OAD_DL;
          reply->zlbsR_preambleOffset = (uint8)PREAMBLE_OFFSET;
          continue;    // done
        }
      }
      // no DL image. we know there's an operational image
      HalOADRead(PREAMBLE_OFFSET, (uint8 *)&preamble, sizeof(preamble_t), HAL_OAD_RC);
      if (!memcmp(cpc, (uint8 *)&preamble.vers, ZL_IMAGE_ID_LENGTH))  {
        // image matches request.
        osal_memcpy(&reply->zlbsR_imgLen, &preamble.len, sizeof(uint32));
        s_State                     = ZLSTATE_SERVER;
        s_itype                     = HAL_OAD_RC;
        reply->zlbsR_preambleOffset = (uint8)PREAMBLE_OFFSET;
      }
      else  {
        reply->zlbsR_errorCode = EC_BS_NO_MATCHES;
      }
    } while (0);

    // if we're good to go, allocate the memory for the SD replies
    // this will make things more efficient during the transfer
    if (!((zlbegsessR_t *)cpr)->zlbsR_errorCode)  {
      uint8 *cp;
      zlbegsessR_t *reply = (zlbegsessR_t *)cpr;

      // need memory for sending replies and for holding flash pages
      if (!(cp=osal_mem_alloc(sizeof(zlmhdr_t)+sizeof(zlsdR_t))))  {
        // oops -- none available. let Client decide what to do.
        s_State                = ZLSTATE_IDLE;
        reply->zlbsR_errorCode = EC_BS_NO_MEM;
      }
      else  {
        s_NumPktGet  = (preamble.len + (ZL_DATA_BLK_SIZE*ZL_NUM_DATA_BLKS-1)) /
                                       (ZL_DATA_BLK_SIZE*ZL_NUM_DATA_BLKS);
        s_blkSize = ZL_DATA_BLK_SIZE * ZL_NUM_DATA_BLKS;
        s_NextPacket = 0;
        s_SessionID  = ((zlbegsessC_t *)cpc)->zlbsC_sessionID;

        // set up other pointers
        s_sdreply               = (zlmhdr_t *)cp;
        s_sdreply->zlhdr_msgid  = ZLMSGID_SEND_DATA | ZLMSGID_REPLY_BIT;
        s_sdreply->zlhdr_msglen = sizeof(zlsdR_t);
        s_sdrpayload            = (zlsdR_t *)(cp+sizeof(zlmhdr_t));
        DO_EVENT_CALLBACK(ZLCB_EVENT_OADBEGIN_SERVER);
      }
    }
#endif
    break;

  case ZLMSGID_SESSION_TERM:
    {
      zlendsessR_t *reply = (zlendsessR_t *)cpr;

      reply->zlesR_state     = s_State;
      reply->zlesR_errorCode = EC_NO_ERROR;

      // guard against terminating the wrong session
      if (((zlendsessC_t *)cpc)->zlesC_sessionID != s_SessionID)  {
        // wrong session ID
        reply->zlesR_errorCode = EC_ES_BAD_SESS_ID;
      }
#if defined ZPORT
      else if (s_serialMsg)  {
        // somehow this came over the serial port and it isn't legal
        ((zlbegsessR_t *)cpr)->zlbsR_errorCode = EC_ES_NOT_SERVER;
      }
#endif
      else  {
#if defined ZPORT
        if (ZLSTATE_PASS_THROUGH == s_State)  {
          // everything is OK. pass this up to host if we're in pass-through mode
          // forward message to host
          zahdrin_t *zain = zlBuildExternalInboundSerialMSG(&s_PTClientInfo, (uint8 *)msg, sizeof(zlmhdr_t) + sizeof(zlendsessC_t));

          if (zain)  {
            if (SUCCESS != osal_start_timerEx(oad_app_taskId, ZLOAD_XFER_DONE_EVT, SDR_WAIT_TO))
            {
              osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
            }

            s_PTClientInfo = *MSGpkt;
            zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlendsessC_t));
            osal_mem_free(zain);
            return;
          }
          else  {
            reply->zlesR_errorCode = EC_ES_NO_MEM;
          }
        }
#else
        if (s_State == ZLSTATE_SERVER)  {
          // everything is OK.
          osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
        }
#endif
        else  {
          // right session ID but I'm not the server
          reply->zlesR_errorCode = EC_ES_NOT_SERVER;
        }
      }
    }
    paylSize = sizeof(zlendsessR_t);
    break;

        case ZLMSGID_CLIENT_CMD:
            {
                zlclientR_t *reply = (zlclientR_t *)cpr;

                reply->zlclR_errorCode = EC_NO_ERROR;
                paylSize               = sizeof(zlclientR_t);
                reply->zlclR_state     = s_State;

#if defined ZPORT
                // we can be the client only if the dongle code itself is being updated
                // and then only if the Host is the server
                if (!s_serialMsg)  {
                    reply->zlclR_errorCode = EC_CL_NOT_CLIENT;
                }
                else                
#endif
                if (s_State != ZLSTATE_IDLE) {
                  if (!osal_memcmp(s_clientInfo, cpc, sizeof(zlclientC_t))) {
                    reply->zlclR_errorCode = EC_CL_NOT_IDLE;
                  }
                }
#if (HAL_OAD_XNV_IS_INT && ((HAL_OAD_DL_OSET % HAL_FLASH_PAGE_SIZE) != 0))
                // Bug 2946 - HalXNVWrite() only triggers a page erase when the first byte of a
                // page boundary is written, so when dividing the available internal flash in half
                // results in a page being split in half, this remedial measure is necessary.
                else if (zlEraseHalfPage() != SUCCESS)
                {
                  reply->zlclR_errorCode = EC_CL_NO_MEM;
                }
#endif
                else  {
                    zlclientC_t *cmd = (zlclientC_t *)cpc;
                    uint8       *cp;

                    // need memory for holding client info, and current send-data cmd
                    // do one alloc and set up the pointers
                    cp = osal_mem_alloc(sizeof(zlclientC_t)+sizeof(zlmhdr_t)+sizeof(zlsdC_t));
                    if (!cp)  {
                        // no more memory. let Commissioner decide what to do. maybe retry later.
                        reply->zlclR_errorCode = EC_CL_NO_MEM;
                        break;
                    }
                    else  {
                        // set up other pointers
                        s_clientInfo  = (zlclientC_t *)cp;
                        s_sdcmd       = (zlmhdr_t *)(cp+sizeof(zlclientC_t));
                        s_sdcpayload  = (zlsdC_t *)(cp+sizeof(zlclientC_t)+sizeof(zlmhdr_t));
                    }

                    // the info on the image to be downloaded and
                    // the address of the Server is now saved both here
                    // and in persistent memory
                    osal_memcpy(s_clientInfo, cpc, sizeof(zlclientC_t));

                    // populate destination address structure for convenience
                    dstAddr.addrMode       = afAddr16Bit;
                    dstAddr.endPoint       = cmd->zlclC_endp;
                    osal_memcpy(&dstAddr.addr.shortAddr, &cmd->zlclC_nwk, sizeof(uint16));

                    // set event to cause the session to begin
                    osal_set_event(oad_app_taskId, ZLOAD_IS_CLIENT_EVT);
                }
            }

            break;

        case ZLMSGID_CODE_ENABLE:
            {
                zlceR_t *reply   = (zlceR_t *)cpr;

                paylSize = sizeof(zlceR_t);

                reply->zlceR_state = s_State;

#if defined ZPORT
                // we can enable code if the dongle code itself is being updated.
                if (!s_serialMsg)  {
                    reply->zlceR_errorCode = EC_CE_NOT_CLIENT;
                }
                else
#endif
                if (s_State != ZLSTATE_IDLE)  {
                    reply->zlceR_errorCode = EC_CE_NOT_IDLE;
                }
                // see if we're supposed to enable the DL image. spec is in command payload
                // see if there is one...
                else {
                  uint8 dlImagePreambleOset;
                  osal_nv_read(PREAMBLE_NV_ID, 0, 1, &dlImagePreambleOset);
                  HalOADRead(dlImagePreambleOset,(uint8 *)&preamble,sizeof(preamble_t),HAL_OAD_DL);
                  if (preamble.vers != 0xFFFF) {
                    // see if they match
                    if (!memcmp(cpc, (uint8 *)&preamble.vers, ZL_IMAGE_ID_LENGTH))  {
                        // DL image there and matches request, see if image is sane.
                        if (SUCCESS == HalOADChkDL(dlImagePreambleOset)) {
                            //set event to cause reset
                            if (SUCCESS != osal_start_timerEx(oad_app_taskId, ZLOAD_CODE_ENABLE_EVT,
                                                              SDC_WAIT_TO_ENABLE))
                            {
                              osal_set_event(oad_app_taskId, ZLOAD_CODE_ENABLE_EVT);
                            }
                            reply->zlceR_errorCode = EC_NO_ERROR;
                        }
                        else  {
                            reply->zlceR_errorCode = EC_CE_IMAGE_INSANE;
                        }
                    }
                    else  {
                        // DL image there but doesn't match request
                        reply->zlceR_errorCode = EC_CE_NO_MATCH;
                    }
                  }
                  else  {
                    // no DL image
                    reply->zlceR_errorCode = EC_CE_NO_IMAGE;
                  }
                }
            }
            break;

        case ZLMSGID_SEND_DATA:
#if defined ZPORT
        {
          zlsdR_t *reply = (zlsdR_t *)cpr;

          paylSize = sizeof(zlsdR_t);

          // do a sanity check.
          if (s_serialMsg)  {
            reply->zlsdR_errorCode = EC_SD_NOT_SERVER;
          }
          else if (ZLSTATE_PASS_THROUGH == s_State)  {
            // this should be good enough...
            if (s_PTClientInfo.srcAddr.addr.shortAddr != MSGpkt->srcAddr.addr.shortAddr)  {
                // wrong client -- not the current client
                reply->zlsdR_errorCode = EC_SD_NOT_CLIENT;
            }
            else if (((zlsdC_t *)cpc)->zlsdC_sessionID != s_SessionID)  {
                reply->zlsdR_errorCode = EC_SD_BAD_SESS_ID;
            }
            else  {
                zahdrin_t *zain = zlBuildExternalInboundSerialMSG(&s_PTClientInfo, (uint8 *)msg, sizeof(zlmhdr_t) + sizeof(zlsdC_t));

                // forward message to host
                if (zain)  {
                    zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlsdC_t));
                    osal_mem_free(zain);
                    // save the AF data. there may be AF parameters we need for the reply
                    s_PTClientInfo = *MSGpkt;
                    return;
                }
                else  {
                    reply->zlsdR_errorCode = EC_SD_NO_MEM;
                }
            }
          }
          else  {
            reply->zlsdR_errorCode = EC_SD_NOT_SERVER;
          }
      }
#else
            paylSize = sizeof(zlsdR_t);
            if (s_State != ZLSTATE_SERVER)  {
                // this is the error case when we get a SD before we've started a session
                break;
            }
            s_sdrpayload->zlsdR_state = s_State;
            buf = (uint8 *)s_sdreply;
            if (((zlsdC_t *)cpc)->zlsdC_sessionID != s_SessionID)  {
                s_sdrpayload->zlsdR_errorCode = EC_SD_BAD_SESS_ID;
            }
            else  {
                // set error code first. it might get reset in the processing routine
                s_sdrpayload->zlsdR_errorCode = EC_NO_ERROR;
                // process the command
                zlProcessSDC((zlsdC_t *)cpc);
            }
#endif
            break;

        case ZLMSGID_RESET:
#if defined ZPORT
            // always legal if over serial port
            if (!s_serialMsg)  {
                return;
            }
            else  
#endif
            {
                zlrstR_t *reply = (zlrstR_t *)cpr;

                reply->zlrstR_state     = s_State;
                reply->zlrstR_errorCode = EC_NO_ERROR;
            }
            paylSize = sizeof(zlrstR_t);
            osal_set_event(oad_app_taskId, ZLOAD_RESET_EVT);
            break;

        default:
            return;
    }

    // the case above has filled in the payload. the header is the
    // same for all. go ahead and fill in header and send reply
    {
        zlmhdr_t *hdr = (zlmhdr_t *)buf;

        hdr->zlhdr_msgid  = msg->zlhdr_msgid | ZLMSGID_REPLY_BIT;
        hdr->zlhdr_seqnum = msg->zlhdr_seqnum;
        hdr->zlhdr_msglen = paylSize;

#if defined ZPORT
        if (!s_serialMsg)  {
#endif
        AF_DataRequest( &MSGpkt->srcAddr,
                         afFindEndPointDesc( MSGpkt->endPoint),
                         MSGpkt->clusterId,
                         sizeof(zlmhdr_t) + paylSize, buf,
                        &MSGpkt->cmd.TransSeqNumber,
                         AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
#if defined ZPORT
        } else  {
            zahdrin_t *zain = zlBuildInternalInboundSerialMSG((uint8 *)hdr, sizeof(zlmhdr_t) + paylSize);

            if (zain)  {
                zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + paylSize);
                osal_mem_free(zain);
            }
        }
#else
        if ((buf != NULL) && 
           ((msg->zlhdr_msgid != ZLMSGID_SEND_DATA) || (s_State != ZLSTATE_SERVER)))
        {
          osal_mem_free(buf);
        }
#endif
    }

    return;
}

#if !defined ZPORT
/**********************************************************************************
 * @fn      zlProcessSDC
 *
 * @brief   Process the SDC command.
 *
 * @param   cmd - A valid alsdC_t structure with valid data.
 *
 * @return  none
 */
static void zlProcessSDC(zlsdC_t *cmd)
{
  uint16 reqPktNum;
  osal_memcpy(&reqPktNum, &cmd->zlsdC_pktNum, sizeof(uint16));

  // if the requested packet is the previous one the Client didn't get the
  // transmission for some reason. just resend the previous one. the data
  // are already in there. if we never got the command this test will fail.
  if (s_NextPacket && (reqPktNum == (s_NextPacket-1)))  {
      return;
  }

  // make sure the request is otherwise valid
  if (!zlIsReqPacketNumOK(reqPktNum))  {
      s_sdrpayload->zlsdR_errorCode = EC_SD_BAD_PKT_NUM;
      return;
  }

  // Read data into reply buffer.
  HalOADRead((uint32)s_NextPacket * s_blkSize, s_sdrpayload->zlsdR_data,
                                    ZL_DATA_BLK_SIZE*ZL_NUM_DATA_BLKS, s_itype);

  // set packet number in reply
  osal_memcpy(&s_sdrpayload->zlsdR_pktNum, &reqPktNum, sizeof(uint16));
  s_NextPacket++;
}

/**********************************************************************************
 * @fn      zlIsReqPacketNumOK
 *
 * @brief   Check validity of resquested packet number as Server
 *
 * @param   input
 *            reqNum  requested packet number
 *
 * @return  0: packet number request illegal
 *          1: packet number request is valid
 */
static uint8 zlIsReqPacketNumOK(uint16 reqNum)
{
    if (reqNum >= s_NumPktGet) {
        return 0;
    }
    return 1;
}
#endif

/**********************************************************************************
 * @fn      ZLOADApp_handleReply
 *
 * @brief   Handle a normal ZLOAD reply
 *
 * @param   input
 *            msg pointer to ZLOAD message
 *
 * @return  none.
 */
static void ZLOADApp_handleReply(afIncomingMSGPacket_t *MSGpkt, zlmhdr_t *msg)
{
  uint8    *cpr;
#if defined ZPORT
  uint8 msgSize = 0;
#endif

#if !defined ZPORT
  // right reply?
  if (s_firstTx)  {
      s_firstTx = 0;
  }
  else  {
    if (msg->zlhdr_seqnum != s_lastTxSeqNum)  {
      return;
    }
    if (msg->zlhdr_seqnum == s_lastSeen)  {
      // duplicate reply to a resend
      return;
    }
  }
  s_lastSeen = msg->zlhdr_seqnum;
#endif

  cpr = (uint8 *)msg + sizeof(zlmhdr_t);   // offset of reply payload

  // generate replies to commands here. reply may generate another command out, for
  // example, if data are being transfered and the dvice is acting as client.
  // Commands handled here. replies in next routine

  // ignore reply bit on switch()
  switch (msg->zlhdr_msgid & (0xFF ^ ZLMSGID_REPLY_BIT))  {
  case ZLMSGID_STATUSQ:
#if defined ZPORT
    msgSize = 20;
    //  * * *  NO BREAK  * * *
  case ZLMSGID_CLIENT_CMD:
  case ZLMSGID_RESET:
  case ZLMSGID_CODE_ENABLE:
    if (!msgSize)  {
      msgSize = 2;
    }
    // right now, only the host could have sent these -- the dongle never sends these
    // commands on its own. it's a proxy reply. forward it on...
    if (!s_serialMsg)  {
      zahdrin_t *zain = zlBuildExternalInboundSerialMSG(MSGpkt, (uint8 *)msg, sizeof(zlmhdr_t) + msgSize);

      if (zain)  {
        zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + msgSize);
        osal_mem_free(zain);
      }
    }
#endif
    break;

  case ZLMSGID_SESSION_START:
#if defined ZPORT
            // this can happen both in pass-through and client mode -- the dongle could be
            // getting updated firmware and is the client receiving the start session reply.
            // in any case we cannot get this reply over air. it has to have come from the Host
            if (!s_serialMsg)  {
                break;
            }
            else if (ZLSTATE_PASS_THROUGH == s_State) {
                // pass reply back to original client
                msg->zlhdr_seqnum = s_PTSeqNum;
                AF_DataRequest( &s_PTClientInfo.srcAddr,
                   afFindEndPointDesc( s_PTClientInfo.endPoint ),
                   s_PTClientInfo.clusterId,
                   sizeof(zlmhdr_t) + sizeof(zlbegsessR_t), (uint8 *)msg,
                   &s_PTClientInfo.cmd.TransSeqNumber,
                   AF_TX_OPTIONS_NONE, AF_DEFAULT_RADIUS );
            }
            else if (((zlbegsessR_t *)cpr)->zlbsR_errorCode)  {
                // Server rejected the real client session. back to Idle state.
                osal_set_event(oad_app_taskId, ZLOAD_RESET_EVT);
            }
            // reply from Server to Begin Session command from Client
            else if (ZLSTATE_CLIENT != s_State) {
                // I didn't send it...ignore.
                break;
            }
#else

		    // reply from Server to Begin Session command from Client
		    if (s_State != ZLSTATE_CLIENT) {
				// I didn't send it...ignore.
				break;
			}
      else if (((zlbegsessR_t *)cpr)->zlbsR_errorCode)  {
        // Server rejected the session. back to Idle state.
        osal_set_event(oad_app_taskId, ZLOAD_RESET_EVT);
      }
#endif
      else  {
        // get length and make sure we have room
        uint32 imglen = ((zlbegsessR_t *)cpr)->zlbsR_imgLen;

        if (imglen > HalOADAvail()) {
          // no room. terminate session
          osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
        }
        else  {
          // TODO: The OAD Dongle Tool is broken and passes the wrong offset.
          //uint8 dlImagePreambleOffset = ((zlbegsessR_t *)cpr)->zlbsR_preambleOffset;
          //osal_nv_write(PREAMBLE_NV_ID, 0, 1, &dlImagePreambleOffset);

          s_blkSize  = ((zlbegsessR_t *)cpr)->zlbsR_blkSize;
          s_blkSize *= ((zlbegsessR_t *)cpr)->zlbsR_numBlks;

          // figure out how many data packets to request.
          s_NumPktGet   = (imglen+(s_blkSize-1)) / s_blkSize;
          s_NextPacket  = 0;
          // request "next" (i.e., first) packet
          zlRequestNextDataPacket();
        }
      }
      break;

  case ZLMSGID_SESSION_TERM:
#if defined ZPORT
            // pass it back if we're in pass-through mode
          if (s_serialMsg && (ZLSTATE_PASS_THROUGH == s_State))  {
              AF_DataRequest( &s_PTClientInfo.srcAddr,
                 afFindEndPointDesc( s_PTClientInfo.endPoint),
                 s_PTClientInfo.clusterId,
                 sizeof(zlmhdr_t) + sizeof(zlendsessR_t), (uint8 *)msg,
                 &s_PTClientInfo.cmd.TransSeqNumber,
                 AF_TX_OPTIONS_NONE, AF_DEFAULT_RADIUS );
              // in any case, clean up
              osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
            }
#else
            // don't care. right now the Commisioner can't be
            // the client so it will never get a reply to the
            // terminate session command because it can't send
            // that command.
#endif
            break;

  case ZLMSGID_SEND_DATA:
#if defined ZPORT
            if (!s_serialMsg)  {
                // reply should not be over air
                break;
            }
            else if (ZLSTATE_CLIENT == s_State)  {
                // not a don't care if we're in client state
                zlProcessSDR((zlsdR_t *)cpr);
            }
            else if (ZLSTATE_PASS_THROUGH == s_State)  {
                // forward to client
                AF_DataRequest( &s_PTClientInfo.srcAddr,
                   afFindEndPointDesc( s_PTClientInfo.endPoint),
                   s_PTClientInfo.clusterId,
                   sizeof(zlmhdr_t) + sizeof(zlsdR_t), (uint8 *)msg,
                   &s_PTClientInfo.cmd.TransSeqNumber,
                   AF_TX_OPTIONS_NONE, AF_DEFAULT_RADIUS );
            }
#else
            // not a don't care if we're in client state
            if (s_State == ZLSTATE_CLIENT)  {
                zlProcessSDR((zlsdR_t *)cpr);
            }
#endif
            break;

  default:
            // don't care
            break;
  }
}

/**********************************************************************************
 * @fn      zlProcessSDR
 *
 * @brief   Process the Send Data Reply as Client
 *
 * @param   input
 *            sdr  pointer to received Send Data Reply
 *
 * @return  none
 */
static void zlProcessSDR(zlsdR_t *sdr)
{
  uint16 tmp;
  osal_memcpy(&tmp, &sdr->zlsdR_pktNum, sizeof(uint16));

  // is this the right packet?
  if (s_NextPacket > tmp)
  {
    return;  // we've already seen this one. ignore it
  }
  else if (s_NextPacket < tmp)  {
    // uh oh. we've missed one. kill session...
    osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
    return;
  }
  else if (sdr->zlsdR_errorCode)  {
    // there are no data here. just ignaore the packet. if things
    // are relly screwed up the timer will expire and we'll quit.
    return;
  }

  // Store the data in Xtra-NV.
  HalOADWrite((uint32)s_NextPacket * s_blkSize, sdr->zlsdR_data, s_blkSize, HAL_OAD_DL);
  s_NextPacket++;  // OK to increment next packet number now.
  zlRequestNextDataPacket();  // ask for next packet
}

/**********************************************************************************
 * @fn      zlRequestNextDataPacket
 *
 * @brief   Set up the next data packet request. Takes care of knowing when
 *          transfer is done. Takes care of retry timer.
 *
 * @param   none
 *
 * @return  none
 */
static void zlRequestNextDataPacket()
{
    if (s_NextPacket >= s_NumPktGet)  {
        osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
        // kill response timeout check
        osal_stop_timerEx(oad_app_taskId, ZLOAD_SDRTIMER_EVT);
        return;
    }

#if defined ZPORT
    s_sdcpayload->zlsdC_pktNum = s_NextPacket;
    s_sdcmd->zlhdr_seqnum      = ++s_lastSeqNum;

    {
        zahdrin_t *zain = zlBuildInternalInboundSerialMSG((uint8 *)s_sdcmd, sizeof(zlmhdr_t) + sizeof(zlsdC_t));

        if (zain)  {
            zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlsdC_t));
            osal_mem_free(zain);
        }
    }
#else
    osal_memcpy(&s_sdcpayload->zlsdC_pktNum, &s_NextPacket, sizeof(uint16));
    s_sdcmd->zlhdr_seqnum      = ++s_lastTxSeqNum;

    zlSendCommand(sizeof(zlmhdr_t) + sizeof(zlsdC_t), (uint8 *)s_sdcmd);
#endif

    if (SUCCESS != osal_start_timerEx(oad_app_taskId, ZLOAD_SDRTIMER_EVT, SDR_WAIT_TO))
    {
      osal_set_event(oad_app_taskId, ZLOAD_SDRTIMER_EVT);
    }
    s_SDCRetryCount = SDC_RETRY_COUNT;

    // set timeout timer
    // need osal timer resource and set event flag and need a method to process timeout event
    return;
}

/**********************************************************************************
 * @fn      zlCleanupOnReset
 *
 * @brief   ZLOAD reset occurred. Free resources and reset state machine and other
 *          supporting constructs
 *
 * @param   none
 *
 * @return  none
 */
static void zlCleanupOnReset()
{
#if !defined ZPORT
  if (ZLSTATE_CLIENT == s_State)  {
    DO_EVENT_CALLBACK(ZLCB_EVENT_OADEND_CLIENT);
  }
  else if (ZLSTATE_SERVER == s_State)  {
    DO_EVENT_CALLBACK(ZLCB_EVENT_OADEND_SERVER);
  }
#endif

  zlResetState();
}

/**************************************************************************************
 * @fn      zlCleanupOnXferDone
 *
 * @brief   Data transfer session complete (not necessarily correctly). Free resources
 *          and reset state machine and other supporting constructs. Send Terminate
 *          Session command to Server.
 *
 * @param   none
 *
 * @return  none
 */
static void  zlCleanupOnXferDone()
{
#if defined ZPORT
    if (ZLSTATE_PASS_THROUGH == s_State)  {
      s_State = ZLSTATE_IDLE;
      return;
    }
#endif

  // check for flashing last block and send Session Terminate command
  if (ZLSTATE_CLIENT == s_State)  {
    zlmhdr_t *hdr = osal_mem_alloc(sizeof(zlmhdr_t) + sizeof(zlendsessC_t));

    if (hdr)  {
      zlendsessC_t *cmd = (zlendsessC_t *)((uint8 *)hdr + sizeof(zlmhdr_t));

      hdr->zlhdr_msgid  = ZLMSGID_SESSION_TERM;
      hdr->zlhdr_msglen = sizeof(zlendsessC_t);

      cmd->zlesC_sessionID = s_SessionID;
      hdr->zlhdr_seqnum    = ++s_lastTxSeqNum;

#if defined ZPORT
            {
                zahdrin_t *zain = zlBuildInternalInboundSerialMSG((uint8 *)hdr, sizeof(zlmhdr_t) + sizeof(zlendsessC_t));

                if (zain)  {
                    zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlendsessC_t));
                    osal_mem_free(zain);
                }
            }
#else
      zlSendCommand(sizeof(zlmhdr_t) + sizeof(zlendsessC_t), (uint8 *)hdr);
#endif

      osal_mem_free(hdr);
    }

#if !defined ZPORT
    DO_EVENT_CALLBACK(ZLCB_EVENT_OADEND_CLIENT);
#endif
  }
#if !defined ZPORT
  else if (ZLSTATE_SERVER == s_State) {
    DO_EVENT_CALLBACK(ZLCB_EVENT_OADEND_SERVER);
  }
#endif

  zlResetState();
}

/**************************************************************************************
 * @fn      zlStartClientSession
 *
 * @brief   We have been commanded to be in the Client role. Set up to start the
 *          session with the Server by sending the Begin Session command
 *
 * @param   none
 *
 * @return  none
 */
static void zlStartClientSession()
{
    uint8 *buf;

    // we already have Server info. Contact Server to set up session
    if (buf=osal_mem_alloc(sizeof(zlmhdr_t) + sizeof(zlbegsessC_t)))  {
        zlmhdr_t     *hdr = (zlmhdr_t *)buf;
        zlbegsessC_t *cmd = (zlbegsessC_t *)((uint8 *)buf+sizeof(zlmhdr_t));

        // initialize session id as lsb of IEEE address
        if (!s_SessionID)  {
            uint8 addr[Z_EXTADDR_LEN];

            ZMacGetReq(ZMacExtAddr, addr);
            s_SessionID = addr[0];
        }

        s_State = ZLSTATE_CLIENT;
        s_SessionID++;

        hdr->zlhdr_msgid     = ZLMSGID_SESSION_START;
        hdr->zlhdr_seqnum    = ++s_lastTxSeqNum;
        hdr->zlhdr_msglen    = sizeof(zlbegsessC_t);
        osal_memcpy(&cmd->zlbsC_ver, &s_clientInfo->zlclC_ver, sizeof(uint16));
        osal_memcpy(&cmd->zlbsC_manu, &s_clientInfo->zlclC_manu, sizeof(uint16));
        osal_memcpy(&cmd->zlbsC_prod, &s_clientInfo->zlclC_prod, sizeof(uint16));
        cmd->zlbsC_sessionID = s_SessionID;
        s_NextPacket         = 0;

        // populate session ID element in the Send data payload
        s_sdcpayload->zlsdC_sessionID = s_SessionID;

        // set up SD command header
        s_sdcmd->zlhdr_msgid  = ZLMSGID_SEND_DATA;
        s_sdcmd->zlhdr_msglen = sizeof(zlsdC_t);

#if defined ZPORT
        {
            zahdrin_t *zain = zlBuildInternalInboundSerialMSG((uint8 *)hdr, sizeof(zlmhdr_t) + sizeof(zlbegsessC_t));

            if (zain)  {
                zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlbegsessC_t));
                osal_mem_free(zain);
            }
        }
#else
        if (zlSendCommand(sizeof(zlmhdr_t) + sizeof(zlbegsessC_t), buf))
        {
        }
#endif

        osal_mem_free(buf);
        DO_EVENT_CALLBACK(ZLCB_EVENT_OADBEGIN_CLIENT);
    }
	else  {
		// we couldn't send the Begin Session command. go back to Idle state.
        osal_set_event(oad_app_taskId, ZLOAD_RESET_EVT);
	}

    return;
}

#if !defined ZPORT
/**************************************************************************************
 * @fn      zlSendCommand
 *
 * @brief   Send command to destination. Can be either over air or to the Host. This
 *          routine takes care of the context.
 *
 * @param   input
 *            length   length of message being sent
 *            buf      pointer to message
 *
 * @return  0: successful send
 *          1: unseccessful send (return code for AF layer)
 */
static uint8 zlSendCommand(uint8 length, uint8 *buf)
{
    uint8  rc;

    rc = AF_DataRequest( &dstAddr,
                          afFindEndPointDesc( OAD_epDesc.endPoint ),
                          OAD_CLUSTERID_CS,
                          length, buf,
                         &transId,
                          AF_DISCV_ROUTE, DEF_NWK_RADIUS );

    return rc;
}
#endif

/**************************************************************************************
 * @fn      zlResendSDC
 *
 * @brief   Resend the previous Send Data Command because a repply was not received
 *          before a timeout.
 *
 * @param   none
 *
 * @return  none.
 */
static void zlResendSDC()
{
    if (s_State != ZLSTATE_CLIENT)  {
        // late expiration -- don't care
        return;
    }

    if (--s_SDCRetryCount)  {
#if defined ZPORT
        s_sdcmd->zlhdr_seqnum = ++s_lastSeqNum;
        {
            zahdrin_t *zain = zlBuildInternalInboundSerialMSG((uint8 *)s_sdcmd, sizeof(zlmhdr_t) + sizeof(zlsdC_t));

            if (zain)  {
                zlSendSerial((uint8 *)zain, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlsdC_t));
                osal_mem_free(zain);
            }
        }
#else
        // the static structure still contains the last packet number to be requested.
        // the only way the packet number gets bumped is if a reply is received.
        zlSendCommand(sizeof(zlmhdr_t) + sizeof(zlsdC_t), (uint8 *)s_sdcmd);
#endif

        if (SUCCESS != osal_start_timerEx(oad_app_taskId, ZLOAD_SDRTIMER_EVT, SDR_WAIT_TO))
        {
          osal_set_event(oad_app_taskId, ZLOAD_SDRTIMER_EVT);
        }
    }
    else  {
        // too many retries. abandon session.
        osal_set_event(oad_app_taskId, ZLOAD_XFER_DONE_EVT);
    }

    return;
}

#if !defined ZPORT
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
void oadAppRegisterCB(void (*pCBFunction)(uint16), uint16 eventMask)
{
  s_pCallback = pCBFunction;
  s_eventMask = eventMask;
  return;
}
#endif

/**************************************************************************************************
 * @fn          zlResetState
 *
 * @brief       This function frees any memory in use and resets the zl state info.
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
static void zlResetState(void)
{
  osal_stop_timerEx(oad_app_taskId, ZLOAD_SDRTIMER_EVT);  // kill response timeout check

  if (s_sdreply)
  {
    osal_mem_free(s_sdreply);
    s_sdreply = NULL;
  }

  if (s_clientInfo)
  {
    osal_mem_free(s_clientInfo);
    s_clientInfo = NULL;
  }

  s_State = ZLSTATE_IDLE;
  s_NextPacket      = 0;
  s_NumPktGet       = 0;
}

#if defined ZPORT
/*********************************************************************
 * @fn      ZLOADApp_SerialMSGCB
 *
 * @brief   Message from Host application. It's a command, a reply,
 *          or a proxy command. Send it on appropriately
 *
 * @param   input
 *            inMsg  pointer to incoming message
 *
 * @return  none
 */
static void ZLOADApp_SerialMessageMSGCB(zahdrout_t *zaout)
{
    zlmhdr_t *zlhdr = (zlmhdr_t *)zaout->zaproxy_payload;

    // is this for me?
    if (zaout->zaproxy_nwkAddr == s_myNwkAddr)  {
      // is this a reply?
      if (zlhdr->zlhdr_msgid & ZLMSGID_REPLY_BIT)  {
          ZLOADApp_handleReply(NULL, zlhdr);
      }
      else  {
          ZLOADApp_handleCommand(NULL, zlhdr);
      }
    }
    else  {
        // it's a proxy command
        zlZArchitectProxyMsg(zaout);
    }
    return;
}

/*********************************************************************
 * @fn      zlSendSerial
 *
 * @brief   Prepare message outgoing over serial port.
 *          malloc and copy to add serial encapsulation.
 *
 * @param   input
 *            buf  pointer to ougoing message buffer
 *            len  length of message
 *
 * @return  none
 */
static void zlSendSerial(uint8 *buf, uint8 len)
{
    uint8 *nbuf;

    if ((nbuf=osal_mem_alloc(len+1)))  {
        osal_memcpy(nbuf+1, buf, len);
        *nbuf = OAD_ENDPOINT;
#if !defined(ZTOOL_SUPPORT)
        MTProcessAppRspMsg(nbuf, len + 1);
#else
        // ZTool expects an 81 byte message
        MTProcessAppRspMsg(nbuf, 81);
#endif
        osal_mem_free(nbuf);
    }
    return;
}

/****************************************************************************
 * @fn      zlZArchitectProxyCommand
 *
 * @brief   Send command to target device on behalf of ZArchitect Host App
 *
 * @param   input
 *            info  pointer to ZArchitect proxy message
 *
 * @return  none
 */
static void zlZArchitectProxyMsg(zahdrout_t *info)
{
    afAddrType_t taddr;
    zlmhdr_t     *zlhdr = (zlmhdr_t *)info->zaproxy_payload;

    // only certain outgoing messages are valid proxy commands
    switch (zlhdr->zlhdr_msgid)  {
        case ZLMSGID_SESSION_START:
        case ZLMSGID_SESSION_TERM:
        case ZLMSGID_SEND_DATA:
            return;
    }

    // fill in address info
    taddr.addrMode       = afAddr16Bit;
    taddr.endPoint       = info->zaproxy_endp;
    taddr.addr.shortAddr = info->zaproxy_nwkAddr;

    AF_DataRequest( &taddr,
                     afFindEndPointDesc( info->zaproxy_endp ),
                     info->zaproxy_ClusterID,
                     info->zaproxy_msglen, info->zaproxy_payload,
                    &transId,
                     AF_TX_OPTIONS_NONE, DEF_NWK_RADIUS );
}

// start a session with the host on behalf of the client from which
// the start session was received. do this by sending self a client
// command using the start session parameters. ZLOAD will think it
// is a client message coming from the host asking it to come get
// the image being requested by the real client.
static uint8 zlPassOnStartSessionOK(uint8 *msg)
{
    zahdrin_t *zamsgin;

    // forward the Begin Session command
    zamsgin = zlBuildExternalInboundSerialMSG(&s_PTClientInfo, msg, sizeof(zlmhdr_t) + sizeof(zlbegsessC_t));

    if (!zamsgin)  {
      return 0;
    }

    zlSendSerial((uint8 *)zamsgin, SIZEOF_ZAIN_HDR + sizeof(zlmhdr_t) + sizeof(zlbegsessC_t));

    osal_mem_free(zamsgin);

    return 1;
}

// inbound messages can be built from address information gleaned from the client
// during a pass-through session setup. but there can also be inbound messages
// such as status replies that are not part of any ongoing client session. in this
// case the address information comes from the AF message. in either case this
// information is requried to populate the header for for the host.
static zahdrin_t *zlBuildExternalInboundSerialMSG(afIncomingMSGPacket_t *addressInfo, uint8 *zlmsg, uint8 len)
{
    zahdrin_t *zamsgin;

    // build message
    zamsgin = (zahdrin_t *)osal_mem_alloc(SIZEOF_ZAIN_HDR + len);

    if (!zamsgin)  {
      return 0;
    }

    zamsgin->zaproxy_nwkAddr   = addressInfo->srcAddr.addr.shortAddr;
    zamsgin->zaproxy_endp      = addressInfo->endPoint;
    zamsgin->zaproxy_ClusterID = addressInfo->clusterId;
    zamsgin->zaproxy_msglen    = len;

    osal_memcpy(zamsgin->zaproxy_payload, zlmsg, len);

    return zamsgin;
}

static zahdrin_t *zlBuildInternalInboundSerialMSG(uint8 *zlmsg, uint8 len)
{
    zahdrin_t *zamsgin;

    // build message
    zamsgin = (zahdrin_t *)osal_mem_alloc(SIZEOF_ZAIN_HDR + len);

    if (!zamsgin)  {
      return 0;
    }

    zamsgin->zaproxy_nwkAddr   = s_myNwkAddr;
    zamsgin->zaproxy_endp      = OAD_ENDPOINT;
    zamsgin->zaproxy_ClusterID = OAD_CLUSTERID_CS;
    zamsgin->zaproxy_msglen    = len;

    osal_memcpy(zamsgin->zaproxy_payload, zlmsg, len);

    return zamsgin;
}

/*********************************************************************
 * @fn      zlHandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
static void zlHandleKeys(uint8 shift, uint8 keys)
{
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      mtxMode = TRUE;
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
      mtxMode = FALSE;
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
}
#else
#if (HAL_OAD_XNV_IS_INT && ((HAL_OAD_DL_OSET % HAL_FLASH_PAGE_SIZE) != 0))
// Bug 2946 - HalXNVWrite() only triggers a page erase when the first byte of a page
// boundary is written, so when dividing the available internal flash in half results in a
// page being split in half, this remedial measure is necessary.
static Status_t zlEraseHalfPage(void)
{
  const uint16 hPgSz = HAL_FLASH_PAGE_SIZE / 2;
  // HalXNVRead/Write routines add HAL_OAD_DL_OSET to the 'addr' parameter, so pass in a 'negative'.
  const uint32 addr = 0L - hPgSz;
  uint8 *pBuf = osal_mem_alloc(hPgSz);

  if (NULL == pBuf)
  {
    return FAILURE;
  }

  HalXNVRead(addr, pBuf, hPgSz);
  // This triggers the full page erase and restores the lower half.
  HalXNVWrite(addr, pBuf, hPgSz);
  osal_mem_free(pBuf);
  return SUCCESS;
}
#endif
#endif
#pragma diag_default=Pa039
