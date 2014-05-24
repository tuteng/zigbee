/**************************************************************************************************
  Filename:       znp_app.c
  Revised:        $Date: 2010-01-17 08:58:03 -0800 (Sun, 17 Jan 2010) $
  Revision:       $Revision: 21533 $

  Description:    This file is the Application implementation for the ZNP.


  Copyright 2009-2010 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_board_cfg.h"
#include "mac_radio_defs.h"
#include "MT.h"
#include "MT_AF.h"
#include "MT_SYS.h"
#include "MT_ZDO.h"
#include "MT_UART.h"
#include "MT_UTIL.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#if defined POWER_SAVING
#include "OSAL_PwrMgr.h"
#endif        
#include "ZComDef.h"
#include "znp_app.h"
#include "znp_spi.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#define ZNP_UART_PORT                      0
#define ZNP_UART_BAUD                      HAL_UART_BR_115200

#if defined CC2531ZNP
#define ZNP_TX_MAX                         255
#else
#define ZNP_TX_MAX                         HAL_UART_DMA_TX_MAX
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros 
 * ------------------------------------------------------------------------------------------------
 */

#define MAC_RADIO_TX_ON()     st( RFST = ISTXON;   )

#define MOD_IF             4     // ~Modulation bit of MDMTEST1 register.
#define TX_PWR_MOD__SET(MOD_) st ( \
  if ((MOD_)) \
  { \
    MDMTEST1 |= BV(MOD_IF); \
  } \
  else \
  { \
    MDMTEST1 &= ~BV(MOD_IF); \
  } \
);

#define TX_PWR_TONE_SET(TONE) st ( \
  MDMTEST0 &= ~0xF0; \
  MDMTEST0 |= (TONE << 4); \
)

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void npInitNV(void);

static void npUartCback(uint8 port, uint8 event);
static void npUartTxReady(void);
static uint8* npMtUartAlloc(uint8 cmd0, uint8 len);
static void npMtUartSend(uint8 *pBuf);

#if !defined CC2531ZNP
static uint8* npMtSpiAlloc(uint8 cmd0, uint8 len);
static void npMtSpiSend(uint8 *pBuf);
uint8* npSpiPollCallback(void);
bool npSpiReadyCallback(void);
#endif

#if ZNP_RUN_CRC
static void znpCrc(void);
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static osal_msg_q_t npTxQueue;

#if ZNP_RUN_CRC
__root __code const unsigned char CRC[4] @ 0x3FFF4 = {
  0xFF,
  0xFF,
  0x5F,
  0xD9
};
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 znpTaskId;
uint8 znpCfg1;
#if ZNP_ZCL
extern uint8 zcl_TaskID;
extern void zclProcessMessageMSG(afIncomingMSGPacket_t *pkt);
#endif        

/**************************************************************************************************
 * @fn          znpInit
 *
 * @brief       This function is the OSAL task initialization callback.
 *
 * input parameters
 *
 * @param taskId - The task ID assigned to this task by the OSAL.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void znpInit(uint8 taskId)
{
#if ZNP_RUN_CRC
  znpCrc();
#endif

  if (ZNP_CFG1_UART == znpCfg1)
  {
    halUARTCfg_t uartConfig;

    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = ZNP_UART_BAUD;
    uartConfig.flowControl          = TRUE;
    uartConfig.flowControlThreshold = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.rx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.tx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.idleTimeout          = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.intEnable            = TRUE;
    uartConfig.callBackFunc         = npUartCback;
    HalUARTOpen(ZNP_UART_PORT, &uartConfig);
    MT_UartRegisterTaskID(taskId);
  }
  else
  {
    //npSpiInit() is called by hal_spi.c: HalSpiInit().
  }

  znpTaskId = taskId;
  MT_Init(taskId);
  npInitNV();

#if ZNP_ZCL
  zcl_TaskID = taskId;
#endif        
#if defined CC2531ZNP
  (void)osal_pwrmgr_task_state(znpTaskId, PWRMGR_HOLD);
#endif
}

/**************************************************************************************************
 * @fn          znpEventLoop
 *
 * @brief       This function processes the OSAL events and messages for the application.
 *
 * input parameters
 *
 * @param taskId - The task ID assigned to this application by OSAL at system initialization.
 * @param events - A bit mask of the pending event(s).
 *
 * output parameters
 *
 * None.
 *
 * @return      The events bit map received via parameter with the bits cleared which correspond to
 *              the event(s) that were processed on this invocation.
 **************************************************************************************************
 */
uint16 znpEventLoop(uint8 taskId, uint16 events)
{
  osal_event_hdr_t  *pMsg;
#if !defined CC2531ZNP
  uint8 *pBuf;
#endif

  if (events & SYS_EVENT_MSG)
  {
    if ((pMsg = (osal_event_hdr_t *) osal_msg_receive(znpTaskId)) != NULL)
    {
      switch (pMsg->event)
      {
      /* incoming message from UART transport */
      case CMD_SERIAL_MSG:
        MT_ProcessIncoming(((mtOSALSerialData_t *)pMsg)->msg);
        break;

#if ZNP_ZCL
#if defined (MT_UTIL_FUNC)
      case ZCL_KEY_ESTABLISH_IND:
        MT_UtilKeyEstablishInd((keyEstablishmentInd_t *)pMsg);
        break;
#endif        
#endif        

      case AF_INCOMING_MSG_CMD:
#if ZNP_ZCL
        if ((ZCL_KEY_ESTABLISHMENT_ENDPOINT == (((afIncomingMSGPacket_t *)pMsg)->endPoint)) ||
            (ZCL_KEY_ESTABLISHMENT_ENDPOINT == (((afIncomingMSGPacket_t *)pMsg)->srcAddr.endPoint)))
        {
          zclProcessMessageMSG((afIncomingMSGPacket_t *)pMsg);
        }
        else
#endif        
        {
          MT_AfIncomingMsg((afIncomingMSGPacket_t *)pMsg);
        }
        break;

#ifdef MT_ZDO_FUNC
      case ZDO_STATE_CHANGE:
        MT_ZdoStateChangeCB(pMsg);
        break;

      case ZDO_CB_MSG:
        MT_ZdoSendMsgCB((zdoIncomingMsg_t *)pMsg);
        break;
#endif

      case AF_DATA_CONFIRM_CMD:
        MT_AfDataConfirm((afDataConfirm_t *)pMsg);
        break;

      default:
        break;
      }

      osal_msg_deallocate((byte *)pMsg);
    }

    events ^= SYS_EVENT_MSG;
  }
#if !defined CC2531ZNP
  else if (events & ZNP_SPI_RX_AREQ_EVENT)
  {
    if ((pBuf = npSpiGetReqBuf()) != NULL )
    {
      MT_ProcessIncoming(pBuf);
      npSpiAReqComplete();
    }

    events ^= ZNP_SPI_RX_AREQ_EVENT;
  }
  else if (events & ZNP_SPI_RX_SREQ_EVENT)
  {
    if ((pBuf = npSpiGetReqBuf()) != NULL)
    {
      MT_ProcessIncoming(pBuf);
    }

    events ^= ZNP_SPI_RX_SREQ_EVENT;
  }
#endif
  else if (events & ZNP_UART_TX_READY_EVENT)
  {
    npUartTxReady();
    events ^= ZNP_UART_TX_READY_EVENT;
  }
#if defined MT_SYS_FUNC
  else if (events & MT_SYS_OSAL_EVENT_0)
  {
    MT_SysOsalTimerExpired(0x00);
    events ^= MT_SYS_OSAL_EVENT_0;
  }
  else if (events & MT_SYS_OSAL_EVENT_1)
  {
    MT_SysOsalTimerExpired(0x01);
    events ^= MT_SYS_OSAL_EVENT_1;
  }
  else if (events & MT_SYS_OSAL_EVENT_2)
  {
    MT_SysOsalTimerExpired(0x02);
    events ^= MT_SYS_OSAL_EVENT_2;
  }
  else if (events & MT_SYS_OSAL_EVENT_3)
  {
    MT_SysOsalTimerExpired(0x03);
    events ^= MT_SYS_OSAL_EVENT_3;
  }
#endif
#if defined POWER_SAVING
  else if (events & ZNP_PWRMGR_CONSERVE_EVENT)
  {
    (void)osal_pwrmgr_task_state(znpTaskId, PWRMGR_CONSERVE);
    events ^= ZNP_PWRMGR_CONSERVE_EVENT;
  }
#endif
#if !defined CC2531ZNP
  else if (events & ZNP_SPI_TIMER_EVENT)
  {
    if (ZNP_CFG1_SPI == znpCfg1)
    {
      npSpiTimer();
    }
    events ^= ZNP_SPI_TIMER_EVENT;
  }
#endif
  else
  {
    events = 0;  /* Discard unknown events. */
  }

  return ( events );
}

/**************************************************************************************************
 * @fn          znpTestRF
 *
 * @brief       This function initializes and checks the ZNP RF Test Mode NV items. It is designed
 *              to be invoked before/instead of MAC radio initialization.
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
void znpTestRF(void)
{
  uint8 rfTestParms[4] = { 0, 0, 0, 0 };

  if ((SUCCESS != osal_nv_item_init(ZNP_NV_RF_TEST_PARMS, 4, rfTestParms))  ||
      (SUCCESS != osal_nv_read(ZNP_NV_RF_TEST_PARMS, 0, 4, rfTestParms)) ||
      (rfTestParms[0] == 0))
  {
    return;
  }
      
  // Settings from SmartRF Studio
  MDMCTRL0 = 0x85;
  RXCTRL = 0x3F;
  FSCTRL = 0x5A;
  FSCAL1 = 0x2B;
  AGCCTRL1 = 0x11;
  ADCTEST0 = 0x10;
  ADCTEST1 = 0x0E;
  ADCTEST2 = 0x03;

  FRMCTRL0 = 0x43;
  FRMCTRL1 = 0x00;

  MAC_RADIO_RXTX_OFF();
  MAC_RADIO_SET_CHANNEL(rfTestParms[1]);
  MAC_RADIO_SET_TX_POWER(rfTestParms[2]);
  TX_PWR_TONE_SET(rfTestParms[3]);

  switch (rfTestParms[0])
  {
  case 1:  // Rx promiscuous mode.
    MAC_RADIO_RX_ON();
    break;

  case 2:  // Un-modulated Tx.
    TX_PWR_MOD__SET(1);
    // no break;

  case 3:  // Modulated Tx.
    // Modulated is default register setting, so no special action.

    // Now turn on Tx power for either mod or un-modulated Tx test.
    MAC_RADIO_TX_ON();
    break;

  default:  // Not expected.
    break;
  }

  // Clear the RF test mode.
  (void)osal_memset(rfTestParms, 0, 4);
  (void)osal_nv_write(ZNP_NV_RF_TEST_PARMS, 0, 4, rfTestParms);

  while (1);  // Spin in RF test mode until a hard reset.
}
 
/**************************************************************************************************
 * @fn          MT_TransportAlloc
 *
 * @brief       This function is the definition of the physical transport API for allocation a msg.
 *
 * input parameters
 *
 * @param cmd0 - The RPC command byte 0.
 * @param len - The RPC data length.
 *
 * output parameters
 *
 * @param uint8 * - Pointer to the buffer to use build and send the RPC message.
 *
 * @return      None.
 **************************************************************************************************
 */
uint8 *MT_TransportAlloc(uint8 cmd0, uint8 len)
{
#if !defined CC2531ZNP
  if (ZNP_CFG1_UART == znpCfg1)
#endif
  {
    return npMtUartAlloc(cmd0, len);
  }
#if !defined CC2531ZNP
  else
  {
    return npMtSpiAlloc(cmd0, len);
  }
#endif
}
 
/**************************************************************************************************
 * @fn          MT_TransportSend
 *
 * @brief       This function is the definition of the physical transport API for sending a message.
 *
 * input parameters
 *
 * @param pBuf - Pointer to the buffer created with MT_TransportAlloc.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MT_TransportSend(uint8 *pBuf)
{
#if !defined CC2531ZNP
  if (ZNP_CFG1_UART == znpCfg1)
#endif
  {
    npMtUartSend(pBuf);
  }
#if !defined CC2531ZNP
  else
  {
    npMtSpiSend(pBuf);
  }
#endif
}

/**************************************************************************************************
 * @fn         npInitNV
 *
 * @brief
 *
 * input parameters
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npInitNV(void)
{
  /* 4 x 2 bytes ZNP_NV_APP_ITEM_X */
  osal_nv_item_init(ZNP_NV_APP_ITEM_1, 2, NULL);
  osal_nv_item_init(ZNP_NV_APP_ITEM_2, 2, NULL);
  osal_nv_item_init(ZNP_NV_APP_ITEM_3, 2, NULL);
  osal_nv_item_init(ZNP_NV_APP_ITEM_4, 2, NULL);

  /* 2 x 16 bytes ZNP_NV_APP_ITEM_X */
  osal_nv_item_init(ZNP_NV_APP_ITEM_5, 16, NULL);
  osal_nv_item_init(ZNP_NV_APP_ITEM_6, 16, NULL);
}
  
/**************************************************************************************************
 * @fn          npUartCback
 *
 * @brief       This function is the UART callback processor.
 *
 * input parameters
 *
 * @param port - The port being used for UART.
 * @param event - The reason for the callback.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npUartCback(uint8 port, uint8 event)
{
  switch (event) {
  case HAL_UART_RX_FULL:
  case HAL_UART_RX_ABOUT_FULL:
  case HAL_UART_RX_TIMEOUT:
    MT_UartProcessZToolData(port, znpTaskId);
    break;

  case HAL_UART_TX_EMPTY:
    osal_set_event(znpTaskId, ZNP_UART_TX_READY_EVENT);
    break;

  default:
    break;
  }
}

/**************************************************************************************************
 * @fn          npUartTxReady
 *
 * @brief       This function gets and writes the next chunk of data to the UART.
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
static void npUartTxReady(void)
{
  static uint16 npUartTxCnt = 0;
  static uint8 *npUartTxMsg = NULL;
  static uint8 *pMsg = NULL;

  if (!npUartTxMsg)
  {
    if ((pMsg = npUartTxMsg = osal_msg_dequeue(&npTxQueue)))
    {
      /* | SOP | Data Length | CMD |  DATA   | FSC |
       * |  1  |     1       |  2  | as dLen |  1  |
       */
      npUartTxCnt = pMsg[1] + MT_UART_FRAME_OVHD + MT_RPC_FRAME_HDR_SZ;
    }
  }

  if (npUartTxMsg)
  {
    uint16 len = MIN(ZNP_TX_MAX, npUartTxCnt);

    len = HalUARTWrite(ZNP_UART_PORT, pMsg, len);
    npUartTxCnt -= len;
    ZNP_RDY(TRUE);  // Signal to Master that Tx is pending - sleep not ok.

    if (npUartTxCnt == 0)
    {
      osal_msg_deallocate(npUartTxMsg);
      npUartTxMsg = NULL;
    }
    else
    {
      pMsg += len;
    }
  }
}

/**************************************************************************************************
 * @fn          npMtUartAlloc
 *
 * @brief       This function allocates a buffer for Txing on UART.
 *
 * input parameters
 *
 * @param cmd0 - The first byte of the MT command id containing the command type and subsystem.
 * @param len - Data length required.
 *
 * output parameters
 *
 * None.
 *
 * @return      Pointer to the buffer obtained; possibly NULL if an allocation failed.
 **************************************************************************************************
 */
static uint8* npMtUartAlloc(uint8 cmd0, uint8 len)
{
  uint8 *p;

  if ((p = osal_msg_allocate(len + MT_RPC_FRAME_HDR_SZ + MT_UART_FRAME_OVHD)) != NULL)
  {
    return p + 1;
  }

  return NULL;
}

/**************************************************************************************************
 * @fn          npMtUartSend
 *
 * @brief       This function transmits or enqueues the buffer for transmitting on UART.
 *
 * input parameters
 *
 * @param pBuf - Pointer to the buffer to transmit on the UART.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npMtUartSend(uint8 *pBuf)
{
  uint8 len = pBuf[0] + MT_RPC_FRAME_HDR_SZ;

  pBuf[len] = MT_UartCalcFCS(pBuf, len);
  pBuf--;
  pBuf[0] = MT_UART_SOF;

  osal_msg_enqueue(&npTxQueue, pBuf);
  osal_set_event(znpTaskId, ZNP_UART_TX_READY_EVENT);
}

#if !defined CC2531ZNP
/**************************************************************************************************
 * @fn          npMtSpiAlloc
 *
 * @brief       This function gets or allocates a buffer for Txing on SPI.
 *
 * input parameters
 *
 * @param cmd0 - The first byte of the MT command id containing the command type and subsystem.
 * @param len - Data length required.
 *
 * output parameters
 *
 * None.
 *
 * @return      Pointer to the buffer obtained; possibly NULL if an allocation failed.
 **************************************************************************************************
 */
static uint8* npMtSpiAlloc(uint8 cmd0, uint8 len)
{
  if ((cmd0 & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
  {
    return npSpiSRspAlloc(len);
  }
  else
  {
    return npSpiAReqAlloc(len);
  }
}

/**************************************************************************************************
 * @fn          npMtSpiSend
 *
 * @brief       This function transmits or enqueues the buffer for transmitting on SPI.
 *
 * input parameters
 *
 * @param pBuf - Pointer to the buffer to transmit on the SPI.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npMtSpiSend(uint8 *pBuf)
{
  if ((pBuf[1] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
  {
    npSpiSRspReady(pBuf);
  }
  else
  {
    osal_msg_enqueue(&npTxQueue, pBuf);
    npSpiAReqReady();
  }
}

/**************************************************************************************************
 * @fn          npSpiPollCallback
 *
 * @brief       This function is called by the SPI driver when a POLL frame is received.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      A pointer to an OSAL message buffer containing the next AREQ frame to transmit,
 *              if any; NULL otherwise.
 **************************************************************************************************
 */
uint8* npSpiPollCallback(void)
{
  return osal_msg_dequeue(&npTxQueue);
}

/**************************************************************************************************
 * @fn          npSpiReadyCallback
 *
 * @brief       This function is called by the SPI driver to check if any data is ready to send.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if data is ready to send; FALSE otherwise.
 **************************************************************************************************
 */
bool npSpiReadyCallback(void)
{
  return !OSAL_MSG_Q_EMPTY(&npTxQueue);
}

/**************************************************************************************************
 * @fn          port0Isr
 *
 * @brief       This function handles the PORT0 interrupt.
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
HAL_ISR_FUNCTION(port0Isr, P0INT_VECTOR)
{
  // Knowing which pin requires a #define from _hal_uart_dma.c
  //if (P0IFG & NP_RDYIn_BIT)
  {
    if (ZNP_CFG1_UART == znpCfg1)
    {
      osal_set_event(znpTaskId, ZNP_UART_TX_READY_EVENT);
    }
    else
    {
      npSpiMrdyIsr();
    }
  }

  P0IFG = 0;
  P0IF = 0;
}
#endif

#if ZNP_RUN_CRC
/*********************************************************************
 * @fn      runPoly
 *
 * @brief   Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * @param   crc - Running CRC calculated so far.
 * @param   val - Value on which to run the CRC16.
 *
 * @return  crc - Updated for the run.
 */
static uint16 runPoly(uint16 crc, uint8 val)
{
  const uint16 poly = 0x1021;
  uint8 cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8 msb = (crc & 0x8000) ? 1 : 0;

    crc <<= 1;
    if (val & 0x80)  crc |= 0x0001;
    if (msb)         crc ^= poly;
  }

  return crc;
}

/**************************************************************************************************
 * @fn          znpCrc
 *
 * @brief       This function handles the PORT0 interrupt.
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
static void znpCrc(void)
{
  preamble_t preamble;
  uint32 oset;
  uint16 crc = 0, crc2;

  HalOADRead(dlImagePreambleOffset, (uint8 *)&preamble, sizeof(preamble_t), HAL_OAD_DL);

  // Run the CRC calculation over the downloaded image.
  for (oset = 0; oset < preamble.len; oset++)
  {
    if ((oset < HAL_OAD_CRC_OSET) || (oset >= HAL_OAD_CRC_OSET+4))
    {
      uint8 buf;
      HalOADRead(oset, &buf, 1, HAL_OAD_DL);
      crc = runPoly(crc, buf);
    }
  }

  // IAR note explains that poly must be run with value zero for each byte of crc.
  crc = runPoly(crc, 0);
  crc = runPoly(crc, 0);

  HalOADRead(HAL_OAD_CRC_OSET, (uint8 *)&crc2, sizeof(crc2), HAL_OAD_DL);
  return (crc2 == crc) ? SUCCESS : FAILURE;
}
#endif

/**************************************************************************************************
*/
