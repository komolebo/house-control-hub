/*
 * network_request.c
 *
 *  Created on: 1 трав. 2020 р.
 *      Author: Oleh
 */

#include "icall_ble_api.h"
#include <icall.h>
#include <uartlog/UartLog.h>
#include <ble_user_config.h>
#include <network/net_info.h>
#include <network/net_request.h>
#include <string.h>

#include "util.h"
#include "profiles_if.h"
#include "ipc/uart_handler.h"
#include "central.h"


uint16_t netMaxPduSize; // TODO: move to needed module


// Currently requested connection handle
static uint16_t connHandle = CONNHANDLE_INVALID;

// Currently requested characteristic handle
static uint16_t charHandle = GATT_INVALID_HANDLE;

// Currently requested characteristic value
static uint16_t charValue = 0xFFFF;

static bStatus_t NetReq_processIpcReadPeripheral(pkgDataPeriphery_t *ipcMsg,
                                                 bool readCfg);
static bStatus_t NetReq_processIpcWritePeripheral(pkgDataPeriphery_t *ipcMsg,
                                                  bool writeCfg);


/*********************************************************************
 * @fn      NetReq_processGATTUpdatesEvent
 *
 * @brief   Process GATT updates event, notification or indication
 *
 * @return  none
 */
void NetReq_processGATTUpdatesEvent(gattMsgEvent_t *pMsg)
{
    Log_info2("%s: GattRead response - method: 0x%x",
                      (uintptr_t )__func__, pMsg->method);

    if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
        attHandleValueNoti_t *notifReq = (attHandleValueNoti_t *)pMsg;
        uint8_t charValue[UUID_DATA_LEN];

        memcpy(charValue,
               NetInfo_getCharValByHandle(pMsg->connHandle, notifReq->handle),
               sizeof(notifReq->handle));

        send_peripheral_ipc_msg(PKG_PERIPHERY_UPDATES,
                                PERIPHERY_MSG_NOTIFY,
                                pMsg->connHandle,
                                (uint8_t *) &charValue,
                                notifReq->len, notifReq->pValue);
    }
    else if (pMsg->method == ATT_HANDLE_VALUE_IND)
    {

    }
}

void NetReq_processGattReadResp(gattMsgEvent_t *pMsg)
{

}


/*********************************************************************
 * @fn      NetReq_processGATTExtEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
void NetReq_processGATTExtEvent(gattMsgEvent_t *pMsg)
{
#if 0
#define READ_RESP_MAX_BUF   (40)
    static uint8_t readBuf[READ_RESP_MAX_BUF];
#endif
    if (pMsg->method == ATT_READ_BY_TYPE_RSP)
    {
        /*if (charValue)
        {
            Log_error2("%s: Unexpected read gatt response received for method: %d",
                       (uintptr_t)__func__, pMsg->method);
        }*/

        attReadByTypeRsp_t* resp = &pMsg->msg.readByTypeRsp;
        static uint32_t data = 0;
        uint16_t len = resp->len;

        if (len)
        {
            memcpy((uint8_t *)&data, resp->pDataList + 2, len - 2);
        }

        Log_info4("%s: GattRead response - for 0x%x, len: %d, data: 0x%x",
                  (uintptr_t )__func__,
                  charValue,
                  len,
                  data);

        // If procedure complete
        if (((pMsg->method == ATT_READ_BY_TYPE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))/*
                || (pMsg->method == ATT_ERROR_RSP)*/)
        {
            send_peripheral_ipc_msg(PKG_PERIPHERY_RESP, PERIPHERY_MSG_READ_VAL,
                                    pMsg->connHandle, (uint8_t *) &charValue,
                                    sizeof(data), (uint8_t *)&data);
            charValue = 0;
        }
    }
    else if (pMsg->method == ATT_WRITE_RSP)
    {
        send_peripheral_ipc_msg(PKG_PERIPHERY_RESP, PERIPHERY_MSG_WRITE_VAL,
                                pMsg->connHandle, (uint8_t*) &charValue,
                                0, NULL);
        charValue = 0;
    }
}

bStatus_t NetReq_processIpcReadCfgPeripheral(pkgDataPeriphery_t *ipcMsg)
{
    return NetReq_processIpcReadPeripheral(ipcMsg, TRUE);
}


bStatus_t NetReq_processIpcReadValPeripheral(pkgDataPeriphery_t *ipcMsg)
{
    return NetReq_processIpcReadPeripheral(ipcMsg, FALSE);
}

bStatus_t NetReq_processIpcWriteCfgPeripheral(pkgDataPeriphery_t *ipcMsg)
{
    return NetReq_processIpcWritePeripheral(ipcMsg, TRUE);
}


bStatus_t NetReq_processIpcWriteValPeripheral(pkgDataPeriphery_t *ipcMsg)
{
    return NetReq_processIpcWritePeripheral(ipcMsg, FALSE);
}


static bStatus_t NetReq_processIpcReadPeripheral(pkgDataPeriphery_t *ipcMsg,
                                                 bool readCfg)
{
    bStatus_t ret_status = FAILURE;

    // change globals
    connHandle = ipcMsg->connHandle;
    charValue = *(uint16_t *) ipcMsg->uuid;
    charHandle = NetInfo_getCharHandleByVal(connHandle, ipcMsg->uuid);

    if (charHandle == CONNHANDLE_INVALID)
    {
        Log_error2("%s: handle doesn't exist for 0x%x", (uintptr_t )__func__,
                   charValue);
    }
    else
    {
        // configuration char handle is next to value
        if (readCfg) { charHandle++; };

        attReadReq_t readReq = { .handle = charHandle };
        if (GATT_ReadCharValue(connHandle, &readReq, selfEntity) != SUCCESS)
        {
            Log_error3(
                    "%s: failed peripheral read %s request for [0x%x]",
                    (uintptr_t )__func__,
                    (uintptr_t )(readCfg ? "config" : "value"),
                    charValue);
        }
        else
        {
            ret_status = SUCCESS;
        }
    }

    return ret_status;
}

static bStatus_t NetReq_processIpcWritePeripheral(pkgDataPeriphery_t *ipcMsg,
                                                  bool writeCfg)
{
    bStatus_t ret_status = FAILURE;
    attWriteReq_t writeReq;

    // change globals
    connHandle = ipcMsg->connHandle;
    charValue = *(uint16_t *) ipcMsg->uuid;
    charHandle = NetInfo_getCharHandleByVal(connHandle, ipcMsg->uuid);

    if (charHandle == CONNHANDLE_INVALID)
    {
        Log_error2("%s: handle doesn't exist for 0x%x", (uintptr_t )__func__,
                   charValue);
    }
    else
    {
        // configuration char handle is next to value
        if (writeCfg) { charHandle++; }

        writeReq.handle = charHandle;
        writeReq.len = ipcMsg->len;
        writeReq.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, writeReq.len,
                                        NULL);
        if (writeReq.pValue == NULL)
        {
            Log_error1("%s: not enough memory in GATT platform",
                       (uintptr_t )__func__);
        }
        else
        {
            writeReq.sig = writeReq.cmd = 0;
            memcpy(writeReq.pValue, ipcMsg->data, writeReq.len);

            Log_info4("%s: writing %s UUID: 0x%x, len: %d",
                      (uintptr_t )__func__,
                      charValue,
                      writeReq.len,
                      (uintptr_t )(writeCfg ? "config" : "value"));

            if (GATT_WriteCharValue(connHandle, &writeReq,
                    selfEntity) != SUCCESS)
            {
                Log_error3("%s: failed %s write request for [0x%x] ",
                           (uintptr_t )__func__,
                           (uintptr_t )(writeCfg ? "config": "value"),
                           charHandle);
                GATT_bm_free((gattMsg_t *) &writeReq, ATT_WRITE_REQ);
            }
            else
            {
                ret_status = SUCCESS;
            }
        }
    }

    return ret_status;
}
