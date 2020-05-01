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
#include <string.h>

#include "util.h"
#include "network_request.h"
#include "network_info.h"
#include "profiles_if.h"
#include "ipc/uart_handler.h"
#include "central.h"


// Currently requested connection handle
static uint16_t connHandle = CONNHANDLE_INVALID;

// Currently requested characteristic handle
static uint16_t charHandle = GATT_INVALID_HANDLE;

// Currently requested characteristic value
static uint16_t charValue = 0xFFFF;


/*********************************************************************
 * @fn      NetReq_processGATTExtEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
void NetReq_processGATTExtEvent(gattMsgEvent_t *pMsg)
{
    if (pMsg->method == ATT_READ_BY_TYPE_RSP)
    {
        if (charValue)
        {
            Log_error2("%s: Unexpected read gatt response received for method: %d",
                       (uintptr_t)__func__, pMsg->method);
        }

        attReadByTypeRsp_t* resp = &pMsg->msg.readByTypeRsp;
//        uint16_t attr_len = pMsg->msg.readByTypeRsp.len;
        static uint16_t data = 0;
        uint16_t len = resp->len;

        if (len)
        {
            memcpy((uint8_t *)&data, resp->pDataList + 2, len - 2);
        }

        Log_info4("%s: GattRead response - dataLen: %d, len: %d, data: 0x%x",
                  (uintptr_t )__func__, len, len, data);

        // If procedure complete
        if (((pMsg->method == ATT_READ_BY_TYPE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))/*
                || (pMsg->method == ATT_ERROR_RSP)*/)
        {
            send_peripheral_ipc_msg(PKG_PERIPHERY_RESP, PERIPHERY_MSG_READ,
                                    pMsg->connHandle, (uint8_t *) &charValue,
                                    sizeof(data), (uint8_t *)&data);
            charValue = 0;
        }
    }
    else if (pMsg->method == ATT_WRITE_RSP)
    {
        send_peripheral_ipc_msg(PKG_PERIPHERY_RESP, PERIPHERY_MSG_WRITE,
                                pMsg->connHandle, (uint8_t*) &charValue,
                                0, NULL);
        charValue = 0;
    }
}


bStatus_t NetReq_processIpcReadPeripheral(pkgDataPeriphery_t *ipcMsg)
{
    bStatus_t ret_status = FAILURE;
    static attAttrType_t attr_type = {.len = UUID_DATA_LEN };

    // change globals
    connHandle = ipcMsg->connHandle;
    charValue = *(uint16_t *)ipcMsg->uuid;

    // TODO: endians
    memcpy(attr_type.uuid, ipcMsg->uuid, sizeof(charValue));

    attReadByTypeReq_t readReq = { 0x001, 0xFFFF, attr_type };


    if (GATT_ReadUsingCharUUID(connHandle, (attReadByTypeReq_t* )&readReq,
                               selfEntity) != SUCCESS)
    {
        Log_error2("%s: peripheral read request for [0x%x] failed to perform",
                   (uintptr_t )__func__, charValue);
    }
    else
    {
        ret_status = SUCCESS;
    }

    return ret_status;
}


bStatus_t NetReq_processIpcWritePeripheral(pkgDataPeriphery_t *ipcMsg)
{
    bStatus_t ret_status = FAILURE;
    attWriteReq_t writeReq;

    // change globals
    connHandle = ipcMsg->connHandle;
    charValue = *(uint16_t *)ipcMsg->uuid;
    charHandle = NetInfo_getCharHandle(connHandle, ipcMsg->uuid);

    if (charHandle == CONNHANDLE_INVALID)
    {
        Log_error2("%s: handle doesn't exist for 0x%x", (uintptr_t )__func__,
                   charValue);
    }
    else
    {
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

            Log_info4("%s: writing UUID: 0x%x, len: %d, value: 0x%x",
                      (uintptr_t )__func__, charValue, writeReq.len,
                      *writeReq.pValue);

            if (GATT_WriteCharValue(connHandle, &writeReq,
                    selfEntity) != SUCCESS)
            {
                Log_error2("%s: failed write request for [0x%x]",
                           (uintptr_t )__func__, charHandle);
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
