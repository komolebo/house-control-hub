/*
 * network_discovery.c
 *
 *  Created on: 1 трав. 2020 р.
 *      Author: Oleh
 */

#include <network/net_discovery.h>
#include <network/net_info.h>
#include <network/net_request.h>
#include <uartlog/UartLog.h>
#include <string.h>

#include "profiles_if.h"
#include "util.h"
#include "ipc/msg_handler.h"
#include "central.h"


uint8_t discState = BLE_DISC_STATE_IDLE;



static uint16_t connHandle = CONNHANDLE_INVALID;


/*********************************************************************
 * @fn      Central_exchangeMtuSize
 *
 * @brief   Start device services discovery.
 *
 * @return  none
 */
void NetDisc_exchangeMtuSize(uint16_t conn_handle)
{
    attExchangeMTUReq_t req;

    discState = BLE_DISC_STATE_MTU;

    connHandle = conn_handle;

    // Discover GATT Server's Rx MTU size
    req.clientRxMTU = netMaxPduSize - L2CAP_HDR_SIZE;

    // ATT MTU size should be set to the minimum of the Client Rx MTU
    // and Server Rx MTU values
    VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);

    Log_info1("Exchanging MTU with 0x%x", connHandle);
}



/*********************************************************************
 * @fn      Central_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
void NetDisc_processGATTEvent(gattMsgEvent_t *pMsg)
{
    static uint8_t ipcRespBuff[CHARS_PER_DEVICE * ATT_BT_UUID_SIZE];

    if (discState == BLE_DISC_STATE_MTU)
    {
        // MTU size response received, discover services
        if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
        {
            discState = BLE_DISC_STATE_SVC;

            /* discover all device's services */
            GATT_DiscAllPrimaryServices(connHandle, selfEntity);
        }
    }
    else if (discState == BLE_DISC_STATE_SVC)
    {
        // Services found, store handles
        uint8_t svcNum = ((attReadByGrpTypeRsp_t *)&pMsg->msg)->numGrps;

        Log_info2("%s: services discovered: %u", (uintptr_t )__func__, svcNum);

        // If procedure complete
        if (((pMsg->method == ATT_READ_BY_GRP_TYPE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            discState = BLE_DISC_STATE_UUID;

            GATT_DiscAllCharDescs(connHandle, 0x001, 0xFFFF, selfEntity);
        }
    }
    else if (discState == BLE_DISC_STATE_UUID)
    {
        if (pMsg->method == ATT_FIND_INFO_RSP)
        {
            // if 2 byte UUID format received
            attFindInfoRsp_t* rsp = (attFindInfoRsp_t *)&pMsg->msg.findInfoRsp;

            if (rsp->format == ATT_HANDLE_BT_UUID_TYPE)
            {
                for (uint8_t i = 0, *p = rsp->pInfo; i < rsp->numInfo;
                        i++, p += sizeof(charInfo_t))
                {
                    uint16_t charHandle = BUILD_UINT16(p[0], p[1]);
                    uint16_t charValue = BUILD_UINT16(p[2], p[3]);

                    // Save only data UUIDs
                    if (charValue != GATT_PRIMARY_SERVICE_UUID
                            && charValue != GATT_SECONDARY_SERVICE_UUID
                            && charValue != GATT_INCLUDE_UUID
                            && charValue != GATT_CHARACTER_UUID
                            && charValue != GATT_CHAR_USER_DESC_UUID
                            && charValue != GATT_CLIENT_CHAR_CFG_UUID)
                    {
                        Log_info2("%s: discovered UUID: 0x%x",
                                  (uintptr_t )__func__, charValue);

                        uint8_t connIndex = NetInfo_addCharHandle(
                                pMsg->connHandle, (uint8_t *) &charValue,
                                charHandle);

                        if (connIndex == CHARS_PER_DEVICE)
                        {
                            Log_error2("%s: Not enough memory for char handle 0x%x",
                                    (uintptr_t )__func__, charValue);
                        }
                    }
                }
            }
        }

        // If procedure complete
        if (((pMsg->method == ATT_FIND_INFO_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            Log_info1("%s: Discovery done", (uintptr_t )__func__);

            // Send IPC report to the back
            uint8_t bytes_populated = NetInfo_populateUuidIpcResp(
                    pMsg->connHandle, &ipcRespBuff[0],
                    CHARS_PER_DEVICE * ATT_BT_UUID_SIZE);

            send_central_ipc_msg_resp(CENTRAL_MSG_DISCOVER_DEVICE_UUIDS,
                                      bytes_populated,
                                      (uint8_t *)&ipcRespBuff);

            // Discovery done
            discState = BLE_DISC_STATE_IDLE;
            connHandle = CONNHANDLE_INVALID;
        }
    }
}
