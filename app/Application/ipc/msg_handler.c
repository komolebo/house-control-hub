/*
 * uart_adapter.c
 *
 *  Created on: 23 квіт. 2020 р.
 *      Author: Oleh
 */

/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <uartlog/UartLog.h>

#include "msg_handler.h"
#include "uart_handler.h"
#include "util.h"
#include "profiles_if.h"
#include "central.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

#define TX_BUFFER_LEN               (2 * IPC_MAX_MSG_SIZE + 1)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
*  LOCAL VARIABLES
*/
// buffer for transmitting into IPC channel
static serialProtoPkg_t txMessage;

/*********************************************************************
 * FUNCTIONS
 */
void process_ipc_msg(const uint8_t *data, const uint16_t len)
{
//    char (*__kaboom)[sizeof(serialProtoPkg_t)] = 1;

    serialProtoPkg_t *pSerialProtoPkg;
    uint8_t packageType;

    if ((data == NULL) || (len < sizeof(pkgHeader_t)))
    {
        return;
    }

    // Select data from raw IPC package
    pSerialProtoPkg = (serialProtoPkg_t *) data;

    // Parse package type
    packageType = pSerialProtoPkg->header.package_type;

    /* Message is sent for central device */
    if (packageType == PKG_CENTRAL_REQ)
    {
        if (len >= (sizeof(pkgHeader_t) + sizeof(pkgDataCentral_t)))
        {
            pkgDataCentral_t *pData = ICall_malloc(sizeof(pkgDataCentral_t));

            memcpy((uint8_t *) pData,
                   (uint8_t *) &pSerialProtoPkg->data.central_msg_data,
                   sizeof(pkgDataCentral_t));

            if (Util_enqueueAppMsg(EVT_IPC_CENTRAL_CMD, SUCCESS,
                                   (uint8_t *) pData) != SUCCESS)
            {
                ICall_free(pData);
            }
        }
        else
        {
            /* Data length is not matching header definitions */
        }
    }
    /* Message is sent for peripheral device */
    else if (packageType == PKG_PERIPHERY_REQ)
    {
        if (len >= (sizeof(pkgHeader_t) + sizeof(pkgDataPeriphery_t)))
        {
            pkgDataPeriphery_t *pData = ICall_malloc(
                    sizeof(pkgDataPeriphery_t));

            memcpy((uint8_t *) pData,
                   (uint8_t *) &pSerialProtoPkg->data.peripheral_msg_data,
                   sizeof(pkgDataPeriphery_t));

            if (Util_enqueueAppMsg(EVT_IPC_MSG_PERIPHERAL, SUCCESS,
                                   (uint8_t *) pData) != SUCCESS)
            {
                ICall_free(pData);
            }
        }
        else
        {
            /* Data length is not matching header definitions */
        }
    }
}

bool send_ipc_msg(uint8_t *data, uint16_t len)
{
    static uint8_t buffer[TX_BUFFER_LEN];

    if (len <= IPC_MAX_MSG_SIZE)
    {
        if (Util_convertHex2Str(data, &buffer[0], len, TX_BUFFER_LEN))
        {
            Log_warning1("IPC TX: %s", (uintptr_t)buffer);
        }
        else
        {
            Log_error1("%s: hex data is too big to send in ASCII",
                       (uintptr_t)__func__);
        }
    }
    else
    {
        Log_error0("IPC message is too big to transmit");
        return FALSE;
    }

    return TRUE;
}

bool send_central_ipc_msg_resp(msgCentral_t msg,
                               uint16_t len,
                               uint8_t *data)
{
    if(len > IPC_MSG_MAX_DATA_SIZE) { return FALSE; }

    memset(&txMessage, 0, IPC_MAX_MSG_SIZE);

    txMessage.header.package_type = PKG_CENTRAL_RESP;
    txMessage.data.central_msg_data.msg = msg;
    txMessage.data.central_msg_data.len = len;
    memcpy(txMessage.data.central_msg_data.data, data, len);

    send_ipc_msg((uint8_t *)&txMessage, sizeof(txMessage));

    return TRUE;
}


bool send_peripheral_ipc_msg(pkgType_t type,
                             msgPeriphery_t msg,
                             uint16_t conn_mask,
                             uint8_t uuid[UUID_DATA_LEN],
                             uint16_t len,
                             uint8_t *data)
{
    if(len > IPC_MSG_MAX_DATA_SIZE) { return FALSE; }

    memset(&txMessage, 0, IPC_MAX_MSG_SIZE);

    txMessage.header.package_type = type;
    txMessage.data.peripheral_msg_data.msg = msg;
    txMessage.data.peripheral_msg_data.connHandle = conn_mask;
    txMessage.data.peripheral_msg_data.len = len;
    memcpy(txMessage.data.peripheral_msg_data.data, data, len);
    memcpy(txMessage.data.peripheral_msg_data.uuid, uuid, UUID_DATA_LEN);

    send_ipc_msg((uint8_t *)&txMessage, sizeof(txMessage));

    return TRUE;
}
