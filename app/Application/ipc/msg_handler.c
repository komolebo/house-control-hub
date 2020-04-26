/*
 * uart_adapter.c
 *
 *  Created on: 23 квіт. 2020 р.
 *      Author: Oleh
 */

/*********************************************************************
 * INCLUDES
 */
#include "msg_handler.h"
#include "central.h"

#include <bcomdef.h>
#include "uart_handler.h"
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * FUNCTIONS
 */
void process_rx_ipc_msg(uint8_t *data, size_t len)
{
    serialProtoMsg_t *pSerialProtoMsg;
//    char (*__kaboom)[sizeof(serialProtoMsg_t)] = 1;

    if ((data != NULL) && (len >= sizeof(packageHeader_t)))
    {
        pSerialProtoMsg = (serialProtoMsg_t *)data;

        /* Message is sent for central device */
        uint16_t packType = pSerialProtoMsg->header.package_type;
        if (pSerialProtoMsg->header.package_type == PACKAGE_CMD)
        {
            if (len >= (sizeof(packageHeader_t) + sizeof(msgCentral_t)))
            {
                msgCentral_t *pData = ICall_malloc(sizeof(msgCentral_t));
                uint16_t event = pSerialProtoMsg->data.central_msg_data.cmd;

                memcpy((uint8_t *) pData,
                       (uint8_t *) &pSerialProtoMsg->data.central_msg_data,
                       sizeof(msgCentral_t));

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
        else if (pSerialProtoMsg->header.package_type < PACKAGE_TYPE_COUNT)
        {
            if (len >= (sizeof(packageHeader_t) + sizeof(msgPeripheral_t)))
            {
                msgPeripheral_t *pData = ICall_malloc(sizeof(msgPeripheral_t));

                memcpy((uint8_t *) pData,
                       (uint8_t *) &pSerialProtoMsg->data.peripheral_msg_data,
                       sizeof(msgPeripheral_t));

                if (Util_enqueueAppMsg(EVT_DEVICE_GATT_REQ, SUCCESS,
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
    else
    {
        /* incorrect IPC package header provided */
    }
}

