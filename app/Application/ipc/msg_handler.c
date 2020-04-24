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
    if ((data != NULL) && (len >= sizeof(packageHeader_t)))
    {
        serialProtoMsg_t *pSerialProtoMsg = (serialProtoMsg_t *)data;

        /* Message is sent for central device */
        if (pSerialProtoMsg->header.package_type == PACKAGE_CMD)
        {
            if (len >= (sizeof(packageHeader_t) + sizeof(msgCentral_t)))
            {
                msgCentral_t *pData = ICall_malloc(sizeof(msgCentral_t));

                memcpy((uint8_t *) pData,
                       (uint8_t *) &pSerialProtoMsg->data.central_msg_data,
                       sizeof(msgCentral_t));

                if (Central_enqueueMsg(EVT_IPC_CENTRAL, SUCCESS, (uint8_t*) pData)
                        != SUCCESS)
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

                if (Central_enqueueMsg(EVT_IPC_PERIPHERAL, SUCCESS, (uint8_t*) pData)
                        != SUCCESS)
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

