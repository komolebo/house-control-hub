/*
 * uart_adapter.h
 *
 *  Created on: 23 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_UART_ADAPTER_H_
#define APPLICATION_UART_ADAPTER_H_



/*********************************************************************
 * INCLUDES
 */
#include "util.h"
#include "profiles_if.h"
#include <string.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#define IPC_MSG_MAX_DATA_SIZE       (62)

/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    PACKAGE_CENTRAL_CMD,
    PACKAGE_CENTRAL_RESP,
    PACKAGE_PERIPHERY_READ_REQ,
    PACKAGE_PERIPHERY_WRITE_REQ,
    PACKAGE_PERIPHERY_READ_RESP,
    PACKAGE_PERIPHERY_WRITE_RESP,
    PACKAGE_PERIPHERAL_NOTIFICATION,
    PACKAGE_PERIPHERAL_INDICATION,
    PACKAGE_TYPE_COUNT
} packageType_t;

typedef enum
{
    CENTRAL_CMD_DISCOVER_DEVICES,
    CENTRAL_CMD_STOP_DEVICES_DISCOVER,
    CENTRAL_CMD_CONNECT_DEVICE,
    CENTRAL_CMD_DISCONNECT_DEVICE,
//    CENTRAL_CMD_DISCOVER_SERVICES_AND_UUIDS,
    CENTRAL_CMD_DISCOVER_DEVICE_UUIDS,
    CENTRAL_CMD_RESET_REGISTRATION
}  cmdCentral_t;


typedef struct
{
    uint8_t addr[B_ADDR_LEN];
} cmdReqDataConnectDevice_t;
typedef struct
{
    uint16_t conn_handle;
    uint8_t addr[B_ADDR_LEN];
    uint8_t *uuids[UUID_DATA_LEN];
} cmdRspDataConnectDevice_t;

typedef struct
{
    uint16_t conn_handle;
//    uint8_t addr[B_ADDR_LEN];
    uint16_t len;
    uint8_t *uuids[UUID_DATA_LEN];
} cmdRspDataDiscoverUuids_t;

typedef struct
{
    uint16_t conn_handle;
} cmdDataDisconnectDevice_t;

typedef struct
{
    uint16_t        conn_mask;
    uint8_t         uuid[UUID_DATA_LEN];
    uint16_t        len;
    uint8_t         data[IPC_MSG_MAX_DATA_SIZE];
} __attribute__ ((packed)) msgPeripheral_t;


typedef struct
{
    cmdCentral_t    cmd;
    uint16_t        len;
    uint8_t         data[IPC_MSG_MAX_DATA_SIZE];
} __attribute__ ((packed)) msgCentral_t;


typedef union
{
    msgPeripheral_t peripheral_msg_data;
    msgCentral_t    central_msg_data;
} packageMsgData_t;

typedef struct
{
//    uint16_t tid;
    packageType_t       package_type;
} __attribute__ ((packed)) packageHeader_t;

typedef struct
{
    packageHeader_t     header;
    packageMsgData_t    data;
} __attribute__ ((packed)) serialProtoMsg_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
void process_ipc_msg(const uint8_t *data, const uint16_t len);


bool send_ipc_msg(uint8_t *data, uint16_t len);

bool send_central_ipc_msg_resp(cmdCentral_t cmd,
                               uint16_t len,
                               uint8_t *data);

bool send_peripheral_ipc_msg(packageType_t type,
                             uint16_t conn_mask,
                             uint8_t uuid[UUID_DATA_LEN],
                             uint16_t len,
                             uint8_t *data);


#endif /* APPLICATION_UART_ADAPTER_H_ */
