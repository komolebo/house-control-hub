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
// Useful IPC data size
#define IPC_MSG_MAX_DATA_SIZE       (61)

// Full package size to be TX/RX'ed into/from IPC channel
#define IPC_MAX_MSG_SIZE            (sizeof(serialProtoPkg_t))

/*********************************************************************
 * TYPEDEFS
 */

// REQ: BACK->CENTRAL, RESP,UPDATES: CENTRAL->BACK
typedef enum
{
    PKG_CENTRAL_REQ,
    PKG_CENTRAL_RESP,
    PKG_PERIPHERY_REQ,
    PKG_PERIPHERY_RESP,
    PKG_PERIPHERY_UPDATES,
    PKG_TYPE_COUNT
} pkgType_t;

typedef enum
{
    CENTRAL_MSG_DISCOVER_DEVICES,
    CENTRAL_MSG_STOP_DEVICES_DISCOVER,
    CENTRAL_MSG_CONNECT_DEVICE,
    CENTRAL_MSG_DISCONNECT_DEVICE,
//    CENTRAL_CMD_DISCOVER_SERVICES_AND_UUIDS,
    CENTRAL_MSG_DISCOVER_DEVICE_UUIDS,
    CENTRAL_MSG_RESET_REGISTRATION
}  msgCentral_t;

typedef enum
{
    PERIPHERY_MSG_READ_VAL,
    PERIPHERY_MSG_WRITE_VAL,
    PERIPHERY_MSG_READ_CFG,
    PERIPHERY_MSG_WRITE_CFG,
    PERIPHERY_MSG_READ_MULTIPLE,
    PERIPHERY_MSG_WRITE_MULTIPLE,
    PERIPHERY_MSG_NOTIFY,
    PERIPHERY_MSG_INDICATE,
    PERIPHERY_MSG_COUNT
}  msgPeriphery_t;


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
} cmdDataDisconnectDevice_t;

typedef struct
{
    msgPeriphery_t  msg;
    uint16_t        connHandle;
    uint8_t         uuid[UUID_DATA_LEN];
    uint16_t        len;
    uint8_t         data[IPC_MSG_MAX_DATA_SIZE];
} __attribute__ ((packed)) pkgDataPeriphery_t;


typedef struct
{
    msgCentral_t    msg;
    uint16_t        len;
    uint8_t         data[IPC_MSG_MAX_DATA_SIZE];
} __attribute__ ((packed)) pkgDataCentral_t;


typedef union
{
    pkgDataPeriphery_t  peripheral_msg_data;
    pkgDataCentral_t    central_msg_data;
} pkgData_t;

typedef struct
{
//    uint16_t tid;
    pkgType_t       package_type;
} __attribute__ ((packed)) pkgHeader_t;

typedef struct
{
    pkgHeader_t     header;
    pkgData_t       data;
} __attribute__ ((packed)) serialProtoPkg_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
void process_ipc_msg(const uint8_t *data, const uint16_t len);


bool send_ipc_msg(uint8_t *data, uint16_t len);

bool send_central_ipc_msg_resp(msgCentral_t msg,
                               uint16_t len,
                               uint8_t *data);

bool send_peripheral_ipc_msg(pkgType_t type,
                             msgPeriphery_t msg,
                             uint16_t conn_mask,
                             uint8_t uuid[UUID_DATA_LEN],
                             uint16_t len,
                             uint8_t *data);


#endif /* APPLICATION_UART_ADAPTER_H_ */
