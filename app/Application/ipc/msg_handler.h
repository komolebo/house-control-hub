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
#define IPC_MSG_DATA_MAX_SIZE       (48)


/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    PACKAGE_CMD,
    PACKAGE_READ_REQ,
    PACKAGE_WRITE_REQ,
    PACKAGE_READ_RESP,
    PACKAGE_WRITE_RESP,
//    PACKAGE_ACKNOWLEDGE,
    PACKAGE_NOTIFICATION,
    PACKAGE_INDICATION,
    PACKAGE_TYPE_COUNT
} packageType_t;

typedef enum
{
    CENTRAL_MSG_DISCOVER,
    CENTRAL_MSG_RESET_REGISTRATION
} cmdCentral_t;


typedef struct
{
    uint16_t        conn_mask;
    uint8_t         uuid[UUID_DATA_LEN];
    uint8_t         data[IPC_MSG_DATA_MAX_SIZE];
} msgPeripheral_t;


typedef struct
{
    cmdCentral_t    cmd;
    uint8_t         data[IPC_MSG_DATA_MAX_SIZE];
} msgCentral_t;


typedef union
{
    msgPeripheral_t peripheral_msg_data;
    msgCentral_t    central_msg_data;
} packageMsgData_t;

typedef struct
{
//    uint16_t tid;
    packageType_t       package_type;
} packageHeader_t;

typedef struct
{
    packageHeader_t     header;
    packageMsgData_t    data;
} serialProtoMsg_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
void process_rx_ipc_msg(uint8_t *data, size_t len);


#endif /* APPLICATION_UART_ADAPTER_H_ */
