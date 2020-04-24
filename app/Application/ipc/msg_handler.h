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

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#define IPC_MSG_DATA_MAX_SIZE       (20)


/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    PACKAGE_REQ_READ,
    PACKAGE_REQ_WRITE,
    PACKAGE_RESP_READ,
    PACKAGE_RESP_WRITE,
//    PACKAGE_ACKNOWLEDGE,
    PACKAGE_NOTIFICATION,
    PACKAGE_INDICATION,
    PACKAGE_TYPE_COUNT
} packageType_t;

typedef struct
{
//    uint16_t tid;
    uint16_t        device_mask;
    packageType_t   package_type;
    uint8_t         uuid[UUID_DATA_LEN];
    uint8_t         data[IPC_MSG_DATA_MAX_SIZE];
} serialProtoMsg_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */


#endif /* APPLICATION_UART_ADAPTER_H_ */
