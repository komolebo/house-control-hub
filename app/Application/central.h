/*
 * central.h
 *
 *  Created on: 22 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_CENTRAL_CENTRAL_H_
#define APPLICATION_CENTRAL_CENTRAL_H_
/*********************************************************************
 * INCLUDES
 */
#include <ti/display/Display.h>
#include <bcomdef.h>
#include "util.h"
#include "icall_ble_api.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan results.
// Note: this value cannot be greater than the number of items reserved in
// scMenuConnect (See simple_central_menu.c)
// This cannot exceed 27 (two-button menu's constraint)
#define DEFAULT_MAX_SCAN_RES                 8

#define UUID_LIMIT_NUM_PER_DEVICE       (16)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
// Application events
typedef enum
{
    EVT_IPC_CENTRAL_CMD     = 0,
    EVT_IPC_CENTRAL_RESP,
    EVT_IPC_PERIPHERAL_REQ,
    EVT_SCAN_DISABLE,
    EVT_SCAN_ENABLED,
    EVT_SCAN_DISABLED,
    EVT_ADV_REPORT,
    EVT_SVC_DISC,
    EVT_READ_RSSI,
    EVT_PAIR_STATE,
    EVT_PASSCODE_NEEDED,
    EVT_READ_RPA,
    EVT_INSUFFICIENT_MEM
} appEvent_t;

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple Central.
 */
extern void Central_createTask();
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

extern Display_Handle dispHandle;

#endif /* APPLICATION_CENTRAL_CENTRAL_H_ */
