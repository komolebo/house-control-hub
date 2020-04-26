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
// Application events
#define EVT_IPC_CENTRAL_CMD     0x00
#define EVT_IPC_PERIPHERAL_REQ  0x01
#define EVT_SCAN_ENABLE         0x02
#define EVT_SCAN_DISABLE        0x03
#define EVT_SCAN_ENABLED        0x04
#define EVT_SCAN_DISABLED       0x05
#define EVT_ADV_REPORT          0x06
#define EVT_SVC_DISC            0x07
#define EVT_READ_RSSI           0x08
#define EVT_PAIR_STATE          0x09
#define EVT_PASSCODE_NEEDED     0x0A
#define EVT_READ_RPA            0x0B
#define EVT_INSUFFICIENT_MEM    0x0C


// Maximum number of scan results.
// Note: this value cannot be greater than the number of items reserved in
// scMenuConnect (See simple_central_menu.c)
// This cannot exceed 27 (two-button menu's constraint)
#define DEFAULT_MAX_SCAN_RES                 8


/*********************************************************************
 * MACROS
 */

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
