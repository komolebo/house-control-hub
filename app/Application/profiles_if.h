/*
 * common_services_uuids.h
 *
 *  Created on: 18 êâ³ò. 2020 ð.
 *      Author: Oleh
 */

#ifndef APPLICATION_SERVICES_COMMON_PROFILE_DEF_H_
#define APPLICATION_SERVICES_COMMON_PROFILE_DEF_H_


/*********************************************************************
 * CONSTANTS
 */


/* ---------------------------------------------------------------------------
 * Service UUID definitions
 * */
#define LED_SERVICE_SERV_UUID       0x1110
#define BUTTON_SERVICE_SERV_UUID    0x1120
#define DATA_SERVICE_SERV_UUID      0x1130
#define CONFIG_SERVICE_SERV_UUID    0x1140

/* ---------------------------------------------------------------------------
 * Characteristic UUID definitions
 * */
// LED0 Characteristic defines
#define LS_LED0_ID                 0
#define LS_LED0_UUID               0x1111
#define LS_LED0_LEN                1
#define LS_LED0_LEN_MIN            1

// LED1 Characteristic defines
#define LS_LED1_ID                 1
#define LS_LED1_UUID               0x1112
#define LS_LED1_LEN                1
#define LS_LED1_LEN_MIN            1

// BUTTON0 Characteristic defines
#define BS_BUTTON0_ID                 0
#define BS_BUTTON0_UUID               0x1121
#define BS_BUTTON0_LEN                1
#define BS_BUTTON0_LEN_MIN            1

// BUTTON1 Characteristic defines
#define BS_BUTTON1_ID                 1
#define BS_BUTTON1_UUID               0x1122
#define BS_BUTTON1_LEN                1
#define BS_BUTTON1_LEN_MIN            1

// String Characteristic defines
#define DS_STRING_ID                 0
#define DS_STRING_UUID               0x1131
#define DS_STRING_LEN                40
#define DS_STRING_LEN_MIN            0

// Stream Characteristic defines
#define DS_STREAM_ID                 1
#define DS_STREAM_UUID               0x1132
#define DS_STREAM_LEN                20
#define DS_STREAM_LEN_MIN            0

// State Characteristic defines
#define CS_STATE_ID                  0
#define CS_STATE_UUID                0x1141
#define CS_STATE_LEN                 1

// Mode Characteristic defines
#define CS_MODE_ID                   1
#define ÑS_MODE_UUID                 0x1142
#define CS_MODE_LEN                  1

// Mode Characteristic defines
#define CS_SENSITIVITY_ID            2
#define ÑS_SENSITIVITY_UUID          0x1143
#define CS_SENSITIVITY_LEN           1


/*********************************************************************
 * MACROS
 */
#define BASE128_FROM_UINT16(uuid) 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, \
    0xF0

#endif /* APPLICATION_SERVICES_COMMON_PROFILE_DEF_H_ */
