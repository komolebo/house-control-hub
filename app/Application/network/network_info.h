/*
 * network_info.h
 *
 *  Created on: 29 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_NETWORK_NETWORK_INFO_H_
#define APPLICATION_NETWORK_NETWORK_INFO_H_

#include <bcomdef.h>
#include <ti/sysbios/knl/Clock.h>

#include "profiles_if.h"


#define CHARS_PER_DEVICE        (20)
#define TOTAL_CHARS_NUM     (CHARS_PER_DEVICE * MAX_NUM_BLE_CONNS)


typedef struct
{
    uint16_t charHandle;
    uint8_t charValue[UUID_DATA_LEN];
} charInfo_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;                      // Connection Handle
  uint8_t  addr[B_ADDR_LEN];                // Peer Device Address
  charInfo_t  charData[CHARS_PER_DEVICE]; // Characteristic Handle
  Clock_Struct *pRssiClock;                 // pointer to clock struct
} connRec_t;



void NetInfo_init(void);
void NetInfo_initCharHandles(uint8_t index);

uint8_t     NetInfo_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
uint8_t     NetInfo_removeConnInfo(uint16_t connHandle);
uint8_t     NetInfo_addCharHandle(uint16_t connHandle,
                                  uint8_t charValue[UUID_DATA_LEN],
                                  uint16_t charHandle);
uint8_t     NetInfo_getConnIndex(uint16_t connHandle);
connRec_t * NetInfo_getConnInfo(uint16_t connHandle);
connRec_t * NetInfo_getConnInfoByInd(uint8_t index);
char *      NetInfo_getConnAddrStr(uint16_t connHandle);
uint16_t    NetInfo_getCharHandleByVal(uint16_t connHandle,
                                       uint8_t charValue[UUID_DATA_LEN]);
uint8_t*    NetInfo_getCharValByHandle(uint16_t connHandle,
                                       uint16_t charHandle);
uint8_t     NetInfo_populateUuidIpcResp(uint16_t connHandle, uint8_t *buf,
                                        uint8_t len);

extern uint8_t numConn;


#endif /* APPLICATION_NETWORK_NETWORK_INFO_H_ */
