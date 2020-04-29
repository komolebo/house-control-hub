/*
 * network_info.c
 *
 *  Created on: 29 квіт. 2020 р.
 *      Author: Oleh
 */

#include "string.h"

#include "network_info.h"
#include "icall_ble_api.h"
#include "icall.h"
#include "util.h"


// Spin if the expression is not true
#define NETINFO_ASSERT(expr) if (!(expr)) Central_spin();



// Number of connected devices
uint8_t numConn = 0;




// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];



void NetInfo_init(void)
{
    // Initialize connection list
    for (uint8_t i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        connList[i].connHandle = CONNHANDLE_INVALID;
        connList[i].pRssiClock = NULL;

        NetInfo_initCharHandles(i);
    }
}

void NetInfo_initCharHandles(uint8_t index)
{
    for (uint8_t j = 0; j < CHARS_PER_DEVICE; j++ )
    {
        connList[index].charData[j].charHandle = GATT_INVALID_HANDLE;
        memset(connList[index].charData[j].charValue, 0, UUID_DATA_LEN);
    }
}


/*********************************************************************
 * @fn      NetInfo_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
uint8_t NetInfo_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;
            memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
            numConn++;

            break;
        }
    }

    return i;
}

/*********************************************************************
 * @fn      NetInfo_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
uint8_t NetInfo_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            break;
        }
    }

    return i;
}

/*********************************************************************
 * @fn      NetInfo_getConnInfoByInd
 *
 * @brief   Find index in the connected device list by index
 *
 * @return  an element has the given connection index.
 *          if there is no match, NULL will be returned.
 */
connRec_t * NetInfo_getConnInfoByInd(uint8_t index)
{
    if (index < MAX_NUM_BLE_CONNS)
    {
        if (connList[index].connHandle != CONNHANDLE_INVALID)
        {
            return &connList[index];
        }
    }

    return NULL;
}

/*********************************************************************
 * @fn      NetInfo_getConnInfo
 *
 * @brief   Find element in the connected device list by connHandle
 *
 * @return  an element that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
connRec_t * NetInfo_getConnInfo(uint16_t connHandle)
{
    uint8_t connIndex = NetInfo_getConnIndex(connHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    if (connIndex < MAX_NUM_BLE_CONNS)
    {
        return NULL;
    }

    return &connList[connIndex];
}

/*********************************************************************
 * @fn      NetInfo_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
char* NetInfo_getConnAddrStr(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            return Util_convertBdAddr2Str(connList[i].addr);
        }
    }

    return NULL;
}

/*********************************************************************
 * @fn      NetInfo_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
uint8_t NetInfo_removeConnInfo(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            // Found the entry to mark as deleted
            connList[i].connHandle = CONNHANDLE_INVALID;
            numConn--;

            break;
        }
    }

    return i;
}
