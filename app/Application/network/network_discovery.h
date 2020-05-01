/*
 * network_discovery.h
 *
 *  Created on: 1 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_NETWORK_NETWORK_DISCOVERY_H_
#define APPLICATION_NETWORK_NETWORK_DISCOVERY_H_



// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_UUID                 // UUIDs discovery
};



void NetDisc_exchangeMtuSize(uint16_t conn_handle);
void NetDisc_processGATTEvent(gattMsgEvent_t *pMsg);


extern uint8_t discState;


#endif /* APPLICATION_NETWORK_NETWORK_DISCOVERY_H_ */
