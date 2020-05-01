/*
 * network_request.h
 *
 *  Created on: 1 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_NETWORK_NETWORK_REQUEST_H_
#define APPLICATION_NETWORK_NETWORK_REQUEST_H_


#include "ipc/msg_handler.h"
#include <bcomdef.h>


void NetReq_processGATTExtEvent(gattMsgEvent_t *pMsg);

bStatus_t NetReq_processIpcReadPeripheral(pkgDataPeriphery_t *ipcMsg);
bStatus_t NetReq_processIpcWritePeripheral(pkgDataPeriphery_t *ipcMsg);


extern uint16_t netMaxPduSize;


#endif /* APPLICATION_NETWORK_NETWORK_REQUEST_H_ */
