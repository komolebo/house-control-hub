/*
 * network_request.h
 *
 *  Created on: 1 ����. 2020 �.
 *      Author: Oleh
 */

#ifndef APPLICATION_NETWORK_NET_REQUEST_H_
#define APPLICATION_NETWORK_NET_REQUEST_H_


#include "ipc/msg_handler.h"
#include <bcomdef.h>


void NetReq_processGATTUpdatesEvent(gattMsgEvent_t *pMsg);
void NetReq_processGATTExtEvent(gattMsgEvent_t *pMsg);


bStatus_t NetReq_processIpcReadCfgPeripheral(pkgDataPeriphery_t *ipcMsg);
bStatus_t NetReq_processIpcReadValPeripheral(pkgDataPeriphery_t *ipcMsg);
bStatus_t NetReq_processIpcWriteCfgPeripheral(pkgDataPeriphery_t *ipcMsg);
bStatus_t NetReq_processIpcWriteValPeripheral(pkgDataPeriphery_t *ipcMsg);



extern uint16_t netMaxPduSize;


#endif /* APPLICATION_NETWORK_NET_REQUEST_H_ */
