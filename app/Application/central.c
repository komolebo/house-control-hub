/*
 * central.c
 *
 *  Created on: 22 ���. 2020 �.
 *      Author: Oleh
 */
/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include <bcomdef.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "board.h"

#include <ble_user_config.h>

//#include "simple_gatt_profile.h"

#include "central.h"

//#include <uartlog/UartLog.h>  // Comment out if using xdc Log
#include "profiles_if.h"

#include "ipc/uart_handler.h"

//#include "UartLog.h"
#include <uartlog/UartLog.h>
#include "ipc/msg_handler.h"
/*********************************************************************
 * CONSTANTS
 */

// Task configuration
#define CENTRAL_TASK_PRIORITY                     1

#ifndef CENTRAL_TASK_STACK_SIZE
#define CENTRAL_TASK_STACK_SIZE                   1024
#endif

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SC_ADDR_STR_SIZE     15

// Spin if the expression is not true
#define CENTRAL_ASSERT(expr) if (!(expr)) Central_spin();


// Simple Central Task Events
#define SC_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SC_ALL_EVENTS                        (SC_ICALL_EVT           | \
                                              SC_QUEUE_EVT)

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                 ADDRMODE_RP_WITH_PUBLIC_ID

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_1M

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                100 // 1 sec

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                  3000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// How often to read current current RPA (in ms)
#define SC_READ_RPA_PERIOD                    3000
/*********************************************************************
 * TYPEDEFS
 */
// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_UUID                 // UUIDs discovery
};


// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;        // Connection Handle
  uint8_t  addr[B_ADDR_LEN];  // Peer Device Address
  uint8_t  charHandle;        // Characteristic Handle
  Clock_Struct *pRssiClock;   // pointer to clock struct
} connRec_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint16_t connHandle;
  uint8_t  status;
} scPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} scPasscodeData_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task configuration
Task_Struct centralTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(centralTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t centralTaskStack[CENTRAL_TASK_STACK_SIZE];

ICall_EntityID selfEntity;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Central_pairStateCb(uint16_t connHandle, uint8_t state,
                                uint8_t status);
static void Central_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                               uint8_t uiInputs, uint8_t uiOutputs,
                               uint32_t numComparison);
static uint8_t Central_processStackMsg(ICall_Hdr *pMsg);
static void Central_processAppMsg(appEvt_t *pMsg);
static void Central_addScanInfo(uint8_t *pAddr, uint8_t addrType);
static uint8_t Central_getConnIndex(uint16_t connHandle);
static void Central_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void Central_processGapMsg(gapEventHdr_t *pMsg);
static void Central_processPairState(uint8_t state,
                                     scPairStateData_t* pPairData);
static void Central_processPasscode(scPasscodeData_t *pData);
static void Central_exchangeMtuSize();
static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
static status_t Central_CancelRssi(uint16_t connHandle);
static char* SimpleCentral_getConnAddrStr(uint16_t connHandle);
static uint8_t Central_removeConnInfo(uint16_t connHandle);
static void Central_processIpcHubReq(pkgDataCentral_t *ipcMsg);
static void Central_processIpcPeripheryReq(pkgDataPeriphery_t *ipcMsg);
static bool connectScannedDevice(uint8_t index);
/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Central";


#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;
static uint8_t discConnHandle;

// Discovered service start and end handle
static uint16_t discSvcStartHdl = 0;
static uint16_t discSvcEndHdl = 0;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Number of connected devices
static uint8_t numConn = 0;

// Connection handle of current connection
#if 0
static uint16_t connHandle = CONNHANDLE_INVALID;
#endif
// Entity ID globally used to check for source and/or destination of messages

// Event globally used to post local events and pend on system and
// local events.


// Queue object used for app messages



// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Maximum PDU size (default = 27 octets)
static uint16_t scMaxPduSize;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;


// Value to write
static uint8_t charVal = 0;

// Bond Manager Callbacks
static gapBondCBs_t bondMgrCBs =
{
    Central_passcodeCb, // Passcode callback
    Central_pairStateCb // Pairing/Bonding state Callback
};




// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Central_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void Central_spin(void)
{
    volatile uint8_t x;

    while (1)
    {
        x++;
    }
}


/*********************************************************************
 * @fn      Central_init
 *
 * @brief   Initialization function for the Simple Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void Central_init( )
{
    uint8_t i;

    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &eventSyncHandle);

    // Create an RTOS queue for message from profile to be sent to app.


    // Initialize internal data
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        connList[i].connHandle = CONNHANDLE_INVALID;
        connList[i].pRssiClock = NULL;
    }

    //  Board_initKeys(SimpleCentral_keyChangeHandler);

    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                     (void * )attDeviceName);

    //Set default values for Data Length Extension
    //Extended Data Length Feature is already enabled by default
    //in build_config.opt in stack project.
    {
        //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
        #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        //This API is documented in hci.h
        //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
        //http://software-dl.ti.com/lprf/ble5stack-latest/
        //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set Bond Manager parameters
    {
        // Don't send a pairing request after connecting; the device waits for the
        // application to start pairing
        uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
        // Do not use authenticated pairing
        uint8_t mitm = FALSE;
        // This is a display only device
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        // Create a bond during the pairing process
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
                                &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t),
                                &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
                                &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
                                &bonding);
    }

    // Start Bond Manager and register callback
    // This must be done before initialing the GAP layer
    VOID GAPBondMgr_Register(&bondMgrCBs);

    // Accept all parameter update requests
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION,
                      GAP_UPDATE_REQ_ACCEPT_ALL);

    // Register with GAP for HCI/Host messages (for RSSI)
    GAP_RegisterForMsgs(selfEntity);

    // Initialize GAP layer for Central role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

    //  dispHandle = Display_open(Display_Type_ANY, NULL);
    //
    //  // Disable all items in the main menu
    //  tbm_setItemStatus(&scMenuMain, SC_ITEM_NONE, SC_ITEM_ALL);
    //  // Initialize Two-button Menu
    //  tbm_initTwoBtnMenu(dispHandle, &scMenuMain, 4, SimpleCentral_menuSwitchCb);
    //  Display_printf(dispHandle, SC_ROW_SEPARATOR, 0, "====================");
}

/*********************************************************************
 * @fn      Central_taskFxn
 *
 * @brief   Application task entry point for the Simple Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void Central_taskFxn(uintptr_t a0, uintptr_t a1)
{
    // Initialize application
    Central_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        events = Util_listenEventMsg(SC_ALL_EVENTS);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void **) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature != 0xffff)
                    {
                        // Process inter-task message
                        safeToDealloc = Central_processStackMsg(
                                (ICall_Hdr *) pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message
            if (events & SC_QUEUE_EVT)
            {
                appEvt_t * pMsg;
                while (pMsg = (appEvt_t *) Util_dequeueAppMsg())
                {
                    // Process message
                    Central_processAppMsg(pMsg);

                    // Free the space from the message
                    ICall_free(pMsg);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      Central_createTask
 *
 * @brief   Task creation function for the Central.
 *
 * @param   none
 *
 * @return  none
 */
void Central_createTask()
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = centralTaskStack;
    taskParams.stackSize = CENTRAL_TASK_STACK_SIZE;
    taskParams.priority = CENTRAL_TASK_PRIORITY;

    Task_construct(&centralTask, Central_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      Central_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void Central_processAppMsg(appEvt_t *pMsg)
{
    bool safeToDealloc = TRUE;

    switch (pMsg->hdr.event)
    {
    // Parse incoming IPC message to request central functionality
    case EVT_IPC_CENTRAL_CMD:
        Central_processIpcHubReq((pkgDataCentral_t *) pMsg->pData);
        ICall_free(pMsg->pData);
        break;

    // Route request for peripheral device
    case EVT_IPC_MSG_PERIPHERAL:
        Central_processIpcPeripheryReq((pkgDataPeriphery_t *) pMsg->pData);
        /* route here peripheral request to device's GATT */
        break;

    case EVT_ADV_REPORT:
    {
        GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

        if (SimpleCentral_findSvcUuid(0xFFF0 /*CONFIG_SERVICE_SERV_UUID*/, // TODO: replace with actual UUID
                        pAdvRpt->pData, pAdvRpt->dataLen))
        {
            Central_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
            send_central_ipc_msg_resp(CENTRAL_MSG_DISCOVER_DEVICES,
                                      B_ADDR_LEN,
                                      pAdvRpt->addr);

            Log_info1("Discovered: %s",
                      (uintptr_t)Util_convertBdAddr2Str(pAdvRpt->addr));

//            connectScannedDevice(0);
        }

        // Free report payload data
        if (pAdvRpt->pData != NULL)
        {
            ICall_free(pAdvRpt->pData);
        }
        break;
    }

    case EVT_SCAN_ENABLED:
        Log_info0("Discovering...");
        break;

    case EVT_SCAN_DISABLED:
    {
        Log_info0("Scan disabled");
        uint8_t numReport;
        uint8_t i;
        static uint8_t* pAddrs = NULL;
        uint8_t* pAddrTemp;
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
        GapScan_Evt_AdvRpt_t advRpt;

        numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

        Log_info1("%d devices discovered", numReport);

#if 0
        if (numReport > 0)
        {
            // Also enable "Connect to"
            itemsToEnable |= SC_ITEM_CONNECT;
        }

        if (numConn > 0)
        {
            // Also enable "Work with"
            itemsToEnable |= SC_ITEM_SELECTCONN;
        }

        // Enable "Discover Devices", "Set Scanning PHY", and possibly
        // "Connect to" and/or "Work with".
        // Disable "Stop Discovering".
        tbm_setItemStatus(&scMenuMain, itemsToEnable, SC_ITEM_STOPDISC);
#endif
        // Allocate buffer to display addresses
        if (pAddrs != NULL)
        {
            // A scan has been done previously, release the previously allocated buffer
            ICall_free(pAddrs);
        }
        pAddrs = ICall_malloc(numReport * SC_ADDR_STR_SIZE);
        if (pAddrs == NULL)
        {
            numReport = 0;
        }
#if 0
        TBM_SET_NUM_ITEM(&scMenuConnect, numReport);
#endif
        if (pAddrs != NULL)
        {
            pAddrTemp = pAddrs;
            for (i = 0; i < numReport; i++, pAddrTemp += SC_ADDR_STR_SIZE)
            {
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
                // Get the address from the list, convert it to string, and
                // copy the string to the address buffer
                memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
                        SC_ADDR_STR_SIZE);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
                // Get the address from the report, convert it to string, and
                // copy the string to the address buffer
                GapScan_getAdvReport(i, &advRpt);
                memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
                       SC_ADDR_STR_SIZE);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
#if 0
                // Assign the string to the corresponding action description of the menu
                TBM_SET_ACTION_DESC(&scMenuConnect, i, pAddrTemp);
#endif
            }

#if 0
            // Disable any non-active scan results
            for (; i < DEFAULT_MAX_SCAN_RES; i++)
            {
                tbm_setItemStatus(&scMenuConnect, TBM_ITEM_NONE, (1 << i));
            }
#endif
            // Note: pAddrs is not freed since it will be used by the two button menu
            // to display the discovered address.
            // This implies that at least the last discovered addresses
            // will be maintained until a new scan is done.
        }
        break;
    }

    case EVT_SVC_DISC:
//        Log_info0("Start service discovery");
//        Central_exchangeMtuSize();
        break;

    case EVT_READ_RSSI:
    {
        Log_info0("Read RSSI event");
        uint8_t connIndex = pMsg->hdr.state;
        uint16_t connHandle = connList[connIndex].connHandle;

        // If link is still valid
        if (connHandle != CONNHANDLE_INVALID)
        {
            // Restart timer
            Util_startClock(connList[connIndex].pRssiClock);

            // Read RSSI
            VOID HCI_ReadRssiCmd(connHandle);
        }

        break;
    }

        // Pairing event
    case EVT_PAIR_STATE:
    {
        Log_info0("Process pair state");
        Central_processPairState(pMsg->hdr.state,
                                       (scPairStateData_t*) (pMsg->pData));
        break;
    }

        // Passcode event
    case EVT_PASSCODE_NEEDED:
    {
        Log_info0("Pass code needed");
        Central_processPasscode((scPasscodeData_t *) (pMsg->pData));
        break;
    }

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case EVT_READ_RPA:
    {
        Log_info0("Reading RPA");
        uint8_t* pRpaNew;

        // Read the current RPA.
        pRpaNew = GAP_GetDevAddress(FALSE);

        if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
        {
            // If the RPA has changed, update the display
            Log_info1("RP Addr: %s", (uintptr_t)Util_convertBdAddr2Str(pRpaNew));
            memcpy(rpa, pRpaNew, B_ADDR_LEN);
        }
        break;
    }
#endif // PRIVACY_1_2_CFG

        // Insufficient memory
    case EVT_INSUFFICIENT_MEM:
    {
        // We are running out of memory.
        Log_info0("Insufficient Memory");

        // We might be in the middle of scanning, try stopping it.
        GapScan_disable();
        break;
    }

    default:
        // Do nothing.
        break;
    }

    if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
    {
        ICall_free(pMsg->pData);
    }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      SimpleCentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
    uint8_t adLen;
    uint8_t adType;
    uint8_t *pEnd;

    if (dataLen > 0)
    {
        pEnd = pData + dataLen - 1;

        // While end of data not reached
        while (pData < pEnd)
        {
            // Get length of next AD item
            adLen = *pData++;
            if (adLen > 0)
            {
                adType = *pData;

                // If AD type is for 16-bit service UUID
                if ((adType == GAP_ADTYPE_16BIT_MORE)
                        || (adType == GAP_ADTYPE_16BIT_COMPLETE))
                {
                    pData++;
                    adLen--;

                    // For each UUID in list
                    while (adLen >= 2 && pData < pEnd)
                    {
                        // Check for match
                        if ((pData[0] == LO_UINT16(uuid))
                                && (pData[1] == HI_UINT16(uuid)))
                        {
                            // Match found
                            return TRUE;
                        }

                        // Go to next
                        pData += 2;
                        adLen -= 2;
                    }

                    // Handle possible erroneous extra byte in UUID list
                    if (adLen == 1)
                    {
                        pData++;
                    }
                }
                else
                {
                    // Go to next item
                    pData += adLen;
                }
            }
        }
    }

    // Match not found
    return FALSE;
}

/*********************************************************************
 * @fn      Central_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void Central_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    // If result count not at max
    if (numScanRes < DEFAULT_MAX_SCAN_RES)
    {
        // Check if device is already in scan results
        for (i = 0; i < numScanRes; i++)
        {
            if (memcmp(pAddr, scanList[i].addr, B_ADDR_LEN) == 0)
            {
                return;
            }
        }

        // Add addr to scan result list
        memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
        scanList[numScanRes].addrType = addrType;

        // Increment scan result count
        numScanRes++;
    }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      Central_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void Central_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status)
{
    scPairStateData_t *pData;

    // Allocate space for the event data.
    if ((pData = ICall_malloc(sizeof(scPairStateData_t))))
    {
        pData->connHandle = connHandle;
        pData->status = status;

        // Queue the event.
        if (Util_enqueueAppMsg(EVT_PAIR_STATE, state, (uint8_t*) pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      Central_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - pointer to device address
 *
 * @param   connHandle - the connection handle
 *
 * @param   uiInputs - pairing User Interface Inputs
 *
 * @param   uiOutputs - pairing User Interface Outputs
 *
 * @param   numComparison - numeric Comparison 20 bits
 *
 * @return  none
 */
static void Central_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison)
{
    scPasscodeData_t *pData = ICall_malloc(sizeof(scPasscodeData_t));

    // Allocate space for the passcode event.
    if (pData)
    {
        pData->connHandle = connHandle;
        memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
        pData->uiInputs = uiInputs;
        pData->uiOutputs = uiOutputs;
        pData->numComparison = numComparison;

        // Enqueue the event.
        if (Util_enqueueAppMsg(EVT_PASSCODE_NEEDED, 0,
                                     (uint8_t *) pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}



/*********************************************************************
 * @fn      Central_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void Central_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
    static uint16_t ipcBuff[UUID_LIMIT_NUM_PER_DEVICE];
    static uint16 discUuid = 0;


    if (discState == BLE_DISC_STATE_MTU)
    {
        // MTU size response received, discover services
        if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
        {
            discState = BLE_DISC_STATE_SVC;

            /* discover all device's services */
            GATT_DiscAllPrimaryServices(discConnHandle, selfEntity);
        }
    }
    else if (discState == BLE_DISC_STATE_SVC)
    {
        // Services found, store handles
        uint8_t svcNum = ((attReadByGrpTypeRsp_t *)&pMsg->msg)->numGrps;

        if ((pMsg->method == ATT_READ_BY_GRP_TYPE_RSP) && (svcNum > 0))
        {
            discSvcStartHdl = ATT_ATTR_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
            discSvcEndHdl = ATT_GRP_END_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, svcNum - 1);

            Log_info2("%s: ServiceDiscovery - found: %d",
                      (uintptr_t )__func__,
                      ((attReadByGrpTypeRsp_t * )&pMsg->msg)->numGrps);
        }

        // If procedure complete
        if (((pMsg->method == ATT_READ_BY_GRP_TYPE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            if (discSvcStartHdl != 0)
            {
                discState = BLE_DISC_STATE_UUID;
                discUuid = 0;

                GATT_DiscAllChars(discConnHandle, discSvcStartHdl, discSvcEndHdl,
                                  selfEntity);
            }
        }
    }
    else if (discState == BLE_DISC_STATE_UUID)
    {
        uint16_t uuidFound = pMsg->msg.readByTypeRsp.numPairs;

        // Characteristic descriptors found
        if (pMsg->method == ATT_READ_BY_TYPE_RSP && uuidFound > 0)
        {
            uint8_t len = pMsg->msg.readByTypeRsp.len;

            Log_info2("%s: UuidDiscovery - found: %d",
                      (uintptr_t )__func__, uuidFound);

            // For each characteristic declaration
            for (uint8_t i = 0, *p = pMsg->msg.readByTypeRsp.pDataList; i < uuidFound;
                    i++, p += len)
            {
                // Parse characteristic declaration, UUID is last 2 bytes of an element
                uint16_t uuid = BUILD_UINT16(p[len - 2], p[len - 1]);

                // If UUID is of interest, store handle
                Log_info2("%s: UuidDiscovery - retain characteristic: 0x%x",
                          (uintptr_t )__func__, uuid);

                ipcBuff[i + discUuid] = uuid;
            }

            discUuid += uuidFound;
        }

        // If procedure complete
        if (((pMsg->method == ATT_READ_BY_TYPE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            // Send IPC report to the back
            send_central_ipc_msg_resp(CENTRAL_MSG_DISCOVER_DEVICE_UUIDS,
                                      discUuid * ATT_BT_UUID_SIZE,
                                      (uint8_t *)&ipcBuff);

            // Discovery done
            discState = BLE_DISC_STATE_IDLE;
        }
    }
}

/*********************************************************************
 * @fn      Central_processGATTMsgEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void Central_processGATTMsgEvent(gattMsgEvent_t *pMsg)
{
    static uint8_t buff_str[50];
    if (pMsg->method == ATT_READ_BY_TYPE_RSP)
    {
        uint8_t len = pMsg->msg.readByTypeRsp.len;
        uint8_t pairs = pMsg->msg.readByTypeRsp.numPairs;

        Log_info2("%s: numPairs: %d", __func__, pairs);
        Log_info2("%s: len: %d", __func__, len);

        for (uint8_t i = 0; i < pairs; i++)
        {
            if (Util_convertHex2Str(pMsg->msg.readByTypeRsp.pDataList, &buff_str[0],
                                len, 50))
            {
                // If UUID is of interest, store handle
                Log_info2("%s: UUID Read result: %s", (uintptr_t )__func__,
                          (uintptr_t )(pMsg->msg.readByTypeRsp.pDataList + 2));
//                          (uintptr_t )buff_str);
            }

        }
    }
}


/*********************************************************************
 * @fn      Central_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void Central_processGATTMsg(gattMsgEvent_t *pMsg)
{
//    Log_error1("DEBUG: RECEIVED GATT MSG: 0x%x", pMsg->method);
    if (linkDB_Up(pMsg->connHandle))
    {
        // See if GATT server was unable to transmit an ATT response
        if (pMsg->hdr.status == blePending)
        {
            // No HCI buffer was available. App can try to retransmit the response
            // on the next connection event. Drop it for now.
            Log_info1("ATT Rsp dropped %d", pMsg->method);
        }
        else if ((pMsg->method == ATT_READ_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
                Log_error1("Read Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a successful read, display the read value
                Log_info1("Read rsp: 0x%02x", pMsg->msg.readRsp.pValue[0]);
            }
        }
        else if ((pMsg->method == ATT_WRITE_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
                Log_error1("Write Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a successful write, display the value that was written and
                // increment value
                Log_info1("Write sent: 0x%02x", charVal);
            }
        }
        else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
        {
            // ATT request-response or indication-confirmation flow control is
            // violated. All subsequent ATT requests or indications will be dropped.
            // The app is informed in case it wants to drop the connection.

            // Display the opcode of the message that caused the violation.
            Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
        }
        else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
        {
            // MTU size updated
            Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
        }
        else if (discState != BLE_DISC_STATE_IDLE)
        {
            Central_processGATTDiscEvent(pMsg);
        }
        else
        {
            // Not being discovered now
            Central_processGATTMsgEvent(pMsg);

        }
    } // else - in case a GATT message came after a connection has dropped, ignore it.

    // Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      Central_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t Central_processStackMsg(ICall_Hdr *pMsg)
{
    uint8_t safeToDealloc = TRUE;

    switch (pMsg->event)
    {
    case GAP_MSG_EVENT:
        Central_processGapMsg((gapEventHdr_t*) pMsg);
        break;

    case GATT_MSG_EVENT:
        Central_processGATTMsg((gattMsgEvent_t *) pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {
        // Process HCI message
        switch (pMsg->status)
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            Central_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
            break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
            break;

            // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *) pMsg;
            switch (pMyMsg->cmdOpcode)
            {
            case HCI_LE_SET_PHY:
            {
                if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                {
                    Log_info0("PHY Change failure, peer does not support this");
                }
                else
                {
                    Log_info1("PHY Update Status: 0x%02x", pMyMsg->cmdStatus);
                }
            }
                break;
            case HCI_DISCONNECT:
                break;

            default:
            {
                Log_info2("Unknown Cmd Status: 0x%04x::0x%02x",
                               pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
            }
                break;
            }
        }
            break;

            // LE Events
        case HCI_LE_EVENT_CODE:
        {
            hciEvt_BLEPhyUpdateComplete_t *pPUC =
                    (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

            if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
            {
                if (pPUC->status != SUCCESS)
                {
                    Log_info1("%s: PHY change failure",
                              (uintptr_t)SimpleCentral_getConnAddrStr(pPUC->connHandle));
                }
                else
                {
                    Log_info2(
                            "%s: PHY updated to %s",
                            (uintptr_t)SimpleCentral_getConnAddrStr(
                                    pPUC->connHandle),
                            // Only symmetrical PHY is supported.
                            // rxPhy should be equal to txPhy.
                            (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ?
                                    (uintptr_t)"1 Mbps" :
                                    (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ?
                                            (uintptr_t)"2 Mbps" :
                                            (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ?
                                                    (uintptr_t)"CODED" :
                                                    (uintptr_t)"Unexpected PHY Value");
                }
            }

            break;
        }

        default:
            break;
        }

        break;
    }

    case L2CAP_SIGNAL_EVENT:
        // place holder for L2CAP Connection Parameter Reply
        break;

    default:
        break;
    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      Central_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void Central_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
    uint8_t event;

    if (evt & GAP_EVT_ADV_REPORT)
    {
        event = EVT_ADV_REPORT;
    }
    else if (evt & GAP_EVT_SCAN_ENABLED)
    {
        event = EVT_SCAN_ENABLED;
    }
    else if (evt & GAP_EVT_SCAN_DISABLED)
    {
        event = EVT_SCAN_DISABLED;
    }
    else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
    {
        event = EVT_INSUFFICIENT_MEM;
    }
    else
    {
        return;
    }

    if (Util_enqueueAppMsg(event, SUCCESS, pMsg) != SUCCESS)
    {
        ICall_free(pMsg);
    }
}

/*********************************************************************
 * @fn      Central_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
void Central_clockHandler(UArg arg)
{
    uint8_t evtId = (uint8_t) (arg & 0xFF);

    switch (evtId)
    {
    case EVT_READ_RSSI:
        Util_enqueueAppMsg(EVT_READ_RSSI, (uint8_t) (arg >> 8), NULL);
        break;

    case EVT_READ_RPA:
        // Restart timer
        Util_startClock(&clkRpaRead);
        // Let the application handle the event
        Util_enqueueAppMsg(EVT_READ_RPA, 0, NULL);
        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      Central_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Central_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
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
 * @fn      Central_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void Central_processGapMsg(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        uint8_t temp8;
        uint16_t temp16;
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *) pMsg;

        // Setup scanning
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Register callback to process Scanner events
        GapScan_registerCb(Central_scanCb, NULL);

        // Set Scanner Event Mask
        GapScan_setEventMask(
                GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED | GAP_EVT_ADV_REPORT);

        // Set Scan PHY parameters
        GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_PASSIVE,
                             SCAN_PARAM_DFLT_INTERVAL,
                             SCAN_PARAM_DFLT_INTERVAL);

        // Set Advertising report fields to keep
        temp16 = ADV_RPT_FIELDS;
        GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);

        // Set Scanning Primary PHY
        temp8 = DEFAULT_SCAN_PHY;
        GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);

        // Set LL Duplicate Filter
        temp8 = SCAN_FLT_DUP_ENABLE;
        GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

        // Set PDU type filter -
        // Only 'Connectable' and 'Complete' packets are desired.
        // It doesn't matter if received packets are
        // whether Scannable or Non-Scannable, whether Directed or Undirected,
        // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
        temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
        GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

        scMaxPduSize = pPkt->dataPktLen;

        Log_info0("Initialized");
        Log_info1("Num Conns: %d", numConn);

        // Display device address
        Log_info2("%s Addr: %s",
                  (uintptr_t )((addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID"),
                  (uintptr_t )Util_convertBdAddr2Str(pPkt->devAddr));

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
        if (addrMode > ADDRMODE_RANDOM)
        {
            // Update the current RPA.
            memcpy(rpa, GAP_GetDevAddress(FALSE), B_ADDR_LEN);

            Log_info1("RP Addr: %s", (uintptr_t)Util_convertBdAddr2Str(rpa));

            // Create one-shot clock for RPA check event.
            Util_constructClock(&clkRpaRead, Central_clockHandler,
                                SC_READ_RPA_PERIOD, 0, true, EVT_READ_RPA);
        }
#endif // PRIVACY_1_2_CFG
        break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
        Log_info0("Conneting attempt cancelled");
        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
        uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
        uint8_t connIndex;
        uint8_t* pStrAddr;

        // Add this connection info to the list
        connIndex = Central_addConnInfo(connHandle, pAddr);

        // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        CENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

        connList[connIndex].charHandle = 0;

        pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

        Log_info1("Connected to %s", (uintptr_t)pStrAddr);
        Log_info1("Num Conns: %d", numConn);

        // Request MTU to be ready for next discovery request
        discConnHandle = connHandle;
        Central_exchangeMtuSize(connHandle);

#if 0
        send_central_ipc_msg_resp(CENTRAL_CONNECT_DEVICE,
                                  sizeof(connHandle),
                                  (uint8_t *)&connHandle);
#endif
        break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
        uint16_t connHandle =
                ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
        uint8_t connIndex;
        uint8_t* pStrAddr;

        // Cancel timers
        Central_CancelRssi(connHandle);

        // Mark this connection deleted in the connected device list.
        connIndex = Central_removeConnInfo(connHandle);

        // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        CENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

        pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

        Log_info1("%s is disconnected", (uintptr_t)pStrAddr);
        Log_info1("Num Conns: %d", numConn);

        send_central_ipc_msg_resp(CENTRAL_MSG_DISCONNECT_DEVICE,
                                          sizeof(connHandle),
                                          (uint8_t *)&connHandle);

#if 0
        for (i = 0; i < TBM_GET_NUM_ITEM(&scMenuConnect); i++)
        {
            if (!memcmp(TBM_GET_ACTION_DESC(&scMenuConnect, i), pStrAddr,
                        SC_ADDR_STR_SIZE))
            {
                // Enable this device in the connection choices
                tbm_setItemStatus(&scMenuConnect, 1 << i, SC_ITEM_NONE);
            }

            if (TBM_IS_ITEM_ACTIVE(&scMenuConnect, i))
            {
                numConnectable++;
            }
        }
#endif
        break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReq_t *pReq;

        pReq = &((gapUpdateLinkParamReqEvent_t *) pMsg)->req;

        rsp.connectionHandle = pReq->connectionHandle;
        rsp.signalIdentifier = pReq->signalIdentifier;

        if (acceptParamUpdateReq)
        {
            rsp.intervalMin = pReq->intervalMin;
            rsp.intervalMax = pReq->intervalMax;
            rsp.connLatency = pReq->connLatency;
            rsp.connTimeout = pReq->connTimeout;
            rsp.accepted = TRUE;
        }
        else
        {
            // Reject the request.
            rsp.accepted = FALSE;
        }

        // Send Reply
        VOID GAP_UpdateLinkParamReqReply(&rsp);

        break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *) pMsg;
        // Get the address from the connection handle
        linkDBInfo_t linkInfo;

        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) == SUCCESS)
        {
            if (pPkt->status == SUCCESS)
            {
                Log_info2("Updated: %s, connTimeout:%d",
                          (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr),
                          linkInfo.connTimeout * CONN_TIMEOUT_MS_CONVERSION);
            }
            else
            {
                // Display the address of the connection update failure
                Log_info2("Update Failed 0x%h: %s", pPkt->opcode,
                          (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));
            }
        }

        break;
    }

    default:
        break;
    }
}


/*********************************************************************
 * @fn      Central_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Central_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
#ifndef Display_DISABLE_ALL
      uint16_t connHandle = BUILD_UINT16(pMsg->pReturnParam[1],
                                         pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];

      Log_info2("%s: RSSI %d dBm",
                (uintptr_t)SimpleCentral_getConnAddrStr(connHandle), rssi);

#endif
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      Central_exchangeMtuSize
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void Central_exchangeMtuSize()
{
    attExchangeMTUReq_t req;

    // Initialize cached handles
    discSvcStartHdl = discSvcEndHdl = 0;

    discState = BLE_DISC_STATE_MTU;

    // Discover GATT Server's Rx MTU size
    req.clientRxMTU = scMaxPduSize - L2CAP_HDR_SIZE;

    // ATT MTU size should be set to the minimum of the Client Rx MTU
    // and Server Rx MTU values
    VOID GATT_ExchangeMTU(discConnHandle, &req, selfEntity);

    Log_info1("Exchanging MTU with 0x%x", discConnHandle);
}

/*********************************************************************
 * @fn      Central_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void Central_processPairState(uint8_t state, scPairStateData_t* pPairData)
{
    uint8_t status = pPairData->status;

    if (state == GAPBOND_PAIRING_STATE_STARTED)
    {
        Log_info0("Pairing started");
    }
    else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if (status == SUCCESS)
        {
            linkDBInfo_t linkInfo;

            Log_info0("Pairing success");

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
            if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
            {
                // If the peer was using private address, update with ID address
                if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID
                        || linkInfo.addrType == ADDRTYPE_RANDOM_ID)
                        && !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))
                {
                    // Update the address of the peer to the ID address
                    Log_info1("Addr updated: %s",
                              (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));

                    // Update the connection list with the ID address
                    uint8_t i = Central_getConnIndex(
                            pPairData->connHandle);

                    CENTRAL_ASSERT(i < MAX_NUM_BLE_CONNS);
                    memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
                }
            }
#endif // PRIVACY_1_2_CFG
        }
        else
        {
            Log_info1("Pairing fail: %d", status);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
    {
        if (status == SUCCESS)
        {
            Log_info0("Encryption success");
        }
        else
        {
            Log_info1("Encryption failed: %d", status);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
    {
        if (status == SUCCESS)
        {
            Log_info0("Bond save success");
        }
        else
        {
            Log_info1("Bond save failed: %d", status);
        }
    }
}

/*********************************************************************
 * @fn      Central_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void Central_processPasscode(scPasscodeData_t *pData)
{
    // Display passcode to user
    if (pData->uiOutputs != 0)
    {
        Log_info1("Passcode: %d", B_APP_DEFAULT_PASSCODE);
    }

    // Send passcode response
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}


/*********************************************************************
 * @fn      Central_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Central_getConnIndex(uint16_t connHandle)
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
 * @fn      Central_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connection handle
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: Has not started
 */
static status_t Central_CancelRssi(uint16_t connHandle)
{
    uint8_t connIndex = Central_getConnIndex(connHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    CENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // If already running
    if (connList[connIndex].pRssiClock == NULL)
    {
        return bleIncorrectMode;
    }

    // Stop timer
    Util_stopClock(connList[connIndex].pRssiClock);

    // Destroy the clock object
    Clock_destruct(connList[connIndex].pRssiClock);

    // Free clock struct
    ICall_free(connList[connIndex].pRssiClock);
    connList[connIndex].pRssiClock = NULL;

    return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleCentral_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* SimpleCentral_getConnAddrStr(uint16_t connHandle)
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
 * @fn      Central_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t Central_removeConnInfo(uint16_t connHandle)
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

/*********************************************************************
 * @fn      connectDevice
 *
 * @brief   Establish a link to a peer device
 *
 * @param   pAddr - 6 byte array of device address
 *
 * @return  always true
 */
static bool connectDevice(uint8_t *pAddr)
{
    GapInit_connect(PEER_ADDRTYPE_PUBLIC_OR_PUBLIC_ID, // TODO:  & MASK_ADDRTYPE_ID
            pAddr, DEFAULT_INIT_PHY, 0);

    Log_info1("Connecting to %s", (uintptr_t )Util_convertBdAddr2Str(pAddr));

    return (true);
}

/*********************************************************************
 * @fn      connectScannedDevice
 *
 * @brief   Establish a link to a peer device
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
static bool connectScannedDevice(uint8_t index)
{
    GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                    scanList[index].addr, DEFAULT_INIT_PHY, 0);

    Log_info1("Connecting to %s...",
              (uintptr_t )Util_convertBdAddr2Str(scanList[index].addr));

    return (true);
}

/*********************************************************************
 * @fn      Central_GattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool Central_GattRead(uint8_t index)
{
    Log_warning0("Gatt read called");
    attReadReq_t req;
    uint8_t connIndex = Central_getConnIndex(connList[0].connHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    CENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[0].charHandle;
    GATT_ReadCharValue(connList[0].connHandle, &req, selfEntity);

    return (true);
}


static void Central_processIpcHubReq(pkgDataCentral_t *ipcMsg)
{
    bStatus_t cmdResult;
    uint16_t connHandle;

    switch (ipcMsg->msg)
    {
    case CENTRAL_MSG_DISCOVER_DEVICES:
        GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
        break;

    case CENTRAL_MSG_STOP_DEVICES_DISCOVER:
        GapScan_disable();
        break;

    case CENTRAL_MSG_CONNECT_DEVICE:
        connectDevice(((cmdReqDataConnectDevice_t*)ipcMsg->data)->addr);
        break;

    case CENTRAL_MSG_DISCONNECT_DEVICE:
        connHandle = ((cmdDataDisconnectDevice_t *)ipcMsg->data)->conn_handle;
        cmdResult = GAP_TerminateLinkReq(connHandle,
                                         HCI_DISCONNECT_REMOTE_USER_TERM);

        Log_info1("Disconnect device request result: %d", cmdResult);

        break;

    default:
        break;
    }
}

static void Central_processIpcPeripheryReq(pkgDataPeriphery_t *ipcMsg)
{
//    bStatus_t cmdResult;
//    uint16_t connHandle;
    uint16_t connHandle = ipcMsg->conn_mask;

    switch (ipcMsg->msg)
    {
    case PERIPHERY_MSG_READ:
    {
        attAttrType_t attr_type = {.len = UUID_DATA_LEN };
        memcpy(attr_type.uuid, ipcMsg->uuid, sizeof(ipcMsg->uuid));
        attReadByTypeReq_t characteristic = {1,65535, attr_type};

        if (GATT_ReadUsingCharUUID(connHandle,
                                   (attReadByTypeReq_t* )&characteristic,
                                   selfEntity) != SUCCESS)
        {
            Log_error1("%s: peripheral read request failed to perform",
                       (uintptr_t )__func__);
        }



        break;
    }


    case PERIPHERY_MSG_WRITE:
        break;

    default:
        break;
    }
}

