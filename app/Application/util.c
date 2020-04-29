/******************************************************************************

 @file  util.c

 @brief This file contains utility functions commonly used by
 BLE applications for CC26xx with TIRTOS.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2014-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
 its contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/hal/Hwi.h>

#ifdef USE_ICALL
#include <icall.h>
#else
#include <stdlib.h>
#endif

#include "bcomdef.h"
#include "util.h"
#include "ipc/uart_handler.h"
#include <uartlog/UartLog.h>

/*********************************************************************
 * TYPEDEFS
 */

// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
    Queue_Elem _elem;          // queue element
    uint8_t *pData;            // pointer to app data
} queueRec_t;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
ICall_SyncHandle eventSyncHandle;

/*********************************************************************
 * LOCAL VARIABLES
 */
static Queue_Handle appMsgQueue;
static Queue_Struct appMsg;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Util_constructClock
 *
 * @brief   Initialize a TIRTOS Clock instance.
 *
 * @param   pClock        - pointer to clock instance structure.
 * @param   clockCB       - callback function upon clock expiration.
 * @param   clockDuration - longevity of clock timer in milliseconds
 * @param   clockPeriod   - if set to a value other than 0, the first
 *                          expiry is determined by clockDuration.  All
 *                          subsequent expiries use the clockPeriod value.
 * @param   startFlag     - TRUE to start immediately, FALSE to wait.
 * @param   arg           - argument passed to callback function.
 *
 * @return  Clock_Handle  - a handle to the clock instance.
 */
Clock_Handle Util_constructClock(Clock_Struct *pClock,
                                 Clock_FuncPtr clockCB,
                                 uint32_t clockDuration, uint32_t clockPeriod,
                                 uint8_t startFlag, UArg arg)
{
    Clock_Params clockParams;

    // Convert clockDuration in milliseconds to ticks.
    uint32_t clockTicks = clockDuration * (1000 / Clock_tickPeriod);

    // Setup parameters.
    Clock_Params_init(&clockParams);

    // Setup argument.
    clockParams.arg = arg;

    // If period is 0, this is a one-shot timer.
    clockParams.period = clockPeriod * (1000 / Clock_tickPeriod);

    // Starts immediately after construction if true, otherwise wait for a call
    // to start.
    clockParams.startFlag = startFlag;

    // Initialize clock instance.
    Clock_construct(pClock, clockCB, clockTicks, &clockParams);

    return Clock_handle(pClock);
}

/*********************************************************************
 * @fn      Util_startClock
 *
 * @brief   Start a clock.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  none
 */
void Util_startClock(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Start clock instance
    Clock_start(handle);
}

/*********************************************************************
 * @fn      Util_restartClock
 *
 * @brief   Restart a clock by changing the timeout.
 *
 * @param   pClock - pointer to clock struct
 * @param   clockTimeout - longevity of clock timer in milliseconds
 *
 * @return  none
 */
void Util_restartClock(Clock_Struct *pClock, uint32_t clockTimeout)
{
    uint32_t clockTicks;
    Clock_Handle handle;

    handle = Clock_handle(pClock);

    if (Clock_isActive(handle))
    {
        // Stop clock first
        Clock_stop(handle);
    }

    // Convert timeout in milliseconds to ticks.
    clockTicks = clockTimeout * (1000 / Clock_tickPeriod);

    // Set the initial timeout
    Clock_setTimeout(handle, clockTicks);

    // Start clock instance
    Clock_start(handle);
}

/*********************************************************************
 * @fn      Util_isActive
 *
 * @brief   Determine if a clock is currently active.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  TRUE if Clock is currently active
 FALSE otherwise
 */
bool Util_isActive(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Start clock instance
    return Clock_isActive(handle);
}

/*********************************************************************
 * @fn      Util_stopClock
 *
 * @brief   Stop a clock.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  none
 */
void Util_stopClock(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Stop clock instance
    Clock_stop(handle);
}

/*********************************************************************
 * @fn      Util_rescheduleClock
 *
 * @brief   Reschedule a clock by changing the timeout and period values.
 *
 * @param   pClock - pointer to clock struct
 * @param   clockPeriod - longevity of clock timer in milliseconds
 * @return  none
 */
void Util_rescheduleClock(Clock_Struct *pClock, uint32_t clockPeriod)
{
    bool running;
    uint32_t clockTicks;
    Clock_Handle handle;

    handle = Clock_handle(pClock);
    running = Clock_isActive(handle);

    if (running)
    {
        Clock_stop(handle);
    }

    // Convert period in milliseconds to ticks.
    clockTicks = clockPeriod * (1000 / Clock_tickPeriod);

    Clock_setTimeout(handle, clockTicks);
    Clock_setPeriod(handle, clockTicks);

    if (running)
    {
        Clock_start(handle);
    }
}

/*********************************************************************
 * @fn      Util_constructQueue
 *
 * @brief   Initialize an RTOS queue to hold messages to be processed.
 *
 * @return  A queue handle.
 */
void Util_constructQueue()
{
    // Construct a Queue instance.
    Queue_construct(&appMsg, NULL);

    appMsgQueue = Queue_handle(&appMsg);
}

/*********************************************************************
 * @fn      Util_putMsg2Queue
 *
 * @brief   Creates a queue node and puts the node in RTOS queue.
 *
 * @param   msgQueue - queue handle.
 * @param   pMsg - pointer to message to be queued
 *
 * @return  TRUE if message was queued, FALSE otherwise.
 */
static uint8_t Util_putQueueMsg(Queue_Handle msgQueue,
                                Event_Handle eventSyncHandle, uint8_t *pMsg)
{
    queueRec_t *pRec;

    // Allocated space for queue node.
#ifdef USE_ICALL
    if ((pRec = ICall_malloc(sizeof(queueRec_t))))
#else
    if ((pRec = (queueRec_t *)malloc(sizeof(queueRec_t))))
#endif
    {
        pRec->pData = pMsg;

        // This is an atomic operation
        Queue_put(msgQueue, &pRec->_elem);

        // Wake up the application thread event handler.
        if (eventSyncHandle)
        {
            Event_post(eventSyncHandle, UTIL_QUEUE_EVENT_ID);
        }

        return TRUE;
    }

    // Free the message.
#ifdef USE_ICALL
    ICall_free(pMsg);
#else
    free(pMsg);
#endif

    return FALSE;
}

/*********************************************************************
 * @fn      Util_getQueueMsg
 *
 * @brief   Dequeues the message from the RTOS queue.
 *
 * @param   msgQueue - queue handle.
 *
 * @return  pointer to dequeued message, NULL otherwise.
 */
static uint8_t *Util_getQueueMsg(Queue_Handle msgQueue)
{
    queueRec_t *pRec = Queue_get(msgQueue);

    if (pRec != (queueRec_t *) msgQueue)
    {
        uint8_t *pData = pRec->pData;

        // Free the queue node
        // Note:  this does not free space allocated by data within the node.
#ifdef USE_ICALL
        ICall_free(pRec);
#else
        free(pRec);
#endif

        return pData;
    }

    return NULL;
}

/*********************************************************************
 * @fn      Util_listenEventMsg
 *
 * @brief   Listen t.
 *
 * @param   msgQueue - queue handle.
 *
 * @return  pointer to dequeued message, NULL otherwise.
 */
uint32_t Util_listenEventMsg(uint32_t event_mask)
{
    uint32_t received_events;

    received_events = Event_pend(eventSyncHandle, Event_Id_NONE, event_mask,
                                 ICALL_TIMEOUT_FOREVER);
    return received_events;
}

/*********************************************************************
 * @fn      Util_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char *Util_convertBdAddr2Str(uint8_t *pAddr)
{
    uint8_t charCnt;
    char hex[] = "0123456789ABCDEF";
    static char str[(2 * B_ADDR_LEN) + 3];
    char *pStr = str;

    *pStr++ = '0';
    *pStr++ = 'x';

    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }
    *pStr = NULL;

    return str;
}


/*********************************************************************
 * @fn      Util_convertHex2Str
 *
 * @brief   Convert hex to string. Only needed when hex displayed.
 *          Function is not safe if wrong length provided!
 *
 *
 * @param   pAddr - hex array address
 *
 *          pStr - string array address
 *
 *          len - length of data to convert and it's not checked for address
 *
 * @return  BD address as a string
 */
bool Util_convertHex2Str(const uint8_t *hex_arr, uint8_t *str_arr, uint16_t hex_len,
                         uint16_t str_len)
{
    static char hex_codes[] = "0123456789ABCDEF";

    uint8_t hexCnt;
    uint8_t *pHex = hex_arr;
    uint8_t *pStr = str_arr;

    /* 2 char symbols needed to encode 1 byte hex value,
     * one more symbols is for terminal NULL
     * */
    if (2 * hex_len + 1 > str_len)
    {
        Log_error1("%s: not enough string space to place hex array",
                   (uintptr_t )__func__);
        return FALSE;
    }

    for (hexCnt = 0; hexCnt < hex_len; hexCnt++)
    {
        *pStr++ = hex_codes[*pHex >> 4];
        *pStr++ = hex_codes[*pHex & 0x0F];
        pHex++;
    }
    *pStr = NULL;

    return TRUE;
}

/*********************************************************************
 * @fn      Util_isBufSet
 *
 * @brief   Check if contents of buffer matches byte pattern.
 *
 * @param   pBuf    - buffer to check
 * @param   pattern - pattern to match
 * @param   len     - len of buffer (in bytes) to iterate over
 *
 * @return  TRUE if buffer matches the pattern, FALSE otherwise.
 */
uint8_t Util_isBufSet(uint8_t *pBuf, uint8_t pattern, uint16_t len)
{
    uint8_t result = FALSE;

    if (pBuf)
    {
        result = TRUE;

        for (uint16_t i = 0; i < len; i++)
        {
            if (pBuf[i] != pattern)
            {
                // Buffer does not match pattern.
                result = FALSE;
                break;
            }
        }
    }

    return (result);
}

/*********************************************************************
 * @fn      Util_enqueueAppMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
status_t Util_enqueueAppMsg(uint8_t event, uint8_t state,
                                         uint8_t *pData)
{
    uint8_t success;
    appEvt_t *pMsg = ICall_malloc(sizeof(appEvt_t));

    // TODO: add semaphore

    // Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        success = Util_putQueueMsg(appMsgQueue, eventSyncHandle, (uint8_t *) pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return (bleMemAllocError);
}

uint8_t *Util_dequeueAppMsg()
{
    return Util_getQueueMsg(appMsgQueue);
}

/*********************************************************************
 *********************************************************************/
