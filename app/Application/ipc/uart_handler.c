/*
 * uart_handler.c
 *
 *  Created on: 22 квіт. 2020 р.
 *      Author: Oleh
 */
#define TEST    (1)

/*******************************************************************************
 * INCLUDES
 */
#include <Board.h>
#include "uart_handler.h"
#include "msg_handler.h"
#include "central.h" // TODO: move queue tools to util.c

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

#if TEST
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Error.h>
#endif
/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define UART_TASK_PRIORITY                      (1)

#ifndef UART_TASK_STACK_SIZE
#define UART_TASK_STACK_SIZE                    (512)
#endif

#define UART_HANDLER_BAUD_RATE                  (9600)  /* TODO: increase to 115200 */

#define IPC_MSG_SIZE                            (sizeof(serialProtoMsg_t))


/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task configuration
Task_Struct uartTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(uartTaskStack, 8)
#else
#pragma data_alignment=8
#endif


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t uartTaskStack[UART_TASK_STACK_SIZE];


// Uart configuration
static UART_Params uartParams;
static UART_Handle uartHandle;



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#if 0
void uartReadCallback(UART_Handle handle, void *buffer, size_t num)
{
    UART_write(uartHandle, buffer, num);
    UART_read(uartHandle, &input, UART_MAX_BUF_SIZE);
}
#endif

void UartHandler_init()
{
    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);

    /* Establist uart TX/RX parameters */
    {
        uartParams.baudRate = UART_HANDLER_BAUD_RATE;
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.writeMode = UART_MODE_BLOCKING;
        uartParams.readMode = UART_MODE_BLOCKING;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
    }

    uartHandle = UART_open(Board_UART0, &uartParams);

    if (uartHandle == NULL)
    {
        System_abort("Error opening the UART");
    }
}

static void UartHandler_Task()
{
    uint8_t rxBuffer[IPC_MSG_SIZE] = { 0 };
//    serialProtoMsg_t

    int32_t uart_status;

#if 0
    Semaphore_Handle semaphore0;
    Semaphore_Params semaphoreParams;
    Error_Block eb = { 0 };

    Semaphore_Params_init(&semaphoreParams);
    semaphore0 = Semaphore_create(0, &semaphoreParams, &eb);

#endif
    uint8_t response[IPC_MSG_SIZE + 2];
    response[0] = response[IPC_MSG_SIZE + 1] = '\n';

    for (;;)
    {
//        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        uart_status = UART_read(uartHandle, &rxBuffer, IPC_MSG_SIZE);

        if (uart_status != UART_STATUS_ERROR)
        {
            /* Decode and send here an event to application */
            process_rx_ipc_msg(rxBuffer, IPC_MSG_SIZE);
        }

        // debug echo
        memcpy(&response[1], &rxBuffer, IPC_MSG_SIZE);
        UART_write(uartHandle, &rxBuffer, IPC_MSG_SIZE);
    }
}

static void uartHandlerFxn(UArg arg0, UArg arg1)
{
    UartHandler_init();


#if TEST
    const char echoPrompt[] = "\fTesting:\r\n";
    UART_write(uartHandle, echoPrompt, sizeof(echoPrompt));
#endif

    UartHandler_Task();
}



/*********************************************************************
 * @fn      UartHandler_createTask
 *
 * @brief   Task creation function for UART TX/RX handle.
 */
void UartHandler_createTask(void)
{
    Task_Params taskParams;

    /* Construct UART parser Task  thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = UART_TASK_STACK_SIZE;
    taskParams.stack = &uartTaskStack;
    taskParams.priority = UART_TASK_PRIORITY;

    Task_construct(&uartTask, (Task_FuncPtr)uartHandlerFxn, &taskParams, NULL);
}
