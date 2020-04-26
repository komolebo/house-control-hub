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
#include <ti/display/Display.h>
#include <Board.h>
#include "uart_handler.h"
#include "msg_handler.h"
#include "central.h" // TODO: move queue tools to util.c

/* Driver Header files */
#include <ti/drivers/GPIO.h>

#if TEST
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
//#include <xdc/runtime/Error.h>
#include "util.h"
#include <uartlog/UartLog.h>
#endif
/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define UART_TASK_PRIORITY                      (1)

#ifndef UART_TASK_STACK_SIZE
#define UART_TASK_STACK_SIZE                    (512)
#endif

#define UART_HANDLER_BAUD_RATE                  (115200)

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

static uint8_t rxBuffer[IPC_MSG_SIZE] = { 0 };

static UART_Handle uartHandle;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void UartHandler_init()
{
    // UART handle
    UART_Params uartParams;

    /* Call driver init functions */
    GPIO_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);

    /* Establish UART TX/RX parameters */
    {
        uartParams.baudRate = UART_HANDLER_BAUD_RATE;
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.writeMode = UART_MODE_BLOCKING;
        uartParams.readMode = UART_MODE_BLOCKING;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
    }

    UART_init();
    UartLog_init(uartHandle = UART_open(Board_UART0, &uartParams));

    if (uartHandle == NULL)
    {
        System_abort("Error opening the UART");
    }
}

static void UartHandler_Task()
{
    int32_t uart_status;

    for (;;)
    {
        uart_status = UART_read(uartHandle, &rxBuffer, IPC_MSG_SIZE);
        if (uart_status != UART_STATUS_ERROR)
        {
            /* Decode and send here an event to application */
            process_rx_ipc_msg(rxBuffer, IPC_MSG_SIZE);
        }

        /* clear whole buffer */
        memset(rxBuffer, 0, IPC_MSG_SIZE);
    }
}

static void uartHandlerFxn(UArg arg0, UArg arg1)
{
    /* initialize UART HW */
    UartHandler_init();

    /* task cyclic call */
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
