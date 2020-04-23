/*
 * uart_handler.c
 *
 *  Created on: 22 квіт. 2020 р.
 *      Author: Oleh
 */
/*******************************************************************************
 * INCLUDES
 */
#include "uart_handler.h"
#include <Board.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define UART_TASK_PRIORITY                      (1)

#ifndef UART_TASK_STACK_SIZE
#define UART_TASK_STACK_SIZE                    (512)
#endif

#define UART_HANDLER_BAUD_RATE                  (9600)

#define UART_MAX_BUF_SIZE                       (1)


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
static UART_Handle uart;



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void UartHandler_init()
{
    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode    = UART_DATA_BINARY;
    uartParams.readDataMode     = UART_DATA_BINARY;
    uartParams.writeMode        = UART_MODE_BLOCKING;
    uartParams.readMode         = UART_MODE_BLOCKING;
//    uartParams.writeCallback    = &uartWriteCallback;
//    uartParams.readCallback     = &uartReadCallback;
    uartParams.readReturnMode   = UART_RETURN_NEWLINE;
    uartParams.readEcho         = UART_ECHO_OFF;
    uartParams.baudRate         = UART_HANDLER_BAUD_RATE; // TODO

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL)
    {
        System_abort("Error opening the UART");
    }
}

static void UartHandler_Task()
{
    static uint8_t input[UART_MAX_BUF_SIZE];

    int32_t uart_status;

    for (;;)
    {
        uart_status = UART_read(uart, &input, UART_MAX_BUF_SIZE);
        UART_write(uart, &input, UART_MAX_BUF_SIZE);
    }
}

static void uartHandlerFxn(UArg arg0, UArg arg1)
{
    UartHandler_init();

    const char echoPrompt[] = "\fTesting:\r\n";
    UART_write(uart, echoPrompt, sizeof(echoPrompt));

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
