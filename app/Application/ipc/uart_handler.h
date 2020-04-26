/*
 * uart_handler.h
 *
 *  Created on: 22 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_CENTRAL_UART_HANDLER_H_
#define APPLICATION_CENTRAL_UART_HANDLER_H_


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>

#include <ti/drivers/UART.h>
#include <xdc/runtime/System.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/
extern UART_Handle uartHandle;

/*********************************************************************
 * CONSTANTS
 */



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
void UartHandler_init(void);
void UartHandler_createTask(void);




#endif /* APPLICATION_CENTRAL_UART_HANDLER_H_ */
