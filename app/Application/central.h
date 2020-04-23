/*
 * central.h
 *
 *  Created on: 22 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_CENTRAL_CENTRAL_H_
#define APPLICATION_CENTRAL_CENTRAL_H_
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan results.
// Note: this value cannot be greater than the number of items reserved in
// scMenuConnect (See simple_central_menu.c)
// This cannot exceed 27 (two-button menu's constraint)
#define DEFAULT_MAX_SCAN_RES                 8


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple Central.
 */
extern void Central_createTask(void);
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);


#endif /* APPLICATION_CENTRAL_CENTRAL_H_ */
