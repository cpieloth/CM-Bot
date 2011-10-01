/**
 * \file	dynamixel.h
 *
 * \brief	Methoden zur Steuerung der Dynamixal AX-12.
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "datatypes.h"
#include "usart_driver.h"

#define DNX_BRDCAST_ID 0xFE

DT_byte DNX_send(DT_byte* const, DT_size, DT_byte* const, DT_bool);
DT_byte DNX_receive(USART_data_t* const, DT_byte* const);

DT_byte DNX_getChecksum(const DT_byte* const, DT_size);
DT_bool DNX_setAngle(DT_byte, DT_double, DT_bool);
DT_bool DNX_setAngleAndSpeed(DT_byte id, DT_double angle, DT_double speed, DT_bool regWrite);
void DNX_setId(DT_byte, DT_byte);
void DNX_setSpeed(DT_byte, DT_byte);
DT_bool DNX_setLed(DT_byte, DT_byte);

DT_double DNX_getAngle(DT_byte);
DT_byte DNX_getSpeed(DT_byte);
DT_byte DNX_getLed(DT_byte);
void DNX_getConnectedIDs(DT_leg* const, DT_leg* const);
void DNX_sendAction(DT_byte);

#endif /* DYNAMIXEL_H_ */
