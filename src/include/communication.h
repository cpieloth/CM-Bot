/**
 * \file	communication.h
 *
 * \brief	Methoden zur Kommunikation der CPUs.
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "datatypes.h"
#include "usart_driver.h"

#define COM_MASTER		0x02
#define COM_SLAVE1B		0x01
#define COM_SLAVE3F		0x03
#define COM_BRDCAST_ID 	0xFE
#define COM_NOCPUID		0x00

// Instructions
#define COM_STATUS		0x01
#define COM_ACTION		0x02
#define COM_POINT		0x03
#define COM_ANGLE		0x04
#define COM_SPEED		0x05

// Status Parameter
#define COM_IS_ALIVE	0x01

// Responses
#define COM_ACK			0x06
#define COM_NAK			0x15

// NAK-Error-Codes
#define COM_ERR_ANGLE_LIMIT			0x01
#define COM_ERR_POINT_OUT_OF_BOUNDS	0x02
#define COM_ERR_DEFAULT_ERROR		0x03

// Config
#define COM_CONF_RIGHT		0x01
#define COM_CONF_LEFT		0x02
#define COM_CONF_GLOB		0x04
#define COM_CONF_HIP		0x08
#define COM_CONF_KNEE		0x10
#define COM_CONF_FOOT		0x20


DT_byte COM_send(DT_byte* const, DT_size, DT_byte* const, DT_bool);
DT_byte COM_receive(USART_data_t* const, DT_byte* const);

DT_size COM_requestStatus(DT_byte, DT_byte, DT_byte* const);
DT_bool COM_sendPoint(DT_byte, const DT_point* const, const DT_byte);
DT_bool COM_sendPointAndSpeed(DT_byte, const DT_point* const, const DT_double, const DT_byte);
DT_bool COM_sendAngle(DT_byte, const DT_double, const DT_byte);
void COM_sendAction(DT_byte);
DT_bool COM_isAlive(DT_byte);
void COM_sendACK(DT_byte);
void COM_sendNAK(DT_byte, DT_byte);

DT_byte COM_getCpuID(const DT_leg* const);
DT_double COM_byteArrayToDouble(const DT_byte* const);
void COM_doubleToByteArray(const DT_double, DT_byte* const);
DT_point COM_getPointFromPacket(const DT_byte* const);
DT_double COM_getAngleFromPacket(const DT_byte* const);
DT_double COM_getSpeedFromPacket(const DT_byte* const);
DT_bool COM_isLeftLeg(const DT_byte* const result);
DT_bool COM_isRightLeg(const DT_byte* const result);
DT_bool COM_isGlobal(const DT_byte* const result);
DT_bool COM_isHip(const DT_byte* const result);
DT_bool COM_isKnee(const DT_byte* const result);
DT_bool COM_isFoot(const DT_byte* const result);
#endif /* COMMUNICATION_H_ */
