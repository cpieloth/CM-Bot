/**
 * \file	remote.h
 *
 * \brief	Methoden zur Steuerung durch den RC-100 Remote Controller.
 */

#ifndef REMOTE_H_
#define REMOTE_H_

#include "datatypes.h"
#include "usart_driver.h"

DT_cmd RMT_getCommand();
DT_byte RMT_receive(USART_data_t* const, DT_byte* const);
DT_bool RMT_NonPressed(DT_cmd);
DT_bool RMT_isUpPressed(DT_cmd);
DT_bool RMT_isDownPressed(DT_cmd);
DT_bool RMT_isLeftPressed(DT_cmd);
DT_bool RMT_isRightPressed(DT_cmd);
DT_bool RMT_isButton1Pressed(DT_cmd);
DT_bool RMT_isButton2Pressed(DT_cmd);
DT_bool RMT_isButton3Pressed(DT_cmd);
DT_bool RMT_isButton4Pressed(DT_cmd);
DT_bool RMT_isButton5Pressed(DT_cmd);
DT_bool RMT_isButton6Pressed(DT_cmd);

#endif /* REMOTE_H_ */
