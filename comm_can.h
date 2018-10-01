/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#pragma once

#include "conf_general.h"

// Settings
#define CAN_STATUS_MSG_INT_MS		1
#define CAN_STATUS_MSGS_TO_STORE	10

namespace comm{
  namespace can{

    // Functions
    void init(void);
    void set_baud(CAN_BAUD baud);
    void transmit_eid(uint32_t id, uint8_t *data, uint8_t len);
    void transmit_sid(uint32_t id, uint8_t *data, uint8_t len);
    void set_sid_rx_callback(void (*p_func)(uint32_t id, uint8_t *data, uint8_t len));
    void send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send);
    void set_duty(uint8_t controller_id, float duty);
    void set_current(uint8_t controller_id, ampere_t current);
    void set_current_brake(uint8_t controller_id, ampere_t current);
    void set_rpm(uint8_t controller_id, float rpm);
    void set_pos(uint8_t controller_id, float pos);
    void set_current_rel(uint8_t controller_id, float current_rel);
    void set_current_brake_rel(uint8_t controller_id, float current_rel);
    can_status_msg *get_status_msg_index(int index);
    can_status_msg *get_status_msg_id(int id);
  }
}
