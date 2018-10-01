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

#include <string.h>
#include <math.h>
#include "comm_can.h"
#include "ch.h"
#include "hal.h"
#include "datatypes.h"
#include "buffer.h"
#include "mc_interface.h"
#include "timeout.h"
#include "commands.h"
#include "app.h"
#include "crc.h"
#include "packet.h"

// Settings
#define CANDx			CAND1
#define RX_FRAMES_SIZE	100
#define RX_BUFFER_SIZE	packet::MAX_PL_LEN

namespace comm{
  namespace can{

    // Threads
    THD_WORKING_AREA(cancom_read_thread_wa, 512);
    THD_WORKING_AREA(cancom_process_thread_wa, 4096);
    THD_WORKING_AREA(cancom_status_thread_wa, 1024);
    THD_FUNCTION(cancom_read_thread, arg);
    THD_FUNCTION(cancom_status_thread, arg);
    THD_FUNCTION(cancom_process_thread, arg);

    // Variables
    can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
    mutex_t can_mtx;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    unsigned int rx_buffer_last_id;
    CANRxFrame rx_frames[RX_FRAMES_SIZE];
    int rx_frame_read;
    int rx_frame_write;
    thread_t *process_tp;

    /*
     * 500KBaud, automatic wakeup, automatic recover
     * from abort mode.
     * See section 22.7.7 on the STM32 reference manual.
     */
    CANConfig cancfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(3) | CAN_BTR_TS2(2) |
            CAN_BTR_TS1(9) | CAN_BTR_BRP(5)
    };

    // Private functions
    void send_packet_wrapper(unsigned char *data, unsigned int len);
    void set_timing(int brp, int ts1, int ts2);

    // Function pointers
    void(*sid_callback)(uint32_t id, uint8_t *data, uint8_t len) = 0;

    void init(void) {
        for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
            stat_msgs[i].id = -1;
        }

        rx_frame_read = 0;
        rx_frame_write = 0;

        chMtxObjectInit(&can_mtx);

        palSetPadMode(GPIOB, 8,
                PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
                PAL_STM32_OTYPE_PUSHPULL |
                PAL_STM32_OSPEED_MID1);
        palSetPadMode(GPIOB, 9,
                PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
                PAL_STM32_OTYPE_PUSHPULL |
                PAL_STM32_OSPEED_MID1);

        canStart(&CANDx, &cancfg);

        chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
                cancom_read_thread, NULL);
        chThdCreateStatic(cancom_status_thread_wa, sizeof(cancom_status_thread_wa), NORMALPRIO,
                cancom_status_thread, NULL);
        chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
                cancom_process_thread, NULL);
    }

    void set_baud(CAN_BAUD baud) {
        switch (baud) {
        case CAN_BAUD::CAN_BAUD_125K:	set_timing(15, 14, 4); break;
        case CAN_BAUD::CAN_BAUD_250K:	set_timing(7, 14, 4); break;
        case CAN_BAUD::CAN_BAUD_500K:	set_timing(5, 9, 2); break;
        case CAN_BAUD::CAN_BAUD_1M:	set_timing(2, 9, 2); break;
        default: break;
        }
    }

    THD_FUNCTION(cancom_read_thread, arg) {
        (void)arg;
        chRegSetThreadName("CAN");

        event_listener_t el;
        CANRxFrame rxmsg;

        chEvtRegister(&CANDx.rxfull_event, &el, 0);

        while(!chThdShouldTerminateX()) {
            if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
                continue;
            }

            msg_t result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

            while (result == MSG_OK) {
                rx_frames[rx_frame_write++] = rxmsg;
                if (rx_frame_write == RX_FRAMES_SIZE) {
                    rx_frame_write = 0;
                }

                chEvtSignal(process_tp, (eventmask_t) 1);

                result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
            }
        }

        chEvtUnregister(&CANDx.rxfull_event, &el);
    }

    THD_FUNCTION(cancom_process_thread, arg) {
        (void)arg;

        chRegSetThreadName("Cancom process");
        process_tp = chThdGetSelfX();

        int32_t ind = 0;
        unsigned int rxbuf_len;
        unsigned int rxbuf_ind;
        uint8_t crc_low;
        uint8_t crc_high;
        bool commands_send;

        for(;;) {
            chEvtWaitAny((eventmask_t) 1);

            while (rx_frame_read != rx_frame_write) {
                CANRxFrame rxmsg = rx_frames[rx_frame_read++];

                if (rxmsg.IDE == CAN_IDE_EXT) {
                    uint8_t id = rxmsg.EID & 0xFF;
                    CAN_PACKET_ID cmd;
                    buffer::get_enum(cmd, rxmsg.EID >> 8);
                    can_status_msg *stat_tmp;

                    using buffer::get_float32;

                    if (id == 255 || id == app::get_configuration().controller_id) {
                        switch (cmd) {
                        case CAN_PACKET_SET_DUTY:
                            ind = 0;
                            mc_interface::set_duty(get_float32(rxmsg.data8, 1e5, &ind));
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_CURRENT:
                            ind = 0;
                            mc_interface::set_current(ampere_t{get_float32(rxmsg.data8, 1e3, &ind)});
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_CURRENT_BRAKE:
                            ind = 0;
                            mc_interface::set_brake_current(ampere_t{get_float32(rxmsg.data8, 1e3, &ind)});
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_RPM:
                            ind = 0;
                            mc_interface::set_pid_speed(rpm_t{get_float32(rxmsg.data8, 1e0, &ind)});
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_POS:
                            ind = 0;
                            mc_interface::set_pid_pos(degree_t{get_float32(rxmsg.data8, 1e6, &ind)});
                            timeout::reset();
                            break;

                        case CAN_PACKET_FILL_RX_BUFFER:
                            memcpy(rx_buffer + rxmsg.data8[0], rxmsg.data8 + 1, rxmsg.DLC - 1);
                            break;

                        case CAN_PACKET_FILL_RX_BUFFER_LONG:
                            rxbuf_ind = (unsigned int)rxmsg.data8[0] << 8;
                            rxbuf_ind |= rxmsg.data8[1];
                            if (rxbuf_ind < RX_BUFFER_SIZE) {
                                memcpy(rx_buffer + rxbuf_ind, rxmsg.data8 + 2, rxmsg.DLC - 2);
                            }
                            break;

                        case CAN_PACKET_PROCESS_RX_BUFFER:
                            ind = 0;
                            rx_buffer_last_id = rxmsg.data8[ind++];
                            commands_send = rxmsg.data8[ind++];
                            rxbuf_len = (unsigned int)rxmsg.data8[ind++] << 8;
                            rxbuf_len |= (unsigned int)rxmsg.data8[ind++];

                            if (rxbuf_len > RX_BUFFER_SIZE) {
                                break;
                            }

                            crc_high = rxmsg.data8[ind++];
                            crc_low = rxmsg.data8[ind++];

                            if (crc16(rx_buffer, rxbuf_len)
                                    == ((unsigned short) crc_high << 8
                                            | (unsigned short) crc_low)) {

                                if (commands_send) {
                                    commands::send_packet(rx_buffer, rxbuf_len);
                                } else {
                                    commands::set_send_func(send_packet_wrapper);
                                    commands::process_packet(rx_buffer, rxbuf_len);
                                }
                            }
                            break;

                        case CAN_PACKET_PROCESS_SHORT_BUFFER:
                            ind = 0;
                            rx_buffer_last_id = rxmsg.data8[ind++];
                            commands_send = rxmsg.data8[ind++];

                            if (commands_send) {
                                commands::send_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
                            } else {
                                commands::set_send_func(send_packet_wrapper);
                                commands::process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
                            }
                            break;

                        case CAN_PACKET_SET_CURRENT_REL:
                            ind = 0;
                            mc_interface::set_current_rel(get_float32(rxmsg.data8, 1e5, &ind));
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_CURRENT_BRAKE_REL:
                            ind = 0;
                            mc_interface::set_brake_current_rel(get_float32(rxmsg.data8, 1e5, &ind));
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_CURRENT_HANDBRAKE:
                            ind = 0;
                            mc_interface::set_handbrake(ampere_t{get_float32(rxmsg.data8, 1e3, &ind)});
                            timeout::reset();
                            break;

                        case CAN_PACKET_SET_CURRENT_HANDBRAKE_REL:
                            ind = 0;
                            mc_interface::set_handbrake_rel(get_float32(rxmsg.data8, 1e5, &ind));
                            timeout::reset();
                            break;

                        default:
                            break;
                        }
                    }

                    switch (cmd) {
                    case CAN_PACKET_STATUS:
                        for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                            stat_tmp = &stat_msgs[i];
                            if (stat_tmp->id == id || stat_tmp->id == -1) {
                                ind = 0;
                                stat_tmp->id = id;
                                stat_tmp->rx_time = chVTGetSystemTime();
                                stat_tmp->rpm = rpm_t{(float)buffer::get_int32(rxmsg.data8, &ind)};
                                stat_tmp->current = ampere_t{buffer::get_int16(rxmsg.data8, &ind) / 10.0};
                                stat_tmp->duty = (float)buffer::get_int16(rxmsg.data8, &ind) / 1000.0;
                                break;
                            }
                        }
                        break;

                    default:
                        break;
                    }
                } else {
                    if (sid_callback) {
                        sid_callback(rxmsg.SID, rxmsg.data8, rxmsg.DLC);
                    }
                }

                if (rx_frame_read == RX_FRAMES_SIZE) {
                    rx_frame_read = 0;
                }
            }
        }
    }

    THD_FUNCTION(cancom_status_thread, arg) {
        (void)arg;
        chRegSetThreadName("CAN status");

        for(;;) {
            if (app::get_configuration().send_can_status) {
                // Send status message
                int32_t send_index = 0;
                uint8_t buffer[8];
                buffer::append_int32(buffer, (int32_t)mc_interface::get_rpm(), &send_index);
                buffer::append_int16(buffer, (int16_t)(mc_interface::get_tot_current() * 10.0), &send_index);
                buffer::append_int16(buffer, (int16_t)(mc_interface::get_duty_cycle_now() * 1000.0), &send_index);
                transmit_eid(app::get_configuration().controller_id |
                        ((uint32_t)CAN_PACKET_STATUS << 8), buffer, send_index);
            }

            systime_t sleep_time = CH_CFG_ST_FREQUENCY / app::get_configuration().send_can_status_rate_hz;
            if (sleep_time == 0) {
                sleep_time = 1;
            }

            chThdSleep(sleep_time);
        }
    }

    void transmit_eid(uint32_t id, uint8_t *data, uint8_t len) {
        if (len > 8) {
            len = 8;
        }

    #if CAN_ENABLE
        CANTxFrame txmsg;
        txmsg.IDE = CAN_IDE_EXT;
        txmsg.EID = id;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.DLC = len;
        memcpy(txmsg.data8, data, len);

        chMtxLock(&can_mtx);
        canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
        chMtxUnlock(&can_mtx);

    #else
        (void)id;
        (void)data;
        (void)len;
    #endif
    }

    void transmit_sid(uint32_t id, uint8_t *data, uint8_t len) {
        if (len > 8) {
            len = 8;
        }

    #if CAN_ENABLE
        CANTxFrame txmsg;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.SID = id;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.DLC = len;
        memcpy(txmsg.data8, data, len);

        chMtxLock(&can_mtx);
        canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
        chMtxUnlock(&can_mtx);

    #else
        (void)id;
        (void)data;
        (void)len;
    #endif
    }

    /**
     * Set function to be called when standard CAN frames are received.
     *
     * @param p_func
     * Pointer to the function.
     */
    void set_sid_rx_callback(void (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
        sid_callback = p_func;
    }

    /**
     * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
     * it will be sent in a single CAN frame, otherwise it will be split into
     * several frames.
     *
     * @param controller_id
     * The controller id to send to.
     *
     * @param data
     * The payload.
     *
     * @param len
     * The payload length.
     *
     * @param send
     * If true, this packet will be passed to the send function of commands.
     * Otherwise, it will be passed to the process function.
     */
    void send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send) {
        uint8_t send_buffer[8];

        if (len <= 6) {
            uint32_t ind = 0;
            send_buffer[ind++] = app::get_configuration().controller_id;
            send_buffer[ind++] = send;
            memcpy(send_buffer + ind, data, len);
            ind += len;
            transmit_eid(controller_id |
                    ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
        } else {
            unsigned int end_a = 0;
            for (unsigned int i = 0;i < len;i += 7) {
                if (i > 255) {
                    break;
                }

                end_a = i + 7;

                uint8_t send_len = 7;
                send_buffer[0] = i;

                if ((i + 7) <= len) {
                    memcpy(send_buffer + 1, data + i, send_len);
                } else {
                    send_len = len - i;
                    memcpy(send_buffer + 1, data + i, send_len);
                }

                transmit_eid(controller_id |
                        ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
            }

            for (unsigned int i = end_a;i < len;i += 6) {
                uint8_t send_len = 6;
                send_buffer[0] = i >> 8;
                send_buffer[1] = i & 0xFF;

                if ((i + 6) <= len) {
                    memcpy(send_buffer + 2, data + i, send_len);
                } else {
                    send_len = len - i;
                    memcpy(send_buffer + 2, data + i, send_len);
                }

                transmit_eid(controller_id |
                        ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
            }

            uint32_t ind = 0;
            send_buffer[ind++] = app::get_configuration().controller_id;
            send_buffer[ind++] = send;
            send_buffer[ind++] = len >> 8;
            send_buffer[ind++] = len & 0xFF;
            unsigned short crc = crc16(data, len);
            send_buffer[ind++] = (uint8_t)(crc >> 8);
            send_buffer[ind++] = (uint8_t)(crc & 0xFF);

            transmit_eid(controller_id |
                    ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
        }
    }

    void set_duty(uint8_t controller_id, float duty) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
    }

    void set_current(uint8_t controller_id, ampere_t current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_int32(buffer, (int32_t)static_cast<float>(current * 1000.0), &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
    }

    void set_current_brake(uint8_t controller_id, ampere_t current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_int32(buffer, (int32_t)static_cast<float>(current * 1000.0), &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
    }

    void set_rpm(uint8_t controller_id, float rpm) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_int32(buffer, (int32_t)rpm, &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
    }

    void set_pos(uint8_t controller_id, float pos) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
    }

    /**
     * Set current relative to the minimum and maximum current limits.
     *
     * @param controller_id
     * The ID of the VESC to set the current on.
     *
     * @param current_rel
     * The relative current value, range [-1.0 1.0]
     */
    void set_current_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_float32(buffer, current_rel, 1e5, &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
    }

    /**
     * Set brake current relative to the minimum current limit.
     *
     * @param controller_id
     * The ID of the VESC to set the current on.
     *
     * @param current_rel
     * The relative current value, range [0.0 1.0]
     */
    void set_current_brake_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_float32(buffer, current_rel, 1e5, &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
    }

    /**
     * Set handbrake current.
     *
     * @param controller_id
     * The ID of the VESC to set the handbrake current on.
     *
     * @param current_rel
     * The handbrake current value
     */
    void set_handbrake(uint8_t controller_id, float current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_float32(buffer, current, 1e3, &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
    }

    /**
     * Set handbrake current relative to the minimum current limit.
     *
     * @param controller_id
     * The ID of the VESC to set the handbrake current on.
     *
     * @param current_rel
     * The relative handbrake current value, range [0.0 1.0]
     */
    void set_handbrake_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer::append_float32(buffer, current_rel, 1e5, &send_index);
        transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
    }

    /**
     * Get status message by index.
     *
     * @param index
     * Index in the array
     *
     * @return
     * The message or 0 for an invalid index.
     */
    can_status_msg *get_status_msg_index(int index) {
        if (index < CAN_STATUS_MSGS_TO_STORE) {
            return &stat_msgs[index];
        } else {
            return 0;
        }
    }

    /**
     * Get status message by id.
     *
     * @param id
     * Id of the controller that sent the status message.
     *
     * @return
     * The message or 0 for an invalid id.
     */
    can_status_msg *get_status_msg_id(int id) {
        for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
            if (stat_msgs[i].id == id) {
                return &stat_msgs[i];
            }
        }

        return 0;
    }

    void send_packet_wrapper(unsigned char *data, unsigned int len) {
        send_buffer(rx_buffer_last_id, data, len, true);
    }

    void set_timing(int brp, int ts1, int ts2) {
        brp &= 0b1111111111;
        ts1 &= 0b1111;
        ts2 &= 0b111;

        cancfg.btr = CAN_BTR_SJW(3) | CAN_BTR_TS2(ts2) |
            CAN_BTR_TS1(ts1) | CAN_BTR_BRP(brp);

        canStop(&CANDx);
        canStart(&CANDx, &cancfg);
    }
  }
}
