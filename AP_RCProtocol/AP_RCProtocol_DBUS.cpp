/*
  DJI DBUS decoder, based on src/modules/px4iofirmware/sbus.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
  modified for adapted in DJI DBUS by Xianhao Ji, DUT Robomaster Team
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol_DBUS.h"
//#include "GCS_MAVLink/GCS.h"

#define SBUS_FRAME_SIZE		18
#define SBUS_INPUT_CHANNELS	7
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 364
#define SBUS_RANGE_MAX 1684
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
};

// constructor
AP_RCProtocol_DBUS::AP_RCProtocol_DBUS(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

// decode a full SBUS frame
bool AP_RCProtocol_DBUS::sbus_decode(const uint8_t frame[18], uint16_t *values, uint16_t *num_values,
                                     bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
#define CHANNEL_SCALE(x) ((int32_t(x) * SBUS_TARGET_RANGE) / SBUS_RANGE_RANGE + SBUS_SCALE_OFFSET)
    uint16_t chancount = SBUS_INPUT_CHANNELS;

        // channel 1 (index 0)
    values[0] = (frame[0] | frame[1] << 8) & 0x07FF;
    values[0] = CHANNEL_SCALE(values[0]);

    // channel 2 (index 1)
    values[1] = (frame[1] >> 3 | frame[2] << 5) & 0x07FF;
    values[1] = CHANNEL_SCALE(values[1]);
    values[1] = SBUS_TARGET_MAX + SBUS_TARGET_MIN - values[1];

    // channel 3 (index 2)
    values[3] = (frame[2] >> 6 | frame[3] << 2 | frame[4] << 10) & 0x07FF;
    values[3] = CHANNEL_SCALE(values[3]);

    // channel 4 (index 3)
    values[2] = (frame[4] >> 1 | frame[5] << 7) & 0x07FF;
    values[2] = CHANNEL_SCALE(values[2]);

    // channel 5 sw 1 (index 4)
    values[4] = ((frame[5] >> 4) & 0x000C) >> 2;
    values[4] = SBUS_TARGET_MIN + (values[4] - 1) * SBUS_TARGET_RANGE / 2;

    values[5] = 0;
    
     // channel 7 sw 2 (index 6)
    values[6] = (frame[5] >> 4) & 0x0003;
    values[6] = SBUS_TARGET_MIN + (values[6] - 1) * SBUS_TARGET_RANGE / 2;

    /* note the number of channels decoded */
    *num_values = chancount;

    return true;
}


/*
  process a SBUS input pulse of the given width
 */
void AP_RCProtocol_DBUS::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_DBUS::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "%x", b);
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);
    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
    }
    // if (b != 0x0F && byte_input.ofs == 0) {
    //     // definately not SBUS, missing header byte
    //     return;
    // }
    if (byte_input.ofs == 0 && !have_frame_gap) {
        // must have a frame gap before the start of a new SBUS frame
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        uint16_t values[SBUS_INPUT_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe = false;
        bool sbus_frame_drop = false;
        if (sbus_decode(byte_input.buf, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop, SBUS_INPUT_CHANNELS) &&
            num_values >= MIN_RCIN_CHANNELS) {
            add_input(num_values, values, sbus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_DBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 100000) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}
