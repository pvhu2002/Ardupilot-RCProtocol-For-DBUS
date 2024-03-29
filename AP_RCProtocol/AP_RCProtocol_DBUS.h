/*
    modified for adapted in DJI DBUS by Xianhao Ji, DUT Robomaster Team
 */
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
 * Modified by pvhu
 */

#pragma once

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

class AP_RCProtocol_DBUS : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_DBUS(AP_RCProtocol &_frontend, bool inverted);
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    bool sbus_decode(const uint8_t frame[18], uint16_t *values, uint16_t *num_values,
                     bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values);

    bool inverted;
    SoftSerial ss{100000, SoftSerial::SERIAL_CONFIG_8E1I};
    uint32_t saved_width;

    struct {
        uint8_t buf[18];
        uint8_t ofs;
        uint32_t last_byte_us;
    } byte_input;
};
