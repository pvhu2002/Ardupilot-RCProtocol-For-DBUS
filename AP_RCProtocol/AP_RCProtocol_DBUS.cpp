#include "AP_RCProtocol_DBUS.h"
#define DBUS_FRAME_SIZE		18
#define DBUS_INPUT_CHANNELS	6
// #define SBUS_FLAGS_BYTE		23
// #define SBUS_FAILSAFE_BIT	3
// #define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 364
#define SBUS_RANGE_MAX 1684
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000
#define SBUS_TARGET_MAX 2000
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))

#ifndef HAL_SBUS_FRAME_GAP
#define HAL_SBUS_FRAME_GAP 2000U
#endif

// constructor
AP_RCProtocol_DBUS::AP_RCProtocol_DBUS(AP_RCProtocol &_frontend, bool _inverted, uint32_t configured_baud) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted),
    ss{configured_baud, SoftSerial::SERIAL_CONFIG_8E1I}
{
    
}

// decode a full SBUS frame
bool AP_RCProtocol_DBUS::dbus_decode(const uint8_t frame[18], uint16_t *values, uint16_t *num_values,
                                     bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
#define CHANNEL_SCALE(x) ((int32_t(x) * SBUS_TARGET_RANGE) / SBUS_RANGE_RANGE + SBUS_SCALE_OFFSET)
    uint16_t chancount = DBUS_INPUT_CHANNELS;

    // decode_11bit_channels((const uint8_t*)(&frame[1]), DBUS_INPUT_CHANNELS, values,
    //     SBUS_TARGET_RANGE, SBUS_RANGE_RANGE, SBUS_SCALE_OFFSET);

    /* decode switch channels if data fields are wide enough */
    // if (max_values > 17 && DBUS_INPUT_CHANNELS > 15) {
    //     chancount = 18;

    //     /* channel 17 (index 16) */
    //     values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0))?1998:998;
    //     /* channel 18 (index 17) */
    //     values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1))?1998:998;
    // }

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

     // channel 6 sw 2 (index 5)
    values[5] = (frame[5] >> 4) & 0x0003;
    values[5] = SBUS_TARGET_MIN + (values[5] - 1) * SBUS_TARGET_RANGE / 2;

    /* note the number of channels decoded */
    *num_values = chancount;

    /* decode and handle failsafe and frame-lost flags */
    // if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
    //     /* report that we failed to read anything valid off the receiver */
    //     *sbus_failsafe = true;
    //     *sbus_frame_drop = true;
    // } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
    //     /* set a special warning flag
    //      *
    //      * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
    //      * condition as fail-safe greatly reduces the reliability and range of the radio link,
    //      * e.g. by prematurely issuing return-to-launch!!! */

    //     *sbus_failsafe = false;
    //     *sbus_frame_drop = true;
    // } else {
    //     *sbus_failsafe = false;
    //     *sbus_frame_drop = false;
    // }

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
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= HAL_SBUS_FRAME_GAP);
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
        log_data(AP_RCProtocol::DBUS, timestamp_us, byte_input.buf, byte_input.ofs);
        uint16_t values[DBUS_INPUT_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe = false;
        bool sbus_frame_drop = false;
        if (dbus_decode(byte_input.buf, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop, DBUS_INPUT_CHANNELS) &&
            num_values >= MIN_RCIN_CHANNELS) {
            add_input(num_values, values, sbus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_DBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    // note that if we're here we're not actually using SoftSerial,
    // but it does record our configured baud rate:
    if (baudrate != 100000) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}