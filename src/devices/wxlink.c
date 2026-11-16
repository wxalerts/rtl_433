/** @file
    WxAlerts WxLink Stevenson screen node (BME680) decoder.
*/
/** @fn int wxlink_decode(r_device *decoder, bitbuffer_t *bitbuffer)
WxAlerts WxLink, a custom OOK node with a BME680 and a battery, 433.92 MHz.

Every bit is one 400 us pulse followed by a gap. Pulse plus gap is 800 us for a
0 and 1400 us for a 1, so the gaps are nominally 400 us (0) and 1000 us (1).
The slicer measures the gap only, never the pulse width.

The train is:

- 12 preamble pulses, 400 us pulse and 400 us gap (a row of twelve 0 bits)
- one sync pulse followed by a 2400 us gap (starts a new row)
- 22 bytes, 176 bits, MSB first
- one closing pulse, which gives the last bit a measurable gap

The train is sent twice per reading, every 300 s.

The 22 bytes are a packed little-endian struct of 21 bytes, then CRC-8:

    VV II SSSS TTTT HHHH PPPPPPPP QQQQ BBBB CCCC A MMMM KK

- V: version, u8, currently 1
- I: node id, u8
- S: sequence number, u16
- T: temperature in 0.01 C, i16
- H: relative humidity in 0.01 %, u16
- P: pressure in Pa, u32
- Q: BSEC IAQ index times 10, u16
- B: breath VOC equivalent in 0.01 ppm, u16
- C: CO2 equivalent in ppm, u16
- A: IAQ accuracy 0..3, u8
- M: battery voltage in mV, u16
- K: CRC-8, polynomial 0x07, init 0x00, over the first 21 bytes

The sender is the firmware in wx433-esp32 (lib/wxlink).
*/

#include "decoder.h"

#define WXLINK_VERSION 1
#define WXLINK_FRAME_BYTES 21
#define WXLINK_WIRE_BITS ((WXLINK_FRAME_BYTES + 1) * 8)

static int wxlink_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int best_fail = DECODE_ABORT_LENGTH;

    for (int row = 0; row < bitbuffer->num_rows; ++row) {
        // The preamble row has twelve bits and is skipped here. Allow one
        // extra trailing bit from the closing pulse.
        unsigned bits = bitbuffer->bits_per_row[row];
        if (bits < WXLINK_WIRE_BITS || bits > WXLINK_WIRE_BITS + 1)
            continue;

        uint8_t *b = bitbuffer->bb[row];

        if (b[0] != WXLINK_VERSION) {
            best_fail = DECODE_FAIL_SANITY;
            continue;
        }

        if (crc8(b, WXLINK_FRAME_BYTES, 0x07, 0x00) != b[WXLINK_FRAME_BYTES]) {
            best_fail = DECODE_FAIL_MIC;
            continue;
        }

        int id          = b[1];
        int seq         = b[2] | (b[3] << 8);
        int temp_raw    = (int16_t)(b[4] | (b[5] << 8));
        int hum_raw     = b[6] | (b[7] << 8);
        uint32_t pa     = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24);
        int iaq_raw     = b[12] | (b[13] << 8);
        int bvoc_raw    = b[14] | (b[15] << 8);
        int co2         = b[16] | (b[17] << 8);
        int iaq_acc     = b[18];
        int vbat_mv     = b[19] | (b[20] << 8);

        /* clang-format off */
        data_t *data = data_make(
                "model",         "",            DATA_STRING, "WxAlerts-WxLink",
                "id",            "Node ID",     DATA_INT,    id,
                "seq",           "Sequence",    DATA_INT,    seq,
                "temperature_C", "Temperature", DATA_FORMAT, "%.2f C",   DATA_DOUBLE, temp_raw / 100.0,
                "humidity",      "Humidity",    DATA_FORMAT, "%.2f %%",  DATA_DOUBLE, hum_raw / 100.0,
                "pressure_hPa",  "Pressure",    DATA_FORMAT, "%.2f hPa", DATA_DOUBLE, pa / 100.0,
                "iaq",           "IAQ",         DATA_FORMAT, "%.1f",     DATA_DOUBLE, iaq_raw / 10.0,
                "iaq_accuracy",  "IAQ accuracy", DATA_INT,   iaq_acc,
                "bvoc_ppm",      "bVOC",        DATA_FORMAT, "%.2f ppm", DATA_DOUBLE, bvoc_raw / 100.0,
                "co2_ppm",       "CO2",         DATA_FORMAT, "%d ppm",   DATA_INT,    co2,
                "battery_mV",    "Battery",     DATA_FORMAT, "%d mV",    DATA_INT,    vbat_mv,
                "mic",           "Integrity",   DATA_STRING, "CRC",
                NULL);
        /* clang-format on */

        decoder_output_data(decoder, data);
        return 1;
    }

    return best_fail;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "seq",
        "temperature_C",
        "humidity",
        "pressure_hPa",
        "iaq",
        "iaq_accuracy",
        "bvoc_ppm",
        "co2_ppm",
        "battery_mV",
        "mic",
        NULL,
};

// The PPM slicer measures the GAP (not the pulse, not pulse plus gap), see
// pulse_slicer_ppm() in src/pulse_slicer.c. With tolerance 0 a gap below
// (short_width + long_width) / 2 is a 0 and a gap up to gap_limit is a 1.
// short_width 400 and long_width 1000 are the nominal gaps, the split is at
// 700 us. gap_limit 1800 sits between the 1000 us bit gap and the 2400 us sync
// gap, so the sync gap starts a new row. reset_limit 5000 is above the sync gap
// and below the silence between transmissions.
r_device const wxlink = {
        .name        = "WxAlerts WxLink Stevenson screen node (BME680)",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 400,
        .long_width  = 1000,
        .gap_limit   = 1800,
        .reset_limit = 5000,
        .decode_fn   = &wxlink_decode,
        .fields      = output_fields,
};
