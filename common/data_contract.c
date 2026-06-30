/*
 * data_contract.c -- ESP32-side copy, byte-for-byte identical logic to
 * stm32-lidar-firmware/Src/data_contract.c. See data_contract.h in this
 * same directory for why this is a deliberate verbatim copy rather than a
 * shared include path, and ../DATA_CONTRACT.md (repo root) for the
 * full wire-format spec. All multi-byte payload fields are little-endian
 * on the wire; the trailing CRC16 is transmitted big-endian -- documented,
 * deliberate asymmetry, not a bug.
 */

#include "data_contract.h"
#include <string.h>

/* ---------------- little-endian byte helpers ---------------- */

static inline void put_u16le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline uint16_t get_u16le(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint32_t get_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------- CRC-16/CCITT-FALSE ----------------
 * poly=0x1021 init=0xFFFF refin=false refout=false xorout=0x0000
 * Bit-by-bit (no lookup table) implementation -- intentionally simple and
 * auditable over raw throughput; at our packet rates (single digit kHz at
 * most) this is not a bottleneck on any of the targets that run this code
 * (Cortex-M4 and Linux/Android CPUs alike).
 */
uint16_t lb_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc; /* xorout = 0x0000, nothing to do */
}

void lb_pack(const lb_frame_t *frame, uint8_t out[LB_WIRE_LEN]) {
    out[0] = frame->sof;
    out[1] = frame->type;
    put_u16le(&out[2], frame->seq);
    memcpy(&out[4], frame->payload, LB_PAYLOAD_LEN);

    uint16_t crc = lb_crc16(out, LB_FRAME_LEN);
    out[14] = (uint8_t)((crc >> 8) & 0xFF); /* big-endian trailer */
    out[15] = (uint8_t)(crc & 0xFF);
}

int lb_unpack(const uint8_t in[LB_WIRE_LEN], lb_frame_t *frame) {
    frame->sof  = in[0];
    frame->type = in[1];
    frame->seq  = get_u16le(&in[2]);
    memcpy(frame->payload, &in[4], LB_PAYLOAD_LEN);

    uint16_t expected = lb_crc16(in, LB_FRAME_LEN);
    uint16_t received = (uint16_t)(((uint16_t)in[14] << 8) | in[15]);
    return expected == received;
}

/* ---------------- payload codecs ---------------- */

void lb_encode_scan_sample(lb_frame_t *f, const lb_scan_sample_t *s) {
    f->type = LB_TYPE_SCAN_SAMPLE;
    put_u16le(&f->payload[0], s->angle_cdeg);
    put_u16le(&f->payload[2], s->distance_mm);
    put_u32le(&f->payload[4], s->timestamp_ms);
    f->payload[8] = 0;
    f->payload[9] = 0;
}

void lb_decode_scan_sample(const lb_frame_t *f, lb_scan_sample_t *s) {
    s->angle_cdeg   = get_u16le(&f->payload[0]);
    s->distance_mm  = get_u16le(&f->payload[2]);
    s->timestamp_ms = get_u32le(&f->payload[4]);
}

void lb_encode_scan_complete(lb_frame_t *f, const lb_scan_complete_t *s) {
    f->type = LB_TYPE_SCAN_COMPLETE;
    f->payload[0] = s->sweep_dir;
    f->payload[1] = 0;
    put_u32le(&f->payload[2], s->timestamp_ms);
    memset(&f->payload[6], 0, 4);
}

void lb_decode_scan_complete(const lb_frame_t *f, lb_scan_complete_t *s) {
    s->sweep_dir    = f->payload[0];
    s->timestamp_ms = get_u32le(&f->payload[2]);
}

void lb_encode_health_status(lb_frame_t *f, const lb_health_status_t *s) {
    f->type = LB_TYPE_HEALTH_STATUS;
    put_u16le(&f->payload[0], s->fault_flags);
    put_u16le(&f->payload[2], s->battery_mv);
    put_u32le(&f->payload[4], s->timestamp_ms);
    f->payload[8] = 0;
    f->payload[9] = 0;
}

void lb_decode_health_status(const lb_frame_t *f, lb_health_status_t *s) {
    s->fault_flags  = get_u16le(&f->payload[0]);
    s->battery_mv   = get_u16le(&f->payload[2]);
    s->timestamp_ms = get_u32le(&f->payload[4]);
}

void lb_encode_control_command(lb_frame_t *f, const lb_control_command_t *s) {
    f->type = LB_TYPE_CONTROL_COMMAND;
    f->payload[0] = s->cmd_id;
    f->payload[1] = 0;
    put_u16le(&f->payload[2], s->param1);
    put_u16le(&f->payload[4], s->param2);
    put_u32le(&f->payload[6], s->timestamp_ms);
}

void lb_decode_control_command(const lb_frame_t *f, lb_control_command_t *s) {
    s->cmd_id       = f->payload[0];
    s->param1       = get_u16le(&f->payload[2]);
    s->param2       = get_u16le(&f->payload[4]);
    s->timestamp_ms = get_u32le(&f->payload[6]);
}

void lb_encode_control_ack(lb_frame_t *f, const lb_control_ack_t *s) {
    f->type = LB_TYPE_CONTROL_ACK;
    f->payload[0] = s->cmd_id;
    f->payload[1] = s->status;
    memset(&f->payload[2], 0, 4);
    put_u32le(&f->payload[6], s->timestamp_ms);
}

void lb_decode_control_ack(const lb_frame_t *f, lb_control_ack_t *s) {
    s->cmd_id       = f->payload[0];
    s->status       = f->payload[1];
    s->timestamp_ms = get_u32le(&f->payload[6]);
}
