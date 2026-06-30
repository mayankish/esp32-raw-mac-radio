/*
 * data_contract.h
 *
 * Canonical C implementation of the lidar-bot-system wire data contract,
 * for the ESP32 side. This is a byte-for-byte copy of
 * stm32-lidar-firmware's Inc/data_contract.h -- the two are NOT linked or
 * shared via a build-system include path, by design (see this project's
 * README: each project folder is meant to be independently `git
 * init`-able as its own repo, so each carries its own copy of this file
 * rather than reaching across project boundaries).
 *
 * This copy is duplicated verbatim once more into bot-radio/main/ and
 * base-radio/main/ for the same reason (each ESP-IDF project must build
 * standalone). If you ever change the wire format, you must update all
 * four C/Kotlin/Python copies plus DATA_CONTRACT.md -- see that file's
 * "keeping this in sync" note.
 *
 * See DATA_CONTRACT.md for the authoritative, prose description of the
 * wire format, including the documented resolution of an early ambiguity
 * in the spec (frame is 16 bytes on the wire: 14-byte struct + 2-byte
 * trailing CRC16 -- read the "Resolved ambiguity" note there before
 * touching this file).
 *
 * ---------------------------------------------------------------------
 * FRAME LAYOUT (14-byte struct, matches struct lb_frame_t exactly,
 * packed, no compiler-inserted padding):
 *
 *   offset  size  field      notes
 *   0       1     sof        0xAA telemetry / 0xAB control
 *   1       1     type       see lb_type_t
 *   2       2     seq        uint16, little-endian, increments per packet
 *   4       10    payload    interpretation depends on `type`
 *
 * WIRE ENCODING (16 bytes total, what actually goes over UART / radio / UDP):
 *
 *   bytes 0-13   the 14-byte struct above, raw
 *   bytes 14-15  crc16, BIG-ENDIAN (high byte first), computed over bytes 0-13
 *
 * CRC16 SPEC: CRC-16/CCITT-FALSE
 *   poly   = 0x1021
 *   init   = 0xFFFF
 *   refin  = false   (process input MSB-first, no bit reversal)
 *   refout = false   (no output bit reversal)
 *   xorout = 0x0000
 *
 * Mismatched CRC parameters (or byte order of the 2-byte CRC trailer) is
 * the single most likely silent bug in this whole system -- if telemetry
 * looks garbled or every packet fails CRC, check this file against the
 * other three implementations byte-for-byte before anything else.
 * ---------------------------------------------------------------------
 */

#ifndef DATA_CONTRACT_H
#define DATA_CONTRACT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LB_FRAME_LEN   14u   /* sof+type+seq+payload, no CRC */
#define LB_WIRE_LEN    16u   /* LB_FRAME_LEN + 2-byte CRC trailer */
#define LB_PAYLOAD_LEN 10u

#define LB_SOF_TELEMETRY 0xAAu
#define LB_SOF_CONTROL   0xABu

typedef enum {
    LB_TYPE_SCAN_SAMPLE    = 0x01,
    LB_TYPE_SCAN_COMPLETE  = 0x02,
    LB_TYPE_HEALTH_STATUS  = 0x03,
    LB_TYPE_CONTROL_COMMAND = 0x10,
    LB_TYPE_CONTROL_ACK    = 0x11,
} lb_type_t;

typedef enum {
    LB_CMD_START_SCAN      = 0x01,
    LB_CMD_STOP_SCAN       = 0x02,
    LB_CMD_SET_SWEEP_RANGE = 0x03,
    LB_CMD_PING            = 0x04,
} lb_cmd_id_t;

/* Raw wire-format frame. Packed attribute is load-bearing -- see the
 * matching note in the STM32 copy of this header. */
typedef struct __attribute__((packed)) {
    uint8_t  sof;
    uint8_t  type;
    uint16_t seq;                 /* host-endian once unpacked */
    uint8_t  payload[LB_PAYLOAD_LEN];
} lb_frame_t;

/* ---- payload views (decoded, host-endian) ---- */

typedef struct {
    uint16_t angle_cdeg;
    uint16_t distance_mm;   /* 0xFFFF = out of range */
    uint32_t timestamp_ms;
} lb_scan_sample_t;

typedef struct {
    uint8_t  sweep_dir;     /* 0 = fwd, 1 = rev */
    uint32_t timestamp_ms;
} lb_scan_complete_t;

typedef struct {
    uint16_t fault_flags;
    uint16_t battery_mv;
    uint32_t timestamp_ms;
} lb_health_status_t;

typedef struct {
    uint8_t  cmd_id;
    uint16_t param1;
    uint16_t param2;
    uint32_t timestamp_ms;
} lb_control_command_t;

typedef struct {
    uint8_t  cmd_id;
    uint8_t  status;        /* 0 = ok, 1 = error */
    uint32_t timestamp_ms;
} lb_control_ack_t;

/* ---- CRC16/CCITT-FALSE ---- */
uint16_t lb_crc16(const uint8_t *data, size_t len);

/* ---- pack: build a 16-byte wire buffer from a frame ----
 * `out` must point to at least LB_WIRE_LEN (16) bytes. */
void lb_pack(const lb_frame_t *frame, uint8_t out[LB_WIRE_LEN]);

/* ---- unpack: parse 16 wire bytes into a frame, verifying CRC ----
 * Returns 1 on success (CRC ok), 0 on CRC failure. On CRC failure the
 * contents of *frame are still filled in for diagnostic logging, but
 * callers must not act on them. */
int lb_unpack(const uint8_t in[LB_WIRE_LEN], lb_frame_t *frame);

/* ---- payload encode/decode helpers ---- */
void lb_encode_scan_sample(lb_frame_t *f, const lb_scan_sample_t *s);
void lb_decode_scan_sample(const lb_frame_t *f, lb_scan_sample_t *s);

void lb_encode_scan_complete(lb_frame_t *f, const lb_scan_complete_t *s);
void lb_decode_scan_complete(const lb_frame_t *f, lb_scan_complete_t *s);

void lb_encode_health_status(lb_frame_t *f, const lb_health_status_t *s);
void lb_decode_health_status(const lb_frame_t *f, lb_health_status_t *s);

void lb_encode_control_command(lb_frame_t *f, const lb_control_command_t *s);
void lb_decode_control_command(const lb_frame_t *f, lb_control_command_t *s);

void lb_encode_control_ack(lb_frame_t *f, const lb_control_ack_t *s);
void lb_decode_control_ack(const lb_frame_t *f, lb_control_ack_t *s);

#ifdef __cplusplus
}
#endif

#endif /* DATA_CONTRACT_H */
