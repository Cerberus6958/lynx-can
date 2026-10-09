#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>
#include <string.h>

// Classic CAN 2.0A frame. Fixed 8-byte payload, no heap allocation,
// so send/receive time stays deterministic.
typedef struct {
    uint32_t id;            // 11-bit standard ID (0x000 to 0x7FF); lower = higher priority
    uint8_t  dlc;           // data length, 0 to 8
    uint8_t  data[8];
    uint64_t timestamp_us;  // filled on receive
} CANframe;

// Little-endian helpers for packing test payloads
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#endif