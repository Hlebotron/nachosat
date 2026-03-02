#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stddef.h>
#include "definitions.h"
#include "config.h"

// Calculate CRC32 over buffer
uint32_t crc32(const uint8_t* data, size_t len);

// Serialize a packet into buffer
// Returns total packet size, or 0 on error
// Buffer must be at least PACKET_HEADER_SIZE + data_len + PACKET_CRC_SIZE bytes
size_t serialize_packet(uint8_t* buf, size_t buf_size,
                        Peripheral type, uint32_t ticks,
                        const uint8_t* data, uint8_t data_len);

#endif // PACKET_H
