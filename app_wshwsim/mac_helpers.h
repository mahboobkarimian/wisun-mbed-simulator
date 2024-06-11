#ifndef MAC_HELPERS_H
#define MAC_HELPERS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Function to extract MAC addresses
void extract_mac_addresses(uint8_t *frame, uint8_t *dest_addr, uint8_t *src_addr);

#endif