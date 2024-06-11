#include "mac_helpers.h"

// Constants for addressing modes
#define ADDR_MODE_NONE 0x00
#define ADDR_MODE_SHORT 0x02
#define ADDR_MODE_LONG 0x03

// Frame Control Field positions
#define FRAME_CTRL_DEST_ADDR_MODE_OFFSET 10
#define FRAME_CTRL_SRC_ADDR_MODE_OFFSET 14
#define FRAME_CTRL_PAN_COMPRESSION_OFFSET 6
#define FRAME_CTRL_SEQ_NB_SUPPRESSION_OFFSET 8

// Function to extract MAC addresses
void extract_mac_addresses(uint8_t *frame, uint8_t *dest_addr, uint8_t *src_addr) {
    uint16_t frame_control = frame[0] | (frame[1] << 8);
    uint8_t dest_addr_mode = (frame_control >> FRAME_CTRL_DEST_ADDR_MODE_OFFSET) & 0x03;
    uint8_t src_addr_mode = (frame_control >> FRAME_CTRL_SRC_ADDR_MODE_OFFSET) & 0x03;
    uint8_t pan_compression = (frame_control >> FRAME_CTRL_PAN_COMPRESSION_OFFSET) & 0x01;
    uint8_t seq_nb_suppression = (frame_control >> FRAME_CTRL_SEQ_NB_SUPPRESSION_OFFSET) & 0x01;

    uint8_t *ptr = frame + 2; // Skip frame control field

    if (!seq_nb_suppression) {
        ptr += 1;   // Skip sequence number
        printf("A\n");
    }

    if (!pan_compression) {
        if (dest_addr_mode != ADDR_MODE_NONE) {
            ptr += 2;   // Skip dest PAN ID
            printf("B\n");
        }
    } else {
        if (dest_addr_mode == ADDR_MODE_NONE && src_addr_mode == ADDR_MODE_NONE) {
            ptr += 2;   // Skip dest PAN ID
            printf("D\n");
        }
    }

    // Extract Destination Address
    if (dest_addr_mode == ADDR_MODE_SHORT) {
        dest_addr[0] = ptr[1];
        dest_addr[1] = ptr[2];
        ptr += 2;
    } else if (dest_addr_mode == ADDR_MODE_LONG) {
        dest_addr[0] = ptr[7];
        dest_addr[1] = ptr[6];
        dest_addr[2] = ptr[5];
        dest_addr[3] = ptr[4];
        dest_addr[4] = ptr[3];
        dest_addr[5] = ptr[2];
        dest_addr[6] = ptr[1];
        dest_addr[7] = ptr[0];
        ptr += 8;
    }

    if (!pan_compression) {
        if (dest_addr_mode == ADDR_MODE_NONE && src_addr_mode != ADDR_MODE_NONE) {
            ptr += 2;   // Skip src PAN ID
            printf("E\n");
        }
    }   // We don't manage the special cases of IEEE 802.15.4-2020 Table 7-2 here as they are not used by Wi-SUN

    // Extract Source Address
    if (src_addr_mode == ADDR_MODE_SHORT) {
        src_addr[0] = ptr[1];
        src_addr[1] = ptr[2];
        ptr += 2;
    } else if (src_addr_mode == ADDR_MODE_LONG) {
        src_addr[0] = ptr[7];
        src_addr[1] = ptr[6];
        src_addr[2] = ptr[5];
        src_addr[3] = ptr[4];
        src_addr[4] = ptr[3];
        src_addr[5] = ptr[2];
        src_addr[6] = ptr[1];
        src_addr[7] = ptr[0];
        ptr += 8;
    }
}