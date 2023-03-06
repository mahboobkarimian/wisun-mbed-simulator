/*
 * Copyright (c) 2016-2021, Pelion and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "nsconfig.h"
#include <string.h>
#include <stdlib.h>
#include "mbed-client-libservice/ns_trace.h"
#include "mbed-client-libservice/common_functions.h"
#include "nanostack/mac/mac_api.h"
#include "nanostack/mac/mac_common_defines.h"
#include "nanostack/mac/mlme.h"

#include "nwk_interface/protocol.h"

#include "mac_helper.h"

#define TRACE_GROUP "MACh"

static const uint8_t mac_helper_default_key_source[8] = {0xff, 0, 0, 0, 0, 0, 0, 0};

uint16_t test_6lowpan_fragmentation_mtu_size_override = 0;

static uint8_t mac_helper_header_security_aux_header_length(uint8_t keyIdmode);
static uint8_t mac_helper_security_mic_length_get(uint8_t security_level);

static int8_t mac_helper_pib_8bit_set(protocol_interface_info_entry_t *interface, mlme_attr_t attribute, uint8_t value)
{
    switch (attribute) {
        case macAutoRequestKeyIdMode:
            interface->mac_parameters.mac_key_id_mode = value;
            break;
        case macAutoRequestKeyIndex:
            interface->mac_parameters.mac_default_key_index = value;
            break;

        case macAutoRequestSecurityLevel:
        default:
            interface->mac_parameters.mac_security_level = value;
            break;
    }

    if (interface->mac_api && interface->mac_api->mlme_req) {
        mlme_set_t set_req;
        set_req.attr = attribute;
        set_req.attr_index = 0;
        set_req.value_pointer = &value;
        set_req.value_size = 1;
        interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
    }

    return 0;
}

void mac_helper_panid_set(protocol_interface_info_entry_t *interface, uint16_t panId)
{
    interface->mac_parameters.pan_id = panId;
    mlme_set_t set_req;
    set_req.attr = macPANId;
    set_req.attr_index = 0;
    set_req.value_pointer = &panId;
    set_req.value_size = 2;
    interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
}

void mac_helper_mac16_address_set(protocol_interface_info_entry_t *interface, uint16_t mac16)
{
    interface->mac_parameters.mac_short_address = mac16;
    if (mac16 < 0xfffe) {
        interface->mac_parameters.shortAdressValid = true;
    } else {
        interface->mac_parameters.shortAdressValid = false;
    }
    mlme_set_t set_req;
    set_req.attr = macShortAddress;
    set_req.attr_index = 0;
    set_req.value_pointer = &mac16;
    set_req.value_size = 2;
    interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
}

uint16_t mac_helper_mac16_address_get(const protocol_interface_info_entry_t *interface)
{
    uint16_t shortAddress = 0xfffe;
    if (interface) {
        shortAddress = interface->mac_parameters.mac_short_address;
    }
    return shortAddress;
}

uint16_t mac_helper_panid_get(const protocol_interface_info_entry_t *interface)
{
    uint16_t panId = 0xffff;
    if (interface) {
        panId = interface->mac_parameters.pan_id;
    }
    return panId;
}

void mac_helper_default_key_index_set(protocol_interface_info_entry_t *interface, uint8_t keyIndex)
{
    interface->mac_parameters.mac_default_key_index = keyIndex;
}

uint8_t mac_helper_default_key_index_get(protocol_interface_info_entry_t *interface)
{
    return interface->mac_parameters.mac_default_key_index;
}

void mac_helper_set_default_key_source(protocol_interface_info_entry_t *interface)
{
    mlme_set_t set_req;
    set_req.attr_index = 0;
    set_req.value_pointer = (void *)mac_helper_default_key_source;
    set_req.value_size = 8;
    //Set first default key source
    set_req.attr = macDefaultKeySource;
    interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
    //Set first default key source
    set_req.attr = macAutoRequestKeySource;
    interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);

}

void mac_helper_default_security_level_set(protocol_interface_info_entry_t *interface, uint8_t securityLevel)
{
    bool security_enabled;
    if (securityLevel) {
        security_enabled = true;
    } else {
        security_enabled = false;
    }
    mac_helper_pib_8bit_set(interface, macAutoRequestSecurityLevel,  securityLevel);
    mac_helper_pib_boolean_set(interface, macSecurityEnabled, security_enabled);

}

uint8_t mac_helper_default_security_level_get(protocol_interface_info_entry_t *interface)
{
    return interface->mac_parameters.mac_security_level;
}

void mac_helper_default_security_key_id_mode_set(protocol_interface_info_entry_t *interface, uint8_t keyIdMode)
{
    mac_helper_pib_8bit_set(interface, macAutoRequestKeyIdMode,  keyIdMode);
}

uint8_t mac_helper_default_security_key_id_mode_get(protocol_interface_info_entry_t *interface)
{
    return interface->mac_parameters.mac_key_id_mode;
}

static void mac_helper_key_lookup_set(mlme_key_id_lookup_descriptor_t *lookup, uint8_t id)
{
    memcpy(lookup->LookupData, mac_helper_default_key_source, 8);
    lookup->LookupData[8] = id;
    lookup->LookupDataSize = 1;
}


static void mac_helper_keytable_descriptor_set(struct mac_api_s *api, const uint8_t *key, uint8_t id, uint8_t attribute_id)
{
    mlme_set_t set_req;
    mlme_key_id_lookup_descriptor_t lookup_description;
    mlme_key_descriptor_entry_t key_description;
    if (key) {
        mac_helper_key_lookup_set(&lookup_description, id);
        memset(&key_description, 0, sizeof(mlme_key_descriptor_entry_t));
        memcpy(key_description.Key, key, 16);
        key_description.KeyIdLookupList = &lookup_description;
        key_description.KeyIdLookupListEntries = 1;
    } else {
        memset(&key_description, 0, sizeof(mlme_key_descriptor_entry_t));
    }
    set_req.attr = macKeyTable;
    set_req.attr_index = attribute_id;
    set_req.value_pointer = &key_description;
    set_req.value_size = sizeof(mlme_key_descriptor_entry_t);

    api->mlme_req(api, MLME_SET, &set_req);
}

int8_t mac_helper_security_default_key_set(protocol_interface_info_entry_t *interface, const uint8_t *key, uint8_t id, uint8_t keyid_mode)
{
    if (id == 0 || keyid_mode > 3) {
        return -1;
    }

    mac_helper_pib_8bit_set(interface, macAutoRequestKeyIndex, id);
    mac_helper_keytable_descriptor_set(interface->mac_api, key, id, interface->mac_parameters.mac_default_key_attribute_id);
    return 0;
}

int8_t mac_helper_security_auto_request_key_index_set(protocol_interface_info_entry_t *interface, uint8_t key_attibute_index, uint8_t id)
{
    if (id == 0) {
        return -1;
    }
    interface->mac_parameters.mac_default_key_attribute_id = key_attibute_index;
    mac_helper_pib_8bit_set(interface, macAutoRequestKeyIndex, id);
    return 0;
}

int8_t mac_helper_security_key_to_descriptor_set(protocol_interface_info_entry_t *interface, const uint8_t *key, uint8_t id, uint8_t descriptor)
{
    if (id == 0) {
        return -1;
    }

    mac_helper_keytable_descriptor_set(interface->mac_api, key, id, descriptor);
    return 0;
}

int8_t mac_helper_security_key_descriptor_clear(protocol_interface_info_entry_t *interface, uint8_t descriptor)
{
    if (!interface->mac_api) {
        return -1;
    }

    mlme_set_t set_req;
    mlme_key_descriptor_entry_t key_description;
    memset(&key_description, 0, sizeof(mlme_key_descriptor_entry_t));

    set_req.attr = macKeyTable;
    set_req.value_pointer = &key_description;
    set_req.value_size = sizeof(mlme_key_descriptor_entry_t);
    set_req.attr_index = descriptor;
    interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
    return 0;
}

void mac_helper_coordinator_address_set(protocol_interface_info_entry_t *interface, addrtype_t adr_type, uint8_t *adr_ptr)
{
    uint16_t short_addr;
    mlme_set_t set_req;
    set_req.attr_index = 0;

    if (adr_type == ADDR_802_15_4_SHORT) {
        memcpy(interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address, adr_ptr, 2);
        interface->mac_parameters.mac_cordinator_info.cord_adr_mode = MAC_ADDR_MODE_16_BIT;
        short_addr = common_read_16_bit(interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address);
        set_req.attr = macCoordShortAddress;
        set_req.value_pointer = &short_addr;
        set_req.value_size = 2;
    } else if (adr_type == ADDR_802_15_4_LONG) {
        memcpy(interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address, adr_ptr, 8);
        interface->mac_parameters.mac_cordinator_info.cord_adr_mode = MAC_ADDR_MODE_64_BIT;
        set_req.attr = macCoordExtendedAddress;
        set_req.value_pointer = &interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address;
        set_req.value_size = 8;
    }

    if (interface->mac_api) {
        interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
    }
}

addrtype_t mac_helper_coordinator_address_get(protocol_interface_info_entry_t *interface, uint8_t *adr_ptr)
{
    addrtype_t ret = ADDR_NONE;
    if (!interface) {
        return ret;
    }

    if (interface->mac_parameters.mac_cordinator_info.cord_adr_mode == MAC_ADDR_MODE_16_BIT) {
        memcpy(adr_ptr, interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address, 2);
        ret = ADDR_802_15_4_SHORT;
    } else if (interface->mac_parameters.mac_cordinator_info.cord_adr_mode == MAC_ADDR_MODE_64_BIT) {
        memcpy(adr_ptr, interface->mac_parameters.mac_cordinator_info.mac_mlme_coord_address, 8);
        ret = ADDR_802_15_4_LONG;
    }
    return ret;
}

int8_t mac_helper_pib_boolean_set(protocol_interface_info_entry_t *interface, mlme_attr_t attribute, bool value)
{

    switch (attribute) {
        case macSecurityEnabled:
            interface->mac_parameters.SecurityEnabled = value;
            break;

        case macRxOnWhenIdle:
            interface->mac_parameters.RxOnWhenIdle = value;
            break;

        default:
            return -1;
    }
    if (interface->mac_api && interface->mac_api->mlme_req) {
        mlme_set_t set_req;
        set_req.attr = attribute;
        set_req.attr_index = 0;
        set_req.value_pointer = &value;
        set_req.value_size = sizeof(bool);
        interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
    }

    return 0;
}

int8_t mac_helper_mac_channel_set(protocol_interface_info_entry_t *interface, uint8_t new_channel)
{

    if (interface->mac_parameters.mac_channel != new_channel) {

        interface->mac_parameters.mac_channel = new_channel;
        if (interface->mac_api && interface->mac_api->mlme_req) {
            mlme_set_t set_req;
            set_req.attr = phyCurrentChannel;
            set_req.attr_index = 0;
            set_req.value_pointer = &new_channel;
            set_req.value_size = 1;
            interface->mac_api->mlme_req(interface->mac_api, MLME_SET, &set_req);
        }
    }
    return 0;
}

static bool mac_helper_write_16bit(uint16_t temp16, uint8_t *addrPtr)
{
    common_write_16_bit(temp16, addrPtr);
    return temp16 != 0xffff;
}

/* Write functions return "false" if they write an "odd" address, true if they
 * write a "normal" address. They still write odd addresses, as certain special
 * packets may want them, but this allows normal data paths to check and block
 * odd cases.
 * "Odd" is currently defined as PAN ID == 0xffff, or short address > 0xfffd.
 */
bool mac_helper_write_our_addr(protocol_interface_info_entry_t *interface, sockaddr_t *ptr)
{
    bool normal = true;

    //Set First PANID
    normal &= mac_helper_write_16bit(interface->mac_parameters.pan_id, ptr->address);

    if (ptr->addr_type != ADDR_802_15_4_LONG && ptr->addr_type != ADDR_802_15_4_SHORT) {
        if (interface->mac_parameters.shortAdressValid) {
            ptr->addr_type = ADDR_802_15_4_SHORT;
        } else {
            ptr->addr_type = ADDR_802_15_4_LONG;
        }
    }

    if (ptr->addr_type == ADDR_802_15_4_SHORT) {
        normal &= mac_helper_write_16bit(interface->mac_parameters.mac_short_address, &ptr->address[2]);
    } else {
        memcpy(&ptr->address[2], interface->mac, 8);
    }

    return normal;
}

int8_t mac_helper_mac64_set(protocol_interface_info_entry_t *interface, const uint8_t *mac64)
{
    memcpy(interface->mac, mac64, 8);
    if (interface->mac_api) {
        interface->mac_api->mac64_set(interface->mac_api, mac64);
    }
    return 0;
}


/*
 * Given a buffer, with address and security flags set, compute the maximum
 * MAC payload that could be put in that buffer.
 */
uint_fast16_t mac_helper_max_payload_size(protocol_interface_info_entry_t *cur, uint_fast16_t frame_overhead)
{
    uint16_t max;

    if (test_6lowpan_fragmentation_mtu_size_override == 0) {
        max = cur->mac_api->phyMTU - frame_overhead;
    } else {
        max = test_6lowpan_fragmentation_mtu_size_override - frame_overhead;
    }

    /* But if we want IEEE 802.15.4-2003 compatibility (and it looks like a
     * standard PHY), limit ourselves to the 2003 maximum */
    if (cur->mac_parameters.MacUnsusecured_2003_cab && max > MAC_IEEE_802_15_4_MAX_MAC_SAFE_PAYLOAD_SIZE &&
            cur->mac_api->phyMTU == MAC_IEEE_802_15_4_MAX_PHY_PACKET_SIZE) {
        max = MAC_IEEE_802_15_4_MAX_MAC_SAFE_PAYLOAD_SIZE;
    }

    return max;
}

/*
 * Given a buffer, with address and security flags set, compute the MAC overhead
 * size once MAC header and footer are added.
 * May not be accurate if MAC_MAX_PHY_PACKET_SIZE isn't set, implying a
 * non-standard MAC.
 */
uint_fast8_t mac_helper_frame_overhead(protocol_interface_info_entry_t *cur, const buffer_t *buf)
{
    uint_fast8_t length = 15;

    /*8bytes src address, 2 frame control, 1 sequence, 2 pan-id, 2 FCS*/
    if (buf->src_sa.addr_type == ADDR_NONE) {
        if (cur->mac_parameters.shortAdressValid) {
            length -= 6; //Cut 6 bytes from src address
        }
    } else if (buf->src_sa.addr_type == ADDR_802_15_4_SHORT) {
        length -= 6; //Cut 6 bytes from src address
    }

    if (memcmp(buf->dst_sa.address, buf->src_sa.address, 2) == 0) {
        length -= 2; // Cut Pan-id
    }

    if (buf->dst_sa.addr_type == ADDR_802_15_4_LONG) {
        length += 10;
    } else if (buf->dst_sa.addr_type == ADDR_802_15_4_SHORT || buf->dst_sa.addr_type == ADDR_BROADCAST) {
        length += 4;
    }

    if (cur->mac_parameters.mac_security_level && (!buf->options.ll_security_bypass_tx)) {
        length += mac_helper_header_security_aux_header_length(cur->mac_parameters.mac_key_id_mode);
        length += mac_helper_security_mic_length_get(cur->mac_parameters.mac_security_level);
    }

    return length;
}

static uint8_t mac_helper_security_mic_length_get(uint8_t security_level)
{
    uint8_t mic_length;
    switch (security_level) {
        case SEC_MIC32:
        case SEC_ENC_MIC32:
            mic_length = 4;
            break;
        case SEC_MIC64:
        case SEC_ENC_MIC64:
            mic_length = 8;
            break;
        case SEC_MIC128:
        case SEC_ENC_MIC128:
            mic_length = 16;
            break;
        case SEC_NONE:
        case SEC_ENC:
        default:
            mic_length = 0;
            break;
    }

    return mic_length;
}

static uint8_t mac_helper_header_security_aux_header_length(uint8_t keyIdmode)
{

    uint8_t header_length = 5; //Header + 32-bit counter
    switch (keyIdmode) {
        case MAC_KEY_ID_MODE_SRC8_IDX:
            header_length += 4; //64-bit key source first part
        /* fall through  */
        case MAC_KEY_ID_MODE_SRC4_IDX:
            header_length += 4; //32-bit key source inline
        /* fall through  */
        case MAC_KEY_ID_MODE_IDX:
            header_length += 1;
            break;
        default:
            break;
    }
    return header_length;
}

int8_t mac_helper_link_frame_counter_read(int8_t interface_id, uint32_t *seq_ptr)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api || !seq_ptr) {
        return -1;
    }

    return mac_helper_key_link_frame_counter_read(interface_id, seq_ptr, cur->mac_parameters.mac_default_key_attribute_id);
}

int8_t mac_helper_key_link_frame_counter_read(int8_t interface_id, uint32_t *seq_ptr, uint8_t descriptor)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api || !seq_ptr) {
        return -1;
    }
    mlme_get_t get_req;
    get_req.attr = macFrameCounter;
    get_req.attr_index = descriptor;
    cur->mac_api->mlme_req(cur->mac_api, MLME_GET, &get_req);
    *seq_ptr = cur->mac_parameters.security_frame_counter;

    return 0;
}

int8_t mac_helper_key_link_frame_counter_set(int8_t interface_id, uint32_t seq_ptr, uint8_t descriptor)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api) {
        return -1;
    }
    mlme_set_t set_req;
    set_req.attr = macFrameCounter;
    set_req.attr_index = descriptor;
    set_req.value_pointer = &seq_ptr;
    set_req.value_size = 4;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}

void mac_helper_devicetable_remove(mac_api_t *mac_api, uint8_t attribute_index, uint8_t *mac64)
{
    (void) mac64;
    if (!mac_api) {
        return;
    }

    mlme_device_descriptor_t device_desc;
    mlme_set_t set_req;
    memset(&device_desc, 0xff, sizeof(mlme_device_descriptor_t));

    set_req.attr = macDeviceTable;
    set_req.attr_index = attribute_index;
    set_req.value_pointer = (void *)&device_desc;
    set_req.value_size = sizeof(mlme_device_descriptor_t);
    if (mac64) {
        tr_debug("Unregister Device %u, mac64: %s", attribute_index, trace_array(mac64, 8));
    }
    mac_api->mlme_req(mac_api, MLME_SET, &set_req);
}

void mac_helper_device_description_write(protocol_interface_info_entry_t *cur, mlme_device_descriptor_t *device_desc, const uint8_t *mac64, uint16_t mac16, uint32_t frame_counter, bool exempt)
{
    memcpy(device_desc->ExtAddress, mac64, 8);
    device_desc->ShortAddress = mac16;
    device_desc->PANId = mac_helper_panid_get(cur);
    device_desc->Exempt = exempt;
    device_desc->FrameCounter = frame_counter;
}

void mac_helper_devicetable_set(const mlme_device_descriptor_t *device_desc, protocol_interface_info_entry_t *cur, uint8_t attribute_index, uint8_t keyID, bool force_set)
{
    if (!force_set && cur->mac_parameters.SecurityEnabled && cur->mac_parameters.mac_default_key_index != keyID) {
        tr_debug("Do not set counter by index %u != %u", cur->mac_parameters.mac_default_key_index, keyID);
        return;
    }

    tr_debug("Register Device %u, mac16 %x mac64: %s, %"PRIu32, attribute_index, device_desc->ShortAddress, trace_array(device_desc->ExtAddress, 8), device_desc->FrameCounter);
    mac_helper_devicetable_direct_set(cur->mac_api, device_desc, attribute_index);
}

void mac_helper_devicetable_direct_set(struct mac_api_s *mac_api, const mlme_device_descriptor_t *device_desc, uint8_t attribute_index)
{
    if (!mac_api) {
        return;
    }

    mlme_set_t set_req;
    set_req.attr = macDeviceTable;
    set_req.attr_index = attribute_index;
    set_req.value_pointer = (void *)device_desc;
    set_req.value_size = sizeof(mlme_device_descriptor_t);
    mac_api->mlme_req(mac_api, MLME_SET, &set_req);
}

int8_t mac_helper_mac_mlme_max_retry_set(int8_t interface_id, uint8_t mac_retry_set)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api) {
        return -1;
    }
    mlme_set_t set_req;
    set_req.attr = macMaxFrameRetries;
    set_req.attr_index = 0;
    set_req.value_pointer = &mac_retry_set;
    set_req.value_size = 1;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}

int8_t mac_helper_mac_mlme_max_csma_backoffs_set(int8_t interface_id, uint8_t csma_backoffs)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api) {
        return -1;
    }
    mlme_set_t set_req;
    set_req.attr = macMaxCSMABackoffs;
    set_req.attr_index = 0;
    set_req.value_pointer = &csma_backoffs;
    set_req.value_size = 1;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}

int8_t mac_helper_mac_mlme_be_set(int8_t interface_id, uint8_t min_be, uint8_t max_be)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api) {
        return -1;
    }
    mlme_set_t set_req;
    set_req.attr = macMinBE;
    set_req.attr_index = 0;
    set_req.value_pointer = &min_be;
    set_req.value_size = 1;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    set_req.attr = macMaxBE;
    set_req.attr_index = 0;
    set_req.value_pointer = &max_be;
    set_req.value_size = 1;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}

int8_t mac_helper_mac_mlme_data_request_restart_set(int8_t interface_id, mlme_request_restart_config_t *request_restart_config)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);

    if (!cur || !cur->mac_api) {
        return -1;
    }
    mlme_set_t set_req;
    set_req.attr = macRequestRestart;
    set_req.attr_index = 0;
    set_req.value_pointer = (void *)request_restart_config;
    set_req.value_size = sizeof(mlme_request_restart_config_t);
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}

int8_t mac_helper_start_auto_cca_threshold(int8_t interface_id, uint8_t number_of_channels, int8_t default_dbm, int8_t high_limit, int8_t low_limit)
{
    protocol_interface_info_entry_t *cur;
    cur = protocol_stack_interface_info_get_by_id(interface_id);
    if (!cur || !cur->mac_api) {
        return -1;
    }
    uint8_t start_cca_thr[4] = {number_of_channels, default_dbm, high_limit, low_limit};
    mlme_set_t set_req;
    set_req.attr = macCCAThresholdStart;
    set_req.value_pointer = &start_cca_thr;
    set_req.value_size = sizeof(start_cca_thr);
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);
    /* Get CCA threshold table. Table is stored to interface structure */
    mlme_get_t get_req;
    get_req.attr = macCCAThreshold;
    cur->mac_api->mlme_req(cur->mac_api, MLME_GET, &get_req);
    return 0;
}

int8_t mac_helper_mac_mlme_filter_start(int8_t interface_id, int16_t lqi_m, int16_t lqi_add, int16_t dbm_m, int16_t dbm_add)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);
    mlme_set_t set_req;
    mlme_request_mac_filter_start_t args = {
        .lqi_m = lqi_m,
        .lqi_add = lqi_add,
        .dbm_m = dbm_m,
        .dbm_add = dbm_add
    };

    if (!cur || !cur->mac_api) {
        return -1;
    }

    set_req.attr = macFilterStart;
    set_req.value_pointer = &args;
    set_req.value_size = sizeof(mlme_request_mac_filter_start_t);
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);
    return 0;
}

int8_t mac_helper_mac_mlme_filter_clear(int8_t interface_id)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);
    mlme_set_t set_req;

    if (!cur || !cur->mac_api) {
        return -1;
    }

    set_req.attr = macFilterClear;
    set_req.value_pointer = NULL;
    set_req.value_size = 0;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);
    return 0;
}

int8_t mac_helper_mac_mlme_filter_add_long(int8_t interface_id, uint8_t mac64[8], int16_t lqi_m, int16_t lqi_add, int16_t dbm_m, int16_t dbm_add)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);
    mlme_set_t set_req;
    mlme_request_mac_filter_add_long_t args = {
        .mac64 = { 0 },
        .lqi_m = lqi_m,
        .lqi_add = lqi_add,
        .dbm_m = dbm_m,
        .dbm_add = dbm_add
    };

    if (!cur || !cur->mac_api) {
        return -1;
    }

    memcpy(&args.mac64, mac64, 8);
    set_req.attr = macFilterAddLong;
    set_req.value_pointer = &args;
    set_req.value_size = sizeof(mlme_request_mac_filter_add_long_t);
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);
    return 0;
}

int8_t mac_helper_mac_mlme_filter_stop(int8_t interface_id)
{
    protocol_interface_info_entry_t *cur = protocol_stack_interface_info_get_by_id(interface_id);
    mlme_set_t set_req;

    if (!cur || !cur->mac_api) {
        return -1;
    }

    set_req.attr = macFilterStop;
    set_req.value_pointer = NULL;
    set_req.value_size = 0;
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);
    return 0;
}

int8_t mac_helper_set_regional_regulation(const struct protocol_interface_info_entry *cur, uint32_t regulation)
{
    mlme_set_t set_req;

    set_req.attr = macRegionalRegulation;
    set_req.value_pointer = &regulation;
    set_req.value_size = sizeof(uint32_t);
    cur->mac_api->mlme_req(cur->mac_api, MLME_SET, &set_req);

    return 0;
}
