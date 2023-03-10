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

#ifndef MAC_HELPER_H
#define MAC_HELPER_H

#include "nanostack/mac/mlme.h"
#include "core/ns_address_internal.h"

struct channel_list_s;
struct nwk_pan_descriptor_t;
struct protocol_interface_info_entry;
struct ns_sockaddr;
struct buffer;
struct mac_api_s;

extern uint16_t test_6lowpan_fragmentation_mtu_size_override;

void mac_helper_panid_set(struct protocol_interface_info_entry *interface, uint16_t panId);

void mac_helper_mac16_address_set(struct protocol_interface_info_entry *interface, uint16_t mac16);

uint16_t mac_helper_mac16_address_get(const struct protocol_interface_info_entry *interface);

uint16_t mac_helper_panid_get(const struct protocol_interface_info_entry *interface);

void mac_helper_set_default_key_source(struct protocol_interface_info_entry *interface);

void mac_helper_default_security_level_set(struct protocol_interface_info_entry *interface, uint8_t securityLevel);

uint8_t mac_helper_default_security_level_get(struct protocol_interface_info_entry *interface);

void mac_helper_default_security_key_id_mode_set(struct protocol_interface_info_entry *interface, uint8_t keyIdMode);

uint8_t mac_helper_default_security_key_id_mode_get(struct protocol_interface_info_entry *interface);

void mac_helper_default_key_index_set(struct protocol_interface_info_entry *interface, uint8_t keyIndex);

uint8_t mac_helper_default_key_index_get(struct protocol_interface_info_entry *interface);

int8_t mac_helper_security_default_key_set(struct protocol_interface_info_entry *interface, const uint8_t *key, uint8_t id, uint8_t keyid_mode);

int8_t mac_helper_security_auto_request_key_index_set(struct protocol_interface_info_entry *interface, uint8_t key_attibute_index, uint8_t id);

int8_t mac_helper_security_key_to_descriptor_set(struct protocol_interface_info_entry *interface, const uint8_t *key, uint8_t id, uint8_t descriptor);

int8_t mac_helper_security_key_descriptor_clear(struct protocol_interface_info_entry *interface, uint8_t descriptor);

void mac_helper_coordinator_address_set(struct protocol_interface_info_entry *interface, addrtype_t adr_type, uint8_t *adr_ptr);

addrtype_t mac_helper_coordinator_address_get(struct protocol_interface_info_entry *interface, uint8_t *adr_ptr);

int8_t mac_helper_pib_boolean_set(struct protocol_interface_info_entry *interface, mlme_attr_t attribute, bool value);

int8_t mac_helper_mac_channel_set(struct protocol_interface_info_entry *interface, uint8_t new_channel);

bool mac_helper_write_our_addr(struct protocol_interface_info_entry *interface, struct ns_sockaddr *ptr);

int8_t mac_helper_mac64_set(struct protocol_interface_info_entry *interface, const uint8_t *mac64);

uint_fast16_t mac_helper_max_payload_size(struct protocol_interface_info_entry *cur, uint_fast16_t frame_overhead);

uint_fast8_t mac_helper_frame_overhead(struct protocol_interface_info_entry *cur, const struct buffer *buf);

int8_t mac_helper_link_frame_counter_read(int8_t interface_id, uint32_t *seq_ptr);

int8_t mac_helper_key_link_frame_counter_read(int8_t interface_id, uint32_t *seq_ptr, uint8_t descriptor);

int8_t mac_helper_key_link_frame_counter_set(int8_t interface_id, uint32_t seq_ptr, uint8_t descriptor);

void mac_helper_devicetable_remove(struct mac_api_s *mac_api, uint8_t attribute_index, uint8_t *mac64);

void mac_helper_device_description_write(struct protocol_interface_info_entry *cur, mlme_device_descriptor_t *device_desc, const uint8_t *mac64, uint16_t mac16, uint32_t frame_counter, bool exempt);

void mac_helper_devicetable_set(const mlme_device_descriptor_t *device_dec, struct protocol_interface_info_entry *cur, uint8_t attribute_index, uint8_t keyID, bool force_set);

void mac_helper_devicetable_direct_set(struct mac_api_s *mac_api, const mlme_device_descriptor_t *device_desc, uint8_t attribute_index);

int8_t mac_helper_mac_mlme_max_retry_set(int8_t interface_id, uint8_t mac_retry_set);

int8_t mac_helper_mac_mlme_max_csma_backoffs_set(int8_t interface_id, uint8_t csma_backoffs);

int8_t mac_helper_mac_mlme_be_set(int8_t interface_id, uint8_t min_be, uint8_t max_be);

int8_t mac_helper_mac_mlme_data_request_restart_set(int8_t interface_id, mlme_request_restart_config_t *request_restart_config);

int8_t mac_helper_start_auto_cca_threshold(int8_t interface_id, uint8_t number_of_channels, int8_t default_dbm, int8_t high_limit, int8_t low_limit);

int8_t mac_helper_mac_mlme_filter_start(int8_t interface_id, int16_t lqi_m, int16_t lqi_add, int16_t dbm_m, int16_t dbm_add);

int8_t mac_helper_mac_mlme_filter_clear(int8_t interface_id);

int8_t mac_helper_mac_mlme_filter_add_long(int8_t interface_id, uint8_t mac64[8], int16_t lqi_m, int16_t lqi_add, int16_t dbm_m, int16_t dbm_add);

int8_t mac_helper_mac_mlme_filter_stop(int8_t interface_id);

int8_t mac_helper_set_regional_regulation(const struct protocol_interface_info_entry *cur, uint32_t regulation);

#endif // MAC_HELPER_H
