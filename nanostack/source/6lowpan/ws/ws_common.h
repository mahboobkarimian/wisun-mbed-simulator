/*
 * Copyright (c) 2018-2021, Pelion and affiliates.
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

#ifndef WS_COMMON_H_
#define WS_COMMON_H_
#include <stdint.h>
#include "mbed-client-libservice/ns_list.h"
#include "service_libs/mac_neighbor_table/mac_neighbor_table.h"
#include "nanostack/mac/fhss_config.h"
#include "nanostack/mac/net_fhss.h"

#include "nwk_interface/protocol.h"
#include "6lowpan/ws/ws_config.h"
#include "6lowpan/ws/ws_common_defines.h"
#include "6lowpan/ws/ws_neighbor_class.h"
#include "6lowpan/ws/ws_regulation.h"

extern uint16_t test_max_child_count_override;

struct ws_pan_information_s;
struct ws_neighbor_class_s;
struct ws_excluded_channel_data_s;
struct ws_cfg_s;
struct ws_neighbor_class_entry;
struct mcps_data_ie_list;

typedef struct parent_info_s {
    uint16_t             pan_id;             /**< PAN ID */
    uint8_t              addr[8];            /**< address */
    uint8_t              link_quality;       /**< LQI value measured during reception of the MPDU */
    uint8_t              tx_fail;
    int8_t               signal_dbm;         /**< This extension for normal IEEE 802.15.4 Data indication */
    ws_pan_information_t pan_information;
    ws_utt_ie_t          ws_utt;
    ws_us_ie_t           ws_us;
    uint32_t             timestamp;                 /**< Timestamp when packet was received */
    uint32_t             age;                       /**< Age of entry in 100ms ticks */
    uint8_t              excluded_channel_data[32]; /**< Channel mask Max length and it accept 8 different range*/
    bool                 link_acceptable: 1;        /**< True when Link quality is in acceptable level*/
    ns_list_link_t       link;
} parent_info_t;

typedef NS_LIST_HEAD(parent_info_t, link) parent_info_list_t;

typedef struct ws_nud_table_entry {
    void                            *neighbor_info;
    uint16_t                        timer;                    /*!< Timer which resolution is 100ms*/
    unsigned                        retry_count: 2;
    bool                            wait_response: 1;           /*!< True when NS is sended and wait NA, False when random timer is active*/
    bool                            nud_process;
    ns_list_link_t  link;
} ws_nud_table_entry_t;

#define NO_PENDING_PROCESS 0
#define PENDING_KEY_INDEX_ADVERTISMENT 1
#define PENDING_KEY_INDEX_ACTIVATE 2

typedef struct {
    unsigned state: 2;
    uint8_t index;
} ws_pending_key_index_t;

typedef struct {
    uint32_t block_time;
    uint16_t old_bsi;
} ws_bsi_block_t;

typedef struct {
    uint16_t eapol_trigger_timer;
    uint16_t pas_trigger_timer;
    uint16_t pcs_trigger_timer;
    uint16_t dis_trigger_timer;
    uint16_t dis_trigger_timer_val;
    uint16_t rpl_trigger_timer;
    uint16_t rpl_trigger_timer_val;
    uint8_t pas_trigger_count;
    uint8_t pcs_trigger_count;
    bool auto_trg_enabled;
} ws_test_proc_trg_t;

typedef struct {
    uint16_t lfn_version;
    bool lfn_version_learned: 1;
    bool active_hash_1: 1;
    bool active_hash_2: 1;
    bool active_hash_3: 1;
    unsigned active_key_index: 2;
    uint8_t lgtkhash[24];
} ws_lfn_lgtk_t;


typedef NS_LIST_HEAD(ws_nud_table_entry_t, link) ws_nud_table_list_t;

typedef struct ws_info_s {
    trickle_t trickle_pan_config_solicit;
    trickle_t trickle_pan_config;
    trickle_t trickle_pan_advertisement_solicit;
    trickle_t trickle_pan_advertisement;
    trickle_params_t trickle_params_pan_discovery;
    uint8_t version; // Wi-SUN version information 1 = 1.0 2 = 1.x
    uint8_t rpl_state; // state from rpl_event_t
    uint8_t pas_requests; // Amount of PAN solicits sent
    uint8_t device_min_sens; // Device min sensitivity set by the application
    int8_t weakest_received_rssi; // Weakest received signal (dBm)
    parent_info_t parent_info[WS_PARENT_LIST_SIZE];
    parent_info_list_t parent_list_free;
    parent_info_list_t parent_list_reserved;
    ws_bsi_block_t ws_bsi_block;
    uint16_t aro_registration_timer;       /**< Aro registration timer */
    uint16_t rpl_version_timer;            /**< RPL version update timeout */
    uint32_t pan_timeout_timer;            /**< routers will fallback to previous state after this */
    uint32_t uptime;                       /**< Seconds after interface has been started */
    uint32_t authentication_time;          /**< When the last authentication was performed */
    uint32_t connected_time;               /**< Time we have been connected to network */
    uint32_t pan_config_sol_max_timeout;
    uint8_t gtkhash[32];
    uint16_t network_pan_id;
    bool configuration_learned: 1;
    bool trickle_pas_running: 1;
    bool trickle_pa_running: 1;
    bool trickle_pcs_running: 1;
    bool trickle_pc_running: 1;
    uint16_t trickle_pc_consistency_block_period;
    ws_pending_key_index_t pending_key_index_info;
    ws_nud_table_entry_t nud_table_entrys[ACTIVE_NUD_PROCESS_MAX];
    ws_nud_table_list_t active_nud_process;
    ws_nud_table_list_t free_nud_entries;
    ws_test_proc_trg_t test_proc_trg;
#ifdef HAVE_WS_VERSION_1_1
    ws_lfn_lgtk_t lfngtk;
    ws_phy_cap_info_t phy_cap_info;
#endif
    struct ws_cfg_s *cfg;                  /**< Wi-SUN configuration */
    struct ws_pan_information_s pan_information;
    ws_hopping_schedule_t hopping_schedule;
    struct ws_statistics *stored_stats_ptr;
    struct ws_neighbor_class_s neighbor_storage;
    struct fhss_timer *fhss_timer_ptr; // Platform adaptation for FHSS timers.
    struct fhss_api *fhss_api;
    ws_regulation_t regulation_ctxt;  /**< Regional regulation context. */
} ws_info_t;


int8_t ws_common_generate_channel_list(const struct protocol_interface_info_entry *cur, uint32_t *channel_mask, uint16_t number_of_channels, uint8_t regulatory_domain, uint8_t operating_class, uint8_t channel_plan_id);

uint16_t ws_common_active_channel_count(uint32_t *channel_mask, uint16_t number_of_channels);

int8_t ws_common_regulatory_domain_config(protocol_interface_info_entry_t *cur, ws_hopping_schedule_t *hopping_schedule);

uint16_t ws_common_channel_number_calc(uint8_t regulatory_domain, uint8_t operating_class, uint8_t channel_plan_id);

int8_t ws_common_allocate_and_init(protocol_interface_info_entry_t *cur);

void ws_common_seconds_timer(protocol_interface_info_entry_t *cur, uint32_t seconds);

void ws_common_fast_timer(protocol_interface_info_entry_t *cur, uint16_t ticks);

void ws_common_create_ll_address(uint8_t *ll_address, const uint8_t *mac64);

void ws_common_neighbor_update(protocol_interface_info_entry_t *cur, const uint8_t *ll_address);

void ws_common_black_list_neighbour(const uint8_t *ll_address, uint8_t nd_status);

void ws_common_aro_failure(protocol_interface_info_entry_t *cur, const uint8_t *ll_address);

void ws_common_neighbor_remove(protocol_interface_info_entry_t *cur, const uint8_t *ll_address);

uint8_t ws_common_allow_child_registration(protocol_interface_info_entry_t *cur, const uint8_t *eui64, uint16_t aro_timeout);

bool ws_common_negative_aro_mark(protocol_interface_info_entry_t *interface, const uint8_t *eui64);

uint32_t ws_common_version_timeout_get(uint8_t config);

uint32_t ws_common_latency_estimate_get(protocol_interface_info_entry_t *cur);

uint32_t ws_common_datarate_get_from_phy_mode(uint8_t phy_mode_id, uint8_t operating_mode);

uint32_t ws_common_datarate_get(protocol_interface_info_entry_t *cur);

uint32_t ws_common_usable_application_datarate_get(protocol_interface_info_entry_t *cur);

uint32_t ws_common_network_size_estimate_get(protocol_interface_info_entry_t *cur);

uint32_t ws_common_connected_time_get(protocol_interface_info_entry_t *cur);

uint32_t ws_common_authentication_time_get(protocol_interface_info_entry_t *cur);

void ws_common_primary_parent_update(protocol_interface_info_entry_t *interface, mac_neighbor_table_entry_t *neighbor);

void ws_common_secondary_parent_update(protocol_interface_info_entry_t *interface);

uint8_t ws_common_temporary_entry_size(uint8_t mac_table_size);

void ws_common_border_router_alive_update(protocol_interface_info_entry_t *interface);

int ws_common_init(int8_t interface_id, net_6lowpan_mode_e bootstrap_mode);

void ws_common_state_machine(protocol_interface_info_entry_t *cur);

fhss_ws_configuration_t ws_common_get_current_fhss_configuration(protocol_interface_info_entry_t *cur);

#define ws_info(cur) ((cur)->ws_info)
#ifdef HAVE_WS_VERSION_1_1
#define ws_version_1_0(cur) (((cur)->ws_info) && ((cur)->ws_info)->version == 1)
#define ws_version_1_1(cur) (((cur)->ws_info) && ((cur)->ws_info)->version > 1)
#define ws_lfn_version_learned(cur) ((cur)->ws_info->lfngtk.lfn_version_learned == true)
#define ws_neighbour_cap_pointer(neighbour) (&neighbour->pcap_info)
#else
#define ws_version_1_1(cur) (false)
#define ws_version_1_0(cur) ((cur)->ws_info)
#define ws_lfn_version_learned(cur) (false)
#define ws_neighbour_cap_pointer(neighbour) NULL
#endif
#define ws_test_proc_auto_trg(cur) ((cur)->ws_info->test_proc_trg.auto_trg_enabled == true)
#endif //WS_COMMON_H_
