/*
 * Copyright (c) 2016-2017, 2019-2020, Pelion and affiliates.
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

/**
 * \file net_ipv6_api.h
 * \brief IPv6 configuration API.
 */

#ifndef NET_IPV6_API_H_
#define NET_IPV6_API_H_

#include <stdint.h>

/**
 * \brief Set maximum IPv6 fragmented datagram reception size.
 *
 * Set the maximum size limit for fragmented datagram reception.
 *
 * RFC 2460 requires this to be at least 1500. It should also be at least
 * as large as the MTU of each attached link.
 *
 * \param frag_mru The fragmented Maximum Receive Unit in octets.
 * \return 0 Change OK - actual MRU is at least the requested value.
 * \return <0 Change invalid - unable to set the specified MRU.
 */
int8_t arm_nwk_ipv6_frag_mru(uint16_t frag_mru);

/**
 * \brief Set the maximum number of entries for the neighbour cache and
 * destination cache. Default value is 64 and minimum allowed value is 4.
 *
 * Note: This must be called before arm_nwk_interface_lowpan_init()
 *
 * \param max_entries The absolute maximum entries allowed in cache at any time.
 * \return 0 Change OK.
 * \return <0 Change invalid - unable to change the maximum for cache.
 */
int8_t arm_nwk_ipv6_max_cache_entries(uint16_t max_entries);

/**
 * \brief Configure destination cache.
 *
 * Set destination cache maximum entry count, thresholds where amount of entries is kept during operation and lifetime.
 *
 * Note: This must be called before arm_nwk_interface_lowpan_init()
 *
 * \param max_entries Maximum number of entries allowed in destination cache at any time.
 * \param short_term_threshold Amount of cache entries kept in memory in short term. Must be less than max_entries.
 * \param long_term_threshold Amount of entries kept in memory over long period of time. Must be less than short_term_threshold.
 * \param lifetime Lifetime of cache entry, must be over 120 seconds
 *
 * \return 0 Change OK.
 * \return <0 Change invalid - unable to change the maximum for cache.
 */
int8_t arm_nwk_ipv6_destination_cache_configure(uint16_t max_entries, uint16_t short_term_threshold, uint16_t long_term_threshold, uint16_t lifetime);

/**
 * \brief Configure neighbour cache.
 *
 * Set neighbour cache maximum entry count, thresholds where amount of entries is kept during operation and lifetime.
 *
 * Note: This must be called before arm_nwk_interface_lowpan_init()
 *
 * \param max_entries Maximum number of entries allowed in neighbour cache at any time.
 * \param short_term_threshold Amount of cache entries kept in memory in short term. Must be less than max_entries.
 * \param long_term_threshold Amount of entries kept in memory over long period of time. Must be less than short_term_threshold.
 * \param lifetime Lifetime of cache entry, must be over 120 seconds
 *
 * \return 0 Change OK.
 * \return <0 Change invalid - unable to change the maximum for cache.
 */
int8_t arm_nwk_ipv6_neighbour_cache_configure(uint16_t max_entries, uint16_t short_term_threshold, uint16_t long_term_threshold, uint16_t lifetime);

/**
 * \brief Configure automatic flow label calculation.
 *
 * Enable or disable automatic generation of IPv6 flow labels for outgoing
 * packets.
 *
 * \param auto_flow_label True to enable auto-generation.
 */
void arm_nwk_ipv6_auto_flow_label(bool auto_flow_label);

/**
 * \brief Set the key for opaque IPv6 interface identifiers
 *
 * This call sets the secret key used to generate opaque interface identifiers,
 * as per RFC 7217. Once this has been set, all interfaces will use opaque
 * interface identifiers by default. If secret_key is NULL, opaque interface
 * identifiers will be disabled.
 *
 * Correct implementation of RFC 7217 would require that this key be
 * randomly generated at first bootstrap, and thereafter remain constant, which
 * would require non-volatile storage. The next closest alternative would be
 * to base this on a MAC address.
 *
 * \param secret_key A pointer to secret key (will be copied by call).
 * \param key_len The length of the key.
 *
 * \return 0 key set okay.
 * \return <0 key set failed (for example due to memory allocation).
 */
int8_t arm_nwk_ipv6_opaque_iid_key(const void *secret_key, uint8_t key_len);

/**
 * \brief Enable/disable opaque IPv6 interface identifiers by interface
 *
 * Enable or disable RFC 7217 opaque IIDs generated by SLAAC, per interface.
 * By default opaque IIDs are enabled if the opaque key is set. If disabled,
 * SLAAC IIDs will be EUI-64-based as per RFC 4291.
 *
 * \param interface_id Interface ID.
 * \param enable True to enable.
 * \return 0 enabled/disabled OK.
 * \return <0 failed (for example invalid interface ID).
 *
 */
int8_t arm_nwk_ipv6_opaque_iid_enable(int8_t interface_id, bool enable);

/**
 * \brief Enable/disable default route in Router advertisements
 *
 * Enable or disable RFC 4861 Default router configuration in router advertisements.
 * This makes the device a default router in the interface.
 *
 * \param interface_id Interface ID.
 * \param enable True to enable.
 * \return 0 enabled/disabled OK.
 * \return <0 failed (for example invalid interface ID).
 *
 */
int8_t arm_nwk_ipv6_default_route_enable(int8_t interface_id, bool enable);

/**
 * \brief add Recursive DNS Server Option information to Router advertisements
 *
 * Add Recursive DNS Server Option from RFC 8106 to router advertisements.
 * This makes it possible to configure DNS server address to other devices connected to the interface.
 *
 * \param interface_id Interface ID.
 * \param address 16 byte array for IPv6 address.
 * \param lifetime advertised lifetime of the entry. 0 to delete address.
 * \return 0 DNS server option option successful.
 * \return <0 failed (for example invalid interface ID).
 *
 */
int8_t arm_nwk_ipv6_dns_server_add(int8_t interface_id, uint8_t *address, uint32_t lifetime);

/**
 * \brief add DNS Search List Option information to Router advertisements
 *
 * Add DNS Search List Option from RFC 8106 to router advertisements.
 * This makes it possible to configure DNS search list to other devices connected to the interface.
 *
 * \param interface_id Interface ID.
 * \param data byte array encoded following https://tools.ietf.org/html/rfc1035#section-3.1.
 * \param data_len Length of the byte array
 * \param lifetime advertised lifetime of the entry. 0 to delete address.
 * \return 0 DNS server option option successful.
 * \return <0 failed (for example invalid interface ID).
 *
 */
int8_t arm_nwk_ipv6_dns_search_list_add(int8_t interface_id, uint8_t *data, uint16_t data_len, uint32_t lifetime);


#endif /* NET_IPV6_API_H_ */
