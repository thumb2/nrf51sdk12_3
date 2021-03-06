/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "conn_mw_nrf_soc.h"
#include "conn_mw_ble.h"
#include "conn_mw_ble_l2cap.h"
#include "conn_mw_ble_gap.h"
#include "conn_mw_ble_gatts.h"
#include "conn_mw_ble_gattc.h"

/**@brief Connectivity middleware handlers table. */
static const conn_mw_item_t conn_mw_item[] = {
    //Functions from nrf_soc.h
    {SD_POWER_SYSTEM_OFF, conn_mw_power_system_off},
    {SD_TEMP_GET, conn_mw_temp_get},
    {SD_ECB_BLOCK_ENCRYPT, conn_mw_ecb_block_encrypt},
    //Functions from ble.h
    {SD_BLE_TX_PACKET_COUNT_GET, conn_mw_ble_tx_packet_count_get},
    {SD_BLE_UUID_VS_ADD, conn_mw_ble_uuid_vs_add},
    {SD_BLE_UUID_DECODE, conn_mw_ble_uuid_decode},
    {SD_BLE_UUID_ENCODE, conn_mw_ble_uuid_encode},
    {SD_BLE_VERSION_GET, conn_mw_ble_version_get},
    {SD_BLE_OPT_GET, conn_mw_ble_opt_get},
    {SD_BLE_OPT_SET, conn_mw_ble_opt_set},
    {SD_BLE_ENABLE, conn_mw_ble_enable},
    {SD_BLE_USER_MEM_REPLY, conn_mw_ble_user_mem_reply},
    //Functions from ble_l2cap.h
    {SD_BLE_L2CAP_CID_REGISTER, conn_mw_ble_l2cap_cid_register},
    {SD_BLE_L2CAP_CID_UNREGISTER, conn_mw_ble_l2cap_cid_unregister},
    {SD_BLE_L2CAP_TX, conn_mw_ble_l2cap_tx},
    //Functions from ble_gap.h
    {SD_BLE_GAP_SCAN_STOP, conn_mw_ble_gap_scan_stop},
    {SD_BLE_GAP_ADDR_SET, conn_mw_ble_gap_addr_set},
    {SD_BLE_GAP_ADDR_GET, conn_mw_ble_gap_addr_get},
    {SD_BLE_GAP_PRIVACY_SET, conn_mw_ble_gap_privacy_set},
    {SD_BLE_GAP_PRIVACY_GET, conn_mw_ble_gap_privacy_get},
    {SD_BLE_GAP_WHITELIST_SET, conn_mw_ble_gap_whitelist_set},
    {SD_BLE_GAP_DEVICE_IDENTITIES_SET, conn_mw_ble_gap_device_identities_set},
    {SD_BLE_GAP_CONNECT, conn_mw_ble_gap_connect},
    {SD_BLE_GAP_CONNECT_CANCEL, conn_mw_ble_gap_connect_cancel},
    {SD_BLE_GAP_SCAN_START, conn_mw_ble_gap_scan_start},
    {SD_BLE_GAP_SEC_INFO_REPLY, conn_mw_ble_gap_sec_info_reply},
    {SD_BLE_GAP_ENCRYPT, conn_mw_ble_gap_encrypt},
    {SD_BLE_GAP_ADV_DATA_SET, conn_mw_ble_gap_adv_data_set},
    {SD_BLE_GAP_ADV_START, conn_mw_ble_gap_adv_start},
    {SD_BLE_GAP_ADV_STOP, conn_mw_ble_gap_adv_stop},
    {SD_BLE_GAP_CONN_PARAM_UPDATE, conn_mw_ble_gap_conn_param_update},
    {SD_BLE_GAP_DISCONNECT, conn_mw_ble_gap_disconnect},
    {SD_BLE_GAP_TX_POWER_SET, conn_mw_ble_gap_tx_power_set},
    {SD_BLE_GAP_APPEARANCE_SET, conn_mw_ble_gap_appearance_set},
    {SD_BLE_GAP_APPEARANCE_GET, conn_mw_ble_gap_appearance_get},
    {SD_BLE_GAP_PPCP_SET, conn_mw_ble_gap_ppcp_set},
    {SD_BLE_GAP_PPCP_GET, conn_mw_ble_gap_ppcp_get},
    {SD_BLE_GAP_DEVICE_NAME_SET, conn_mw_ble_gap_device_name_set},
    {SD_BLE_GAP_DEVICE_NAME_GET, conn_mw_ble_gap_device_name_get},
    {SD_BLE_GAP_AUTHENTICATE, conn_mw_ble_gap_authenticate},
    {SD_BLE_GAP_SEC_PARAMS_REPLY, conn_mw_ble_gap_sec_params_reply},
    {SD_BLE_GAP_AUTH_KEY_REPLY, conn_mw_ble_gap_auth_key_reply},
    {SD_BLE_GAP_SEC_INFO_REPLY, conn_mw_ble_gap_sec_info_reply},
    {SD_BLE_GAP_CONN_SEC_GET, conn_mw_ble_gap_conn_sec_get},
    {SD_BLE_GAP_RSSI_START, conn_mw_ble_gap_rssi_start},
    {SD_BLE_GAP_RSSI_STOP, conn_mw_ble_gap_rssi_stop},
    {SD_BLE_GAP_KEYPRESS_NOTIFY, conn_mw_ble_gap_keypress_notify},
    {SD_BLE_GAP_LESC_DHKEY_REPLY, conn_mw_ble_gap_lesc_dhkey_reply},
    {SD_BLE_GAP_LESC_OOB_DATA_SET, conn_mw_ble_gap_lesc_oob_data_set},
    {SD_BLE_GAP_LESC_OOB_DATA_GET, conn_mw_ble_gap_lesc_oob_data_get},
    {SD_BLE_GAP_RSSI_GET, conn_mw_ble_gap_rssi_get},
    //Functions from ble_gattc.h
    {SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER, conn_mw_ble_gattc_primary_services_discover},
    {SD_BLE_GATTC_RELATIONSHIPS_DISCOVER, conn_mw_ble_gattc_relationships_discover},
    {SD_BLE_GATTC_CHARACTERISTICS_DISCOVER, conn_mw_ble_gattc_characteristics_discover},
    {SD_BLE_GATTC_DESCRIPTORS_DISCOVER, conn_mw_ble_gattc_descriptors_discover},
    {SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ, conn_mw_ble_gattc_char_value_by_uuid_read},
    {SD_BLE_GATTC_READ, conn_mw_ble_gattc_read},
    {SD_BLE_GATTC_CHAR_VALUES_READ, conn_mw_ble_gattc_char_values_read},
    {SD_BLE_GATTC_WRITE, conn_mw_ble_gattc_write},
    {SD_BLE_GATTC_HV_CONFIRM, conn_mw_ble_gattc_hv_confirm},
    {SD_BLE_GATTC_ATTR_INFO_DISCOVER, conn_mw_ble_gattc_attr_info_discover},
    {SD_BLE_GATTC_EXCHANGE_MTU_REQUEST, conn_mw_ble_gattc_exchange_mtu_request},
    //Functions from ble_gatts.h
    {SD_BLE_GATTS_SERVICE_ADD, conn_mw_ble_gatts_service_add},
    {SD_BLE_GATTS_INCLUDE_ADD, conn_mw_ble_gatts_include_add},
    {SD_BLE_GATTS_CHARACTERISTIC_ADD, conn_mw_ble_gatts_characteristic_add},
    {SD_BLE_GATTS_DESCRIPTOR_ADD, conn_mw_ble_gatts_descriptor_add},
    {SD_BLE_GATTS_VALUE_SET, conn_mw_ble_gatts_value_set},
    {SD_BLE_GATTS_VALUE_GET, conn_mw_ble_gatts_value_get},
    {SD_BLE_GATTS_HVX, conn_mw_ble_gatts_hvx},
    {SD_BLE_GATTS_SERVICE_CHANGED, conn_mw_ble_gatts_service_changed},
    {SD_BLE_GATTS_RW_AUTHORIZE_REPLY, conn_mw_ble_gatts_rw_authorize_reply},
    {SD_BLE_GATTS_SYS_ATTR_SET, conn_mw_ble_gatts_sys_attr_set},
    {SD_BLE_GATTS_SYS_ATTR_GET, conn_mw_ble_gatts_sys_attr_get},
    {SD_BLE_GATTS_INITIAL_USER_HANDLE_GET, conn_mw_ble_gatts_initial_user_handle_get},
    {SD_BLE_GATTS_ATTR_GET, conn_mw_ble_gatts_attr_get},
    {SD_BLE_GATTS_EXCHANGE_MTU_REPLY, conn_mw_ble_gatts_exchange_mtu_reply},
};
