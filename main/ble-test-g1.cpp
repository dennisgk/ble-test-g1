// main.c — ESP-IDF Bluedroid: GATT Client (central) + GATT Server (peripheral)
// - GATTS: Advertises Nordic UART Service (NUS) as a server + accepts incoming connections
// - GATTC: Scans (finite) -> stop -> connect (python-like) -> request encryption -> discover NUS -> subscribe
//
// Notes:
// - This is designed to be robust across IDF struct variations.
// - If your glasses truly require a specific "server" on the host, you must advertise their expected UUIDs.

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_defs.h"

#include "TextTree.hpp"
#include "anki_remote.hpp"

// -------------------- Logging --------------------
static const char *TAG = "G1_DUAL_ROLE";

texttree::TextTree g_tree;

// -------------------- UUIDs (NUS) --------------------
// 6e400001-b5a3-f393-e0a9-e50e24dcca9e (service)
// 6e400002... (TX) notify from server to central
// 6e400003... (RX) write from central to server
static const uint8_t UUID_NUS_SERVICE[16] = {
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e
};
static const uint8_t UUID_NUS_TX[16] = {
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e
};
static const uint8_t UUID_NUS_RX[16] = {
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e
};

static TaskHandle_t g_hb_task = NULL;
static uint16_t g_hb_seq = 0;

static void build_heartbeat(uint16_t seq, uint8_t out6[6]) {
    // Same 6-byte frame you were using in Python
    const uint16_t len = 6;
    out6[0] = 0x25;
    out6[1] = (uint8_t)(len & 0xff);
    out6[2] = (uint8_t)((len >> 8) & 0xff);
    out6[3] = (uint8_t)(seq % 0xff);
    out6[4] = 0x04;
    out6[5] = (uint8_t)(seq % 0xff);
}

static esp_bt_uuid_t uuid128(const uint8_t b[16]) {
    esp_bt_uuid_t u = {0};
    u.len = ESP_UUID_LEN_128;
    memcpy(u.uuid.uuid128, b, 16);
    return u;
}

// -------------------- Security knobs --------------------
static void setup_security(void) {
    // Good baseline: bonding, no MITM (switch to BOND_MITM + YESNO if glasses require it)
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
}

// Use privacy like Windows does
static void setup_privacy(void) {
    // Makes device use RPA / random addresses; helps many consumer BLE peripherals.
    esp_ble_gap_config_local_privacy(true);
}

// -------------------- GATTS (Server) --------------------
static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_gatts_conn_id = 0;
static bool g_gatts_connected = false;

static uint16_t g_handle_svc = 0;
static uint16_t g_handle_tx  = 0;
static uint16_t g_handle_rx  = 0;
static uint16_t g_handle_cccd = 0;

static uint8_t g_tx_value[20] = {0};
static uint16_t g_tx_value_len = 0;

static bool g_tx_notify_enabled = false;

// Attribute table indexes
enum {
    IDX_SVC,
    IDX_TX_CHAR,
    IDX_TX_VAL,
    IDX_TX_CCCD,
    IDX_RX_CHAR,
    IDX_RX_VAL,
    IDX_NB,
};

static uint16_t g_attr_handle_table[IDX_NB];

// NUS service definition (GATTS)
static const uint16_t PRIMARY_SERVICE_UUID = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t CHAR_DECL_UUID       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t CCCD_UUID            = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t  CHAR_PROP_NOTIFY = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  CHAR_PROP_WRITE  = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

uint16_t zero = 0;

static const esp_gatts_attr_db_t g_gatts_db[IDX_NB] = {
    // Service
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&PRIMARY_SERVICE_UUID, ESP_GATT_PERM_READ,
         ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *)UUID_NUS_SERVICE}
    },

    // TX Characteristic Declaration
    [IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CHAR_DECL_UUID, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&CHAR_PROP_NOTIFY}
    },

    // TX Characteristic Value (notify)
    [IDX_TX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)UUID_NUS_TX, ESP_GATT_PERM_READ,
         sizeof(g_tx_value), 0, g_tx_value}
    },

    // TX CCCD
    [IDX_TX_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CCCD_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(uint16_t), reinterpret_cast<uint8_t*>(&zero)}
    },

    // RX Characteristic Declaration
    [IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CHAR_DECL_UUID, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&CHAR_PROP_WRITE}
    },

    // RX Characteristic Value (write)
    [IDX_RX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)UUID_NUS_RX, ESP_GATT_PERM_WRITE,
         512, 0, NULL}
    },
};

// Advertising (GATTS peripheral)
static esp_ble_adv_params_t g_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Put service UUID in adv data so centrals can discover it.
static esp_ble_adv_data_t g_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x10,
    .max_interval        = 0x20,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = (uint8_t *)UUID_NUS_SERVICE,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static void start_advertising(void) {
    ESP_LOGI(TAG, "GATTS: start advertising (NUS)");
    esp_ble_gap_start_advertising(&g_adv_params);
}

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS registered, if=%d", gatts_if);
        g_gatts_if = gatts_if;

        // Configure advertising payload (name + uuid)
        esp_ble_gap_config_adv_data(&g_adv_data);

        // Create attribute table
        esp_ble_gatts_create_attr_tab(g_gatts_db, gatts_if, IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Create attr table failed, status=%d", param->add_attr_tab.status);
            break;
        }
        if (param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(TAG, "Create attr table num_handle mismatch (%d)", param->add_attr_tab.num_handle);
            break;
        }
        memcpy(g_attr_handle_table, param->add_attr_tab.handles, sizeof(g_attr_handle_table));
        g_handle_svc  = g_attr_handle_table[IDX_SVC];
        g_handle_tx   = g_attr_handle_table[IDX_TX_VAL];
        g_handle_cccd = g_attr_handle_table[IDX_TX_CCCD];
        g_handle_rx   = g_attr_handle_table[IDX_RX_VAL];

        esp_ble_gatts_start_service(g_handle_svc);
        ESP_LOGI(TAG, "GATTS service started: svc=0x%04x tx=0x%04x rx=0x%04x cccd=0x%04x",
                 g_handle_svc, g_handle_tx, g_handle_rx, g_handle_cccd);
        break;

    case ESP_GATTS_CONNECT_EVT:
        g_gatts_connected = true;
        g_gatts_conn_id = param->connect.conn_id;
        g_tx_notify_enabled = false;
        ESP_LOGI(TAG, "GATTS connected (incoming), conn_id=%u", g_gatts_conn_id);

        // Request encryption on incoming link too (if peer supports)
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, "GATTS disconnected");
        g_gatts_connected = false;
        g_tx_notify_enabled = false;
        start_advertising();
        break;

    case ESP_GATTS_WRITE_EVT: {
        // RX writes from the central (glasses) arrive here.
        if (!param->write.is_prep) {
            uint16_t handle = param->write.handle;

            // CCCD write toggles notify
            if (handle == g_handle_cccd && param->write.len == 2) {
                uint16_t cfg = param->write.value[0] | (param->write.value[1] << 8);
                g_tx_notify_enabled = (cfg == 0x0001);
                ESP_LOGI(TAG, "GATTS CCCD write: notify=%s", g_tx_notify_enabled ? "ON" : "OFF");
                break;
            }

            if (handle == g_handle_rx) {
                ESP_LOGI(TAG, "GATTS RX write len=%d", param->write.len);
                ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);

                // OPTIONAL: echo back via notify on TX if enabled
                if (g_gatts_connected && g_tx_notify_enabled) {
                    uint16_t send_len = param->write.len > 20 ? 20 : param->write.len;
                    esp_ble_gatts_send_indicate(
                        g_gatts_if, g_gatts_conn_id, g_handle_tx,
                        send_len, param->write.value, false
                    );
                }
            }
        }
        break;
    }

    default:
        break;
    }
}

// -------------------- GATTC (Client / Central) --------------------
static esp_gatt_if_t g_gattc_if = ESP_GATT_IF_NONE;

static bool g_scan_active = false;
static bool g_scan_pending = false;
static uint32_t g_scan_pending_seconds = 0;
static bool g_have_target = false;
static bool g_connecting = false;
static bool g_connected = false;
static bool g_target_is_left = false;

static esp_bd_addr_t g_target_bda;
static esp_ble_addr_type_t g_target_addr_type = BLE_ADDR_TYPE_PUBLIC;
static esp_bd_addr_t g_left_bda = {0};
static esp_bd_addr_t g_right_bda = {0};

static uint16_t g_conn_id = 0;

// For discovery
static bool g_service_found = false;
static uint16_t g_svc_start = 0;
static uint16_t g_svc_end   = 0;
static bool g_tx_found = false;
static bool g_rx_found = false;
static uint16_t g_tx_handle = 0;
static uint16_t g_rx_handle = 0;

// “Python-like” scan: finite window; stop before connect
static esp_ble_scan_params_t g_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_RANDOM,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// Change these to match your glasses advertising names more precisely
static const char *TARGET_NAME_CONTAINS_1 = "G1_59_L";
static const char *TARGET_NAME_CONTAINS_2 = "G1_59_R";

enum class ConnectStage {
    WAIT_ANKI,
    LEFT,
    RIGHT,
    READY,
};

static ConnectStage g_stage = ConnectStage::WAIT_ANKI;
static bool g_left_connected = false;
static bool g_right_connected = false;

static uint8_t g_text_seq = 0;

void g1_send_text(const char *text)
{
    if (!text || !g_tx_handle) {
        ESP_LOGW("G1_TEXT", "Cannot send text (not ready)");
        return;
    }

    uint8_t pkt[256];
    size_t text_len = strlen(text);

    /* BLE payload safety (leave room for header) */
    if (text_len > 200) text_len = 200;

    uint8_t seq = g_text_seq++;

    /*
     * Text Send Packet (0x4E)
     * [0]  command              = 0x4E
     * [1]  seq
     * [2]  total_pkts           = 1
     * [3]  current_pkt          = 0
     * [4]  newscreen            = 0x71 (new content + text show)
     * [5]  char_pos_hi          = 0
     * [6]  char_pos_lo          = 0
     * [7]  current_page         = 0
     * [8]  max_page             = 1
     * [9+] UTF-8 text bytes
     */

    pkt[0] = 0x4E;
    pkt[1] = seq;
    pkt[2] = 1;
    pkt[3] = 0;
    pkt[4] = 0x71;
    pkt[5] = 0x00;
    pkt[6] = 0x00;
    pkt[7] = 0x00;
    pkt[8] = 0x01;

    memcpy(&pkt[9], text, text_len);

    esp_err_t err = esp_ble_gattc_write_char(
        g_gattc_if,
        g_conn_id,
        g_tx_handle,
        9 + text_len,
        pkt,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err == ESP_OK) {
        ESP_LOGI("G1_TEXT", "Text sent: \"%s\"", text);
    } else {
        ESP_LOGE("G1_TEXT", "Send failed: %s", esp_err_to_name(err));
    }
}


static void heartbeat_task(void *arg) {
    ESP_LOGI(TAG, "Heartbeat task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(4000));

        // Only send if GATTC link is up and we found the remote TX handle
        if (!g_connected || !g_tx_found || g_tx_handle == 0) continue;

        uint8_t hb[6];
        build_heartbeat(g_hb_seq++, hb);

        esp_err_t err = esp_ble_gattc_write_char(
            g_gattc_if,
            g_conn_id,
            g_tx_handle,          // remote TX char handle on the glasses
            sizeof(hb),
            hb,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE
        );

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HB -> glasses (GATTC write): %02x %02x %02x %02x %02x %02x",
                     hb[0], hb[1], hb[2], hb[3], hb[4], hb[5]);
        } else {
            ESP_LOGW(TAG, "HB write failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(4000));

        // Only send if GATTC link is up and we found the remote TX handle
        if (!g_connected || !g_tx_found || g_tx_handle == 0) continue;

        std::string_view v = g_tree.get_value({"Vroman Effect", "MW"});
        if (!v.empty()) {
            char* buf = static_cast<char*>(malloc(v.size() + 1)); // +1 for NUL
            if (buf) {
                memcpy(buf, v.data(), v.size());
                buf[v.size()] = '\0';

                g1_send_text(buf);

                free(buf);
            }
        }
    }
}

static void stop_scan(void);

static void scan_once(uint32_t seconds) {
    if (g_scan_active) {
        // Defer until current scan window ends/stops
        g_scan_pending = true;
        g_scan_pending_seconds = seconds;
        stop_scan();
        return;
    }

    g_scan_pending = false;
    ESP_LOGI(TAG, "GATTC: scan for %us", (unsigned)seconds);
    esp_err_t err = esp_ble_gap_start_scanning(seconds);
    if (err == ESP_OK) g_scan_active = true;
    else ESP_LOGW(TAG, "start_scanning failed: %s", esp_err_to_name(err));
}

static void stop_scan(void) {
    if (!g_scan_active) return;
    esp_ble_gap_stop_scanning();
    // clear g_scan_active only on STOP_COMPLETE
}

static void request_notify_enable(void);

// Compatibility: some IDF versions don’t have INVALID_HANDLE
#ifndef INVALID_HANDLE
#define INVALID_HANDLE 0
#endif

static void connect_target(void) {
    if (!g_have_target) return;
    if (g_connecting) return;
    if (g_connected && g_stage != ConnectStage::RIGHT) return;
    if (g_scan_active) {
        stop_scan(); // connect after scan stop complete
        return;
    }

    g_connecting = true;
    ESP_LOGI(TAG, "GATTC: connecting...");

    esp_ble_gatt_creat_conn_params_t params = {0};
    memcpy(params.remote_bda, g_target_bda, ESP_BD_ADDR_LEN);
    params.remote_addr_type = g_target_addr_type;
    params.own_addr_type    = BLE_ADDR_TYPE_RANDOM;
    params.is_direct        = true;
    params.is_aux           = false;
    params.phy_mask         = 0;

    esp_ble_gattc_enh_open(g_gattc_if, &params);
}

static void discover_services(void) {
    g_service_found = false;
    g_tx_found = false;
    g_rx_found = false;
    esp_ble_gattc_search_service(g_gattc_if, g_conn_id, NULL);
}

static void advance_stage_if_ready(void) {
    if (g_stage == ConnectStage::WAIT_ANKI && anki_remote_is_connected()) {
        ESP_LOGI(TAG, "Stage advance: WAIT_ANKI -> LEFT");
        g_stage = ConnectStage::LEFT;
        g_have_target = false;
        scan_once(5);
        return;
    }

    if (g_stage == ConnectStage::LEFT && g_left_connected) {
        ESP_LOGI(TAG, "Stage advance: LEFT -> RIGHT");
        g_stage = ConnectStage::RIGHT;
        g_have_target = false;
        g_connecting = false;
        g_connected = false;
        scan_once(5);
        return;
    }

    if (g_stage == ConnectStage::RIGHT && g_right_connected) {
        ESP_LOGI(TAG, "Stage advance: RIGHT -> READY");
        g_stage = ConnectStage::READY;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    anki_remote_on_gattc_event(event, gattc_if, param);

    advance_stage_if_ready();

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATTC registered, if=%d", gattc_if);
        g_gattc_if = gattc_if;
        esp_ble_gap_set_scan_params(&g_scan_params);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "GATTC OPEN failed status=%d", param->open.status);
            g_connecting = false;
            g_connected = false;
            g_have_target = false;
            scan_once(5);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "GATTC connected");
        g_connected = true;
        g_connecting = false;
        g_conn_id = param->connect.conn_id;

        // Request encryption immediately (important)
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);

        // MTU request helps some stacks
        esp_ble_gattc_send_mtu_req(gattc_if, g_conn_id);

        // Don’t start aggressive GATT ops until auth completes; but we’ll kick discovery anyway.
        discover_services();
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "GATTC disconnected reason=0x%x", param->disconnect.reason);
        g_connected = false;
        g_connecting = false;
        if (memcmp(param->disconnect.remote_bda, g_left_bda, ESP_BD_ADDR_LEN) == 0) {
            g_left_connected = false;
        }
        if (memcmp(param->disconnect.remote_bda, g_right_bda, ESP_BD_ADDR_LEN) == 0) {
            g_right_connected = false;
        }

        if (!g_right_connected) {
            g_stage = ConnectStage::RIGHT;
        } else if (!g_left_connected) {
            g_stage = ConnectStage::LEFT;
        }
        g_have_target = false;

        if (g_hb_task) {
            vTaskDelete(g_hb_task);
            g_hb_task = NULL;
        }
        g_hb_seq = 0;

        scan_once(5);
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        // IDF variation: srvc_id can be esp_gatt_id_t or wrapped; use as esp_gatt_id_t
        esp_gatt_id_t *srvc_id = &param->search_res.srvc_id;
        if (srvc_id->uuid.len == ESP_UUID_LEN_128 &&
            memcmp(srvc_id->uuid.uuid.uuid128, UUID_NUS_SERVICE, 16) == 0) {
            g_service_found = true;
            g_svc_start = param->search_res.start_handle;
            g_svc_end   = param->search_res.end_handle;
            ESP_LOGI(TAG, "Found NUS svc: 0x%04x..0x%04x", g_svc_start, g_svc_end);
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (!g_service_found) {
            ESP_LOGW(TAG, "NUS service not found on target.");
            break;
        }

        esp_bt_uuid_t tx_uuid = uuid128(UUID_NUS_TX);
        esp_bt_uuid_t rx_uuid = uuid128(UUID_NUS_RX);

        uint16_t count = 0;
        esp_gatt_status_t st;

        // TX char
        st = esp_ble_gattc_get_attr_count(
            gattc_if, g_conn_id, ESP_GATT_DB_CHARACTERISTIC,
            g_svc_start, g_svc_end, INVALID_HANDLE, &count);

        if (st == ESP_GATT_OK && count) {
            esp_gattc_char_elem_t *chars = (esp_gattc_char_elem_t*)malloc(sizeof(*chars) * count);
            if (chars) {
                uint16_t c = count;
                st = esp_ble_gattc_get_char_by_uuid(gattc_if, g_conn_id, g_svc_start, g_svc_end, tx_uuid, chars, &c);
                if (st == ESP_GATT_OK && c) {
                    g_tx_handle = chars[0].char_handle;
                    g_tx_found = true;
                    ESP_LOGI(TAG, "Found TX handle=0x%04x", g_tx_handle);

                    if (g_tx_found && g_hb_task == NULL) {
                        xTaskCreate(heartbeat_task, "hb_task", 4096, NULL, 5, &g_hb_task);
                    }
                }
                free(chars);
            }
        }

        // RX char
        count = 0;
        st = esp_ble_gattc_get_attr_count(
            gattc_if, g_conn_id, ESP_GATT_DB_CHARACTERISTIC,
            g_svc_start, g_svc_end, INVALID_HANDLE, &count);

        if (st == ESP_GATT_OK && count) {
            esp_gattc_char_elem_t *chars = (esp_gattc_char_elem_t*)malloc(sizeof(*chars) * count);
            if (chars) {
                uint16_t c = count;
                st = esp_ble_gattc_get_char_by_uuid(gattc_if, g_conn_id, g_svc_start, g_svc_end, rx_uuid, chars, &c);
                if (st == ESP_GATT_OK && c) {
                    g_rx_handle = chars[0].char_handle;
                    g_rx_found = true;
                    ESP_LOGI(TAG, "Found RX handle=0x%04x", g_rx_handle);
                }
                free(chars);
            }
        }

        if (g_rx_found) {
            request_notify_enable();
        }

        if (g_service_found && g_tx_found && g_rx_found) {
            if (g_target_is_left) {
                g_left_connected = true;
            } else {
                g_right_connected = true;
            }
            advance_stage_if_ready();
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(TAG, "GATTC notify len=%d", param->notify.value_len);
        ESP_LOG_BUFFER_HEX(TAG, param->notify.value, param->notify.value_len);
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        // IDF variants differ; we’ll just proceed to CCCD write using g_rx_handle/g_conn_id
        // (We registered for notify on g_rx_handle)
        ESP_LOGI(TAG, "Registered for notify");
        // CCCD write happens in request_notify_enable() flow below
        break;

    default:
        break;
    }
}

static void request_notify_enable(void) {
    if (!g_connected || !g_rx_found) return;
    ESP_LOGI(TAG, "Register for notify on handle=0x%04x", g_rx_handle);
    esp_ble_gattc_register_for_notify(g_gattc_if, g_target_bda, g_rx_handle);

    // Write CCCD = 0x0001
    esp_bt_uuid_t cccd_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}};
    uint16_t count = 0;

    esp_gatt_status_t st = esp_ble_gattc_get_attr_count(
        g_gattc_if, g_conn_id, ESP_GATT_DB_DESCRIPTOR,
        g_svc_start, g_svc_end, g_rx_handle, &count);

    if (st != ESP_GATT_OK || !count) {
        ESP_LOGW(TAG, "No descriptors on RX");
        return;
    }

    esp_gattc_descr_elem_t *descs = (esp_gattc_descr_elem_t*)malloc(sizeof(*descs) * count);
    if (!descs) return;

    uint16_t dcount = count;
    st = esp_ble_gattc_get_descr_by_char_handle(
        g_gattc_if, g_conn_id, g_rx_handle, cccd_uuid, descs, &dcount);

    if (st == ESP_GATT_OK && dcount) {
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
            g_gattc_if, g_conn_id, descs[0].handle,
            sizeof(notify_en), (uint8_t *)&notify_en,
            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE
        );
        ESP_LOGI(TAG, "CCCD written to enable notify");
    } else {
        ESP_LOGW(TAG, "CCCD not found");
    }

    free(descs);
}

// -------------------- GAP callback (shared) --------------------
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    anki_remote_on_gap_event(event, param);

    advance_stage_if_ready();

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // Start advertising once adv data configured
        start_advertising();
        break;

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Start first scan (finite window)
        scan_once(5);
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        g_scan_active = (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS);
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        g_scan_active = false;
        // If we found target while scanning, connect now
        if (g_have_target && !g_connected && !g_connecting) connect_target();

        if (g_scan_pending) {
            uint32_t secs = g_scan_pending_seconds;
            g_scan_pending = false;
            g_scan_pending_seconds = 0;
            scan_once(secs);
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;

        advance_stage_if_ready();

        if (g_stage == ConnectStage::WAIT_ANKI) break;
        if (g_stage == ConnectStage::LEFT && !anki_remote_is_connected()) break;

        // Resolve adv name
        uint8_t name_len = 0;
        const uint8_t *name = esp_ble_resolve_adv_data(
            param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &name_len
        );
        if (!name || !name_len) break;

        char nbuf[32] = {0};
        int cpy = name_len < 31 ? name_len : 31;
        memcpy(nbuf, name, cpy);

        // Extremely loose matching (you should tighten this!)
        bool match_left = strstr(nbuf, TARGET_NAME_CONTAINS_1) != NULL;
        bool match_right = strstr(nbuf, TARGET_NAME_CONTAINS_2) != NULL;
        bool match = false;

        if (g_stage == ConnectStage::LEFT) {
            match = match_left;
        } else if (g_stage == ConnectStage::RIGHT) {
            match = match_right;
        }
        if (match) {
            ESP_LOGI(TAG, "Found candidate: %s", nbuf);
            memcpy(g_target_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
            g_target_addr_type = param->scan_rst.ble_addr_type;
            g_have_target = true;
            g_target_is_left = (g_stage == ConnectStage::LEFT);
            if (g_target_is_left) {
                memcpy(g_left_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
            } else {
                memcpy(g_right_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
            }

            // Stop scan then connect (python-style)
            if (g_scan_active) {
                stop_scan();
            } else {
                ESP_LOGI(TAG, "Scan already stopped; connecting now");
                connect_target();
            }
        }
        break;
    }

    // Security events
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "SEC_REQ: accepting");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        // Numeric comparison request (LESC). If you don't confirm, pairing can fail.
        ESP_LOGW(TAG, "NC_REQ: auto-confirming");
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "AUTH %s fail_reason=0x%x",
                 param->ble_security.auth_cmpl.success ? "OK" : "FAIL",
                 param->ble_security.auth_cmpl.fail_reason);
        break;

    default:
        break;
    }
}

// -------------------- app_main --------------------
extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    g_tree.set_text(kData, sizeof(kData) - 1); // no copy
    g_tree.build();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    setup_privacy();
    setup_security();

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_cb));

    anki_remote_init();

    // Register both apps
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0xAB));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0xCD));

    // Scan params set triggers SCAN_PARAM_SET_COMPLETE_EVT -> scan_once
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&g_scan_params));
}
