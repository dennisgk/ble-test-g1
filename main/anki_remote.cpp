#include "anki_remote.hpp"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "ANKI_REMOTE";
static const char *TARGET_NAME = "Anki Remote";

static const uint16_t APP_ID_ANKI = 0x42;

static bool s_inited = false;
static bool s_found = false;
static bool s_connecting = false;

static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0xFFFF;
static esp_bd_addr_t s_bda = {0};
static esp_ble_addr_type_t s_addr_type = BLE_ADDR_TYPE_PUBLIC;

// HID service range
static bool s_hid_service_found = false;
static uint16_t s_hid_start = 0;
static uint16_t s_hid_end = 0;

// Handles for report notify
static uint16_t s_report_char_handle = 0;
static uint16_t s_report_cccd_handle = 0;

static char hid_key_to_ascii(uint8_t key, bool shift) {
    if (key >= 0x04 && key <= 0x1D) { // a-z
        char c = (char)('a' + (key - 0x04));
        return shift ? (char)(c - 32) : c;
    }
    if (key >= 0x1E && key <= 0x27) { // 1-0
        static const char no_shift[] = {'1','2','3','4','5','6','7','8','9','0'};
        static const char yes_shift[] = {'!','@','#','$','%','^','&','*','(',')'};
        return shift ? yes_shift[key - 0x1E] : no_shift[key - 0x1E];
    }
    switch (key) {
        case 0x28: return '\n'; // Enter
        case 0x2C: return ' ';  // Space
        default:   return 0;
    }
}

static void print_keyboardish_report(const uint8_t *data, uint16_t len) {
    ESP_LOGI(TAG, "HID report len=%u", (unsigned)len);
    ESP_LOG_BUFFER_HEX(TAG, data, len);

    if (len < 8) return;

    uint8_t mods = data[0];
    bool shift = (mods & (1 << 1)) || (mods & (1 << 5)); // LShift or RShift

    for (int i = 2; i < 8; i++) {
        uint8_t key = data[i];
        if (!key) continue;

        char c = hid_key_to_ascii(key, shift);
        if (c == '\n') ESP_LOGI(TAG, "Key: ENTER");
        else if (c)    ESP_LOGI(TAG, "Key: '%c' (usage 0x%02X) shift=%d", c, key, (int)shift);
        else           ESP_LOGI(TAG, "Key: usage 0x%02X shift=%d", key, (int)shift);
    }
}

static bool adv_name_matches(const uint8_t *adv, uint8_t adv_len) {
    (void)adv_len;

    uint8_t name_len = 0;
    uint8_t *name = esp_ble_resolve_adv_data((uint8_t*)adv, ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
    if (!name || !name_len) {
        name = esp_ble_resolve_adv_data((uint8_t*)adv, ESP_BLE_AD_TYPE_NAME_SHORT, &name_len);
        if (!name || !name_len) return false;
    }

    char nbuf[64] = {0};
    int cpy = (name_len < (sizeof(nbuf) - 1)) ? name_len : (sizeof(nbuf) - 1);
    memcpy(nbuf, name, cpy);

    ESP_LOGI("TEST", "Found: %s", nbuf);

    return (strcmp(nbuf, TARGET_NAME) == 0);
}

static void start_connect(void) {
    if (s_gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGW(TAG, "Not registered yet (no gattc_if)");
        return;
    }
    if (s_connecting) return;

    s_connecting = true;
    ESP_LOGI(TAG, "Connecting to \"%s\"...", TARGET_NAME);
    ESP_LOG_BUFFER_HEX(TAG, s_bda, 6);

    esp_err_t err = esp_ble_gattc_open(s_gattc_if, s_bda, s_addr_type, true);
    if (err != ESP_OK) {
        s_connecting = false;
        ESP_LOGE(TAG, "esp_ble_gattc_open failed: %s", esp_err_to_name(err));
    }
}

static void discover_report_char_and_subscribe(void) {
    if (!s_hid_service_found) return;

    // Report characteristic UUID 0x2A4D
    esp_bt_uuid_t report_uuid = {};
    report_uuid.len = ESP_UUID_LEN_16;
    report_uuid.uuid.uuid16 = 0x2A4D;

    uint16_t count = 0;
    esp_gatt_status_t st = esp_ble_gattc_get_attr_count(
        s_gattc_if,
        s_conn_id,
        ESP_GATT_DB_CHARACTERISTIC,
        s_hid_start,
        s_hid_end,
        0,
        &count
    );

    if (st != ESP_GATT_OK || count == 0) {
        ESP_LOGE(TAG, "No characteristics in HID service (st=%d count=%u)", (int)st, (unsigned)count);
        return;
    }

    esp_gattc_char_elem_t *chars = (esp_gattc_char_elem_t*)calloc(count, sizeof(esp_gattc_char_elem_t));
    if (!chars) return;

    uint16_t out = count;
    st = esp_ble_gattc_get_char_by_uuid(
        s_gattc_if,
        s_conn_id,
        s_hid_start,
        s_hid_end,
        report_uuid,
        chars,
        &out
    );

    if (st != ESP_GATT_OK || out == 0) {
        ESP_LOGE(TAG, "Report char (0x2A4D) not found (st=%d out=%u)", (int)st, (unsigned)out);
        free(chars);
        return;
    }

    s_report_char_handle = chars[0].char_handle;
    ESP_LOGI(TAG, "Report char handle=0x%04X props=0x%02X",
             s_report_char_handle, chars[0].properties);

    free(chars);

    // CCCD UUID 0x2902
    esp_bt_uuid_t cccd_uuid = {};
    cccd_uuid.len = ESP_UUID_LEN_16;
    cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    uint16_t dcount = 0;
    st = esp_ble_gattc_get_attr_count(
        s_gattc_if,
        s_conn_id,
        ESP_GATT_DB_DESCRIPTOR,
        s_hid_start,
        s_hid_end,
        s_report_char_handle,   // here IDF expects "char_handle" for descriptor count
        &dcount
    );

    if (st != ESP_GATT_OK || dcount == 0) {
        ESP_LOGE(TAG, "No descriptors for report char (st=%d dcount=%u)", (int)st, (unsigned)dcount);
        return;
    }

    esp_gattc_descr_elem_t *descs = (esp_gattc_descr_elem_t*)calloc(dcount, sizeof(esp_gattc_descr_elem_t));
    if (!descs) return;

    uint16_t dout = dcount;

    // IMPORTANT: get_descr_by_uuid uses *characteristic UUID*, not handle, in your IDF.
    st = esp_ble_gattc_get_descr_by_uuid(
        s_gattc_if,
        s_conn_id,
        s_hid_start,
        s_hid_end,
        report_uuid,   // <-- UUID, not handle
        cccd_uuid,
        descs,
        &dout
    );

    if (st != ESP_GATT_OK || dout == 0) {
        ESP_LOGE(TAG, "CCCD not found for report char (st=%d dout=%u)", (int)st, (unsigned)dout);
        free(descs);
        return;
    }

    // In your IDF, the field is "handle"
    s_report_cccd_handle = descs[0].handle;
    ESP_LOGI(TAG, "CCCD handle=0x%04X", s_report_cccd_handle);
    free(descs);

    esp_err_t err = esp_ble_gattc_register_for_notify(s_gattc_if, s_bda, s_report_char_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register_for_notify failed: %s", esp_err_to_name(err));
        return;
    }

    uint8_t notify_en[2] = {0x01, 0x00};
    err = esp_ble_gattc_write_char_descr(
        s_gattc_if,
        s_conn_id,
        s_report_cccd_handle,
        sizeof(notify_en),
        notify_en,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write CCCD failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Subscribed to Anki Remote key reports.");
}

void anki_remote_init(void) {
    if (s_inited) return;
    s_inited = true;

    esp_err_t err = esp_ble_gattc_app_register(APP_ID_ANKI);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gattc_app_register failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "anki_remote_init ok (app_id=%u)", (unsigned)APP_ID_ANKI);
    }
}

void anki_remote_on_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (!s_inited) return;

    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) return;
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) return;

    if (s_found) return;

    if (!adv_name_matches(param->scan_rst.ble_adv, param->scan_rst.adv_data_len)) return;

    s_found = true;
    memcpy(s_bda, param->scan_rst.bda, sizeof(esp_bd_addr_t));
    s_addr_type = (esp_ble_addr_type_t)param->scan_rst.ble_addr_type;

    ESP_LOGI(TAG, "Found target \"%s\" in scan. Stopping scan + connecting.", TARGET_NAME);
    esp_ble_gap_stop_scanning();

    start_connect();
}

void anki_remote_on_gattc_event(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param) {
    if (!s_inited) return;

    switch (event) {
        case ESP_GATTC_REG_EVT: {
            if (param->reg.app_id == APP_ID_ANKI) {
                s_gattc_if = gattc_if;
                ESP_LOGI(TAG, "Registered GATTC app (if=%d)", (int)s_gattc_if);

                if (s_found) start_connect();
            }
            break;
        }

        case ESP_GATTC_OPEN_EVT: {
            if (gattc_if != s_gattc_if) break;

            if (param->open.status != ESP_GATT_OK) {
                s_connecting = false;
                ESP_LOGE(TAG, "OPEN failed status=%d", (int)param->open.status);
                break;
            }

            s_conn_id = param->open.conn_id;
            memcpy(s_bda, param->open.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(TAG, "Connected conn_id=%u", (unsigned)s_conn_id);

            s_hid_service_found = false;
            s_hid_start = s_hid_end = 0;
            s_report_char_handle = s_report_cccd_handle = 0;

            esp_ble_gattc_search_service(s_gattc_if, s_conn_id, nullptr);
            break;
        }

        case ESP_GATTC_SEARCH_RES_EVT: {
            if (gattc_if != s_gattc_if) break;

            // In your IDF, srvc_id is esp_gatt_id_t (not esp_gatt_srvc_id_t)
            esp_gatt_id_t *sid = &param->search_res.srvc_id;

            if (sid->uuid.len == ESP_UUID_LEN_16 && sid->uuid.uuid.uuid16 == 0x1812) {
                s_hid_service_found = true;
                s_hid_start = param->search_res.start_handle;
                s_hid_end   = param->search_res.end_handle;
                ESP_LOGI(TAG, "Found HID service 0x1812 start=0x%04X end=0x%04X", s_hid_start, s_hid_end);
            }
            break;
        }

        case ESP_GATTC_SEARCH_CMPL_EVT: {
            if (gattc_if != s_gattc_if) break;

            ESP_LOGI(TAG, "Service discovery complete (status=%d)", (int)param->search_cmpl.status);
            if (param->search_cmpl.status == ESP_GATT_OK && s_hid_service_found) {
                discover_report_char_and_subscribe();
            } else {
                ESP_LOGE(TAG, "HID service not found or discovery failed.");
            }
            break;
        }

        case ESP_GATTC_NOTIFY_EVT: {
            if (gattc_if != s_gattc_if) break;

            if (param->notify.handle == s_report_char_handle) {
                ESP_LOGI(TAG, "Key report notify:");
                print_keyboardish_report(param->notify.value, param->notify.value_len);
            }
            break;
        }

        case ESP_GATTC_DISCONNECT_EVT: {
            if (gattc_if != s_gattc_if) break;

            ESP_LOGW(TAG, "Disconnected reason=0x%02X", param->disconnect.reason);
            s_connecting = false;
            s_conn_id = 0xFFFF;
            s_hid_service_found = false;
            s_report_char_handle = 0;
            s_report_cccd_handle = 0;
            s_found = false; // allow re-find/reconnect
            break;
        }

        default:
            break;
    }
}
