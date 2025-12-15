#pragma once
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void anki_remote_init(void);

// Forward your existing GAP callback events into this
void anki_remote_on_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// Forward your existing GATTC callback events into this
void anki_remote_on_gattc_event(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param);

// Simple connectivity helpers for orchestration
bool anki_remote_is_connected(void);

#ifdef __cplusplus
}
#endif
