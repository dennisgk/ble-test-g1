idf_component_register(SRCS "ble-test-g1.cpp" "TextTree.cpp" "anki_remote.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt esp_hid)
