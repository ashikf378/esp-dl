set(src_dirs ./)
set(include_dirs ./)

# Base requirements
set(requires hand_detect nvs_flash bt)

# Board‑specific LCD support
if (IDF_TARGET STREQUAL "esp32s3")
    list(APPEND requires esp32_s3_eye_noglib esp_lcd)
elseif (IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp32_p4_function_ev_board_noglib esp_lcd)
endif()

# Embedded file (used by hand detection example)
set(embed_files "hand.jpg")

# Register both source files
idf_component_register(SRCS "lcd_display.cpp" "app_main.cpp"
                       INCLUDE_DIRS ${include_dirs}
                       REQUIRES ${requires})