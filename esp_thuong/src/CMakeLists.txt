# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.c" "sub.c"
    INCLUDE_DIRS "." "../include"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_i2c
                  esp_rom esp_lcd esp_timer heap mbedtls esp_hw_support
                  nvs_flash esp_wifi esp_event esp_netif esp_http_client
)
