FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "../include"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_ledc
                  esp_timer esp_rom esp_hw_support heap
)