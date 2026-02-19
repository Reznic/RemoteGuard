#
# ESP32-CAM custom SPI camera driver
#

zephyr_library()
zephyr_library_sources(esp32cam.c)
zephyr_include_directories(.)

target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/esp32cam.c)

