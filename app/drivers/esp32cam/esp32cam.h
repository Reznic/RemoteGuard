/*
 * ESP32-CAM high-level driver API
 *
 * This header exposes high-level operations (power, flash, capture)
 * implemented by the ESP32-CAM driver bound to the esp32,cam-spi node.
 */

#ifndef ESP32CAM_H__
#define ESP32CAM_H__

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* Get the ESP32-CAM device instance (first esp32,cam-spi node) */
const struct device *esp32cam_get_device(void);

/* High-level control APIs */
int esp32cam_turn_on(const struct device *dev);
int esp32cam_turn_off(const struct device *dev);
int esp32cam_flash_on(const struct device *dev);
int esp32cam_flash_off(const struct device *dev);

/* Capture APIs:
 *  - esp32cam_capture_length: trigger capture and return total image length
 *  - esp32cam_capture_read: read 'len' bytes from the capture stream
 */
int esp32cam_capture_length(const struct device *dev, uint32_t *image_length);
int esp32cam_capture_read(const struct device *dev, uint8_t *buf, size_t len);

#endif /* ESP32CAM_H__ */

