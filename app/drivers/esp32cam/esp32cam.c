/*
 * ESP32-CAM custom SPI camera driver
 *
 * This driver binds to the esp32,cam-spi devicetree node and exposes
 * a Zephyr device that encapsulates the SPI connection to the external
 * ESP32-CAM module, providing high-level camera operations.
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/logging/log.h>
 
 #include "esp32cam.h"
 
 LOG_MODULE_REGISTER(esp32cam_drv, CONFIG_ESP32CAM_DRIVER_LOG_LEVEL);
 
 #define DT_DRV_COMPAT esp32_cam_spi
 
 /* ESP32-CAM command opcodes */
 #define ESP32CAM_CMD_ON           0x01
 #define ESP32CAM_CMD_OFF          0x02
 #define ESP32CAM_CMD_CAPTURE      0x03
 #define ESP32CAM_CMD_FLASH_ON     0x04
 #define ESP32CAM_CMD_FLASH_OFF    0x05
 
 /* ESP32-CAM status codes */
 #define ESP32CAM_STATUS_NOT_RESPONDING        0x00
 #define ESP32CAM_STATUS_ACK                   0x01
 #define ESP32CAM_STATUS_ERR                   0x02
 #define ESP32CAM_STATUS_CAMERA_ACCESS_FAILED  0x03
 
 struct esp32cam_config {
	 struct spi_dt_spec spi;
 };
 
 static int esp32cam_init(const struct device *dev)
 {
	 const struct esp32cam_config *cfg = dev->config;
 
	 if (!spi_is_ready_dt(&cfg->spi)) {
		 LOG_ERR("Underlying SPI bus/device not ready");
		 return -ENODEV;
	 }
 
	 LOG_INF("ESP32-CAM driver initialized");
	 return 0;
 }
 
 #define ESP32CAM_INIT(inst)                                                     \
	 static const struct esp32cam_config esp32cam_cfg_##inst = {             \
		 .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) |             \
							 SPI_TRANSFER_MSB, 0),         \
	 };                                                                     \
	 DEVICE_DT_INST_DEFINE(inst, &esp32cam_init, NULL, NULL,               \
				   &esp32cam_cfg_##inst, POST_KERNEL,                \
				   CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
 
 DT_INST_FOREACH_STATUS_OKAY(ESP32CAM_INIT)
 
 static inline const struct esp32cam_config *esp32cam_get_cfg(const struct device *dev)
 {
	 return dev->config;
 }
 
 /* Low-level SPI helpers */
 
 static int esp32cam_spi_send_cmd(const struct esp32cam_config *cfg,
				  uint8_t cmd, uint8_t *status)
 {
	 int ret;
	 uint8_t tx_buf[1] = { cmd };
	 uint8_t rx_buf[1] = { 0 };
 
	 const struct spi_buf tx_bufs[] = {
		 {
			 .buf = tx_buf,
			 .len = sizeof(tx_buf),
		 },
	 };
	 const struct spi_buf rx_bufs[] = {
		 {
			 .buf = rx_buf,
			 .len = sizeof(rx_buf),
		 },
	 };
	 const struct spi_buf_set tx = {
		 .buffers = tx_bufs,
		 .count = 1,
	 };
	 const struct spi_buf_set rx = {
		 .buffers = rx_bufs,
		 .count = 1,
	 };
 
	 if (!spi_is_ready_dt(&cfg->spi)) {
		 LOG_ERR("SPI device not ready");
		 return -ENODEV;
	 }
 
	 ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	 if (ret) {
		 LOG_ERR("SPI transceive failed: %d", ret);
		 return ret;
	 }
 
	 if (status) {
		 *status = rx_buf[0];
		 if (*status == ESP32CAM_STATUS_NOT_RESPONDING) {
			 LOG_ERR("ESP32-CAM not responding (status = 0x00)");
			 return -ENODEV;
		 }
	 }
 
	 return 0;
 }
 
 static int esp32cam_spi_read(const struct esp32cam_config *cfg,
				  uint8_t *data, size_t len)
 {
	 int ret;
	 const struct spi_buf rx_bufs[] = {
		 {
			 .buf = data,
			 .len = len,
		 },
	 };
	 const struct spi_buf_set rx = {
		 .buffers = rx_bufs,
		 .count = 1,
	 };
 
	 if (!spi_is_ready_dt(&cfg->spi)) {
		 LOG_ERR("SPI device not ready");
		 return -ENODEV;
	 }
 
	 ret = spi_read_dt(&cfg->spi, &rx);
	 if (ret) {
		 LOG_ERR("SPI read failed: %d", ret);
		 return ret;
	 }
 
	 return 0;
 }
 
 /* Public API */
 
 const struct device *esp32cam_get_device(void)
 {
	 const struct device *dev = DEVICE_DT_GET_ANY(esp32_cam_spi);
 
	 if (dev == NULL) {
		 LOG_ERR("No ESP32-CAM device found");
	 }
 
	 return dev;
 }
 
 int esp32cam_turn_on(const struct device *dev)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
	 uint8_t status = 0;
	 int ret = esp32cam_spi_send_cmd(cfg, ESP32CAM_CMD_ON, &status);
 
	 if (ret) {
		 return ret;
	 }
 
	 if (status != ESP32CAM_STATUS_ACK) {
		 LOG_ERR("TURN_ON returned status 0x%02x", status);
		 return -EIO;
	 }
 
	 return 0;
 }
 
 int esp32cam_turn_off(const struct device *dev)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
	 uint8_t status = 0;
	 int ret = esp32cam_spi_send_cmd(cfg, ESP32CAM_CMD_OFF, &status);
 
	 if (ret) {
		 return ret;
	 }
 
	 if (status != ESP32CAM_STATUS_ACK) {
		 LOG_ERR("TURN_OFF returned status 0x%02x", status);
		 return -EIO;
	 }
 
	 return 0;
 }
 
 int esp32cam_flash_on(const struct device *dev)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
	 uint8_t status = 0;
	 int ret = esp32cam_spi_send_cmd(cfg, ESP32CAM_CMD_FLASH_ON, &status);
 
	 if (ret) {
		 return ret;
	 }
 
	 if (status != ESP32CAM_STATUS_ACK) {
		 LOG_ERR("FLASH_ON returned status 0x%02x", status);
		 return -EIO;
	 }
 
	 return 0;
 }
 
 int esp32cam_flash_off(const struct device *dev)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
	 uint8_t status = 0;
	 int ret = esp32cam_spi_send_cmd(cfg, ESP32CAM_CMD_FLASH_OFF, &status);
 
	 if (ret) {
		 return ret;
	 }
 
	 if (status != ESP32CAM_STATUS_ACK) {
		 LOG_ERR("FLASH_OFF returned status 0x%02x", status);
		 return -EIO;
	 }
 
	 return 0;
 }
 
 int esp32cam_capture_length(const struct device *dev, uint32_t *image_length)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
	 uint8_t status = 0;
	 int ret;
	 uint8_t length_buf[4];
 
	 if (image_length == NULL) {
		 return -EINVAL;
	 }
 
	 ret = esp32cam_spi_send_cmd(cfg, ESP32CAM_CMD_CAPTURE, &status);
	 if (ret) {
		 return ret;
	 }
 
	 if (status == ESP32CAM_STATUS_CAMERA_ACCESS_FAILED) {
		 LOG_ERR("ESP32 camera access failed");
		 return -EIO;
	 } else if (status != ESP32CAM_STATUS_ACK) {
		 LOG_ERR("CAPTURE returned status 0x%02x", status);
		 return -EIO;
	 }
 
	 ret = esp32cam_spi_read(cfg, length_buf, sizeof(length_buf));
	 if (ret) {
		 LOG_ERR("Failed to read image length: %d", ret);
		 return ret;
	 }
 
	 *image_length = (length_buf[3] << 24) |
			 (length_buf[2] << 16) |
			 (length_buf[1] << 8)  |
			 length_buf[0];
 
	 if (*image_length == 0 || *image_length > 1024 * 1024) {
		 LOG_ERR("Invalid image length: %u", *image_length);
		 return -EINVAL;
	 }
 
	 LOG_INF("Image length: %u bytes", *image_length);
	 return 0;
 }
 
 int esp32cam_capture_read(const struct device *dev, uint8_t *buf, size_t len)
 {
	 const struct esp32cam_config *cfg = esp32cam_get_cfg(dev);
 
	 if (buf == NULL || len == 0) {
		 return -EINVAL;
	 }
 
	 return esp32cam_spi_read(cfg, buf, len);
 }