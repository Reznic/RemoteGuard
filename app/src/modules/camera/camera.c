/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>
#include <string.h>

#include "message_channel.h"
#include "esp32cam.h"

/* Register log module */
LOG_MODULE_REGISTER(camera, CONFIG_APP_LOG_LEVEL);

/* Register subscriber */
ZBUS_SUBSCRIBER_DEFINE(camera, CONFIG_MQTT_SAMPLE_CAMERA_MESSAGE_QUEUE_SIZE);

static enum camera_error_type map_esp32cam_error(int ret)
{
	if (ret == -ENODEV) {
		return CAMERA_ERROR_ESP32_NOT_RESPONDING;
	} else if (ret == -EIO) {
		return CAMERA_ERROR_ESP32_CAMERA_ACCESS_FAILED;
	} else {
		return CAMERA_ERROR_NRF_INTERNAL;
	}
}

/* Handle camera command via esp32cam driver */
static int handle_camera_cmd(const struct device *esp32cam_dev,
			     enum camera_cmd cmd)
{
	int ret;

	switch (cmd) {
	case CAMERA_CMD_CAMERA_ON:
		ret = esp32cam_turn_on(esp32cam_dev);
		if (ret) {
			enum camera_error_type error = map_esp32cam_error(ret);
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
		}
		return ret;

	case CAMERA_CMD_CAMERA_OFF:
		ret = esp32cam_turn_off(esp32cam_dev);
		if (ret) {
			enum camera_error_type error = map_esp32cam_error(ret);
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
		}
		return ret;

	case CAMERA_CMD_FLASH_ON:
		ret = esp32cam_flash_on(esp32cam_dev);
		if (ret) {
			enum camera_error_type error = map_esp32cam_error(ret);
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
		}
		return ret;

	case CAMERA_CMD_FLASH_OFF:
		ret = esp32cam_flash_off(esp32cam_dev);
		if (ret) {
			enum camera_error_type error = map_esp32cam_error(ret);
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
		}
		return ret;

	case CAMERA_CMD_TAKE_PHOTO: {
		uint32_t image_length = 0;

		ret = esp32cam_capture_length(esp32cam_dev, &image_length);
		if (ret) {
			enum camera_error_type error = map_esp32cam_error(ret);
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
			return ret;
		}

		uint32_t chunk_size = CONFIG_MQTT_SAMPLE_CAMERA_CHUNK_SIZE;
		uint32_t total_chunks = (image_length + chunk_size - 1) / chunk_size;

		uint32_t remaining = image_length;
		uint32_t sequence = 0;
		uint8_t *read_buf = k_malloc(chunk_size);
		if (!read_buf) {
			LOG_ERR("Failed to allocate read buffer");
			enum camera_error_type error = CAMERA_ERROR_NRF_INTERNAL;
			zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
			return -ENOMEM;
		}

		while (remaining > 0) {
			uint32_t to_read = (remaining > chunk_size) ? chunk_size : remaining;

			ret = esp32cam_capture_read(esp32cam_dev, read_buf, to_read);
			if (ret) {
				LOG_ERR("Failed to read image data: %d", ret);
				k_free(read_buf);
				enum camera_error_type error = map_esp32cam_error(ret);
				zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
				return ret;
			}

			struct camera_chunk chunk;
			memcpy(chunk.data, read_buf, to_read);
			chunk.size = to_read;
			chunk.sequence = sequence;
			chunk.total_chunks = total_chunks;
			chunk.total_size = image_length;

			ret = zbus_chan_pub(&CAMERA_CHUNK_CHAN, &chunk, K_SECONDS(1));
			if (ret) {
				LOG_ERR("Failed to publish chunk: %d", ret);
				k_free(read_buf);
				enum camera_error_type error = CAMERA_ERROR_NRF_INTERNAL;
				zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
				return ret;
			}

			LOG_DBG("Published chunk %u/%u (%u bytes)",
				sequence + 1, total_chunks, to_read);
			remaining -= to_read;
			sequence++;
		}

		k_free(read_buf);
		LOG_INF("Image capture complete: %u chunks", total_chunks);
		return 0;
	}

	default:
		LOG_ERR("Unknown camera command: %d", cmd);
		return -EINVAL;
	}
}

/* Camera task */
static void camera_task(void)
{
	int err;
	const struct zbus_channel *chan;
	enum camera_cmd cmd;

	const struct device *esp32cam_dev = esp32cam_get_device();
	if (esp32cam_dev == NULL || !device_is_ready(esp32cam_dev)) {
		LOG_ERR("ESP32-CAM driver not ready");
		enum camera_error_type error = CAMERA_ERROR_NRF_INTERNAL;
		zbus_chan_pub(&CAMERA_ERROR_CHAN, &error, K_SECONDS(1));
		return;
	}

	LOG_INF("Camera module initialized");

	while (!zbus_sub_wait(&camera, &chan, K_FOREVER)) {
		if (chan == &CAMERA_CMD_CHAN) {
			err = zbus_chan_read(&CAMERA_CMD_CHAN, &cmd, K_SECONDS(1));
			if (err) {
				LOG_ERR("zbus_chan_read, error: %d", err);
				continue;
			}

			LOG_INF("Received camera command: %d", cmd);
			err = handle_camera_cmd(esp32cam_dev, cmd);
			if (err) {
				LOG_ERR("Failed to handle camera command: %d", err);
			}
		}
	}
}

K_THREAD_DEFINE(camera_task_id,
		CONFIG_MQTT_SAMPLE_CAMERA_THREAD_STACK_SIZE,
		camera_task, NULL, NULL, NULL, 3, 0, 0);
