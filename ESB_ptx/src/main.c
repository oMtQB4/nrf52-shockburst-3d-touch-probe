/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <pm/pm.h>

#include <logging/log_ctrl.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static volatile bool button_pressed = false;
static struct gpio_callback button_cb_data;


static bool ready = true;
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x01, 0x00);

#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload) == 0) {
			LOG_DBG("Packet received, len %d : "
				"0x%02x, 0x%02x",
				rx_payload.length, rx_payload.data[0],
				rx_payload.data[1]);
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE2, 0xE5, 0xEA, 0xE3};
	uint8_t base_addr_1[4] = {0xC5, 0xC1, 0xCA, 0xC3};
	uint8_t addr_prefix[8] = {0xEA, 0xCA, 0xC4, 0xC2, 0xC7, 0xB1, 0xC3, 0xC1};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;
  config.tx_output_power = ESB_TX_POWER_4DBM;

	err = esb_init(&config);

	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

static int led_init(void)
{
	if (!device_is_ready(led.port)) {
		LOG_ERR("LEDs port not ready");
		return -ENODEV;
	}

  int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT);

  if (err) {
    LOG_ERR("Unable to configure LED, err %d.", err);
    return err;
  }

	return 0;
}

static void led_update(bool value)
{
	gpio_port_pins_t mask = BIT(led.pin);

	gpio_port_value_t val = value << led.pin;

	(void)gpio_port_set_masked_raw(led.port, mask, val);
}

static void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  button_pressed = gpio_pin_get(button.port, button.pin) == 1;

  LOG_INF("Button pressed: %d", button_pressed);
}

static int button_initialize(gpio_callback_handler_t btn_prs_cb) {
  int ret;
	if (!device_is_ready(button.port)) {
		LOG_ERR("Error: button device %s is not ready", button.port->name);
		return -1;
	}
  
  ret = gpio_pin_configure(button.port, button.pin, GPIO_INPUT);
  //ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d", ret, button.port->name, button.pin);
		return ret;
	}
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, button.port->name, button.pin);
		return ret;
	}

	gpio_init_callback(&button_cb_data, btn_prs_cb, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	LOG_INF("Initialized on port %s, pin %d", button.port->name, button.pin);

  return ret;
}

void main(void)
{
	int err;

	LOG_INF("Starting ShockBurst 3D Touch Probe firmware");

	err = clocks_start();
	if (err) {
		return;
	}

	err = led_init();
	if (err) {
		return;
	}

  err = button_initialize(button_pressed_cb);
  if (err) {
    return;
  }

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	LOG_INF("Initialization complete. Starting main loop");

	tx_payload.noack = false;
	while (1) {
		if (button_pressed) {
      esb_flush_tx();

			err = esb_write_payload(&tx_payload);
			if (err) {
				LOG_ERR("Payload write failed, err %d", err);
			}
			tx_payload.data[1]++;

      k_sleep(K_MSEC(10));
		}

  	led_update(!button_pressed);

    if (LOG_PROCESS() == false) {
			pm_state_set(PM_STATE_RUNTIME_IDLE, 0);
		}
	}
}
