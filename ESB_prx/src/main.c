/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <logging/log_ctrl.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec status = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

#define STATUS_ACTIVE     (true)
#define STATUS_INACTIVE   (false)

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11);

static bool rx_received = false;


static void status_update(bool value) {
  gpio_port_pins_t mask = BIT(status.pin);

	gpio_port_value_t val = value << status.pin;

	(void)gpio_port_set_masked_raw(status.port, mask, val);

  LOG_INF("STATUS %d set to %d.", status.pin, value);
}

static void led_update(bool value)
{
	gpio_port_pins_t mask = BIT(led.pin);

	gpio_port_value_t val = value << led.pin;

	(void)gpio_port_set_masked_raw(led.port, mask, val);

  LOG_INF("LED %d set to %d.", led.pin, value);
}

static int gpios_init(void)
{
	if (!device_is_ready(led.port) || !device_is_ready(status.port)) {
		LOG_ERR("LED or status port not ready");
		return -ENODEV;
	}

  int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
  if (err) {
    LOG_ERR("Unable to configure LED gpio, err %d.", err);
    return err;
  }
  led_update(true);

  err = gpio_pin_configure_dt(&status, GPIO_OUTPUT);
  if (err) {
    LOG_ERR("Unable to configure STATUS gpio, err %d.", err);
    return err;
  }
  status_update(STATUS_INACTIVE);

	return 0;
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			LOG_INF("Packet received, len %d : "
				"0x%02x, 0x%02x",
				rx_payload.length, rx_payload.data[0],
				rx_payload.data[1]);

      rx_received = true;
		} else {
			LOG_ERR("Error while reading rx packet");
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
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;

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

static bool led_state = false;

static void recheck_timer_expired(struct k_timer *timer_id) {
  status_update(STATUS_INACTIVE);

  led_state = true;
  led_update(led_state);
}
K_TIMER_DEFINE(recheck_timer, recheck_timer_expired, NULL);

static void activity_timer_expired(struct k_timer *timer_id) {
  led_state = !led_state;
  led_update(led_state);
  k_timer_start(timer_id, (led_state ? K_MSEC(5000) : K_MSEC(100)), K_NO_WAIT);
  LOG_INF("Timer expired");
}
K_TIMER_DEFINE(activity_timer, activity_timer_expired, NULL);

void main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst prx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	err = gpios_init();
	if (err) {
		return;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	LOG_INF("Initialization complete");

	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Write payload, err %d", err);
		return;
	}

	LOG_INF("Setting up for packet receiption");

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return;
	}

  led_state = true;

  k_timer_start(&activity_timer, K_MSEC(5000), K_NO_WAIT);

  while(true) {
    if (rx_received) {
      status_update(STATUS_ACTIVE);

      led_state = false;
      led_update(led_state);
      k_timer_start(&recheck_timer, K_MSEC(50), K_NO_WAIT);

      rx_received = false;
    }

    if (LOG_PROCESS() == false) {
			//pm_state_set(PM_STATE_RUNTIME_IDLE, 0);
      k_sleep(K_MSEC(1));
		}
  }

	/* return to idle thread */
	return;
}
