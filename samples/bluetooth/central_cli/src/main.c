/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include <drivers/uart.h>

#define BT_UUID_UART_VAL 0xFFF0
#define BT_UUID_UART_RX_VAL 0xFFF1
#define BT_UUID_UART_TX_VAL 0xFFF2

#define BT_UUID_UART BT_UUID_DECLARE_16(BT_UUID_UART_VAL)
#define BT_UUID_UART_RX BT_UUID_DECLARE_16(BT_UUID_UART_RX_VAL)
#define BT_UUID_UART_TX BT_UUID_DECLARE_16(BT_UUID_UART_TX_VAL)

static void start_scan(void);

static struct bt_conn *default_conn;

static uint16_t uart_rx_value_handle;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static const struct device *uart_dev;

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	for (int i = 0; i < length; i++)
	{
		uart_poll_out(uart_dev, ((uint8_t *)data)[i]);
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_UART)) {
		memcpy(&uuid, BT_UUID_UART_RX, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		printk("UART service start handle %u\n", discover_params.start_handle);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_UART_RX)) {
		memcpy(&uuid, BT_UUID_UART_TX, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;

		uart_rx_value_handle = bt_gatt_attr_value_handle(attr);

		printk("UART Rx value handle %u\n", uart_rx_value_handle);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_UART_TX)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		printk("UART Tx value handle %u\n", subscribe_params.value_handle);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		printk("UART Tx CCC handle %u\n", subscribe_params.ccc_handle);

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("Listen on UART Tx notification\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	/* connect only to devices in close proximity */
	if (rssi < -30) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn) {
		memcpy(&uuid, BT_UUID_UART, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = 0x0001;
		discover_params.end_handle = 0xffff;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	unsigned char c;
	int err;
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

	start_scan();

	uart_dev = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	while (1) {
		if (uart_poll_in(uart_dev, &c) == 0)
		{
			// uart_poll_out(uart_dev, c);
			bt_gatt_write_without_response(default_conn, uart_rx_value_handle, &c, 1, false);
		}
		k_sleep(K_MSEC(1));
	}
}
