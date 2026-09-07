/* SPDX-License-Identifier: Apache-2.0 */
/*
 * USB CDC ACM ↔ FDCAN1 桥：PC 虚拟串口收发 CAN/CAN-FD 帧。
 *
 * 行协议（\\n 结尾，空格分隔）：
 *   t <id> <flags> <nbytes> <hex...>   主机发送
 *   r <id> <flags> <nbytes> <hex...>   总线接收回传
 *   S <arb_bps> <data_bps>             改波特率（须先停总线）
 *   O / C                              打开 / 暂停转发
 *   s                                  查询总线状态
 * flags：x=扩展帧 f=CAN-FD b=BRS r=远程帧；无标志写 -
 */
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(usb_canfd, LOG_LEVEL_INF);

#define USB_RX_RING_SIZE  2048
#define USB_TX_RING_SIZE  4096
#define LINE_MAX          256
#define CAN_RX_DEPTH      64
#define FRAME_TEXT_MAX    200

#define CAN_NODE DT_CHOSEN(zephyr_canbus)
#define USB_NODE DT_NODELABEL(cdc_acm_uart)

static const struct device *const can_dev = DEVICE_DT_GET(CAN_NODE);
static const struct device *const usb_dev = DEVICE_DT_GET(USB_NODE);

CAN_MSGQ_DEFINE(can_rx_msgq, CAN_RX_DEPTH);

K_MUTEX_DEFINE(can_lock);
K_SEM_DEFINE(usb_rx_sem, 0, 1);

static uint8_t usb_rx_mem[USB_RX_RING_SIZE];
static uint8_t usb_tx_mem[USB_TX_RING_SIZE];
static struct ring_buf usb_rx_rb;
static struct ring_buf usb_tx_rb;
static bool usb_rx_throttled;
static volatile bool bridge_open = true;
static int64_t last_tx_err_log;

BUILD_ASSERT(LINE_MAX > 32, "line buffer too small");

static void can_tx_done(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(error);
}

static bool valid_payload_len(bool fd, uint8_t nbytes)
{
	static const uint8_t fd_lens[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
	};
	size_t i;

	if (!fd) {
		return nbytes <= 8U;
	}
	for (i = 0; i < ARRAY_SIZE(fd_lens); i++) {
		if (fd_lens[i] == nbytes) {
			return true;
		}
	}
	return false;
}

static int hex_nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -EINVAL;
}

static int parse_flags(const char *s, uint8_t *flags)
{
	*flags = 0;
	if (s[0] == '-' && s[1] == '\0') {
		return 0;
	}
	for (; *s != '\0'; s++) {
		switch (*s) {
		case 'x':
		case 'X':
			*flags |= CAN_FRAME_IDE;
			break;
		case 'f':
		case 'F':
			*flags |= CAN_FRAME_FDF;
			break;
		case 'b':
		case 'B':
			*flags |= CAN_FRAME_BRS;
			break;
		case 'r':
		case 'R':
			*flags |= CAN_FRAME_RTR;
			break;
		default:
			return -EINVAL;
		}
	}
	if ((*flags & CAN_FRAME_BRS) && (*flags & CAN_FRAME_FDF) == 0) {
		return -EINVAL;
	}
	return 0;
}

static int parse_hex_bytes(char *s, uint8_t *out, uint8_t want)
{
	uint8_t n = 0;
	int hi, lo;

	while (*s != '\0') {
		while (*s == ' ' || *s == '\t') {
			s++;
		}
		if (*s == '\0') {
			break;
		}
		hi = hex_nibble(*s++);
		if (hi < 0) {
			return -EINVAL;
		}
		while (*s == ' ' || *s == '\t') {
			s++;
		}
		if (*s == '\0') {
			return -EINVAL;
		}
		lo = hex_nibble(*s++);
		if (lo < 0) {
			return -EINVAL;
		}
		if (n >= want || n >= CAN_MAX_DLEN) {
			return -EINVAL;
		}
		out[n++] = (uint8_t)((hi << 4) | lo);
	}
	if (n != want) {
		return -EINVAL;
	}
	return 0;
}

static int format_frame_line(char *buf, size_t cap, char dir,
			     const struct can_frame *frame)
{
	char flags[5];
	uint8_t nbytes = can_dlc_to_bytes(frame->dlc);
	size_t fi = 0;
	int pos;
	uint8_t i;

	if ((frame->flags & CAN_FRAME_IDE) != 0) {
		flags[fi++] = 'x';
	}
	if ((frame->flags & CAN_FRAME_FDF) != 0) {
		flags[fi++] = 'f';
	}
	if ((frame->flags & CAN_FRAME_BRS) != 0) {
		flags[fi++] = 'b';
	}
	if ((frame->flags & CAN_FRAME_RTR) != 0) {
		flags[fi++] = 'r';
	}
	if (fi == 0) {
		flags[fi++] = '-';
	}
	flags[fi] = '\0';

	pos = snprintk(buf, cap, "%c %x %s %u", dir, frame->id, flags, nbytes);
	if (pos < 0 || (size_t)pos >= cap) {
		return -ENOBUFS;
	}
	for (i = 0; i < nbytes; i++) {
		int n = snprintk(buf + pos, cap - pos, " %02X", frame->data[i]);

		if (n < 0 || (size_t)n >= cap - pos) {
			return -ENOBUFS;
		}
		pos += n;
	}
	if ((size_t)pos + 2U > cap) {
		return -ENOBUFS;
	}
	buf[pos++] = '\n';
	buf[pos] = '\0';
	return pos;
}

static int usb_write(const uint8_t *data, size_t len)
{
	size_t wrote;

	if (len == 0) {
		return 0;
	}
	wrote = ring_buf_put(&usb_tx_rb, data, len);
	uart_irq_tx_enable(usb_dev);
	if (wrote < len) {
		return -ENOBUFS;
	}
	return 0;
}

static void usb_reply(const char *s)
{
	(void)usb_write((const uint8_t *)s, strlen(s));
}

static void usb_reply_err(int err)
{
	char line[24];
	int n = snprintk(line, sizeof(line), "ERR %d\n", err);

	if (n > 0) {
		(void)usb_write((const uint8_t *)line, n);
	}
}

static int can_send_frame(const struct can_frame *frame)
{
	int ret;

	k_mutex_lock(&can_lock, K_FOREVER);
	ret = can_send(can_dev, frame, K_NO_WAIT, can_tx_done, NULL);
	k_mutex_unlock(&can_lock);
	if (ret < 0 && (k_uptime_get() - last_tx_err_log) >= 1000) {
		last_tx_err_log = k_uptime_get();
		LOG_WRN("CAN TX failed: %d", ret);
	}
	return ret;
}

static int cmd_bitrate(uint32_t arb, uint32_t data)
{
	int ret;

	if (arb == 0U || data == 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&can_lock, K_FOREVER);
	ret = can_stop(can_dev);
	if (ret == 0) {
		ret = can_set_bitrate(can_dev, arb);
	}
	if (ret == 0) {
		ret = can_set_bitrate_data(can_dev, data);
	}
	if (ret == 0) {
		ret = can_start(can_dev);
	}
	k_mutex_unlock(&can_lock);
	return ret;
}

static int handle_tx_line(char *rest)
{
	struct can_frame frame = {0};
	char *id_tok, *flag_tok, *len_tok, *end;
	unsigned long id, nbytes;
	int ret;

	id_tok = strtok(rest, " \t");
	flag_tok = strtok(NULL, " \t");
	len_tok = strtok(NULL, " \t");
	if (id_tok == NULL || flag_tok == NULL || len_tok == NULL) {
		return -EINVAL;
	}

	id = strtoul(id_tok, &end, 16);
	if (*end != '\0') {
		return -EINVAL;
	}
	nbytes = strtoul(len_tok, &end, 10);
	if (*end != '\0' || nbytes > CAN_MAX_DLEN) {
		return -EINVAL;
	}
	ret = parse_flags(flag_tok, &frame.flags);
	if (ret < 0) {
		return ret;
	}
	if (!valid_payload_len((frame.flags & CAN_FRAME_FDF) != 0, (uint8_t)nbytes)) {
		return -EINVAL;
	}
	if ((frame.flags & CAN_FRAME_IDE) != 0) {
		if (id > CAN_EXT_ID_MASK) {
			return -EINVAL;
		}
	} else if (id > CAN_STD_ID_MASK) {
		return -EINVAL;
	}

	frame.id = (uint32_t)id;
	frame.dlc = can_bytes_to_dlc((uint8_t)nbytes);
	if ((frame.flags & CAN_FRAME_RTR) == 0 && nbytes > 0) {
		char *payload = strtok(NULL, "");

		if (payload == NULL) {
			return -EINVAL;
		}
		ret = parse_hex_bytes(payload, frame.data, (uint8_t)nbytes);
		if (ret < 0) {
			return ret;
		}
	}
	return can_send_frame(&frame);
}

static void handle_line(char *line)
{
	char *p = line;
	int ret;

	while (*p == ' ' || *p == '\t') {
		p++;
	}
	if (*p == '\0') {
		return;
	}

	switch (*p) {
	case 't':
	case 'T':
		if (!bridge_open) {
			usb_reply_err(-EAGAIN);
			return;
		}
		ret = handle_tx_line(p + 1);
		if (ret < 0) {
			usb_reply_err(ret);
		}
		return;
	case 'O':
	case 'o':
		bridge_open = true;
		usb_reply("OK\n");
		return;
	case 'C':
	case 'c':
		bridge_open = false;
		usb_reply("OK\n");
		return;
	case 'S':
		if (p[1] == '\0' || !isspace((unsigned char)p[1])) {
			usb_reply_err(-EINVAL);
			return;
		}
		{
			unsigned long arb, data;
			char *end;

			arb = strtoul(p + 1, &end, 10);
			data = strtoul(end, &end, 10);
			while (*end == ' ' || *end == '\t') {
				end++;
			}
			if (*end != '\0') {
				usb_reply_err(-EINVAL);
				return;
			}
			ret = cmd_bitrate((uint32_t)arb, (uint32_t)data);
			if (ret < 0) {
				usb_reply_err(ret);
			} else {
				LOG_INF("bitrate %lu / %lu", arb, data);
				usb_reply("OK\n");
			}
		}
		return;
	case 's':
		{
			enum can_state state;
			struct can_bus_err_cnt err;
			char buf[48];
			int n;

			ret = can_get_state(can_dev, &state, &err);
			if (ret < 0) {
				usb_reply_err(ret);
				return;
			}
			n = snprintk(buf, sizeof(buf), "OK %d tec=%u rec=%u\n",
				     (int)state, err.tx_err_cnt, err.rx_err_cnt);
			if (n > 0) {
				(void)usb_write((const uint8_t *)buf, n);
			}
		}
		return;
	default:
		usb_reply_err(-EINVAL);
		return;
	}
}

static void usb_irq(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (true) {
		uart_irq_update(dev);
		if (uart_irq_is_pending(dev) <= 0) {
			break;
		}

		if (!usb_rx_throttled && uart_irq_rx_ready(dev)) {
			uint8_t tmp[64];
			size_t space = MIN(ring_buf_space_get(&usb_rx_rb), sizeof(tmp));
			int n;

			if (space == 0) {
				uart_irq_rx_disable(dev);
				usb_rx_throttled = true;
				continue;
			}
			n = uart_fifo_read(dev, tmp, space);
			if (n > 0) {
				(void)ring_buf_put(&usb_rx_rb, tmp, n);
				k_sem_give(&usb_rx_sem);
			}
		}

		if (uart_irq_tx_ready(dev) > 0) {
			uint8_t tmp[64];
			int n = (int)ring_buf_get(&usb_tx_rb, tmp, sizeof(tmp));

			if (n == 0) {
				uart_irq_tx_disable(dev);
				continue;
			}
			(void)uart_fifo_fill(dev, tmp, n);
			if (usb_rx_throttled && ring_buf_space_get(&usb_rx_rb) > 64) {
				usb_rx_throttled = false;
				uart_irq_rx_enable(dev);
			}
		}
	}
}

static void usb_in_thread(void *p1, void *p2, void *p3)
{
	char line[LINE_MAX];
	size_t linelen = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		uint8_t tmp[64];
		uint32_t n;
		uint32_t i;

		k_sem_take(&usb_rx_sem, K_FOREVER);
		while ((n = ring_buf_get(&usb_rx_rb, tmp, sizeof(tmp))) > 0) {
			if (usb_rx_throttled && ring_buf_space_get(&usb_rx_rb) > 64) {
				usb_rx_throttled = false;
				uart_irq_rx_enable(usb_dev);
			}
			for (i = 0; i < n; i++) {
				char c = (char)tmp[i];

				if (c == '\r') {
					continue;
				}
				if (c == '\n') {
					line[linelen] = '\0';
					handle_line(line);
					linelen = 0;
					continue;
				}
				if (linelen + 1U >= sizeof(line)) {
					linelen = 0;
					usb_reply_err(-ENOBUFS);
					continue;
				}
				line[linelen++] = c;
			}
		}
	}
}

static void usb_out_thread(void *p1, void *p2, void *p3)
{
	struct can_frame frame;
	char line[FRAME_TEXT_MAX];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		int n;

		k_msgq_get(&can_rx_msgq, &frame, K_FOREVER);
		if (!bridge_open) {
			continue;
		}
		n = format_frame_line(line, sizeof(line), 'r', &frame);
		if (n < 0) {
			continue;
		}
		if (usb_write((const uint8_t *)line, n) < 0) {
			LOG_WRN("USB TX ring full, drop CAN RX");
		}
	}
}

static int can_bridge_start(void)
{
	const struct can_filter std_all = {
		.id = 0,
		.mask = 0,
		.flags = 0,
	};
	const struct can_filter ext_all = {
		.id = 0,
		.mask = 0,
		.flags = CAN_FILTER_IDE,
	};
	int ret;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device not ready");
		return -ENODEV;
	}

	ret = can_set_mode(can_dev, CAN_MODE_FD);
	if (ret < 0) {
		LOG_ERR("CAN FD mode: %d", ret);
		return ret;
	}
	ret = can_start(can_dev);
	if (ret < 0) {
		LOG_ERR("CAN start: %d", ret);
		return ret;
	}
	ret = can_add_rx_filter_msgq(can_dev, &can_rx_msgq, &std_all);
	if (ret < 0) {
		LOG_ERR("std filter: %d", ret);
		return ret;
	}
	ret = can_add_rx_filter_msgq(can_dev, &can_rx_msgq, &ext_all);
	if (ret < 0) {
		LOG_ERR("ext filter: %d", ret);
		return ret;
	}
	return 0;
}

K_THREAD_STACK_DEFINE(usb_in_stack, 2048);
K_THREAD_STACK_DEFINE(usb_out_stack, 2048);
static struct k_thread usb_in_thread_data;
static struct k_thread usb_out_thread_data;

int main(void)
{
	int ret;

	ring_buf_init(&usb_rx_rb, sizeof(usb_rx_mem), usb_rx_mem);
	ring_buf_init(&usb_tx_rb, sizeof(usb_tx_mem), usb_tx_mem);

	if (!device_is_ready(usb_dev)) {
		LOG_ERR("CDC ACM not ready");
		return 0;
	}

	uart_irq_callback_set(usb_dev, usb_irq);
	uart_irq_rx_enable(usb_dev);

	ret = can_bridge_start();
	if (ret < 0) {
		return 0;
	}

	k_thread_create(&usb_in_thread_data, usb_in_stack,
			K_THREAD_STACK_SIZEOF(usb_in_stack), usb_in_thread,
			NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_create(&usb_out_thread_data, usb_out_stack,
			K_THREAD_STACK_SIZEOF(usb_out_stack), usb_out_thread,
			NULL, NULL, NULL, 6, 0, K_NO_WAIT);
	k_thread_name_set(&usb_in_thread_data, "usb_in");
	k_thread_name_set(&usb_out_thread_data, "usb_out");

	LOG_INF("USB-CANFD ready (FDCAN1 1M/5M, CDC ACM)");
	return 0;
}
