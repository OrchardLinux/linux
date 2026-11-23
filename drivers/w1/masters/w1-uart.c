// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * w1-uart - UART 1-Wire bus driver
 *
 * Uses the UART interface (via Serial Device Bus) to create the 1-Wire
 * timing patterns. Implements the following 1-Wire master interface:
 *
 * - reset_bus: requests baud-rate 9600
 *
 * - touch_bit: requests baud-rate 115200
 *
 * Author: Christoph Winklhofer <cj.winklhofer@gmail.com>
 */

#include <linux/completion.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/w1.h>

/* UART packet contains start and stop bit */
#define W1_UART_BITS_PER_PACKET (BITS_PER_BYTE + 2)

/* Timeout to wait for completion of serdev-receive */
#define W1_UART_TIMEOUT msecs_to_jiffies(500)

#define W1_UART_HDQ_BAUDRATE		57600
/* A transaction takes a few ms; do not hold the bus for long on a miss. */
#define W1_UART_HDQ_TIMEOUT		msecs_to_jiffies(100)
#define W1_UART_HDQ_TRIES		3
#define W1_UART_HDQ_BREAK_MIN_US		250
#define W1_UART_HDQ_BREAK_MAX_US		500
#define W1_UART_HDQ_RECOVERY_MIN_US	150
#define W1_UART_HDQ_RECOVERY_MAX_US	500
#define W1_UART_HDQ_BITS_PER_BYTE	8
#define W1_UART_HDQ_RX_BITS		16
#define W1_UART_HDQ_ONE			0xfe
#define W1_UART_HDQ_ZERO			0xc0
#define W1_UART_HDQ_RX_ONE_MIN		0xf0
#define W1_UART_HDQ_FAMILY		0x01
#define W1_UART_HDQ_RAW_MAX		24

enum w1_uart_mode {
	W1_UART_MODE_1WIRE,
	W1_UART_MODE_HDQ,
};

struct w1_uart_variant {
	bool hdq_fail_stop;
	const u8 *hdq_read_regs;
	size_t hdq_num_read_regs;
	unsigned int hdq_budget_ms;
	enum w1_uart_mode mode;
	bool voltage_diagnostic;
};

/**
 * struct w1_uart_config - configuration for 1-Wire operation
 * @baudrate: baud-rate returned from serdev
 * @delay_us: delay to complete a 1-Wire cycle (in us)
 * @tx_byte: byte to generate 1-Wire timing pattern
 */
struct w1_uart_config {
	unsigned int baudrate;
	unsigned int delay_us;
	u8 tx_byte;
};

/**
 * struct w1_uart_device - 1-Wire UART device structure
 * @serdev: serial device
 * @bus: w1-bus master
 * @mode: selected 1-Wire or HDQ protocol
 * @hdq_mux: optional mux state routing HDQ to the application processor
 * @cfg_reset: config for 1-Wire reset
 * @cfg_touch_0: config for 1-Wire write-0 cycle
 * @cfg_touch_1: config for 1-Wire write-1 and read cycle
 * @rx_byte_received: completion for serdev receive
 * @rx_mutex: mutex to protect rx_err and rx_byte
 * @rx_err: indicates an error in serdev-receive
 * @rx_byte: result byte from serdev-receive
 * @hdq_rx_lock: protects HDQ receive state from the UART callback
 * @hdq_command: encoded HDQ command and expected UART echo
 * @hdq_response: decoded HDQ response byte
 * @hdq_rx_bit: number of accepted echo and response symbols
 * @hdq_receiving: whether an HDQ transaction is receiving symbols
 * @hdq_response_valid: whether @hdq_response is ready for the W1 client
 * @hdq_raw: first symbols received in the current HDQ cycle, for debugging
 * @hdq_raw_len: number of symbols in @hdq_raw
 */
struct w1_uart_device {
	bool hdq_faulted;
	bool hdq_fail_stop;
	const u8 *hdq_read_regs;
	size_t hdq_num_read_regs;
	unsigned long hdq_deadline;
	unsigned int hdq_budget_ms;
	struct serdev_device *serdev;
	struct w1_bus_master bus;
	enum w1_uart_mode mode;
	struct mux_state *hdq_mux;

	struct w1_uart_config cfg_reset;
	struct w1_uart_config cfg_touch_0;
	struct w1_uart_config cfg_touch_1;

	struct completion rx_byte_received;
	/*
	 * protect rx_err and rx_byte from concurrent access in
	 * w1-callbacks and serdev-receive.
	 */
	struct mutex rx_mutex;
	int rx_err;
	u8 rx_byte;

	spinlock_t hdq_rx_lock; /* Protects the HDQ receive state. */
	u8 hdq_command[W1_UART_HDQ_BITS_PER_BYTE];
	u8 hdq_response;
	u8 hdq_rx_bit;
	bool hdq_receiving;
	bool hdq_response_valid;
	bool voltage_diagnostic;
	unsigned long diagnostic_attempted;
	u8 hdq_raw[W1_UART_HDQ_RAW_MAX];
	u8 hdq_raw_len;
};

/**
 * struct w1_uart_limits - limits for 1-Wire operations
 * @baudrate: Requested baud-rate to create 1-Wire timing pattern
 * @bit_min_us: minimum time for a bit (in us)
 * @bit_max_us: maximum time for a bit (in us)
 * @sample_us: timespan to sample 1-Wire response
 * @cycle_us: duration of the 1-Wire cycle
 */
struct w1_uart_limits {
	unsigned int baudrate;
	unsigned int bit_min_us;
	unsigned int bit_max_us;
	unsigned int sample_us;
	unsigned int cycle_us;
};

static inline unsigned int baud_to_bit_ns(unsigned int baud)
{
	return NSEC_PER_SEC / baud;
}

static inline unsigned int to_ns(unsigned int us)
{
	return us * NSEC_PER_USEC;
}

/*
 * Set baud-rate, delay and tx-byte to create a 1-Wire pulse and adapt
 * the tx-byte according to the actual baud-rate.
 *
 * Reject when:
 * - time for a bit outside min/max range
 * - a 1-Wire response is not detectable for sent byte
 */
static int w1_uart_set_config(struct serdev_device *serdev,
			      const struct w1_uart_limits *limits,
			      struct w1_uart_config *w1cfg)
{
	unsigned int packet_ns;
	unsigned int bits_low;
	unsigned int bit_ns;
	unsigned int low_ns;

	w1cfg->baudrate = serdev_device_set_baudrate(serdev, limits->baudrate);
	if (w1cfg->baudrate == 0)
		return -EINVAL;

	/* Compute in nanoseconds for accuracy */
	bit_ns = baud_to_bit_ns(w1cfg->baudrate);
	bits_low = to_ns(limits->bit_min_us) / bit_ns;
	/* start bit is always low */
	low_ns = bit_ns * (bits_low + 1);

	if (low_ns < to_ns(limits->bit_min_us))
		return -EINVAL;

	if (low_ns > to_ns(limits->bit_max_us))
		return -EINVAL;

	/* 1-Wire response detectable for sent byte */
	if (limits->sample_us > 0 &&
	    bit_ns * BITS_PER_BYTE < low_ns + to_ns(limits->sample_us))
		return -EINVAL;

	/* delay: 1-Wire cycle takes longer than the UART packet */
	packet_ns = bit_ns * W1_UART_BITS_PER_PACKET;
	w1cfg->delay_us = 0;
	if (to_ns(limits->cycle_us) > packet_ns)
		w1cfg->delay_us =
			(to_ns(limits->cycle_us) - packet_ns) / NSEC_PER_USEC;

	/* byte to create 1-Wire pulse */
	w1cfg->tx_byte = 0xff << bits_low;

	return 0;
}

/*
 * Configuration for reset and presence detect
 * - bit_min_us is 480us, add margin and use 485us
 * - limits for sample time 60us-75us, use 65us
 */
static int w1_uart_set_config_reset(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	struct device_node *np = serdev->dev.of_node;

	struct w1_uart_limits limits = { .baudrate = 9600,
					 .bit_min_us = 485,
					 .bit_max_us = 640,
					 .sample_us = 65,
					 .cycle_us = 960 };

	of_property_read_u32(np, "reset-bps", &limits.baudrate);

	return w1_uart_set_config(serdev, &limits, &w1dev->cfg_reset);
}

/*
 * Configuration for write-0 cycle (touch bit 0)
 * - bit_min_us is 60us, add margin and use 65us
 * - no sampling required, sample_us = 0
 */
static int w1_uart_set_config_touch_0(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	struct device_node *np = serdev->dev.of_node;

	struct w1_uart_limits limits = { .baudrate = 115200,
					 .bit_min_us = 65,
					 .bit_max_us = 120,
					 .sample_us = 0,
					 .cycle_us = 70 };

	of_property_read_u32(np, "write-0-bps", &limits.baudrate);

	return w1_uart_set_config(serdev, &limits, &w1dev->cfg_touch_0);
}

/*
 * Configuration for write-1 and read cycle (touch bit 1)
 * - bit_min_us is 5us, add margin and use 6us
 * - limits for sample time 5us-15us, use 15us
 */
static int w1_uart_set_config_touch_1(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	struct device_node *np = serdev->dev.of_node;

	struct w1_uart_limits limits = { .baudrate = 115200,
					 .bit_min_us = 6,
					 .bit_max_us = 15,
					 .sample_us = 15,
					 .cycle_us = 70 };

	of_property_read_u32(np, "write-1-bps", &limits.baudrate);

	return w1_uart_set_config(serdev, &limits, &w1dev->cfg_touch_1);
}

/*
 * Configure and open the serial device
 */
static int w1_uart_serdev_open(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	struct device *dev = &serdev->dev;
	int ret;

	ret = devm_serdev_device_open(dev, serdev);
	if (ret < 0)
		return ret;

	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret < 0) {
		dev_err(dev, "set parity failed\n");
		return ret;
	}

	serdev_device_set_flow_control(serdev, false);

	if (w1dev->mode == W1_UART_MODE_HDQ) {
		unsigned int baudrate;

		baudrate = serdev_device_set_baudrate(serdev,
						      W1_UART_HDQ_BAUDRATE);
		if (baudrate != W1_UART_HDQ_BAUDRATE)
			return dev_err_probe(dev, -EINVAL,
					     "failed to set %u baud (got %u)\n",
					     W1_UART_HDQ_BAUDRATE, baudrate);

		ret = serdev_device_set_stopbits(serdev, SERDEV_STOPBITS_2);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to set two stop bits\n");

		return 0;
	}

	ret = w1_uart_set_config_reset(w1dev);
	if (ret < 0) {
		dev_err(dev, "config for reset failed\n");
		return ret;
	}

	ret = w1_uart_set_config_touch_0(w1dev);
	if (ret < 0) {
		dev_err(dev, "config for touch-0 failed\n");
		return ret;
	}

	ret = w1_uart_set_config_touch_1(w1dev);
	if (ret < 0) {
		dev_err(dev, "config for touch-1 failed\n");
		return ret;
	}

	return 0;
}

static void w1_uart_hdq_cancel_receive(struct w1_uart_device *w1dev)
{
	unsigned long flags;

	spin_lock_irqsave(&w1dev->hdq_rx_lock, flags);
	w1dev->hdq_receiving = false;
	w1dev->hdq_response_valid = false;
	spin_unlock_irqrestore(&w1dev->hdq_rx_lock, flags);
}

static void w1_uart_hdq_stop_on_error(struct w1_uart_device *w1dev,
				     const char *operation, int ret)
{
	if (!ret || !w1dev->hdq_fail_stop)
		return;

	w1_uart_hdq_cancel_receive(w1dev);
	w1dev->hdq_faulted = true;
	dev_err(&w1dev->serdev->dev, "HDQ %s failed (%d), stopping transfers\n",
		operation, ret);
}

static int w1_uart_hdq_break(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	int ret;

	ret = serdev_device_break_ctl(serdev, -1);
	if (ret)
		return ret;

	usleep_range(W1_UART_HDQ_BREAK_MIN_US, W1_UART_HDQ_BREAK_MAX_US);
	ret = serdev_device_break_ctl(serdev, 0);
	if (ret)
		return ret;

	usleep_range(W1_UART_HDQ_RECOVERY_MIN_US,
		     W1_UART_HDQ_RECOVERY_MAX_US);

	return 0;
}

/*
 * One break + command + response cycle. A corrupted echo symbol is never
 * accepted by the receive callback, so the response then never completes
 * and the cycle ends with -ETIMEDOUT.
 */
/* Zero is rejected by the caller: serdev treats a zero timeout as infinite. */
static unsigned long w1_uart_hdq_wait_budget(struct w1_uart_device *w1dev,
					     unsigned long legacy)
{
	unsigned long now = jiffies;

	if (!w1dev->hdq_budget_ms)
		return legacy;
	if (time_after_eq(now, w1dev->hdq_deadline))
		return 0;
	return min(legacy, w1dev->hdq_deadline - now);
}

static int w1_uart_hdq_xfer_once(struct w1_uart_device *w1dev)
{
	struct serdev_device *serdev = w1dev->serdev;
	unsigned long flags;
	unsigned long timeout;
	ssize_t written;
	int ret;

	if (!w1_uart_hdq_wait_budget(w1dev, HZ))
		return -ETIMEDOUT;
	ret = w1_uart_hdq_break(w1dev);
	if (ret)
		return ret;

	reinit_completion(&w1dev->rx_byte_received);
	spin_lock_irqsave(&w1dev->hdq_rx_lock, flags);
	w1dev->hdq_response = 0;
	w1dev->hdq_rx_bit = 0;
	w1dev->hdq_raw_len = 0;
	w1dev->hdq_receiving = true;
	w1dev->hdq_response_valid = false;
	spin_unlock_irqrestore(&w1dev->hdq_rx_lock, flags);

	timeout = w1_uart_hdq_wait_budget(w1dev, HZ);
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto cancel_receive;
	}
	written = serdev_device_write(serdev, w1dev->hdq_command,
				      sizeof(w1dev->hdq_command), timeout);
	if (written < 0) {
		ret = written;
		goto cancel_receive;
	}
	if (written != sizeof(w1dev->hdq_command)) {
		ret = -EIO;
		goto cancel_receive;
	}

	timeout = w1_uart_hdq_wait_budget(w1dev, HZ);
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto cancel_receive;
	}
	serdev_device_wait_until_sent(serdev, timeout);

	timeout = w1_uart_hdq_wait_budget(w1dev, W1_UART_HDQ_TIMEOUT);
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto cancel_receive;
	}
	timeout = wait_for_completion_timeout(&w1dev->rx_byte_received,
					      timeout);
	if (!timeout || !w1_uart_hdq_wait_budget(w1dev, HZ)) {
		u8 raw[W1_UART_HDQ_RAW_MAX];
		u8 len, bit;

		spin_lock_irqsave(&w1dev->hdq_rx_lock, flags);
		len = w1dev->hdq_raw_len;
		bit = w1dev->hdq_rx_bit;
		memcpy(raw, w1dev->hdq_raw, len);
		spin_unlock_irqrestore(&w1dev->hdq_rx_lock, flags);
		/* What the line gave back: no echo, a bent echo, or no answer */
		dev_dbg(&serdev->dev,
			"HDQ cycle: %u of %u symbols accepted, %u received: %*ph\n",
			bit, W1_UART_HDQ_RX_BITS, len, len, raw);
		ret = -ETIMEDOUT;
		goto cancel_receive;
	}

	return 0;

cancel_receive:
	w1_uart_hdq_cancel_receive(w1dev);

	return ret;
}

static bool w1_uart_hdq_command_allowed(struct w1_uart_device *w1dev,
					u8 command)
{
	size_t i;

	if (!w1dev->hdq_read_regs)
		return true;
	for (i = 0; i < w1dev->hdq_num_read_regs; i++)
		if (command == w1dev->hdq_read_regs[i])
			return true;
	return false;
}

static int w1_uart_hdq_xfer(struct w1_uart_device *w1dev, u8 command,
			    unsigned int tries)
{
	unsigned int bit;
	unsigned int try;
	int ret;
	int deselect_ret;

	w1_uart_hdq_cancel_receive(w1dev);
	if (w1dev->hdq_faulted)
		return -ESHUTDOWN;
	if (!w1_uart_hdq_command_allowed(w1dev, command))
		return -EACCES;
	if (w1dev->hdq_budget_ms)
		w1dev->hdq_deadline = jiffies +
			msecs_to_jiffies(w1dev->hdq_budget_ms);

	if (w1dev->hdq_mux) {
		ret = mux_state_select(w1dev->hdq_mux);
		if (ret) {
			w1_uart_hdq_stop_on_error(w1dev, "mux select", ret);
			return ret;
		}
	}

	for (bit = 0; bit < W1_UART_HDQ_BITS_PER_BYTE; bit++)
		w1dev->hdq_command[bit] = command & BIT(bit) ?
			W1_UART_HDQ_ONE : W1_UART_HDQ_ZERO;

	/*
	 * Retry a lost cycle: the gauge's response window is long over after
	 * the timeout, and the next break resynchronizes its HDQ state.
	 */
	for (try = 0; try < tries; try++) {
		ret = w1_uart_hdq_xfer_once(w1dev);
		if (ret != -ETIMEDOUT || w1dev->hdq_fail_stop)
			break;
		dev_dbg(&w1dev->serdev->dev,
			"HDQ command %02x timed out (try %u/%u)\n",
			command, try + 1, tries);
	}

	if (w1dev->hdq_mux) {
		deselect_ret = mux_state_deselect(w1dev->hdq_mux);
		if (!ret && deselect_ret) {
			w1_uart_hdq_cancel_receive(w1dev);
			ret = deselect_ret;
		}
	}

	w1_uart_hdq_stop_on_error(w1dev, "read", ret);
	return ret;
}

/*
 * Send one byte (tx_byte) and read one byte (rx_byte) via serdev.
 */
static int w1_uart_serdev_tx_rx(struct w1_uart_device *w1dev,
				const struct w1_uart_config *w1cfg, u8 *rx_byte)
{
	struct serdev_device *serdev = w1dev->serdev;
	int ret;

	serdev_device_write_flush(serdev);
	serdev_device_set_baudrate(serdev, w1cfg->baudrate);

	/* write and immediately read one byte */
	reinit_completion(&w1dev->rx_byte_received);
	ret = serdev_device_write_buf(serdev, &w1cfg->tx_byte, 1);
	if (ret != 1)
		return -EIO;
	ret = wait_for_completion_interruptible_timeout(
		&w1dev->rx_byte_received, W1_UART_TIMEOUT);
	if (ret <= 0)
		return -EIO;

	/* locking could fail when serdev is unexpectedly receiving. */
	if (!mutex_trylock(&w1dev->rx_mutex))
		return -EIO;

	ret = w1dev->rx_err;
	if (ret == 0)
		*rx_byte = w1dev->rx_byte;

	mutex_unlock(&w1dev->rx_mutex);

	if (w1cfg->delay_us > 0)
		fsleep(w1cfg->delay_us);

	return ret;
}

/*
 * The command echo must arrive as eight contiguous symbols. After a
 * mismatch, keep the longest tail of what was received (the echo matched
 * so far, then @c) that is still a prefix of the command, so a stray
 * symbol neither leaves an old partial match valid nor drops a real echo
 * that started inside it.
 */
static u8 w1_uart_hdq_echo_restart(const struct w1_uart_device *w1dev,
				   u8 matched, u8 c)
{
	const u8 *cmd = w1dev->hdq_command;
	u8 len;

	for (len = matched; len > 0; len--) {
		/* received tail: cmd[matched - len + 1 .. matched - 1], c */
		if (cmd[len - 1] == c &&
		    !memcmp(cmd, cmd + matched - len + 1, len - 1))
			return len;
	}

	return 0;
}

static size_t w1_uart_serdev_receive_buf(struct serdev_device *serdev,
					  const u8 *buf, size_t count)
{
	struct w1_uart_device *w1dev = serdev_device_get_drvdata(serdev);
	unsigned long flags;
	size_t i;

	if (w1dev->mode == W1_UART_MODE_HDQ) {
		spin_lock_irqsave(&w1dev->hdq_rx_lock, flags);
		for (i = 0; i < count && w1dev->hdq_receiving; i++) {
			if (w1dev->hdq_raw_len < W1_UART_HDQ_RAW_MAX)
				w1dev->hdq_raw[w1dev->hdq_raw_len++] = buf[i];

			if (w1dev->hdq_rx_bit < W1_UART_HDQ_BITS_PER_BYTE &&
			    buf[i] != w1dev->hdq_command[w1dev->hdq_rx_bit]) {
				w1dev->hdq_rx_bit =
					w1_uart_hdq_echo_restart(w1dev,
								 w1dev->hdq_rx_bit,
								 buf[i]);
				continue;
			}

			if (w1dev->hdq_rx_bit >= W1_UART_HDQ_BITS_PER_BYTE &&
			    buf[i] >= W1_UART_HDQ_RX_ONE_MIN)
				w1dev->hdq_response |=
					BIT(w1dev->hdq_rx_bit -
					    W1_UART_HDQ_BITS_PER_BYTE);

			w1dev->hdq_rx_bit++;
			if (w1dev->hdq_rx_bit == W1_UART_HDQ_RX_BITS) {
				w1dev->hdq_receiving = false;
				w1dev->hdq_response_valid = true;
				complete(&w1dev->rx_byte_received);
			}
		}
		spin_unlock_irqrestore(&w1dev->hdq_rx_lock, flags);

		return count;
	}

	mutex_lock(&w1dev->rx_mutex);

	/* sent a single byte and receive one single byte */
	if (count == 1) {
		w1dev->rx_byte = buf[0];
		w1dev->rx_err = 0;
	} else {
		w1dev->rx_err = -EIO;
	}

	mutex_unlock(&w1dev->rx_mutex);
	complete(&w1dev->rx_byte_received);

	return count;
}

static const struct serdev_device_ops w1_uart_serdev_ops = {
	.receive_buf = w1_uart_serdev_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/*
 * 1-wire reset and presence detect: A present slave will manipulate
 * the received byte by pulling the 1-Wire low.
 */
static u8 w1_uart_reset_bus(void *data)
{
	struct w1_uart_device *w1dev = data;
	const struct w1_uart_config *w1cfg = &w1dev->cfg_reset;
	int ret;
	u8 val;

	ret = w1_uart_serdev_tx_rx(w1dev, w1cfg, &val);
	if (ret < 0)
		return -1;

	/* Device present (0) or no device (1) */
	return val != w1cfg->tx_byte ? 0 : 1;
}

/*
 * 1-Wire read and write cycle: Only the read-0 manipulates the
 * received byte, all others left the line untouched.
 */
static u8 w1_uart_touch_bit(void *data, u8 bit)
{
	struct w1_uart_device *w1dev = data;
	const struct w1_uart_config *w1cfg = bit ? &w1dev->cfg_touch_1 :
						   &w1dev->cfg_touch_0;
	int ret;
	u8 val;

	ret = w1_uart_serdev_tx_rx(w1dev, w1cfg, &val);

	/* return inactive bus state on error */
	if (ret < 0)
		return 1;

	return val == w1cfg->tx_byte ? 1 : 0;
}

static void w1_uart_hdq_write_byte(void *data, u8 byte)
{
	struct w1_uart_device *w1dev = data;
	int ret;

	ret = w1_uart_hdq_xfer(w1dev, byte, W1_UART_HDQ_TRIES);
	if (ret)
		dev_dbg(&w1dev->serdev->dev, "HDQ transfer failed: %d\n", ret);
}

static bool w1_uart_hdq_take_response(struct w1_uart_device *w1dev,
				      u8 *response)
{
	unsigned long flags;
	bool valid;

	spin_lock_irqsave(&w1dev->hdq_rx_lock, flags);
	valid = w1dev->hdq_response_valid;
	if (valid) {
		*response = w1dev->hdq_response;
		w1dev->hdq_response_valid = false;
	}
	spin_unlock_irqrestore(&w1dev->hdq_rx_lock, flags);

	return valid;
}

static u8 w1_uart_hdq_reset_bus(void *data)
{
	struct w1_uart_device *w1dev = data;
	int ret;

	if (w1dev->hdq_faulted)
		return 1;
	w1_uart_hdq_cancel_receive(w1dev);
	if (w1dev->hdq_mux) {
		ret = mux_state_select(w1dev->hdq_mux);
		if (ret) {
			w1_uart_hdq_stop_on_error(w1dev, "reset mux select", ret);
			return 1;
		}
	}

	ret = w1_uart_hdq_break(w1dev);
	if (w1dev->hdq_mux && mux_state_deselect(w1dev->hdq_mux) && !ret)
		ret = -EIO;

	w1_uart_hdq_stop_on_error(w1dev, "reset", ret);
	return ret ? 1 : 0;
}

static u8 w1_uart_hdq_read_byte(void *data)
{
	struct w1_uart_device *w1dev = data;
	u8 response;

	return w1_uart_hdq_take_response(w1dev, &response) ? response : 0xff;
}

static u8 w1_uart_hdq_read_block(void *data, u8 *buf, int len)
{
	struct w1_uart_device *w1dev = data;

	if (len != 1)
		return 0;

	return w1_uart_hdq_take_response(w1dev, buf) ? 1 : 0;
}

static void w1_uart_hdq_search(void *data, struct w1_master *master,
			       u8 search_type,
			       w1_slave_found_callback slave_found)
{
	u64 module_id = W1_UART_HDQ_FAMILY;
	u64 rn_le = cpu_to_le64(module_id);
	u8 crc;

	crc = w1_calc_crc8((u8 *)&rn_le, 7);
	slave_found(master, (u64)crc << 56 | module_id);
}

/* Local J120 RAM diagnostic: one coherent voltage word, no W1 scan. */
static ssize_t voltage_mv_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct w1_uart_device *w1dev = dev_get_drvdata(dev);
	static const u8 command[] = { 0x09, 0x08, 0x09 };
	u8 bytes[ARRAY_SIZE(command)];
	unsigned int i;
	int ret;

	if (test_and_set_bit(0, &w1dev->diagnostic_attempted))
		return -EALREADY;

	for (i = 0; i < ARRAY_SIZE(command); i++) {
		ret = w1_uart_hdq_xfer(w1dev, command[i], 1);
		if (ret)
			return dev_err_probe(dev, ret, "HDQ voltage read %#x failed\n",
					     command[i]);
		if (!w1_uart_hdq_take_response(w1dev, &bytes[i]))
			return -EIO;
	}

	if (bytes[0] != bytes[2])
		return -EAGAIN;
	if (!(bytes[0] | bytes[1]) || (bytes[0] == 0xff && bytes[1] == 0xff))
		return -ENODATA;
	return sysfs_emit(buf, "%u\n", (unsigned int)bytes[0] << 8 | bytes[1]);
}
static DEVICE_ATTR_RO(voltage_mv);

static int w1_uart_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	const struct w1_uart_variant *variant;
	struct w1_uart_device *w1dev;
	int ret;

	w1dev = devm_kzalloc(dev, sizeof(*w1dev), GFP_KERNEL);
	if (!w1dev)
		return -ENOMEM;
	w1dev->serdev = serdev;
	w1dev->bus.data = w1dev;
	variant = device_get_match_data(dev);
	if (variant) {
		w1dev->mode = variant->mode;
		w1dev->hdq_fail_stop = variant->hdq_fail_stop;
		w1dev->hdq_read_regs = variant->hdq_read_regs;
		w1dev->hdq_num_read_regs = variant->hdq_num_read_regs;
		w1dev->hdq_budget_ms = variant->hdq_budget_ms;
		w1dev->voltage_diagnostic = variant->voltage_diagnostic;
	}

	if (w1dev->mode == W1_UART_MODE_HDQ) {
		w1dev->hdq_mux = devm_mux_state_get_optional(dev, "hdq");
		if (IS_ERR(w1dev->hdq_mux))
			return dev_err_probe(dev, PTR_ERR(w1dev->hdq_mux),
					     "failed to get HDQ mux state\n");

		w1dev->bus.write_byte = w1_uart_hdq_write_byte;
		w1dev->bus.read_byte = w1_uart_hdq_read_byte;
		w1dev->bus.read_block = w1_uart_hdq_read_block;
		w1dev->bus.reset_bus = w1_uart_hdq_reset_bus;
		w1dev->bus.search = w1_uart_hdq_search;
	} else {
		w1dev->bus.reset_bus = w1_uart_reset_bus;
		w1dev->bus.touch_bit = w1_uart_touch_bit;
	}

	init_completion(&w1dev->rx_byte_received);
	mutex_init(&w1dev->rx_mutex);
	spin_lock_init(&w1dev->hdq_rx_lock);

	serdev_device_set_drvdata(serdev, w1dev);
	serdev_device_set_client_ops(serdev, &w1_uart_serdev_ops);
	ret = w1_uart_serdev_open(w1dev);
	if (ret < 0)
		return ret;

	if (w1dev->voltage_diagnostic)
		return device_create_file(dev, &dev_attr_voltage_mv);

	if (w1dev->hdq_fail_stop)
		return w1_add_master_device_parent(&w1dev->bus, dev);
	return w1_add_master_device(&w1dev->bus);
}

static void w1_uart_remove(struct serdev_device *serdev)
{
	struct w1_uart_device *w1dev = serdev_device_get_drvdata(serdev);

	if (w1dev->voltage_diagnostic) {
		device_remove_file(&serdev->dev, &dev_attr_voltage_mv);
		return;
	}

	/*
	 * Waits until w1-uart callbacks are finished, serdev is closed
	 * and its device data released automatically by devres (waits
	 * until serdev-receive is finished).
	 */
	w1_remove_master_device(&w1dev->bus);
}

static const struct w1_uart_variant w1_uart_1wire = {
	.mode = W1_UART_MODE_1WIRE,
};

static const struct w1_uart_variant w1_uart_hdq = {
	.mode = W1_UART_MODE_HDQ,
};

static const struct w1_uart_variant w1_uart_j120_voltage = {
	.mode = W1_UART_MODE_HDQ,
	.voltage_diagnostic = true,
};

/* Read-command bytes only; HDQ write opcodes (bit7) are never admitted. */
static const u8 w1_uart_j120_hdq_read_regs[] = {
	0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x10,
	0x11, 0x12, 0x13, 0x14, 0x15, 0x2a, 0x2b, 0x2c, 0x2d,
};

static const struct w1_uart_variant w1_uart_j120_hdq = {
	.hdq_fail_stop = true,
	.hdq_read_regs = w1_uart_j120_hdq_read_regs,
	.hdq_num_read_regs = ARRAY_SIZE(w1_uart_j120_hdq_read_regs),
	.mode = W1_UART_MODE_HDQ,
	.hdq_budget_ms = 100,
};

static const struct of_device_id w1_uart_of_match[] = {
	{ .compatible = "apple,j120-hdq-uart", .data = &w1_uart_j120_hdq },
	{ .compatible = "apple,j120-hdq-voltage-diagnostic", .data = &w1_uart_j120_voltage },
	{ .compatible = "w1-uart", .data = &w1_uart_1wire },
	{ .compatible = "ti,hdq-uart", .data = &w1_uart_hdq },
	{},
};
MODULE_DEVICE_TABLE(of, w1_uart_of_match);

static struct serdev_device_driver w1_uart_driver = {
	.driver	= {
		.name		= "w1-uart",
		.of_match_table = w1_uart_of_match,
	},
	.probe	= w1_uart_probe,
	.remove	= w1_uart_remove,
};

module_serdev_device_driver(w1_uart_driver);

MODULE_DESCRIPTION("UART w1 bus driver");
MODULE_AUTHOR("Christoph Winklhofer <cj.winklhofer@gmail.com>");
MODULE_LICENSE("GPL");
