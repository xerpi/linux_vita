/*
 * PlayStation Vita UART driver
 *
 * Copyright (C) 2018 Sergi Granell <xerpi.g.12@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/types.h>

#define SERIAL_NAME	"ttyS"
#define DRIVER_NAME	"vita-uart"
#define MAKE_NAME(x)	(DRIVER_NAME # x)

#define VITA_UART_STATUS		0x28
#define VITA_UART_READ_CNT		0x68
#define VITA_UART_WRITE_FIFO		0x70
#define VITA_UART_READ_FIFO		0x78

#define VITA_UART_STATUS_TXREADY	(1 << 8)
#define VITA_UART_STATUS_DEVREADY	(1 << 9)

#define VITA_UART_READ_CNT_MASK		(0x3F)

#define VITA_UART_MAX_PORTS		7

struct vita_uart_port {
	struct uart_port port;
	struct delayed_work rx_poll_work;
};

static inline struct vita_uart_port *to_vita_port(struct uart_port *port)
{
	return container_of(port, struct vita_uart_port, port);
}

static void vita_uart_write32(struct uart_port *port, u32 val, unsigned int off)
{
	struct vita_uart_port *vita_port = to_vita_port(port);

	writel_relaxed(val, vita_port->port.membase + off);
}

static u32 vita_uart_read32(struct uart_port *port, unsigned int off)
{
	struct vita_uart_port *vita_port = to_vita_port(port);

	return readl(vita_port->port.membase + off);
}

static bool vita_uart_tx_ready(struct uart_port *port)
{
	return !!(vita_uart_read32(port, VITA_UART_STATUS) & VITA_UART_STATUS_TXREADY);
}

static bool vita_uart_dev_ready(struct uart_port *port)
{
	return !!(vita_uart_read32(port, VITA_UART_STATUS) & VITA_UART_STATUS_DEVREADY);
}

static u32 vita_uart_read_cnt(struct uart_port *port)
{
	return vita_uart_read32(port, VITA_UART_READ_CNT) & VITA_UART_READ_CNT_MASK;
}

static unsigned int vita_uart_tx_empty(struct uart_port *port)
{
	return vita_uart_tx_ready(port) ? TIOCSER_TEMT : 0;
}

static void vita_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int vita_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void vita_uart_stop_tx(struct uart_port *port)
{
}

static void vita_uart_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	while (!uart_circ_empty(xmit)) {
		while (!vita_uart_tx_ready(port))
			cpu_relax();

		if (port->x_char) {
			vita_uart_write32(port, port->x_char, VITA_UART_WRITE_FIFO);
			port->x_char = 0;
			port->icount.tx++;
			continue;
		}

		if (uart_tx_stopped(port))
			break;

		vita_uart_write32(port, xmit->buf[xmit->tail], VITA_UART_WRITE_FIFO);
		xmit->tail = (xmit->tail + 1) % UART_XMIT_SIZE;
		port->icount.tx++;

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}
}

static void vita_uart_start_tx(struct uart_port *port)
{
	vita_uart_tx_chars(port);
}

static void vita_uart_stop_rx(struct uart_port *port)
{
}

static void vita_uart_break_ctl(struct uart_port *port, int ctl)
{
}

static void vita_uart_rx_chars(struct vita_uart_port *vita_port)
{
	struct uart_port *port = &vita_port->port;

	while (vita_uart_read_cnt(port) > 0) {
		u8 ch = vita_uart_read32(port, VITA_UART_READ_FIFO);
		port->icount.rx++;
		uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(&port->state->port);

}

static int vita_uart_startup(struct uart_port *port)
{
	struct vita_uart_port *vita_port = to_vita_port(port);

	schedule_delayed_work(&vita_port->rx_poll_work, msecs_to_jiffies(100));

	return 0;
}

static void vita_uart_shutdown(struct uart_port *port)
{
	struct vita_uart_port *vita_port = to_vita_port(port);

	cancel_delayed_work_sync(&vita_port->rx_poll_work);
}

static void
vita_uart_set_termios(struct uart_port *port, struct ktermios *termios,
		      struct ktermios *old)
{
}

static const char *vita_uart_type(struct uart_port *port)
{
	return (port->type == PORT_VITAUART) ? DRIVER_NAME : NULL;
}

static void vita_uart_release_port(struct uart_port *port)
{
}

static int vita_uart_request_port(struct uart_port *port)
{
	return 0;
}

static void vita_uart_config_port(struct uart_port *port, int type)
{
	if (type & UART_CONFIG_TYPE && !vita_uart_request_port(port))
		port->type = PORT_VITAUART;
}

static int vita_uart_verify_port(struct uart_port *port, struct serial_struct *serinfo)
{
	return -EINVAL;
}

static const struct uart_ops vita_uart_pops = {
	.tx_empty = vita_uart_tx_empty,
	.set_mctrl = vita_uart_set_mctrl,
	.get_mctrl = vita_uart_get_mctrl,
	.stop_tx = vita_uart_stop_tx,
	.start_tx = vita_uart_start_tx,
	.stop_rx = vita_uart_stop_rx,
	.break_ctl = vita_uart_break_ctl,
	.startup = vita_uart_startup,
	.shutdown = vita_uart_shutdown,
	.set_termios = vita_uart_set_termios,
	.type = vita_uart_type,
	.release_port = vita_uart_release_port,
	.request_port = vita_uart_request_port,
	.config_port = vita_uart_config_port,
	.verify_port = vita_uart_verify_port,
};

static struct vita_uart_port vita_uart_ports[VITA_UART_MAX_PORTS];

#ifdef CONFIG_SERIAL_VITA_UART_CONSOLE
static void vita_uart_console_putchar(struct uart_port *port, int ch)
{
	while (!vita_uart_tx_ready(port))
		cpu_relax();

	vita_uart_write32(port, ch, VITA_UART_WRITE_FIFO);

	while (!vita_uart_dev_ready(port))
		cpu_relax();
}

static void vita_uart_console_write(struct console *co, const char *s, unsigned int cnt)
{
	struct uart_port *port = &vita_uart_ports[co->index].port;

	uart_console_write(port, s, cnt, vita_uart_console_putchar);
}

static int vita_uart_console_setup(struct console *co, char *options)
{
	struct vita_uart_port *vita_port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= VITA_UART_MAX_PORTS)
		return -ENODEV;

	vita_port = &vita_uart_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&vita_port->port, co, baud, parity, bits, flow);
}

static struct uart_driver vita_uart_driver;

static struct console vita_uart_console = {
	.name = SERIAL_NAME,
	.device = uart_console_device,
	.write = vita_uart_console_write,
	.setup = vita_uart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &vita_uart_driver,
};

#define VITA_SERIAL_CONSOLE (&vita_uart_console)

static void vita_early_putchar(struct uart_port *port, int ch)
{
	while (!vita_uart_tx_ready(port))
		cpu_relax();

	writel((unsigned char)ch, port->membase + VITA_UART_STATUS);

	while (!vita_uart_dev_ready(port))
		cpu_relax();
}

static void vita_early_write(struct console *con, const char *s, unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, vita_early_putchar);
}

static int __init vita_early_console_setup(struct earlycon_device *device,
					   const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = vita_early_write;

	return 0;
}

OF_EARLYCON_DECLARE(vita, "vita,vita-uart-earlycon", vita_early_console_setup);

#else
#define VITA_SERIAL_CONSOLE NULL
#endif

static struct uart_driver vita_uart_driver = {
	.driver_name = DRIVER_NAME,
	.dev_name = SERIAL_NAME,
	.nr = VITA_UART_MAX_PORTS,
	.cons = VITA_SERIAL_CONSOLE,
};

static void vita_rx_poll(struct work_struct *work)
{
	struct vita_uart_port *port =
		container_of(to_delayed_work(work),
			     struct vita_uart_port, rx_poll_work);

	vita_uart_rx_chars(port);

	schedule_delayed_work(&port->rx_poll_work, msecs_to_jiffies(100));
}

static struct vita_uart_port *vita_of_get_port(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int id;

	if (!np)
		return NULL;

	id = of_alias_get_id(np, "serial");
	if (id < 0)
		id = 0;

	if (WARN_ON(id >= VITA_UART_MAX_PORTS))
		return NULL;

	vita_uart_ports[id].port.line = id;
	return &vita_uart_ports[id];
}

static int vita_init_port(struct vita_uart_port *vita_port,
			  struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vita_port->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vita_port->port.membase))
		return PTR_ERR(vita_port->port.membase);

	vita_port->port.mapbase = res->start;
	vita_port->port.mapsize = resource_size(res);
	vita_port->port.iotype = UPIO_MEM;
	vita_port->port.flags = UPF_BOOT_AUTOCONF;
	vita_port->port.fifosize = 1;
	vita_port->port.ops = &vita_uart_pops;
	vita_port->port.dev = &pdev->dev;

	return 0;
}

static int vita_serial_probe(struct platform_device *pdev)
{
	struct vita_uart_port *vita_port;
	int ret;

	vita_port = vita_of_get_port(pdev);
	if (!vita_port)
		return -ENODEV;

	ret = vita_init_port(vita_port, pdev);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&vita_port->rx_poll_work, vita_rx_poll);

	ret = uart_add_one_port(&vita_uart_driver, &vita_port->port);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, vita_port);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vita_match[] = {
	{ .compatible = "vita,vita-uart", },
	{},
};
#endif

static struct platform_driver vita_serial_driver = {
	.probe = vita_serial_probe,

	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(vita_match),
		.suppress_bind_attrs = true,
	},
};

static int __init vita_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&vita_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&vita_serial_driver);
	if (ret)
		uart_unregister_driver(&vita_uart_driver);

	return ret;
}

static void __exit vita_uart_exit(void)
{
	platform_driver_unregister(&vita_serial_driver);
	uart_unregister_driver(&vita_uart_driver);
}

module_init(vita_uart_init);
module_exit(vita_uart_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita serial driver");
