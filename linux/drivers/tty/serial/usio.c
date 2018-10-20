/**
 * Copyright (C) 2014 SOCIONEXT All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#if defined(CONFIG_SERIAL_SN_USIO_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/clk.h>


#define USIO_NAME		"sn-usio-uart"

static struct uart_port usio_ports[CONFIG_SERIAL_SN_USIO_PORTS];

#define RX	0
#define TX	1
static int usio_irq[CONFIG_SERIAL_SN_USIO_PORTS][2];

#ifdef CONFIG_PM_WARP

static struct uart_pm_regs {
	unsigned char smr, scr, escr;
	unsigned short bgr, fcr, fbyte;
} uart_pm_regs[CONFIG_SERIAL_SN_USIO_PORTS];

#endif

enum {
	SN_USIO_REG_SMR			= 0,
		SN_USIO_SMR_SOE			= BIT(0),
		SN_USIO_SMR_SBL			= BIT(3),
	SN_USIO_REG_SCR			= 1,
		SN_USIO_SCR_TXE			= BIT(0),
		SN_USIO_SCR_RXE			= BIT(1),
		SN_USIO_SCR_TBIE		= BIT(2),
		SN_USIO_SCR_TIE			= BIT(3),
		SN_USIO_SCR_RIE			= BIT(4),
		SN_USIO_SCR_UPCL		= BIT(7),
	SN_USIO_REG_ESCR		= 2,
		SN_USIO_ESCR_L_8BIT		= 0,
		SN_USIO_ESCR_L_5BIT		= 1,
		SN_USIO_ESCR_L_6BIT		= 2,
		SN_USIO_ESCR_L_7BIT		= 3,
		SN_USIO_ESCR_P			= BIT(3),
		SN_USIO_ESCR_PEN		= BIT(4),
		SN_USIO_ESCR_FLWEN		= BIT(7),

	SN_USIO_REG_SSR			= 3,
		SN_USIO_SSR_TBI			= BIT(0),
		SN_USIO_SSR_TDRE		= BIT(1),
		SN_USIO_SSR_RDRF		= BIT(2),
		SN_USIO_SSR_ORE			= BIT(3),
		SN_USIO_SSR_FRE			= BIT(4),
		SN_USIO_SSR_PE			= BIT(5),
		SN_USIO_SSR_REC			= BIT(7),
		SN_USIO_SSR_BRK			= BIT(8),
	SN_USIO_REG_DR			= 4,
	SN_USIO_REG_BGR			= 6,
	SN_USIO_REG_FCR			= 0xc,
		SN_USIO_FCR_FE1			= BIT(0),
		SN_USIO_FCR_FE2			= BIT(1),
		SN_USIO_FCR_FCL1		= BIT(2),
		SN_USIO_FCR_FCL2		= BIT(3),
		SN_USIO_FCR_FSET		= BIT(4),
		SN_USIO_FCR_FTIE		= BIT(9),
		SN_USIO_FCR_FDRQ		= BIT(10),
		SN_USIO_FCR_FRIIE		= BIT(11),
		
	SN_USIO_REG_FBYTE		= 0xe,
};

static void usio_stop_tx(struct uart_port *port)
{
	writew(readw(port->membase + SN_USIO_REG_FCR) & ~SN_USIO_FCR_FTIE,
	       port->membase + SN_USIO_REG_FCR);
	writeb(readb(port->membase + SN_USIO_REG_SCR) & ~SN_USIO_SCR_TBIE,
	       port->membase + SN_USIO_REG_SCR);
}

static void usio_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int count;

	writew(readw(port->membase + SN_USIO_REG_FCR) & ~SN_USIO_FCR_FTIE,
	       port->membase + SN_USIO_REG_FCR);
	writeb(readb(port->membase + SN_USIO_REG_SCR) &
	       ~(SN_USIO_SCR_TIE | SN_USIO_SCR_TBIE),
	       port->membase + SN_USIO_REG_SCR);

	if (port->x_char) {
		writew(port->x_char, port->membase + SN_USIO_REG_DR);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		usio_stop_tx(port);
		return;
	}

	count = port->fifosize -
		(readw(port->membase + SN_USIO_REG_FBYTE) & 0xff);

	do {
		writew(xmit->buf[xmit->tail], port->membase + SN_USIO_REG_DR);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;

	} while (--count > 0);

	writew(readw(port->membase + SN_USIO_REG_FCR) & ~SN_USIO_FCR_FDRQ,
	       port->membase + SN_USIO_REG_FCR);

	writeb(readb(port->membase + SN_USIO_REG_SCR) | SN_USIO_SCR_TBIE,
	       port->membase + SN_USIO_REG_SCR);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		usio_stop_tx(port);
}

static void usio_start_tx(struct uart_port *port)
{
	u16 fcr = readw(port->membase + SN_USIO_REG_FCR);

	writew(fcr | SN_USIO_FCR_FTIE, port->membase + SN_USIO_REG_FCR);
	if (!(fcr & SN_USIO_FCR_FDRQ))
		return;

	writeb(readb(port->membase + SN_USIO_REG_SCR) | SN_USIO_SCR_TBIE,
	       port->membase + SN_USIO_REG_SCR);

	if (readb(port->membase + SN_USIO_REG_SSR) & SN_USIO_SSR_TBI)
		usio_tx_chars(port);
}

static void usio_stop_rx(struct uart_port *port)
{
	writeb(readb(port->membase + SN_USIO_REG_SCR) & ~SN_USIO_SCR_RIE,
	       port->membase + SN_USIO_REG_SCR);
}

static void usio_enable_ms(struct uart_port *port)
{
	writeb(readb(port->membase + SN_USIO_REG_SCR) |
	       SN_USIO_SCR_RIE | SN_USIO_SCR_RXE,
	       port->membase + SN_USIO_REG_SCR);
}

static void usio_rx_chars(struct uart_port *port)
{
	struct tty_port *ttyport = &port->state->port;
	unsigned long flag = 0;
	char ch = 0;
	u8 status;
	int max_count = 2; // SN_USIO_FBYTE_GETFIFO2(readw(port->membase + SN_USIO_REG_FBYTE) );

	while (max_count--) {
		status = readb(port->membase + SN_USIO_REG_SSR);

		if (!(status & SN_USIO_SSR_RDRF))
			break;

		if (!(status & (SN_USIO_SSR_ORE | SN_USIO_SSR_FRE |
				SN_USIO_SSR_PE))) {
			ch = readw(port->membase + SN_USIO_REG_DR);
			flag = TTY_NORMAL;
			port->icount.rx++;
			if (uart_handle_sysrq_char(port, ch))
				continue;
			uart_insert_char(port, status, SN_USIO_SSR_ORE,
					 ch, flag);
			continue;
		}

#if defined(CONFIG_SERIAL_SN_USIO_FRE2BREAK)
		if (status & SN_USIO_SSR_FRE) {
			status &= ~(SN_USIO_SSR_ORE | SN_USIO_SSR_FRE |
				    SN_USIO_SSR_PE);
			port->icount.brk++;
			if (!uart_handle_break(port))
				flag = SN_USIO_SSR_BRK;
		} else
#endif
		if (status & SN_USIO_SSR_PE)
				port->icount.parity++;

#if defined(CONFIG_SERIAL_SN_USIO_FRE2BREAK)
		else if (status & SN_USIO_SSR_FRE)
			port->icount.frame++;
#endif
		if (status & SN_USIO_SSR_ORE)
			port->icount.overrun++;

		status &= port->read_status_mask;
		if (status & SN_USIO_SSR_BRK) {
			flag = TTY_BREAK;
			ch = 0;
		} else
			if (status & SN_USIO_SSR_PE) {
				flag = TTY_PARITY;
				ch = 0;
			} else
				if (status & SN_USIO_SSR_FRE) {
					flag = TTY_FRAME;
					ch = 0;
				}
		if (flag)
			uart_insert_char(port, status, SN_USIO_SSR_ORE,
					 ch, flag);

		writeb(readb(port->membase + SN_USIO_REG_SSR) | SN_USIO_SSR_REC,
			port->membase + SN_USIO_REG_SSR);

		max_count = readw(port->membase + SN_USIO_REG_FBYTE) >> 8;
		writew(readw(port->membase + SN_USIO_REG_FCR) |
		       SN_USIO_FCR_FE2 | SN_USIO_FCR_FRIIE,
		port->membase + SN_USIO_REG_FCR);
	}

	tty_flip_buffer_push(ttyport);
}

static irqreturn_t usio_rx_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;

	spin_lock(&port->lock);
	usio_rx_chars(port);
	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static irqreturn_t usio_tx_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;

	spin_lock(&port->lock);
	if (readb(port->membase + SN_USIO_REG_SSR) & SN_USIO_SSR_TBI)
		usio_tx_chars(port);
	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int usio_tx_empty(struct uart_port *port)
{
	return (readb(port->membase + SN_USIO_REG_SSR) & SN_USIO_SSR_TBI) ?
		TIOCSER_TEMT : 0;
}

static void usio_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int usio_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;

}

static void usio_break_ctl(struct uart_port *port, int break_state)
{
}

static int usio_startup(struct uart_port *port)
{
	const char *portname = to_platform_device(port->dev)->name;
	unsigned long flags;
	int ret, index = port->line;
	int flwen = 0;
	unsigned char  escr;
	ret = request_irq(usio_irq[index][RX], usio_rx_irq, 0, portname, port);
	if (ret)
		return ret;
	ret = request_irq(usio_irq[index][TX], usio_tx_irq, 0, portname, port);
	if (ret) {
		free_irq(usio_irq[index][RX], port);
		return ret;
	}

	of_property_read_u32(port->dev->of_node, "flwen", &flwen);
	escr = readb(port->membase + SN_USIO_REG_ESCR);
	if (flwen > 0) {
		escr |= SN_USIO_ESCR_FLWEN;
	}
	spin_lock_irqsave(&port->lock, flags);
	writeb(0, port->membase + SN_USIO_REG_SCR);
	writeb(escr, port->membase + SN_USIO_REG_ESCR);
	writeb(SN_USIO_SCR_UPCL, port->membase + SN_USIO_REG_SCR);
	writeb(SN_USIO_SSR_REC, port->membase + SN_USIO_REG_SSR);
	writew(0, port->membase + SN_USIO_REG_FCR);
	writew(SN_USIO_FCR_FCL1 | SN_USIO_FCR_FCL2,
	       port->membase + SN_USIO_REG_FCR);
	writew(SN_USIO_FCR_FE1 | SN_USIO_FCR_FE2 | SN_USIO_FCR_FRIIE,
	       port->membase + SN_USIO_REG_FCR);
	writew(0, port->membase + SN_USIO_REG_FBYTE);
 	writew(BIT(12), port->membase + SN_USIO_REG_FBYTE); 

	writeb(SN_USIO_SCR_TXE  | SN_USIO_SCR_RIE | SN_USIO_SCR_TBIE |
	       SN_USIO_SCR_RXE, port->membase + SN_USIO_REG_SCR);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void usio_shutdown(struct uart_port *port)
{
	int index = port->line;
#if 0
	unsigned long flags;
	unsigned short fcr;

	writeb(readb(port->membase + SN_USIO_REG_SCR) &
	       ~(SN_USIO_SCR_TIE | SN_USIO_SCR_TBIE | SN_USIO_SCR_RIE |
		 SN_USIO_SCR_RXE | SN_USIO_SCR_TXE),
	       port->membase + SN_USIO_REG_SCR);

	spin_lock_irqsave(&port->lock, flags);
	writeb(0, port->membase + SN_USIO_REG_SCR);
	fcr = readw(port->membase + SN_USIO_REG_FCR);
	fcr &= ~(SN_USIO_FCR_FTIE | SN_USIO_FCR_FRIIE);
	writew(fcr, port->membase + SN_USIO_REG_FCR);
	fcr |= SN_USIO_FCR_FCL1 | SN_USIO_FCR_FCL2;
	writew(fcr, port->membase + SN_USIO_REG_FCR);
	fcr &= ~(SN_USIO_FCR_FE1 | SN_USIO_FCR_FE2);
	writew(fcr, port->membase + SN_USIO_REG_FCR);
	spin_unlock_irqrestore(&port->lock, flags);
#endif
	free_irq(usio_irq[index][RX], port);
	free_irq(usio_irq[index][TX], port);
}

static void usio_set_termios(struct uart_port *port, struct ktermios *termios,
			     struct ktermios *old)
{
	unsigned int escr, smr = SN_USIO_SMR_SOE;
	unsigned long flags, baud, quot;
	int flwen = 0;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		escr = SN_USIO_ESCR_L_5BIT;
		break;
	case CS6:
		escr = SN_USIO_ESCR_L_6BIT;
		break;
	case CS7:
		escr = SN_USIO_ESCR_L_7BIT;
		break;
	case CS8:
	default:
		escr = SN_USIO_ESCR_L_8BIT;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		smr |= SN_USIO_SMR_SBL;

	if (termios->c_cflag & PARENB) {
		escr |= SN_USIO_ESCR_PEN;
		if (termios->c_cflag & PARODD)
			escr |= SN_USIO_ESCR_P;
	}
	/* Set hard flow control */
	of_property_read_u32(port->dev->of_node, "flwen", &flwen);
	if ((termios->c_cflag & CRTSCTS) || (flwen > 0)) {
		escr |= SN_USIO_ESCR_FLWEN;
	}

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);
	if (baud > 1)
		quot = port->uartclk / baud - 1 ;
	else
		quot = 0;

	spin_lock_irqsave(&port->lock, flags);
	uart_update_timeout(port, termios->c_cflag, baud);
	port->read_status_mask = SN_USIO_SSR_ORE | SN_USIO_SSR_RDRF |
				 SN_USIO_SSR_TDRE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= SN_USIO_SSR_FRE | SN_USIO_SSR_PE;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= SN_USIO_SSR_FRE | SN_USIO_SSR_PE;
	if ((termios->c_iflag & IGNBRK) && (termios->c_iflag & IGNPAR))
		port->ignore_status_mask |= SN_USIO_SSR_ORE;
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= SN_USIO_SSR_RDRF;

	writeb(0, port->membase + SN_USIO_REG_SCR);
	writeb(SN_USIO_SCR_UPCL, port->membase + SN_USIO_REG_SCR);
	writeb(SN_USIO_SSR_REC, port->membase + SN_USIO_REG_SSR);
	writew(0, port->membase + SN_USIO_REG_FCR);
	writeb(smr, port->membase + SN_USIO_REG_SMR);
	writeb(escr, port->membase + SN_USIO_REG_ESCR);
	writew(quot, port->membase + SN_USIO_REG_BGR);
	writew(0, port->membase + SN_USIO_REG_FCR);
	writew(SN_USIO_FCR_FCL1 | SN_USIO_FCR_FCL2 | SN_USIO_FCR_FE1 |
	       SN_USIO_FCR_FE2 | SN_USIO_FCR_FRIIE,
	       port->membase + SN_USIO_REG_FCR);
	writew(0, port->membase + SN_USIO_REG_FBYTE);
	writew(BIT(12), port->membase + SN_USIO_REG_FBYTE); 
	writeb(SN_USIO_SCR_RIE | SN_USIO_SCR_RXE | SN_USIO_SCR_TBIE |
	       SN_USIO_SCR_TXE, port->membase + SN_USIO_REG_SCR);
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char * usio_type(struct uart_port *port)
{
	return ((port->type == PORT_SN_USIO) ? USIO_NAME : NULL);
}

static void usio_release_port(struct uart_port *port)
{
}

static int usio_request_port(struct uart_port *port)
{
	return -EBUSY;
}

static void usio_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SN_USIO;
}

static struct uart_ops usio_ops = {
	.tx_empty	= usio_tx_empty,
	.set_mctrl	= usio_set_mctrl,
	.get_mctrl	= usio_get_mctrl,
	.stop_tx	= usio_stop_tx,
	.start_tx	= usio_start_tx,
	.stop_rx	= usio_stop_rx,
	.enable_ms	= usio_enable_ms,
	.break_ctl	= usio_break_ctl,
	.startup	= usio_startup,
	.shutdown	= usio_shutdown,
	.set_termios	= usio_set_termios,
	.type		= usio_type,
	.release_port	= usio_release_port,
	.request_port	= usio_request_port,
	.config_port	= usio_config_port,
};

#ifdef CONFIG_SERIAL_SN_USIO_CONSOLE

static void usio_console_putchar(struct uart_port *port, int c)
{
	while (!(readb(port->membase + SN_USIO_REG_SSR) & SN_USIO_SSR_TDRE))
		cpu_relax();

	writew(c, port->membase + SN_USIO_REG_DR);
}

static void usio_console_write(struct console *co, const char *s,
			       unsigned int count)
{
	struct uart_port *port = &usio_ports[co->index];

	uart_console_write(port, s, count, usio_console_putchar);
}

static int __init usio_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int parity = 'n';
	int flow = 'n';
	int bits = 8;
	int flwen = 0;
	if (co->index >= CONFIG_SERIAL_SN_USIO_PORTS)
		return -ENODEV;

	port = &usio_ports[co->index];
	if (!port->membase)
		return -ENODEV;


	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	of_property_read_u32(port->dev->of_node, "flwen", &flwen);
	if (flwen > 0) {
		flow = 'r';
	}

	return uart_set_options(port, co, baud, parity, bits, flow);
}


static struct uart_driver usio_reg;
static struct console usio_console = {
	.name   = "ttyUSI",
	.write  = usio_console_write,
	.device = uart_console_device,
	.setup  = usio_console_setup,
	.flags  = CON_PRINTBUFFER,
	.index  = -1,
	.data   = &usio_reg,
};

static int __init usio_console_init(void)
{
	register_console(&usio_console);
        return 0;
}
console_initcall(usio_console_init);

#define USIO_CONSOLE	&usio_console
#else
#define USIO_CONSOLE	NULL
#endif


static struct  uart_driver usio_reg = {
	.owner 		= THIS_MODULE,
	.driver_name	= USIO_NAME,
	.dev_name	= "ttyUSI",
	.cons           = USIO_CONSOLE,
	.nr		= CONFIG_SERIAL_SN_USIO_PORTS,
};

static int usio_probe(struct platform_device *pdev)
{
	struct clk *clk = of_clk_get(pdev->dev.of_node, 0);
	struct uart_port *port;
	struct resource *res;
	int index = 0;
	int ret;

	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Missing clock\n");
		return PTR_ERR(clk);
	}
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "Clock enable failed: %d\n", ret);
		return ret;
	}
	of_property_read_u32(pdev->dev.of_node, "index", &index);
	port = &usio_ports[index];

	port->private_data = (void *)clk;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Missing regs\n");
		ret = -ENODEV;
		goto failed;
	}
	port->mapbase = res->start;
	port->membase = ioremap(res->start,(res->end - res->start + 1));

	ret = platform_get_irq_byname(pdev, "rx");
	usio_irq[index][RX] = ret;

	ret = platform_get_irq_byname(pdev, "tx");
	usio_irq[index][TX] = ret;

	port->irq = usio_irq[index][RX];
	port->uartclk = clk_get_rate(clk);
	port->fifosize = 128;
	port->iotype = UPIO_MEM32;
	port->flags = UPF_BOOT_AUTOCONF | UPF_SPD_VHI ;
	port->line = index;
	port->ops = &usio_ops;
	port->dev = &pdev->dev;

	ret = uart_add_one_port(&usio_reg, port);
	if (ret) {
		dev_err(&pdev->dev, "Adding port failed: %d\n", ret);
		goto failed1;
	}
#ifdef CONFIG_PM_WARP
	platform_set_drvdata(pdev, port);
#endif
	return 0;

failed1:
	iounmap(port->membase);

failed:
	clk_disable_unprepare(clk);
	clk_put(clk);

	return ret;
}

static int usio_remove(struct platform_device *pdev)
{
	struct uart_port *port = &usio_ports[pdev->id];
	struct clk *clk = port->private_data;

	uart_remove_one_port(&usio_reg, port);
	clk_disable_unprepare(clk);
	clk_put(clk);

	return 0;
}

#ifdef CONFIG_PM
static int usio_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM_WARP
	if (pm_device_down) {
		struct uart_port *port = platform_get_drvdata(pdev);
		struct uart_pm_regs *pm_regs = &uart_pm_regs[port->line];
		unsigned long flags;

		spin_lock_irqsave(&port->lock, flags);
		pm_regs->smr = readb(port->membase + SN_USIO_REG_SMR);
		pm_regs->scr = readb(port->membase + SN_USIO_REG_SCR);
		pm_regs->escr = readb(port->membase + SN_USIO_REG_ESCR);
		pm_regs->bgr = readw(port->membase + SN_USIO_REG_BGR);
		pm_regs->fcr = readw(port->membase + SN_USIO_REG_FCR);
		pm_regs->fbyte = readw(port->membase + SN_USIO_REG_FBYTE);
		spin_unlock_irqrestore(&port->lock, flags);
	}
#endif
	return 0;
}
static int usio_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM_WARP
	if (pm_device_down) {
		struct uart_port *port = platform_get_drvdata(pdev);
		struct uart_pm_regs *pm_regs = &uart_pm_regs[port->line];
		unsigned long flags;

		spin_lock_irqsave(&port->lock, flags);
		writeb(0, port->membase + SN_USIO_REG_SCR);
		writeb(SN_USIO_SCR_UPCL, port->membase + SN_USIO_REG_SCR);
		writeb(SN_USIO_SSR_REC, port->membase + SN_USIO_REG_SSR);
		writew(0, port->membase + SN_USIO_REG_FCR);
		writeb(pm_regs->smr, port->membase + SN_USIO_REG_SMR);
		writeb(pm_regs->escr, port->membase + SN_USIO_REG_ESCR);
		writew(pm_regs->bgr, port->membase + SN_USIO_REG_BGR);
		writew(0, port->membase + SN_USIO_REG_FCR);
		writew(pm_regs->fcr | SN_USIO_FCR_FCL1 | SN_USIO_FCR_FCL2,
		       port->membase + SN_USIO_REG_FCR);
		writew(0, port->membase + SN_USIO_REG_FBYTE);
		writew(pm_regs->fbyte, port->membase + SN_USIO_REG_FBYTE);
		writeb(pm_regs->scr, port->membase + SN_USIO_REG_SCR);
		spin_unlock_irqrestore(&port->lock, flags);
	}
#endif
	return 0;
}
#else
#define usio_suspend NULL
#define usio_resume NULL
#endif

static struct of_device_id m8m_usio_dt_ids[] = {
	{ .compatible = "socionext,m8m-usio-uart" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, m8m_usio_dt_ids);

static struct platform_driver usio_driver = {
	.probe          = usio_probe,
	.remove         = usio_remove,
	.suspend        = usio_suspend,
	.resume         = usio_resume,
	.driver         = {
		.name   = USIO_NAME,
		.of_match_table = m8m_usio_dt_ids,
	},
};

static int __init usio_init(void)
{
	int ret = uart_register_driver(&usio_reg);

	if (ret) {
		pr_err("%s: uart registration failed: %d\n", __func__, ret);
		return ret;
	}
	ret = platform_driver_register(&usio_driver);
	if (ret) {
		uart_unregister_driver(&usio_reg);
		pr_err("%s: drv registration failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void __exit usio_exit(void)
{
	platform_driver_unregister(&usio_driver);
	uart_unregister_driver(&usio_reg);
}

module_init(usio_init);
module_exit(usio_exit);

MODULE_AUTHOR("SOCIONEXT");
MODULE_DESCRIPTION("F_USIO/UART Driver");
MODULE_LICENSE("GPL");

