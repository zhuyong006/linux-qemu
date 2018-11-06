/*
 * Copyright (C) Maxime Coquelin 2015
 * Author:  Maxime Coquelin <mcoquelin.n76e003@gmail.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Inspired by st-asc.c from STMicroelectronics (c)
 */

#if defined(CONFIG_SERIAL_N76E003_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/serial_core.h>
#include <linux/clk.h>

#define DRIVER_NAME "n76e003-usart"

/* Register offsets */
#define USART_SR		0x00
#define USART_DR		0x04
#define USART_BRR		0x08
#define USART_CR1		0x0c
#define USART_CR2		0x10
#define USART_CR3		0x14
#define USART_GTPR		0x18

/* USART_SR */
#define USART_SR_PE		BIT(0)
#define USART_SR_FE		BIT(1)
#define USART_SR_NF		BIT(2)
#define USART_SR_ORE		BIT(3)
#define USART_SR_IDLE		BIT(4)
#define USART_SR_RXNE		BIT(5)
#define USART_SR_TC		BIT(6)
#define USART_SR_TXE		BIT(7)
#define USART_SR_LBD		BIT(8)
#define USART_SR_CTS		BIT(9)
#define USART_SR_ERR_MASK	(USART_SR_LBD | USART_SR_ORE | \
				 USART_SR_FE | USART_SR_PE)
/* Dummy bits */
#define USART_SR_DUMMY_RX	BIT(16)

/* USART_DR */
#define USART_DR_MASK		GENMASK(8, 0)

/* USART_BRR */
#define USART_BRR_DIV_F_MASK	GENMASK(3, 0)
#define USART_BRR_DIV_M_MASK	GENMASK(15, 4)
#define USART_BRR_DIV_M_SHIFT	4

/* USART_CR1 */
#define USART_CR1_SBK		BIT(0)
#define USART_CR1_RWU		BIT(1)
#define USART_CR1_RE		BIT(2)
#define USART_CR1_TE		BIT(3)
#define USART_CR1_IDLEIE	BIT(4)
#define USART_CR1_RXNEIE	BIT(5)
#define USART_CR1_TCIE		BIT(6)
#define USART_CR1_TXEIE		BIT(7)
#define USART_CR1_PEIE		BIT(8)
#define USART_CR1_PS		BIT(9)
#define USART_CR1_PCE		BIT(10)
#define USART_CR1_WAKE		BIT(11)
#define USART_CR1_M		BIT(12)
#define USART_CR1_UE		BIT(13)
#define USART_CR1_OVER8		BIT(15)
#define USART_CR1_IE_MASK	GENMASK(8, 4)
#define PORT_N76E003     113

/* USART_CR2 */
#define USART_CR2_ADD_MASK	GENMASK(3, 0)
#define USART_CR2_LBDL		BIT(5)
#define USART_CR2_LBDIE		BIT(6)
#define USART_CR2_LBCL		BIT(8)
#define USART_CR2_CPHA		BIT(9)
#define USART_CR2_CPOL		BIT(10)
#define USART_CR2_CLKEN		BIT(11)
#define USART_CR2_STOP_2B	BIT(13)
#define USART_CR2_STOP_MASK	GENMASK(13, 12)
#define USART_CR2_LINEN		BIT(14)

/* USART_CR3 */
#define USART_CR3_EIE		BIT(0)
#define USART_CR3_IREN		BIT(1)
#define USART_CR3_IRLP		BIT(2)
#define USART_CR3_HDSEL		BIT(3)
#define USART_CR3_NACK		BIT(4)
#define USART_CR3_SCEN		BIT(5)
#define USART_CR3_DMAR		BIT(6)
#define USART_CR3_DMAT		BIT(7)
#define USART_CR3_RTSE		BIT(8)
#define USART_CR3_CTSE		BIT(9)
#define USART_CR3_CTSIE		BIT(10)
#define USART_CR3_ONEBIT	BIT(11)

/* USART_GTPR */
#define USART_GTPR_PSC_MASK	GENMASK(7, 0)
#define USART_GTPR_GT_MASK	GENMASK(15, 8)

#define DRIVER_NAME "n76e003-usart"
#define n76e003_SERIAL_NAME "ttyS"
#define n76e003_MAX_PORTS 6

struct n76e003_port {
	struct uart_port port;
	bool hw_flow_control;
};

static struct n76e003_port n76e003_ports[n76e003_MAX_PORTS];
static struct uart_driver n76e003_usart_driver;

static void n76e003_stop_tx(struct uart_port *port);

static inline struct n76e003_port *to_n76e003_port(struct uart_port *port)
{
	return container_of(port, struct n76e003_port, port);
}

static void n76e003_set_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val |= bits;
	writel_relaxed(val, port->membase + reg);
}

static void n76e003_clr_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val &= ~bits;
	writel_relaxed(val, port->membase + reg);
}

static void n76e003_receive_chars(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	unsigned long c;
	u32 sr;
	char flag;

	if (port->irq_wake)
		pm_wakeup_event(tport->tty->dev, 0);

	while ((sr = readl_relaxed(port->membase + USART_SR)) & USART_SR_RXNE) {
		sr |= USART_SR_DUMMY_RX;
		c = readl_relaxed(port->membase + USART_DR);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (sr & USART_SR_ERR_MASK) {
			if (sr & USART_SR_LBD) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (sr & USART_SR_ORE) {
				port->icount.overrun++;
			} else if (sr & USART_SR_PE) {
				port->icount.parity++;
			} else if (sr & USART_SR_FE) {
				port->icount.frame++;
			}

			sr &= port->read_status_mask;

			if (sr & USART_SR_LBD)
				flag = TTY_BREAK;
			else if (sr & USART_SR_PE)
				flag = TTY_PARITY;
			else if (sr & USART_SR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, c))
			continue;
		uart_insert_char(port, sr, USART_SR_ORE, c, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tport);
	spin_lock(&port->lock);
}

static void n76e003_transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	if (port->x_char) {
		writel_relaxed(port->x_char, port->membase + USART_DR);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	if (uart_tx_stopped(port)) {
		n76e003_stop_tx(port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		n76e003_stop_tx(port);
		return;
	}

	writel_relaxed(xmit->buf[xmit->tail], port->membase + USART_DR);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	port->icount.tx++;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		n76e003_stop_tx(port);
}

static irqreturn_t n76e003_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	u32 sr;

	spin_lock(&port->lock);

	sr = readl_relaxed(port->membase + USART_SR);

	if (sr & USART_SR_RXNE)
		n76e003_receive_chars(port);

	if (sr & USART_SR_TXE)
		n76e003_transmit_chars(port);

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int n76e003_tx_empty(struct uart_port *port)
{

}

static void n76e003_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}

static unsigned int n76e003_get_mctrl(struct uart_port *port)
{
	/* This routine is used to get signals of: DCD, DSR, RI, and CTS */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

/* Transmit stop */
static void n76e003_stop_tx(struct uart_port *port)
{
	//n76e003_clr_bits(port, USART_CR1, USART_CR1_TXEIE);
}

/* There are probably characters waiting to be transmitted. */
static void n76e003_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	if (uart_circ_empty(xmit))
		return;

	//n76e003_set_bits(port, USART_CR1, USART_CR1_TXEIE | USART_CR1_TE);
}

/* Throttle the remote when input buffer is about to overflow. */
static void n76e003_throttle(struct uart_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	//n76e003_clr_bits(port, USART_CR1, USART_CR1_RXNEIE);
	spin_unlock_irqrestore(&port->lock, flags);
}

/* Unthrottle the remote, the input buffer can now accept data. */
static void n76e003_unthrottle(struct uart_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	//n76e003_set_bits(port, USART_CR1, USART_CR1_RXNEIE);
	spin_unlock_irqrestore(&port->lock, flags);
}

/* Receive stop */
static void n76e003_stop_rx(struct uart_port *port)
{
	//n76e003_clr_bits(port, USART_CR1, USART_CR1_RXNEIE);
}

/* Handle breaks - ignored by us */
static void n76e003_break_ctl(struct uart_port *port, int break_state)
{
}

static int n76e003_startup(struct uart_port *port)
{
	const char *name = to_platform_device(port->dev)->name;
	u32 val;
	int ret;
	printk("%s LINE = %d\n",__func__,__LINE__);
	ret = request_irq(port->irq, n76e003_interrupt, 0, name, port);


	val = USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
//	n76e003_set_bits(port, USART_CR1, val);

	return 0;
}

static void n76e003_shutdown(struct uart_port *port)
{
	u32 val;

	val = USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
//	n76e003_set_bits(port, USART_CR1, val);
	printk("%s LINE = %d\n",__func__,__LINE__);

	free_irq(port->irq, port);
}

static void n76e003_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	struct n76e003_port *n76e003_port = to_n76e003_port(port);
	unsigned int baud;
	u32 usartdiv, mantissa, fraction, oversampling;
	tcflag_t cflag = termios->c_cflag;
	u32 cr1, cr2, cr3;
	unsigned long flags;

	if (!n76e003_port->hw_flow_control)
		cflag &= ~CRTSCTS;
	
	printk("%s LINE = %d\n",__func__,__LINE__);

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 8);

	spin_lock_irqsave(&port->lock, flags);

	/* Stop serial port and reset value */
	writel_relaxed(0, port->membase + USART_CR1);

	cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
	cr2 = 0;
	cr3 = 0;

	if (cflag & CSTOPB)
		cr2 |= USART_CR2_STOP_2B;

	if (cflag & PARENB) {
		cr1 |= USART_CR1_PCE;
		if ((cflag & CSIZE) == CS8)
			cr1 |= USART_CR1_M;
	}

	if (cflag & PARODD)
		cr1 |= USART_CR1_PS;

	port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	if (cflag & CRTSCTS) {
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
		cr3 |= USART_CR3_CTSE;
	}

	usartdiv = DIV_ROUND_CLOSEST(port->uartclk, baud);

	/*
	 * The USART supports 16 or 8 times oversampling.
	 * By default we prefer 16 times oversampling, so that the receiver
	 * has a better tolerance to clock deviations.
	 * 8 times oversampling is only used to achieve higher speeds.
	 */
	if (usartdiv < 16) {
		oversampling = 8;
		n76e003_set_bits(port, USART_CR1, USART_CR1_OVER8);
	} else {
		oversampling = 16;
		n76e003_clr_bits(port, USART_CR1, USART_CR1_OVER8);
	}

	mantissa = (usartdiv / oversampling) << USART_BRR_DIV_M_SHIFT;
	fraction = usartdiv % oversampling;
	writel_relaxed(mantissa | fraction, port->membase + USART_BRR);

	uart_update_timeout(port, cflag, baud);

	port->read_status_mask = USART_SR_ORE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= USART_SR_PE | USART_SR_FE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= USART_SR_LBD;

	/* Characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask = USART_SR_PE | USART_SR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= USART_SR_LBD;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= USART_SR_ORE;
	}

	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= USART_SR_DUMMY_RX;

	writel_relaxed(cr3, port->membase + USART_CR3);
	writel_relaxed(cr2, port->membase + USART_CR2);
	writel_relaxed(cr1, port->membase + USART_CR1);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *n76e003_type(struct uart_port *port)
{
	return (port->type == PORT_N76E003) ? DRIVER_NAME : NULL;
}

static void n76e003_release_port(struct uart_port *port)
{
}

static int n76e003_request_port(struct uart_port *port)
{
	return 0;
}

static void n76e003_config_port(struct uart_port *port, int flags)
{
//	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_N76E003;
}

static int
n76e003_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No user changeable parameters */
	return 0;
}

static void n76e003_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
//	struct n76e003_port *n76e003port = container_of(port,
//			struct n76e003_port, port);
	unsigned long flags = 0;

	switch (state) {
	case UART_PM_STATE_ON:
		break;
	case UART_PM_STATE_OFF:
		spin_lock_irqsave(&port->lock, flags);
//		n76e003_clr_bits(port, USART_CR1, USART_CR1_UE);
		spin_unlock_irqrestore(&port->lock, flags);
		break;
	}
}

static const struct uart_ops n76e003_uart_ops = {
	.tx_empty	= n76e003_tx_empty,
	.set_mctrl	= n76e003_set_mctrl,
	.get_mctrl	= n76e003_get_mctrl,
	.stop_tx	= n76e003_stop_tx,
	.start_tx	= n76e003_start_tx,
	.throttle	= n76e003_throttle,
	.unthrottle	= n76e003_unthrottle,
	.stop_rx	= n76e003_stop_rx,
	.break_ctl	= n76e003_break_ctl,
	.startup	= n76e003_startup,
	.shutdown	= n76e003_shutdown,
	.set_termios	= n76e003_set_termios,
	.pm		= n76e003_pm,
	.type		= n76e003_type,
	.release_port	= n76e003_release_port,
	.request_port	= n76e003_request_port,
	.config_port	= n76e003_config_port,
	.verify_port	= n76e003_verify_port,
};

static int n76e003_init_port(struct n76e003_port *n76e003port,
			  struct platform_device *pdev)
{
	struct uart_port *port = &n76e003port->port;


	port->iotype	= UPIO_MEM;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &n76e003_uart_ops;
	port->dev	= &pdev->dev;
	port->irq	= platform_get_irq(pdev, 0);
	port->iobase = 1;



	spin_lock_init(&port->lock);

	return 0;
}

static struct n76e003_port *n76e003_of_get_n76e003_port(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int id;

	if (!np)
		return NULL;

	id = of_alias_get_id(np, "serial");
	if (id < 0)
		id = 0;

	if (WARN_ON(id >= n76e003_MAX_PORTS))
		return NULL;

	n76e003_ports[id].hw_flow_control = of_property_read_bool(np,
							"auto-flow-control");
	n76e003_ports[id].port.line = id;
	return &n76e003_ports[id];
}

#ifdef CONFIG_OF
static const struct of_device_id n76e003_match[] = {
	{ .compatible = "st,n76e003-usart", },
	{ .compatible = "st,n76e003-uart", },
	{},
};

MODULE_DEVICE_TABLE(of, n76e003_match);
#endif

static int n76e003_serial_probe(struct platform_device *pdev)
{
	int ret;
	struct n76e003_port *n76e003port;

	printk("%s LINE = %d\n",__func__,__LINE__);
	n76e003port = n76e003_of_get_n76e003_port(pdev);
	if (!n76e003port)
		return -ENODEV;

	ret = n76e003_init_port(n76e003port, pdev);
	if (ret)
		return ret;

	ret = uart_add_one_port(&n76e003_usart_driver, &n76e003port->port);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &n76e003port->port);

	return 0;
}

static int n76e003_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	return uart_remove_one_port(&n76e003_usart_driver, port);
}


#ifdef CONFIG_SERIAL_N76E003_CONSOLE
static void n76e003_console_putchar(struct uart_port *port, int ch)
{
	printk("%s LINE = %d\n",__func__,__LINE__);

}

static void n76e003_console_write(struct console *co, const char *s, unsigned cnt)
{
	struct uart_port *port = &n76e003_ports[co->index].port;
	unsigned long flags;
	u32 old_cr1, new_cr1;
	int locked = 1;

	local_irq_save(flags);
	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else
		spin_lock(&port->lock);


	uart_console_write(port, s, cnt, n76e003_console_putchar);
	printk("%s LINE = %d\n",__func__,__LINE__);



	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static int n76e003_console_setup(struct console *co, char *options)
{
	struct n76e003_port *n76e003port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= n76e003_MAX_PORTS)
		return -ENODEV;

	n76e003port = &n76e003_ports[co->index];



	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&n76e003port->port, co, baud, parity, bits, flow);
}

static struct console n76e003_console = {
	.name		= n76e003_SERIAL_NAME,
	.device		= uart_console_device,
	.write		= n76e003_console_write,
	.setup		= n76e003_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &n76e003_usart_driver,
};

#define n76e003_SERIAL_CONSOLE (&n76e003_console)

#else
#define n76e003_SERIAL_CONSOLE NULL
#endif /* CONFIG_SERIAL_N76E003_CONSOLE */

static struct uart_driver n76e003_usart_driver = {
	.driver_name	= DRIVER_NAME,
	.dev_name	= n76e003_SERIAL_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= n76e003_MAX_PORTS,
	.cons		= n76e003_SERIAL_CONSOLE,
};

static struct platform_driver n76e003_serial_driver = {
	.probe		= n76e003_serial_probe,
	.remove		= n76e003_serial_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(n76e003_match),
	},
};

static int __init usart_init(void)
{
	static char banner[] __initdata = "n76e003 USART driver initialized";
	int ret;

	pr_info("%s\n", banner);

	ret = uart_register_driver(&n76e003_usart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&n76e003_serial_driver);
	if (ret)
		uart_unregister_driver(&n76e003_usart_driver);

	return ret;
}

static void __exit usart_exit(void)
{
	platform_driver_unregister(&n76e003_serial_driver);
	uart_unregister_driver(&n76e003_usart_driver);
}

module_init(usart_init);
module_exit(usart_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("STMicroelectronics n76e003 serial port driver");
MODULE_LICENSE("GPL v2");
