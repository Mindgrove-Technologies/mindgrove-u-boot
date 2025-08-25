// #include <common.h>
#include <clk.h>
#include <debug_uart.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <serial.h>
#include <linux/err.h>

DECLARE_GLOBAL_DATA_PTR;
#define UART0_BASE_ADDR 0x11300

#ifdef CONFIG_TARGET_SECURE_IOT
#define SHAKTI_DEFAULT_CLOCK 700000000
#else
#define SHAKTI_DEFAULT_CLOCK 40000000
#endif
#define REG_BAUD	    0x00
#define REG_TX		    0x04
#define REG_RX		    0x08
#define REG_STATUS	    0x0C
#define REG_DELAY	    0x10
#define REG_CONTROL	    0x14
#define REG_INT_EN	    0x18
#define REG_IQ_CYCLES	0x1C
#define REG_RX_THRES	0x20

#define STS_RX_NOT_EMPTY    1 << 2
#define STS_TX_FULL 	    1 << 1
#define STS_TX_EMPTY 	    1 << 0

struct mindgrove_uart_plat {
	unsigned long clock;
	volatile void *regs;
};

static struct mindgrove_uart_plat uart_debug;

static int _mindgrove_serial_setbrg(struct mindgrove_uart_plat *plat, int baudrate)
{
	int ret;
	u16 baud = 0;
/*
	ret = clk_get_by_index(dev, 0, &clk);
	if (IS_ERR_VALUE(ret)) {
		debug("mindgrove UART failed to get clock\n");
		ret = dev_read_u32(dev, "clock-frequency", &clock);
		if (IS_ERR_VALUE(ret)) {
			debug("mindgrove UART clock not defined\n");
			return 0;
		}
	} else {
		clock = clk_get_rate(&clk);
		if (IS_ERR_VALUE(clock)) {
			debug("mindgrove UART clock get rate failed\n");
			return 0;
		}
	}
	plat->clock = clock;
*/
    baud = (u16)(plat->clock/(16 * baudrate));
    writew(baud, plat->regs + REG_BAUD);

	return 0;
}

static int mindgrove_serial_probe(struct udevice *dev)
{
	return 0;
}

static int _mindgrove_serial_getc(struct mindgrove_uart_plat *plat)
{
	int c;

    while ((readw(plat->regs + REG_STATUS) & STS_RX_NOT_EMPTY) == 0);
    return readb(plat->regs + REG_RX);
}

static int _mindgrove_serial_putc(struct mindgrove_uart_plat *plat, const char ch)
{
	int rc;

	while(readw(plat->regs + REG_STATUS) &  STS_TX_FULL);
	writeb(ch, plat->regs + REG_TX);
/*
    if ((ch == '\n') || (ch == '\r')) {
	    writeb('\n', plat->regs + REG_TX);
	    writeb('\r', plat->regs + REG_TX);
    }*/
}

static int mindgrove_serial_of_to_plat(struct udevice *dev)
{
	struct mindgrove_uart_plat *plat = dev_get_plat(dev);

	plat->regs = (void *)UART0_BASE_ADDR;
    plat->clock = SHAKTI_DEFAULT_CLOCK;
	return 0;
}

static int mindgrove_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct mindgrove_uart_plat *plat = dev_get_plat(dev);
	return _mindgrove_serial_setbrg(plat, baudrate);
}


static int mindgrove_serial_getc(struct udevice *dev)
{
	struct mindgrove_uart_plat *plat = dev_get_plat(dev);
    return _mindgrove_serial_getc(plat);
}

static int mindgrove_serial_putc(struct udevice *dev, const char ch)
{
	struct mindgrove_uart_plat *plat = dev_get_plat(dev);
    return _mindgrove_serial_putc(plat, ch);
}


static const struct dm_serial_ops mindgrove_serial_ops = {
	.putc = mindgrove_serial_putc,
	.getc = mindgrove_serial_getc,
	.setbrg = mindgrove_serial_setbrg,
};

static const struct udevice_id mindgrove_serial_ids[] = {
	{ .compatible = "mindgrove,uart0" },
	{ }
};

U_BOOT_DRIVER(serial_mindgrove) = {
	.name	= "serial_mindgrove",
	.id	= UCLASS_SERIAL,
	.of_match = mindgrove_serial_ids,
	.of_to_plat = mindgrove_serial_of_to_plat,
	.plat_auto	= sizeof(struct mindgrove_uart_plat),
	.probe = mindgrove_serial_probe,
	.ops	= &mindgrove_serial_ops,
};
#ifdef CONFIG_DEBUG_UART_MINDGROVE
static inline void _debug_uart_init(void)
{
	uart_debug.regs = (void *)UART0_BASE_ADDR;
    uart_debug.clock = SHAKTI_DEFAULT_CLOCK;

	_mindgrove_serial_setbrg(&uart_debug, CONFIG_BAUDRATE);
}

static inline void _debug_uart_putc(int ch)
{
    _mindgrove_serial_putc(&uart_debug, ch);
}

DEBUG_UART_FUNCS

#endif
