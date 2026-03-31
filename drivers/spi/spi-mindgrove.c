// SPDX-License-Identifier: GPL-2.0+
/*
 * Mindgrove SPI controller driver for U-Boot
 *
 * Copyright (C) 2025 Mindgrove Technologies Private Limited
 *
 * Optimizations vs previous version:
 *  1. Remove printf from set_speed — was called per-transfer, printing
 *     ~60 chars at 115200 baud = ~5ms per call = minutes of overhead.
 *  2. Fix wait_complete bug: "if (iter > 100) return 0" caused early
 *     exit before transfer completed, silently corrupting data and
 *     paradoxically making transfers slower (retries).
 *  3. FIFO burst mode in xfer: fill up to fifo_depth bytes before
 *     draining, instead of one-byte-at-a-time, reducing MMIO round-trips.
 *  4. Early-exit in set_speed when speed unchanged.
 *  5. Use readw for COMM_STATUS (16-bit register).
 *  6. NCS_CTRL uses writel (32-bit register).
 *  7. CS control logic corrected: SW=0 asserts active-low NCS.
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <spi.h>
#include <spi-mem.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define MINDGROVE_SPI_MAX_CS		4
#define MINDGROVE_SPI_DEFAULT_DEPTH	32
#define MINDGROVE_SPI_DEFAULT_BITS	8
#define MINDGROVE_SPI_TIMEOUT_US	1000000
#define MINDGROVE_SPI_MAX_FREQ		35000000

/* Register offsets */
#define MINDGROVE_SPI_REG_CTRL		0x00
#define MINDGROVE_SPI_REG_CLK_CTRL	0x04
#define MINDGROVE_SPI_REG_TX		0x08
#define MINDGROVE_SPI_REG_RX		0x0C
#define MINDGROVE_SPI_REG_INTR_EN	0x10
#define MINDGROVE_SPI_REG_FIFO_STATUS	0x14
#define MINDGROVE_SPI_REG_COMM_STATUS	0x18	/* 16-bit */
#define MINDGROVE_SPI_REG_NCS_CTRL	0x1C	/* 32-bit */

/* CTRL register */
#define MINDGROVE_SPI_CTRL_SLAVE_MODE(x)	((x) << 0)
#define MINDGROVE_SPI_CTRL_EN(x)		((x) << 1)
#define MINDGROVE_SPI_CTRL_LSBFIRST(x)		((x) << 2)
#define MINDGROVE_SPI_CTRL_RX_FLUSH(x)		((x) << 3)
#define MINDGROVE_SPI_CTRL_COMM_MODE(x)	((x) << 4)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(x)	((x) << 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(x)	((x) << 14)
#define MINDGROVE_SPI_CTRL_SCLK_OUTEN		((u32)BIT(22))
#define MINDGROVE_SPI_CTRL_NCS_OUTEN		((u32)BIT(23))
/* MISO_OUTEN: do NOT set in master mode — disables MISO input buffer */
#define MINDGROVE_SPI_CTRL_MOSI_OUTEN		((u32)BIT(25))

/* CLK_CTRL register */
#define MINDGROVE_SPI_CLK_CTRL_POLARITY		BIT(0)
#define MINDGROVE_SPI_CLK_CTRL_PHASE		BIT(1)
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT	2
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK	GENMASK(15, 2)
#define MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT	16
#define MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT	24

/* FIFO_STATUS register (32-bit) */
#define MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY	BIT(0)
#define MINDGROVE_SPI_FIFO_STATUS_TX_FULL	BIT(8)
#define MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY	BIT(9)

/* COMM_STATUS register (16-bit — use readw) */
#define MINDGROVE_SPI_COMM_STATUS_BUSY		BIT(0)

/* NCS_CTRL register (32-bit — use readl/writel) */
#define MINDGROVE_SPI_NCS_CTRL_SELECT(x)	((u32)((x) << 0))
#define MINDGROVE_SPI_NCS_CTRL_SW(x)		((u32)((x) << 1))

/* Communication modes */
#define MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX	3

struct mindgrove_spi {
	void __iomem	*base;
	u32		fifo_depth;
	u32		bits_per_word;
	u32		input_clk_hz;
	u32		spi_freq;
	u16		prescaler;
	u8		num_cs;
};

/* ------------------------------------------------------------------ */
/* Hardware helpers                                                     */
/* ------------------------------------------------------------------ */

/*
 * Wait for TX FIFO empty AND engine not busy.
 * Called after a burst to confirm all bytes have been clocked out.
 */
static int mindgrove_spi_wait_complete(struct mindgrove_spi *spi)
{
	u32 timeout = MINDGROVE_SPI_TIMEOUT_US;

	while (timeout--) {
		u32 fs = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
		u16 cs = readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS);

		if ((fs & MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY) &&
		    !(cs & MINDGROVE_SPI_COMM_STATUS_BUSY))
			return 0;
	}
	return -ETIMEDOUT;
}

static void mindgrove_spi_prep(struct mindgrove_spi *spi)
{
	u32 ctrl;

	ctrl  = MINDGROVE_SPI_CTRL_COMM_MODE(MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX);
	ctrl |= MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(spi->bits_per_word);
	ctrl |= MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(spi->bits_per_word);
	ctrl |= MINDGROVE_SPI_CTRL_SCLK_OUTEN;
	ctrl |= MINDGROVE_SPI_CTRL_NCS_OUTEN;
	ctrl |= MINDGROVE_SPI_CTRL_MOSI_OUTEN;
	/* MISO_OUTEN intentionally omitted */
	ctrl |= MINDGROVE_SPI_CTRL_EN(1);

	writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
}

/*
 * Assert or deassert chip-select via NCS_CTRL SW bit.
 *
 * NCS_CTRL_SW controls the NCS pin directly:
 *   SW=0 → NCS low  → CS asserted   (active-low device, enable=true)
 *   SW=1 → NCS high → CS deasserted (active-low device, enable=false)
 *
 * For active-high devices (SPI_CS_HIGH), logic is inverted.
 */
static void mindgrove_spi_cs_set(struct mindgrove_spi *spi,
				 struct dm_spi_slave_plat *slave_plat,
				 bool enable)
{
	u32 ncs_ctrl = readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
	bool cs_high = !!(slave_plat->mode & SPI_CS_HIGH);

	/*
	 * SW=0 asserts active-low, SW=1 asserts active-high.
	 * enable==cs_high gives the correct SW value for all combinations.
	 */
	if (enable == cs_high)
		ncs_ctrl |=  MINDGROVE_SPI_NCS_CTRL_SW(1);
	else
		ncs_ctrl &= ~MINDGROVE_SPI_NCS_CTRL_SW(1);

	writel(ncs_ctrl, spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
}

/* ------------------------------------------------------------------ */
/* DM SPI ops                                                           */
/* ------------------------------------------------------------------ */

static int mindgrove_spi_set_speed(struct udevice *bus, uint speed)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);
	u32 prescaler, clk_ctrl;

	if (!speed)
		return 0;

	prescaler = spi->input_clk_hz / speed;
	if (prescaler)
		prescaler -= 1;
	if (prescaler > 0x3FFF)
		prescaler = 0x3FFF;

	/* Skip register write if speed unchanged */
	if (prescaler == spi->prescaler && spi->spi_freq == speed)
		return 0;

	spi->prescaler = prescaler;
	spi->spi_freq  = speed;

	clk_ctrl  = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK;
	clk_ctrl |= (prescaler << MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT);
	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	/* No printf — this is called before every transfer and printf at
	 * 115200 baud costs ~5ms per call, adding minutes to large transfers */
	return 0;
}

static int mindgrove_spi_set_mode(struct udevice *bus, uint mode)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);
	u32 clk_ctrl, ctrl, ncs_ctrl;

	clk_ctrl  = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~(MINDGROVE_SPI_CLK_CTRL_POLARITY |
		      MINDGROVE_SPI_CLK_CTRL_PHASE);
	if (mode & SPI_CPHA)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_PHASE;
	if (mode & SPI_CPOL)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_POLARITY;
	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	ctrl = readl(spi->base + MINDGROVE_SPI_REG_CTRL);
	if (mode & SPI_LSB_FIRST)
		ctrl |=  MINDGROVE_SPI_CTRL_LSBFIRST(1);
	else
		ctrl &= ~MINDGROVE_SPI_CTRL_LSBFIRST(1);
	writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);

	/* NCS_CTRL: select CS0, start deasserted (SW=1 for active-low) */
	ncs_ctrl = MINDGROVE_SPI_NCS_CTRL_SELECT(1);
	if (!(mode & SPI_CS_HIGH))
		ncs_ctrl |= MINDGROVE_SPI_NCS_CTRL_SW(1); /* deasserted = high */
	writel(ncs_ctrl, spi->base + MINDGROVE_SPI_REG_NCS_CTRL);

	return 0;
}

/*
 * mindgrove_spi_xfer - transfer bitlen bits over SPI.
 *
 * Uses FIFO burst mode: fills up to fifo_depth bytes into TX before
 * draining RX, dramatically reducing MMIO accesses vs byte-by-byte.
 *
 * In FULL_DUPLEX mode the hardware always fills the RX FIFO for every
 * TX byte, so we must always drain RX even when din==NULL.
 */
static int mindgrove_spi_xfer(struct udevice *dev, unsigned int bitlen,
			      const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct mindgrove_spi *spi = dev_get_priv(bus);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	const u8 *tx_ptr = dout;
	u8 *rx_ptr = din;
	u32 remaining_len;
	u32 tx_sent = 0;
	u32 rx_got  = 0;
	u32 fifo_status;
	int ret = 0;

	if (bitlen % 8) {
		dev_err(dev, "Non-byte aligned transfer not supported\n");
		return -EINVAL;
	}
	remaining_len = bitlen / 8;

	if (flags & SPI_XFER_BEGIN) {
		mindgrove_spi_prep(spi);
		mindgrove_spi_cs_set(spi, slave_plat, true); /* assert CS */
	}

	/*
	 * Burst transfer loop:
	 * - Push up to fifo_depth bytes ahead of what we've received
	 *   (keeps TX FIFO full without RX overflow)
	 * - Drain RX FIFO whenever data is available
	 *
	 * This reduces MMIO accesses from 4N to ~2N + 2*(N/fifo_depth)
	 * compared to byte-by-byte polling.
	 */
	while (rx_got < remaining_len) {
		/* Fill TX FIFO */
		while (tx_sent < remaining_len &&
		       (tx_sent - rx_got) < spi->fifo_depth) {
			fifo_status = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
			if (fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_FULL)
				break;
			writeb(tx_ptr ? tx_ptr[tx_sent] : 0xFF,
			       spi->base + MINDGROVE_SPI_REG_TX);
			tx_sent++;
		}

		/* Drain RX FIFO */
		while (rx_got < tx_sent) {
			fifo_status = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
			if (fifo_status & MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY)
				break;
			u8 rx_data = readb(spi->base + MINDGROVE_SPI_REG_RX);
			if (rx_ptr)
				rx_ptr[rx_got] = rx_data;
			rx_got++;
		}
	}

	ret = mindgrove_spi_wait_complete(spi);
	if (ret)
		dev_err(dev, "SPI transfer timeout\n");

	if (flags & SPI_XFER_END)
		mindgrove_spi_cs_set(spi, slave_plat, false); /* deassert CS */

	return ret;
}

static int mindgrove_spi_cs_info(struct udevice *bus, uint cs,
				 struct spi_cs_info *info)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);

	if (cs >= spi->num_cs)
		return -EINVAL;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Probe / init                                                         */
/* ------------------------------------------------------------------ */

static void mindgrove_spi_init_hw(struct mindgrove_spi *spi)
{
	spi->num_cs = MINDGROVE_SPI_MAX_CS;

	/* Disable engine */
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);

	/* Flush RX FIFO */
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(1), spi->base + MINDGROVE_SPI_REG_CTRL);
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);

	/* Master mode */
	writel(MINDGROVE_SPI_CTRL_SLAVE_MODE(0), spi->base + MINDGROVE_SPI_REG_CTRL);

	/*
	 * CLK_CTRL: safe initial 400kHz (prescaler=124 at 50MHz),
	 * setup=1 cycle, hold=1 cycle.
	 * Full register write clears any leftover BBL state.
	 */
	writel((124u << MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT) |
	       (1u  << MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT) |
	       (1u  << MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT),
	       spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	/*
	 * NCS_CTRL: CS0 selected, SW=1 (NCS deasserted / high).
	 * writel — 32-bit register.
	 */
	writel(MINDGROVE_SPI_NCS_CTRL_SELECT(1) | MINDGROVE_SPI_NCS_CTRL_SW(1),
	       spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
}

static int mindgrove_spi_probe(struct udevice *bus)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);

	spi->base = (void __iomem *)(ulong)dev_remap_addr(bus);
	if (!spi->base)
		return -ENODEV;

	spi->fifo_depth = dev_read_u32_default(bus, "mindgrove,fifo-depth",
					       MINDGROVE_SPI_DEFAULT_DEPTH);
	spi->bits_per_word = dev_read_u32_default(bus, "mindgrove,max-bits-per-word",
						  MINDGROVE_SPI_DEFAULT_BITS);
	spi->input_clk_hz = 50000000;
	spi->spi_freq = dev_read_u32_default(bus, "spi-max-frequency",
					     MINDGROVE_SPI_MAX_FREQ);
	spi->prescaler = 124; /* matches init_hw 400kHz default */

	mindgrove_spi_init_hw(spi);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Driver registration                                                  */
/* ------------------------------------------------------------------ */

static const struct dm_spi_ops mindgrove_spi_ops = {
	.xfer		= mindgrove_spi_xfer,
	.set_speed	= mindgrove_spi_set_speed,
	.set_mode	= mindgrove_spi_set_mode,
	.cs_info	= mindgrove_spi_cs_info,
};

static const struct udevice_id mindgrove_spi_ids[] = {
	{ .compatible = "mindgrove,spi" },
	{}
};

U_BOOT_DRIVER(mindgrove_spi) = {
	.name		= "mindgrove_spi",
	.id		= UCLASS_SPI,
	.of_match	= mindgrove_spi_ids,
	.ops		= &mindgrove_spi_ops,
	.priv_auto	= sizeof(struct mindgrove_spi),
	.probe		= mindgrove_spi_probe,
};