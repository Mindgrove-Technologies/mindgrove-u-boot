// SPDX-License-Identifier: GPL-2.0+
/*
 * Mindgrove SPI controller driver for U-Boot
 * Based on Mindgrove Technologies SPI controller IP
 *
 * Copyright (C) 2025 Mindgrove Technologies Private Limited
 * Author: Harini P <harininisha1219@gmail.com>
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <spi.h>
#include <spi-mem.h>
#include <wait_bit.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/log2.h>
#include <clk.h>

#define MINDGROVE_SPI_MAX_CS 4
#define MINDGROVE_SPI_DEFAULT_DEPTH 32
#define MINDGROVE_SPI_DEFAULT_BITS 8
#define MINDGROVE_SPI_TIMEOUT_US 1000000
#define MINDGROVE_SPI_MAX_FREQ 35000000

#define NCS_ENABLE 1
#define NCS_DISABLE 0

/* Register offsets */
#define MINDGROVE_SPI_REG_CTRL 0x00		   /* Control register */
#define MINDGROVE_SPI_REG_CLK_CTRL 0x04	   /* Clock control register */
#define MINDGROVE_SPI_REG_TX 0x08		   /* TX data register */
#define MINDGROVE_SPI_REG_RX 0x0C		   /* RX data register */
#define MINDGROVE_SPI_REG_INTR_EN 0x10	   /* Interrupt enable */
#define MINDGROVE_SPI_REG_FIFO_STATUS 0x14 /* FIFO status */
#define MINDGROVE_SPI_REG_COMM_STATUS 0x18 /* Communication status */
#define MINDGROVE_SPI_REG_NCS_CTRL 0x1C	   /* NCS control */

/* CTRL register bit definitions */
#define MINDGROVE_SPI_CTRL_SLAVE_MODE(x) ((x) << 0)
#define MINDGROVE_SPI_CTRL_EN(x) ((x) << 1)
#define MINDGROVE_SPI_CTRL_LSBFIRST(x) ((x) << 2)
#define MINDGROVE_SPI_CTRL_RX_FLUSH(x) ((x) << 3)
#define MINDGROVE_SPI_CTRL_COMM_MODE_MASK GENMASK(5, 4)
#define MINDGROVE_SPI_CTRL_COMM_MODE(x) ((x) << 4)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK GENMASK(13, 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(x) ((x) << 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK GENMASK(21, 14)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(x) ((x) << 14)
#define MINDGROVE_SPI_CTRL_SCLK_OUTEN ((uint32_t)1UL << 22)
#define MINDGROVE_SPI_CTRL_NCS_OUTEN ((uint32_t)1UL << 23)
#define MINDGROVE_SPI_CTRL_MISO_OUTEN ((uint32_t)1UL << 24)
#define MINDGROVE_SPI_CTRL_MOSI_OUTEN ((uint32_t)1UL << 25)

// #define MINDGROVE_SPI_CTRL_SLAVE_MODE	         BIT(0)
// #define MINDGROVE_SPI_CTRL_EN			         BIT(1)
// #define MINDGROVE_SPI_CTRL_LSBFIRST		         BIT(2)
// #define  MINDGROVE_SPI_CTRL_RX_FLUSH(x)          ((x) << 3)
// // #define MINDGROVE_SPI_CTRL_RX_FLUSH		         BIT(3)
// #define MINDGROVE_SPI_CTRL_COMM_MODE_SHIFT	    4
// #define MINDGROVE_SPI_CTRL_COMM_MODE_MASK	    GENMASK(5, 4)
// #define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_SHIFT	6
// #define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK	GENMASK(13, 6)
// #define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_SHIFT	14
// #define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK	GENMASK(21, 14)
// #define MINDGROVE_SPI_CTRL_SCLK_OUTEN		    BIT(22)
// #define MINDGROVE_SPI_CTRL_NCS_OUTEN		    BIT(23)
// #define MINDGROVE_SPI_CTRL_MISO_OUTEN		    BIT(24)
// #define MINDGROVE_SPI_CTRL_MOSI_OUTEN		    BIT(25)

/* CLK_CTRL register bit definitions */
#define MINDGROVE_SPI_CLK_CTRL_POLARITY BIT(0)
#define MINDGROVE_SPI_CLK_CTRL_PHASE BIT(1)
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT 2
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK GENMASK(15, 2)
#define MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT 16
#define MINDGROVE_SPI_CLK_CTRL_SETUP_MASK GENMASK(23, 16)
#define MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT 24
#define MINDGROVE_SPI_CLK_CTRL_HOLD_MASK GENMASK(31, 24)

/* FIFO_STATUS register bit definitions */
#define MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY BIT(0)
#define MINDGROVE_SPI_FIFO_STATUS_TX_FULL BIT(8)
#define MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY BIT(9)
#define MINDGROVE_SPI_FIFO_STATUS_RX_FULL BIT(17)

/* COMM_STATUS register bit definitions */
#define MINDGROVE_SPI_COMM_STATUS_BUSY BIT(0)
#define MINDGROVE_SPI_COMM_STATUS_TX_DEPTH_SHIFT 3
#define MINDGROVE_SPI_COMM_STATUS_TX_DEPTH_MASK GENMASK(5, 3)
#define MINDGROVE_SPI_COMM_STATUS_RX_DEPTH_SHIFT 6
#define MINDGROVE_SPI_COMM_STATUS_RX_DEPTH_MASK GENMASK(8, 6)

/* NCS_CTRL register bit definitions */
#define MINDGROVE_SPI_NCS_CTRL_SELECT(x) ((x) << 0)
#define MINDGROVE_SPI_NCS_CTRL_SW(x) ((x) << 1)

/* Communication modes */
#define MINDGROVE_SPI_COMM_MODE_TX 0
#define MINDGROVE_SPI_COMM_MODE_RX 1
#define MINDGROVE_SPI_COMM_MODE_HALF_DUPLEX 2
#define MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX 3

struct mindgrove_spi
{
	void __iomem *base; // Base address for SPI...
	u32 fifo_depth;		// FIFO length/depth of the SPI Controller.
	u32 bits_per_word;	// The Transaction bits per word count...
	u32 input_clk_hz;	// This is the board freq is set from the DTS file...
	u32 spi_freq;		// This is the operational freq of the SPI...
	u32 comm_mode;		// The communication mode of the SPI Controller...
	u16 prescaler;
	u8 num_cs;
};

// ncs_status : 1-> enable , 0-> disable
static int mindgrove_spi_cs_control(struct mindgrove_spi *spi,
                                   struct dm_spi_slave_plat *slave_plat,
                                   bool enable)
{
    u8 ncs_ctrl = readb(spi->base + MINDGROVE_SPI_REG_NCS_CTRL);

    /* Clear SW bit */
    ncs_ctrl &= ~MINDGROVE_SPI_NCS_CTRL_SW(1);

    if (enable) {
        /* Assert CS */
        if (!(slave_plat->mode & SPI_CS_HIGH))
            ncs_ctrl |= MINDGROVE_SPI_NCS_CTRL_SW(1);
    } else {
        /* Deassert CS */
        if (slave_plat->mode & SPI_CS_HIGH)
            ncs_ctrl |= MINDGROVE_SPI_NCS_CTRL_SW(1);
    }

    writeb(ncs_ctrl, spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
    return 0;
}


static int mindgrove_spi_wait_complete(struct mindgrove_spi *spi)
{
	u32 timeout = MINDGROVE_SPI_TIMEOUT_US;
	u32 fifo_status, comm_status;

	while (timeout--)
	{
		fifo_status = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
		comm_status = readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS);

		if ((fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY) &&
			!(comm_status & MINDGROVE_SPI_COMM_STATUS_BUSY))
			return 0;

		udelay(1);
	}

	return -ETIMEDOUT;
}

static void mindgrove_spi_prep(struct mindgrove_spi *spi)
{
	u32 ctrl;

	/* Set communication mode */
	ctrl = (MINDGROVE_SPI_CTRL_COMM_MODE(MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX));
	/* Set transfer size in bits */
	ctrl |= (MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(spi->bits_per_word)) |
			(MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(spi->bits_per_word)) |
			MINDGROVE_SPI_CTRL_SCLK_OUTEN |
			MINDGROVE_SPI_CTRL_NCS_OUTEN |
			MINDGROVE_SPI_CTRL_MOSI_OUTEN;

	/* Configure control register */
	ctrl |= MINDGROVE_SPI_CTRL_EN(1);

	writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
}

static int mindgrove_spi_xfer(struct udevice *dev, unsigned int bitlen,
							  const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct mindgrove_spi *spi = dev_get_priv(bus);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	u32 remaining_len;
	int ret;
	u32 fifo_status;
	const u8 *tx_ptr = dout;
	u8 rx_data[remaining_len];
	u8 *rx_ptr = din ? din : rx_data;
	
	if (bitlen % 8)
	{
		dev_err(dev, "Non-byte aligned transfer not supported\n");
		return -EINVAL;
	}
	
	remaining_len = bitlen / 8;
	

	mindgrove_spi_prep(spi);

	// printk(KERN_ALERT "[MG-INFO] The mode is %x\n\r",spi->comm_mode);
	// printk(KERN_ALERT "[MG-INFO] The length in bits: %d, bytes: %d\n\r", bitlen, remaining_len);
	// printk(KERN_ALERT "[MG-INFO] The Communication Control Register has the following value : %x\n\r",readl(spi->base + MINDGROVE_SPI_REG_CTRL));
	// printk(KERN_ALERT "[MG-INFO] The Clock Control Register has the following value : %x\n\r",readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL));
	// printk(KERN_ALERT "[MG-INFO] The FIFO Status Register has the following value : %x\n\r",readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS));
	// printk(KERN_ALERT "[MG-INFO] The Communication Status Register has the following value : %x\n\r",readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS));
	// printk(KERN_ALERT "Entering the loop.\n\r");

	if (flags & SPI_XFER_BEGIN)
	{
		// ret = mindgrove_spi_cs_control(spi, slave_plat, NCS_ENABLE);
		if (ret)
		{
			return ret;
		}
	}

	unsigned int nword;
	for (nword = 0; nword < remaining_len; nword++)
	{
		u8 tx_data = (tx_ptr != NULL) ? tx_ptr[nword] : 0xFF;
		do
		{
			fifo_status = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
		} while (fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_FULL);

		// printk(KERN_ALERT "Transmitting the data: 0x%x\n\r", tx_data);
		writeb(tx_data, spi->base + MINDGROVE_SPI_REG_TX);


		do
		{
			fifo_status = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
		} while (fifo_status & MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY);

		rx_ptr[nword] = readb(spi->base + MINDGROVE_SPI_REG_RX);
		// printk(KERN_ALERT "Received the data: 0x%x\n\r", rx_ptr[nword]);
	}

	/* Wait for completion */
	ret = mindgrove_spi_wait_complete(spi);
	if (ret)
	{
		dev_err(dev, "Transfer timeout\n\r");
		return ret;
	}
	if (flags & SPI_XFER_END)
	{
		// mindgrove_spi_cs_control(spi, slave_plat, NCS_DISABLE);
		/* Disable SPI */
		// writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);
	}

	return 0;
}

static int mindgrove_spi_exec_op(struct spi_slave *slave,
								 const struct spi_mem_op *op)
{
	struct udevice *dev = slave->dev;
	unsigned long flags = SPI_XFER_BEGIN;
	u8 opcode = op->cmd.opcode;
	unsigned int pos = 0;
	const void *tx_buf = NULL;
	void *rx_buf = NULL;
	int op_len, i;
	int ret;

	if (!op->addr.nbytes && !op->dummy.nbytes && !op->data.nbytes)
		flags |= SPI_XFER_END;

	/* send the opcode */
	ret = mindgrove_spi_xfer(dev, 8, (void *)&opcode, NULL, flags);
	if (ret < 0)
	{
		dev_err(dev, "failed to xfer opcode\n");
		return ret;
	}
	op_len = op->addr.nbytes + op->dummy.nbytes;
	if (op_len > 0)
	{
		u8 op_buf[op_len];

		/* send the addr + dummy */
		if (op->addr.nbytes)
		{
			/* fill address */
			for (i = 0; i < op->addr.nbytes; i++)
				op_buf[pos + i] = op->addr.val >>
								  (8 * (op->addr.nbytes - i - 1));

			pos += op->addr.nbytes;

			/* fill dummy */
			if (op->dummy.nbytes)
				memset(op_buf + pos, 0xff, op->dummy.nbytes);

			/* make sure to set end flag, if no data bytes */
			if (!op->data.nbytes)
				flags |= SPI_XFER_END;

			ret = mindgrove_spi_xfer(dev, op_len * 8, op_buf, NULL, flags);
			if (ret < 0)
			{
				dev_err(dev, "failed to xfer addr + dummy\n");
				return ret;
			} 
		}
	}
	/* send/received the data */
	if (op->data.nbytes)
	{
		if (op->data.dir == SPI_MEM_DATA_IN)
			rx_buf = op->data.buf.in;
		else
			tx_buf = op->data.buf.out;

		ret = mindgrove_spi_xfer(dev, op->data.nbytes * 8,
								 tx_buf, rx_buf, SPI_XFER_END);
		if (ret)
		{
			dev_err(dev, "failed to xfer data\n");
			return ret;
		}
	}
	return 0;
}

static int mindgrove_spi_set_speed(struct udevice *bus, uint speed)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);
	u32 prescaler, clk_ctrl;
	if (spi->spi_freq != speed)
		spi->spi_freq = speed;
	/* Calculate prescaler */
	prescaler = (spi->input_clk_hz / spi->spi_freq) - 1U;

	if (((spi->input_clk_hz / (prescaler + 1)) > MINDGROVE_SPI_MAX_FREQ) ||
		((spi->input_clk_hz / (prescaler + 1)) < (spi->input_clk_hz / 0x3FFF)))
	{
		dev_err(bus, "Invalid SPI frequency : %d Hz \r\n", speed);
		return -EINVAL;
	}
	spi->prescaler = prescaler;
	clk_ctrl = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK;
	clk_ctrl |= (prescaler << MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT);
	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	return 0;
}

static int mindgrove_spi_set_mode(struct udevice *bus, uint mode)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);
	u32 clk_ctrl;
	u8 ncs_ctrl = 0;

	/* Switch clock mode bits */
	clk_ctrl = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~(MINDGROVE_SPI_CLK_CTRL_POLARITY | MINDGROVE_SPI_CLK_CTRL_PHASE);

	if (mode & SPI_CPHA)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_PHASE;
	if (mode & SPI_CPOL)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_POLARITY;

	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	/* Set LSB first if required */
	if (mode & SPI_LSB_FIRST)
	{
		u32 ctrl = readl(spi->base + MINDGROVE_SPI_REG_CTRL) |
				   MINDGROVE_SPI_CTRL_LSBFIRST(1);
		writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
	}
	/* Configure NCS control for software mode */
	ncs_ctrl = readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL) |
			   MINDGROVE_SPI_NCS_CTRL_SELECT(1);

	/* Update the chip select polarity */
	if (mode & SPI_CS_HIGH)
		ncs_ctrl |= MINDGROVE_SPI_NCS_CTRL_SW(1);

	writeb(ncs_ctrl, spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
	return 0;
}

static int mindgrove_spi_cs_info(struct udevice *bus, uint cs,
								 struct spi_cs_info *info)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);
	if (cs >= spi->num_cs)
		return -EINVAL;
	return 0;
}

static void mindgrove_spi_init_hw(struct mindgrove_spi *spi)
{
	/* Set number of chip selects */
	spi->num_cs = MINDGROVE_SPI_MAX_CS;
	/* Disable SPI initially */
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);
	/* Flush RX FIFO */
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(1), spi->base + MINDGROVE_SPI_REG_CTRL);
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(0), spi->base + MINDGROVE_SPI_REG_CTRL);

	/* Set default setup and hold times */
	writel((1 << MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT) |
			   (1 << MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT),
		   spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	/* Configure NCS for software control */
	writeb(MINDGROVE_SPI_NCS_CTRL_SELECT(1), spi->base + MINDGROVE_SPI_REG_NCS_CTRL);
}

static int mindgrove_spi_probe(struct udevice *bus)
{
	struct mindgrove_spi *spi = dev_get_priv(bus);

	spi->base = (void *)(ulong)dev_remap_addr(bus);
	if (!spi->base)
		return -ENODEV;

	spi->fifo_depth = dev_read_u32_default(bus, "mindgrove,fifo-depth",
										   MINDGROVE_SPI_DEFAULT_DEPTH);

	spi->bits_per_word = dev_read_u32_default(bus, "mindgrove,max-bits-per-word",
											  MINDGROVE_SPI_DEFAULT_BITS);

	spi->input_clk_hz = 50000000; /* Default 50MHz */

	spi->spi_freq = dev_read_u32_default(bus, "spi-max-frequency", 35000000);

	/* init the mindgrove spi hw */
	mindgrove_spi_init_hw(spi);

	return 0;
}

static const struct spi_controller_mem_ops mindgrove_spi_mem_ops = {
	.exec_op = mindgrove_spi_exec_op,
};

static const struct dm_spi_ops mindgrove_spi_ops = {
	.xfer = mindgrove_spi_xfer,
	.set_speed = mindgrove_spi_set_speed,
	.set_mode = mindgrove_spi_set_mode,
	.cs_info = mindgrove_spi_cs_info,
	// .mem_ops	= &mindgrove_ spi_mem_ops,
};

static const struct udevice_id mindgrove_spi_ids[] = {
	{.compatible = "mindgrove,spi"},
	{}};

U_BOOT_DRIVER(mindgrove_spi) = {
	.name = "mindgrove_spi",
	.id = UCLASS_SPI,
	.of_match = mindgrove_spi_ids,
	.ops = &mindgrove_spi_ops,
	.priv_auto = sizeof(struct mindgrove_spi),
	.probe = mindgrove_spi_probe,
};