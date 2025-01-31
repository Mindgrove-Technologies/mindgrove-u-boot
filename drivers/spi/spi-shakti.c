// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 IIT Madras.
 *
 * Author(s) : Venkatakrishnan Sutharsan <venkatakrishnan.sutharsan@gmail.com>, Sambhav Jain <sambhav.jv@gmail.com> 
 *
 * Shakti SPI controller driver (master mode only)
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


/* SHAKTI MACROS */

#define SHAKTI_SPI_DRIVER_NAME "shakti_spi"

/* Comment the next line to disable debug printk statements in 
 * the Shakti SPI Driver code.
 */
// #define SHAKTI_DEBUG

#define SHAKTI_SPI_DEFAULT_DEPTH         		 16

//SHAKTI SPI
#define SHAKTI_SPI1_BASE_ADDRESS            0x20100 /*! Standard Serial Peripheral Interface 0 Base address*/
#define SHAKTI_SPI_MAX_COUNT 			          1 /*! Number of Standard SHAKTI_SPI used in the SOC */
#ifdef CONFIG_TARGET_SHAKTI_VCU118
#define CLOCK_FREQUENCY 		          100000000
#else
#define CLOCK_FREQUENCY 		          40000000
#endif

#define SHAKTI_SPI_DRIVER 		     	          1
#define SHAKTI_SPI_DEFAULT_SPEED            2000000

#define SHAKTI_SPI_COMM_CTRL_REG	           0x00 // Standard SPI Communication Control Register (32-bit Read and Write register)
#define SHAKTI_SPI_CLK_CTRL_REG		           0x04 // Standard SPI Clock Control Register (32-bit Read and Write register)
#define SHAKTI_SPI_TX_DATA			           0x08 // Standard SPI Transmitter Data Register (32-bit Write only register)
#define SHAKTI_SPI_RX_DATA		   	           0x0C // Standard SPI Receiver Data Register (32-bit Read only register)
#define SHAKTI_SPI_INTR_EN    		           0x10 // Standard SPI Interrupt Enable Register (16-bit Read and Write register)
#define SHAKTI_SPI_FIFO_STATUS                 0x14 // Standard SPI FIFO Status Register (8-bit Read only register)
#define SHAKTI_SPI_COMM_STATUS                 0x18 // Standard SPI Communication Register (8-bit Read only register)
#define SHAKTI_SPI_QUAD                        0x1C // Standard SPI Input Qualification Control Register (8-bit Read and Write register)

/*! SPIx Communication Control Register */
#define SHAKTI_SPI_MASTER                   (1 << 0)
#define SHAKTI_SPI_ENABLE_BIT               (1 << 1)
#define SHAKTI_SPI_LSB_FIRST                (1 << 2)
#define SHAKTI_SPI_COMM_MODE(x)             (x << 4)
#define SHAKTI_SPI_TOTAL_BITS_TX(x)         (x << 6)
#define SHAKTI_SPI_TOTAL_BITS_RX(x)        (x << 14)
#define SHAKTI_SPI_OUT_EN_SCLK             (1 << 22)
#define SHAKTI_SPI_OUT_EN_NCS              (1 << 23)
#define SHAKTI_SPI_OUT_EN_MISO             (1 << 24)
#define SHAKTI_SPI_OUT_EN_MOSI             (1 << 25)

/*! SPIx Clock Control Register */
#define SHAKTI_SPI_CLK_POLARITY             (1 << 0)
#define SHAKTI_SPI_CLK_PHASE                (1 << 1)
#define SHAKTI_SPI_PRESCALE(x)              (x << 2)
#define SHAKTI_SPI_SS2TX_DELAY(x)          (x << 10)
#define SHAKTI_SPI_TX2SS_DELAY(x)          (x << 18)

/*! SPIx Interrupt Enable Register */
#define SHAKTI_SPI_TX_EMPTY_INTR_EN    (1<<0)
#define SHAKTI_SPI_TX_DUAL_INTR_EN     (1<<1)
#define SHAKTI_SPI_TX_QUAD_INTR_EN     (1<<2)
#define SHAKTI_SPI_TX_OCTAL_INTR_EN    (1<<3)
#define SHAKTI_SPI_TX_HALF_INTR_EN     (1<<4)
#define SHAKTI_SPI_TX_24_INTR_EN       (1<<5)
#define SHAKTI_SPI_TX_28_INTR_EN       (1<<6)
#define SHAKTI_SPI_TX_30_INTR_EN       (1<<7)
#define SHAKTI_SPI_TX_FULL_INTR_EN     (1<<8)
#define SHAKTI_SPI_RX_EMPTY_INTR_EN    (1<<9)
#define SHAKTI_SPI_RX_DUAL_INTR_EN     (1<<10)
#define SHAKTI_SPI_RX_QUAD_INTR_EN     (1<<11)
#define SHAKTI_SPI_RX_OCTAL_INTR_EN    (1<<12)
#define SHAKTI_SPI_RX_HALF_INTR_EN     (1<<13)
#define SHAKTI_SPI_RX_24_INTR_EN       (1<<14)
#define SHAKTI_SPI_RX_28_INTR_EN       (1<<15)
#define SHAKTI_SPI_RX_30_INTR_EN       (1<<16)
#define SHAKTI_SPI_RX_FULL_INTR_EN     (1<<17)
#define SHAKTI_SPI_RX_OVERRUN_INTR_EN  (1<<18)

/*! SPIx Communication Status Register */
#define SHAKTI_SPI_BUSY                     (1 << 0)
#define SHAKTI_SPI_TXE                      (1 << 1)
#define SHAKTI_SPI_RXNE                     (1 << 2)
#define SHAKTI_SPI_TX_FIFO                  (3 << 3)
#define SHAKTI_SPI_RX_FIFO                  (3 << 6)
#define SHAKTI_SPI_OVR                      (1 << 9)

/*! SPIx FIFO Status Register */
#define SHAKTI_SPI_TX_EMPTY          (1<<0)
#define SHAKTI_SPI_TX_DUAL           (1<<1)
#define SHAKTI_SPI_TX_QUAD           (1<<2)
#define SHAKTI_SPI_TX_OCTAL          (1<<3)
#define SHAKTI_SPI_TX_HALF           (1<<4)       
#define SHAKTI_SPI_TX_24             (1<<5)
#define SHAKTI_SPI_TX_28             (1<<6)
#define SHAKTI_SPI_TX_30             (1<<7)
#define SHAKTI_SPI_TX_FULL           (1<<8)
#define SHAKTI_SPI_RX_EMPTY          (1<<9)
#define SHAKTI_SPI_RX_DUAL           (1<<10)
#define SHAKTI_SPI_RX_QUAD           (1<<11)
#define SHAKTI_SPI_RX_OCTAL          (1<<12)
#define SHAKTI_SPI_RX_HALF           (1<<13)
#define SHAKTI_SPI_RX_24             (1<<14)
#define SHAKTI_SPI_RX_28             (1<<15)
#define SHAKTI_SPI_RX_30             (1<<16)
#define SHAKTI_SPI_RX_FULL           (1<<17)

// #define SHAKTI_SPI_MASTER                         1
// #define SHAKTI_SPI_SLAVE                          0

#define SHAKTI_SPI_DISABLE                        0
#define SHAKTI_SPI_ENABLE                         1

#define SHAKTI_SPI_LSB_FIRST                      1
#define SHAKTI_SPI_MSB_FIRST                      0

/* Shakti SPI Communication Mode */
#define SHAKTI_SPI_SIMPLEX_TX                     0
#define SHAKTI_SPI_SIMPLEX_RX                     1
#define SHAKTI_SPI_HALF_DUPLEX                    2
#define SHAKTI_SPI_FULL_DUPLEX                    3

/* Transfer Status */
#define SHAKTI_SPI_SUCCESS                        0
#define SHAKTI_SPI_FAILURE                       -1
#define SHAKTI_SPI_TIMEOUT                       -2

/* Shakti SPI Baudrate */
#define SHAKTI_SPI_MIN_BR                         19 // 50MHz/ 50  = 1MHz 
#define SHAKTI_SPI_MAX_BR                     	 165 // 50MHz/ 166 = 300 KHz

/* Pinmux related Macros */
#define PINMUX_CONFIGURE_REG 			   	 0x41510
#define SHAKTI_SPI_PINMUX_MASK			  0xFFC03FFF  //Clearing the SPI concerned bits						
#define SHAKTI_SPI_PINMUX_SET				0x154000						

/*
 * shakti_spi struct - The private struct used by
 * Shakti SPI Controller to track parameters.
 */
struct shakti_spi{
    void __iomem *base;       // Base address for SPI...
	struct clk clk;           // Clock struct...
	u32		fifo_depth;		  // FIFO length/depth of the SPI Controller in Shakti...
	u32		bits_per_word;	  // The Transaction bits per word count...
	u32		base_freq;        // This is the board freq is set from the DTS file...
	u32 	op_freq;          // This is the operational freq of the SPI...
	u32 	cur_mode;         // The current mode of operation of the SPI Controller...
	u32 	cur_xferlen;      // The Transfer length of the current transaction...
	u32 	cur_bpw;          // The current bits-per-word...
	u8 		prescalar;        // The prescalar value for the clock...
	u8 		lsb_first;		  // lsb first or msb first flag...
	unsigned int tx_len;	  // number of data to be transmitted in bytes 
	unsigned int rx_len;	  // number of data to be received in bytes 
	const void *tx_buf;	      // data to be transmitted buffer, or NULL 
	void *rx_buf;		      // data to be received buffer, or NULL
	// u32		cs_inactive;  // Level of the CS pins when inactive
	bool cs_high;             // Active status of chipselect in Shakti...
	// u32		num_cs;
	// u8		fmt_proto;
};

/* Reading Shakti Registers */
u8 shakti_spi_read8(struct shakti_spi *spi, u32 offset)
{
	return readb(spi->base + offset);
}

u16 shakti_spi_read16(struct shakti_spi *spi, u32 offset)
{
	return readw(spi->base + offset);
}

u32 shakti_spi_read32(struct shakti_spi *spi, u32 offset)
{
	return readl(spi->base + offset);
}

/* Setting Bits for Shakti Registers */
static inline void shakti_spi_set8(struct shakti_spi *spi,
				      u32 offset, u8 bits)
{
	writeb(readb(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void shakti_spi_set16(struct shakti_spi *spi,
				      u32 offset, u16 bits)
{
	writew(readw(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void shakti_spi_set32(struct shakti_spi *spi,
				      u32 offset, u32 bits)
{
	writel(readl(spi->base + offset) | bits,
		       spi->base + offset);
}

/* Clearing Bits for Shakti Registers */
static inline void shakti_spi_clr8(struct shakti_spi *spi,
				      u32 offset, u8 bits)
{
	writeb(readb(spi->base + offset) & ~bits,
		       spi->base + offset);
}

static inline void shakti_spi_clr16(struct shakti_spi *spi,
				      u32 offset, u16 bits)
{
	writew(readw(spi->base + offset) & ~bits,
		       spi->base + offset);
}

static inline void shakti_spi_clr32(struct shakti_spi *spi,
				      u32 offset, u32 bits)
{
	writel(readl(spi->base + offset) & ~bits,
		       spi->base + offset);
}

/*
 * shakti_spi_cs_high - Function to make the chip select active high or active low.
 */
// void shakti_spi_cs_high(u8 cs_high){
	// if(cs_high)
	// 	// set the Chip select as active high...
	// else
	// 	// set the Chip select as active low...
// }

/*
 * shakti_spi_set_mode - Function used by the core to change the mode of transfer
 * for Shakti SPI Controller.
 */
static int shakti_spi_set_mode(struct udevice *bus, uint mode)
{
    struct shakti_spi *spi = dev_get_priv(bus);
	u32 cr;

	dev_dbg(bus, "mode=%d\n", mode);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "mode=%d\n", mode);
	#endif

	u8 shakti_mode = 0;
	if (mode & SPI_CPOL)
		shakti_mode |= SHAKTI_SPI_CLK_PHASE;

	if (mode & SPI_CPHA)
		shakti_mode |= SHAKTI_SPI_CLK_POLARITY;
	
	if (mode & SPI_LSB_FIRST)
		spi->lsb_first = SHAKTI_SPI_LSB_FIRST;
	else
		spi->lsb_first = SHAKTI_SPI_MSB_FIRST;
	
	if (mode & SPI_CS_HIGH){
		spi->cs_high = true;
		// shakti_spi_cs_high(SHAKTI_SPI_ENABLE);
	}
	else{
		spi->cs_high = false;
		// shakti_spi_cs_high(SHAKTI_SPI_DISABLE);
	}
	shakti_lsb_first(spi);

	uint32_t temp = readl(spi->base + SHAKTI_SPI_CLK_CTRL_REG) & ~(SHAKTI_SPI_CLK_POLARITY | SHAKTI_SPI_CLK_PHASE);
    writel( (temp | shakti_mode), spi->base + SHAKTI_SPI_CLK_CTRL_REG);

	return 0;
}

/*
 * shakti_spi_set_speed - Function used by the core to change the speed of the 
 * Shakti SPI Controller.
 */
static int shakti_spi_set_speed(struct udevice *bus, uint speed)
{
    struct shakti_spi *spi = dev_get_priv(bus);

	if (spi->op_freq == speed)
		return 0;
	
	u8 clock_prescaler = (u8)((spi->base_freq / speed) - 1);

	if (clock_prescaler < SHAKTI_SPI_MIN_BR || clock_prescaler > SHAKTI_SPI_MAX_BR){
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "The clock prescalar is not in range and value is : %d\n",clock_prescaler);
		#endif
		return -EINVAL;
	}

	uint32_t temp = readl(spi->base + SHAKTI_SPI_CLK_CTRL_REG) & ~SHAKTI_SPI_PRESCALE(255);
	writel((temp | SHAKTI_SPI_PRESCALE(clock_prescaler)), spi->base + SHAKTI_SPI_CLK_CTRL_REG);

	dev_dbg(bus, "Speed changed from %d to %d\n",spi->op_freq, speed);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "Speed changed from %d to %d\n",spi->op_freq, speed);
	#endif
	spi->op_freq = speed;
	return 0;
}

/*
 * shakti_spi_tx - The main function which writes the data from data register
 * of the calling function and writes into TX FIFO.
 */
static void shakti_spi_tx(struct shakti_spi *spi,
				      u8 *tx_ptr)
{
	writeb(*tx_ptr, spi->base + SHAKTI_SPI_TX_DATA);	//Writing in bytes
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The Current data sent to buffer is : %x\n",*tx_ptr);
	#endif
}

/*
 * shakti_spi_rx - The main function which gets the data from RX FIFO and
 * writes into data register of the calling function.
 */
static void shakti_spi_rx(struct shakti_spi *spi,
				      u8 *rx_ptr)
{
	*rx_ptr = readb(spi->base + SHAKTI_SPI_RX_DATA);	//Reading as bytes
	//FIXME - The FIFO status registers value dosnt change when the below printk is removed.
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "%x",*rx_ptr);
	#endif
}

/*
 * shakti_spi_dump() - 
 */
void shakti_spi_dump(struct shakti_spi *spi, u32 words){
	u8 temp=0;
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "inside dump\n");
	#endif
	while((shakti_spi_read8(spi,SHAKTI_SPI_FIFO_STATUS) & (0x10)) == 0){
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "looping till words\n");
		#endif
		shakti_spi_rx(spi, &temp);
	}
}

/*
 *
 */
void shakti_spi_busy(struct shakti_spi* spi){
	u8 comm_status = shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS);
	do{
		comm_status = shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS);
	}while(comm_status & SHAKTI_SPI_BUSY);
}

/**
 * shakti_show_registers - Show the main register values at the calling point...
 */
static void shakti_show_registers(struct shakti_spi *spi)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_COMM_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The Clock Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_CLK_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The FIFO Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_FIFO_STATUS));
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_COMM_STATUS));
	#endif
}

static char* show_mode(u32 mode){
	switch(mode){
		case SHAKTI_SPI_FULL_DUPLEX: return "Full Duplex";
		case SHAKTI_SPI_SIMPLEX_RX: return "Simplex RX";
		case SHAKTI_SPI_SIMPLEX_TX: return "Simplex TX";
		case SHAKTI_SPI_HALF_DUPLEX: return "Half Duplex";
	}
	return "?";
}
/*
 * shakti_spi_xfer - The transfer function which is called for SPI transfer 
 * through the Shakti SPI Controller.
 */
static int shakti_spi_xfer(struct udevice *dev, unsigned int bitlen,
			    const void *dout, void *din, unsigned long flags)
{
    struct udevice *bus = dev->parent;
	struct shakti_spi *spi = dev_get_priv(bus);
    struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	u32 sr, ifcr = 0, xferlen, mode;
	// u32 mode;
	int xfer_status = 0;

	xferlen = bitlen / 8; // xferlen is the length in bytes...

	// r_words = xferlen;
	spi->tx_buf = dout;
	spi->rx_buf = din;
	spi->tx_len = spi->tx_buf ? bitlen / 8 : 0;
	spi->rx_len = spi->rx_buf ? bitlen / 8 : 0;
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "TX %x RX %x \n", spi->tx_buf, spi->rx_buf);
	#endif
	u8 dummy_tx = 0xFF;

	// Checking in the transfer length in the private struct...
	int r_words = spi->cur_xferlen = xferlen;
	unsigned int i;

	mode = SHAKTI_SPI_FULL_DUPLEX;
	if (!spi->tx_buf)
		mode = SHAKTI_SPI_SIMPLEX_RX;
	else if (!spi->rx_buf)
		mode = SHAKTI_SPI_SIMPLEX_TX;

	// Condition to change the mode if there are any missing/NULL data pointers...
	if (spi->cur_mode != mode)
	{
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_COMM_MODE(SHAKTI_SPI_FULL_DUPLEX));
		spi->cur_mode = mode;
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_COMM_MODE(spi->cur_mode));
	}
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The mode is %s\n",show_mode(spi->cur_mode));
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_COMM_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The Clock Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_CLK_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The FIFO Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_FIFO_STATUS));
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_COMM_STATUS));
	printk(KERN_ALERT "Entering the loop.\n");
	#endif
	// Need to implement overrun flag check for -EIO xfer_status...
	while(r_words)
	{
		unsigned int n_words = min(r_words, spi->fifo_depth);
		#ifdef SHAKTI_DEBUG
		printk("[TO]r_words %d n_words %d",r_words, n_words);
		#endif
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_RX(255));
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_TX(255));
		//Assuming we have no Simplex Commnuication (tx or rx also )
		if(spi->cur_mode == SHAKTI_SPI_SIMPLEX_RX || spi->cur_mode == SHAKTI_SPI_FULL_DUPLEX)
			shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_RX((n_words) * 8));
		// if(spi->cur_mode == SHAKTI_SPI_SIMPLEX_TX || spi->cur_mode == SHAKTI_SPI_FULL_DUPLEX)
			shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_TX((n_words) * 8));

		//16 by default max upto 31 and the rest r_words = r_words - n_words 
		if(spi->tx_buf)
		{
			for(i=0; i< n_words;i++)
			{
				//Adding Tx data to the fifo 
				#ifdef SHAKTI_DEBUG
				printk(KERN_ALERT "transmitting %d\n",i);
				#endif
				shakti_spi_tx(spi, spi->tx_buf++);
			}
		}
		else{
			for(i=0; i< n_words;i++)
			{
				//Adding Tx data to the fifo 
				#ifdef SHAKTI_DEBUG
				printk(KERN_ALERT "transmitting %d\n",i);
				#endif
				shakti_spi_tx(spi, &dummy_tx);
			}
		}
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "Before Enabling \n");
		#endif
		shakti_show_registers(spi);
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT);
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "After enabling\n");
		#endif
		shakti_show_registers(spi);
		// Wait for Tx to complete
		while(!shakti_spi_read8(spi, SHAKTI_SPI_FIFO_STATUS) & SHAKTI_SPI_TX_EMPTY);
		// mdelay(10);
		shakti_spi_busy(spi);
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "[SHAKTI-INFO] Transmit Completed...\n");
		#endif
		// If receive option is been set condition is true and data received is transfered to din.
		if(spi->rx_buf)
		{
			//Read till the RXE is set and store in the rx_ptrs 
			//Check if the len of r_words is not greater than num of
			// recieved words 
			#ifdef SHAKTI_DEBUG
			printk(KERN_ALERT "Inside RX condition\n");
			#endif
			u8 rx_counter = n_words;
			while( (shakti_spi_read8(spi,SHAKTI_SPI_FIFO_STATUS) & (0x10)) == 0 && (rx_counter--))
			{
				#ifdef SHAKTI_DEBUG
				printk(KERN_ALERT "Inside loop RX");
				#endif
				shakti_spi_rx(spi, spi->rx_buf++);
			}
		}
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "n_words completed\n");
		#endif
		// mdelay(10);
		r_words -= n_words;
	}
	// shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT);
	return xfer_status;
}

/*
 * shakti_spi_configure_pins - Function to configure the MISO, MOSI, CS_PIN and CLK_PIN in the Hardware...
 */
void shakti_spi_configure_pins(struct shakti_spi *spi, uint8_t  mosi_pin_cntrl, uint8_t miso_pin_cntrl, uint8_t cs_pin_cntrl, uint8_t clk_pin_cntrl)
{
    uint32_t temp = 0;
    
    if(mosi_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_MOSI;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_MOSI;  

    if(miso_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_MISO;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_MISO;  

    if(cs_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_NCS;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_NCS;  
		
    if(clk_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_SCLK;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_SCLK;

	// Write to comm ctrl register
	shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, temp);
}

/*
 * shakti_lsb_first - Function to set the MSB as first bit to be transmitted.
 */
void shakti_lsb_first(struct shakti_spi *spi){
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Configuring as LSB First\n");
	#endif
	if(spi->lsb_first)
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_LSB_FIRST);
	else
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_LSB_FIRST);
}

/*
 * shakti_spi_probe - Function which is called during probing of the driver.
 */
static int shakti_spi_probe(struct udevice *bus)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Shakti SPI Driver Probing...\n");
	#endif
	struct shakti_spi *spi = dev_get_priv(bus);
	struct clk clkdev;
	unsigned long clk_rate;
	int ret;
	u32 mode;

	spi->base = dev_remap_addr(bus);
	if (!spi->base)
		return -EINVAL;

	void __iomem *pinmux_register = 0x41510;//ioremap_nocache(PINMUX_CONFIGURE_REG, 4);

	// Writing the pinmux register value for SPI...
	u32 pinmux_reg_read = (readl(pinmux_register) | SHAKTI_SPI_PINMUX_MASK );
	writel((SHAKTI_SPI_PINMUX_SET | pinmux_reg_read) ,pinmux_register);

	spi->fifo_depth = dev_read_u32_default(bus, "shakti,fifo-size", 0);	
	if(!spi->fifo_depth)
		spi->fifo_depth = SHAKTI_SPI_DEFAULT_DEPTH;

	spi->op_freq = dev_read_u32_default(bus, "shakti,spi-frequency", 0);
	if(!spi->op_freq)
		spi->op_freq = SHAKTI_SPI_DEFAULT_SPEED;
	
	spi->lsb_first = dev_read_u32_default(bus, "shakti,lsb-first", 0);

	ret = clk_get_by_index(bus, 0, &spi->clk);
	if (ret < 0)
		return ret;
	
	ret = clk_enable(&spi->clk);
	if (ret < 0)
		return ret;

	clk_rate = clk_get_rate(&spi->clk);
	if (!clk_rate) {
		ret = -EINVAL;
		goto clk_err;
	}
	spi->base_freq = clk_rate;
	spi->prescalar = (spi->base_freq / spi->op_freq)-1;
	spi->cur_mode = SHAKTI_SPI_FULL_DUPLEX;
	spi->cur_xferlen = 0;
	spi->cur_bpw = SPI_DEFAULT_WORDLEN;
	if (spi->op_freq > spi->base_freq)
		return -EINVAL;

	// To set the SPI Controller as MASTER...
	shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_MASTER);

	// Configuring pins for MASTER MODE...
	shakti_spi_configure_pins(spi, SHAKTI_SPI_ENABLE,SHAKTI_SPI_DISABLE,SHAKTI_SPI_ENABLE,SHAKTI_SPI_ENABLE);

	// Setting the lsb first or msb first mode...
	shakti_lsb_first(spi);

	// To set FULL DUPLEX TRANSFER MODE...
	spi->cur_mode = SHAKTI_SPI_FULL_DUPLEX;
	shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_COMM_MODE(spi->cur_mode));

	// To set the clock speed...
	u8 clock_prescaler = (u8)((spi->base_freq / spi->op_freq) - 1);
	uint32_t temp = readl(spi->base + SHAKTI_SPI_CLK_CTRL_REG) & ~SHAKTI_SPI_PRESCALE(255);
	writel((temp | SHAKTI_SPI_PRESCALE(clock_prescaler)), spi->base + SHAKTI_SPI_CLK_CTRL_REG);

	shakti_show_registers(spi);

	printk(KERN_ALERT "[SHAKTI-INFO] Shakti SPI Driver Initialized...\n");
	return 0;

clk_err:
	clk_disable(&spi->clk);
	clk_free(&spi->clk);

	return ret;
}

/*
 * shakti_spi_remove - Function which is called when the driver is removed.
 * Useful only when used as a module...
 */
static int shakti_spi_remove(struct udevice *dev)
{
	struct shakti_spi *spi = dev_get_priv(dev);
	int ret;

	// stm32_spi_stopxfer(dev);
	// stm32_spi_disable(priv);

	// ret = reset_assert(&priv->rst_ctl);
	// if (ret < 0)
	// 	return ret;

	// reset_free(&priv->rst_ctl);

	ret = clk_disable(&spi->clk);
	if (ret < 0)
		return ret;

	clk_free(&spi->clk);

	return ret;
};

static const struct dm_spi_ops shakti_spi_ops = {
	// .claim_bus	= shakti_spi_claim_bus,
	// .release_bus	= shakti_spi_release_bus,
	.set_mode	= shakti_spi_set_mode,
	.set_speed	= shakti_spi_set_speed,
	.xfer		= shakti_spi_xfer,
};


static const struct udevice_id shakti_spi_ids[] = {
	{ .compatible = "shakti,spi1", },
	{ }
};

U_BOOT_DRIVER(shakti_spi) = {
	.name			= "shakti_spi",
	.id			= UCLASS_SPI,
	.of_match		= shakti_spi_ids,
	.ops			= &shakti_spi_ops,
	.priv_auto	= sizeof(struct shakti_spi),
	.probe			= shakti_spi_probe,
	.remove			= shakti_spi_remove,
};
