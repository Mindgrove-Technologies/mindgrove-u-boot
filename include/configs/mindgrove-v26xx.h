/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __MINDGROVE_V26XX_H
#define __MINDGROVE_V26XX_H

#include <linux/sizes.h>

#define CONFIG_SYS_SDRAM_BASE		0x80000000

/* Environment */
#define CONFIG_ENV_OFFSET		0x100000

/* Memory */
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_4M)

/* Boot */
#define CONFIG_BOOTDELAY		5

#endif /* __MINDGROVE_V26XX_H */
