CONFIG_EXTRA_ENV_SETTINGS="fdt_high=0xffffffffffffffff0initrd_high=0xffffffffffffffff0kernel_addr_r=0x840000000fdt_addr_r=0x880000000scriptaddr=0x881000000pxefile_addr_r=0x882000000ramdisk_addr_r=0x883000000type_guid_gpt_loader1=" TYPE_GUID_LOADER1 "0type_guid_gpt_loader2=" TYPE_GUID_LOADER2 "0type_guid_gpt_system=" TYPE_GUID_SYSTEM "0partitions=" PARTS_DEFAULT "0"
CONFIG_STANDALONE_LOAD_ADDR=0x80200000
CONFIG_SYS_SDRAM_BASE=0x80000000
CONFIG_SYS_INIT_SP_ADDR="(CONFIG_SYS_SDRAM_BASE + SZ_2M)"
CONFIG_PREBOOT="setenv fdt_addr ${fdtcontroladdr};fdt addr ${fdtcontroladdr};"
