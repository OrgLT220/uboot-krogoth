CONFIG_CMD_FAT=y
CONFIG_VIDEO_BMP_LOGO=y
CONFIG_BOOTCOMMAND="mmc dev ${mmcdev};mmc dev ${mmcdev}; if mmc rescan; then if run loadbootscript; then run bootscript; else if run loadimage; then run mmcboot; else run netboot; fi; fi; else run netboot; fi"
CONFIG_BOARD_EARLY_INIT_F=y
CONFIG_USB_GADGET_DOWNLOAD=y
CONFIG_VGA_AS_SINGLE_DEVICE=y
CONFIG_MXC_USB_FLAGS=0
CONFIG_IMX_CONFIG="board/freescale/mx7dsabresd/imximage.cfg"
CONFIG_BOOTM_VXWORKS=y
CONFIG_SYS_LONGHELP=y
CONFIG_IS_MODULE(option)="config_enabled(CONFIG_VAL(option ##_MODULE))"
CONFIG_VIDEO_MXS=y
CONFIG_SYS_LOAD_ADDR=$(CONFIG_LOADADDR)
CONFIG_DISPLAY_BOARDINFO=y
CONFIG_CMD_CACHE=y
CONFIG_POWER_PFUZE3000_I2C_ADDR=0x08
CONFIG_STACKSIZE="SZ_128K"
CONFIG_BOOTDELAY=3
CONFIG_SYS_HELP_CMD_WIDTH=8
CONFIG_NR_DRAM_BANKS=y
CONFIG_IMX_VIDEO_SKIP=y
CONFIG_FS_FAT=y
CONFIG_SYS_CBSIZE=512
CONFIG_EHCI_HCD_INIT_AFTER_RESET=y
CONFIG_BOOTM_LINUX=y
CONFIG_BOARD_LATE_INIT=y
CONFIG_MII=y
CONFIG_SYS_CACHELINE_SIZE=64
CONFIG_MMC=y
CONFIG_CMD_USB_MASS_STORAGE=y
CONFIG_SYS_FSL_CLK=y
CONFIG_SYS_FSL_SEC_ADDR="(CAAM_IPS_BASE_ADDR)"
CONFIG_CMD_FUSE=y
CONFIG_SYSCOUNTER_TIMER=y
CONFIG_ENV_OFFSET="(12 * SZ_64K)"
CONFIG_MXC_OCOTP=y
CONFIG_ENV_OVERWRITE=y
CONFIG_ENV_SIZE="SZ_8K"
CONFIG_G_DNL_VENDOR_NUM=0x0525
CONFIG_SYS_MALLOC_LEN="(32 * SZ_1M)"
CONFIG_SYS_NO_FLASH=y
CONFIG_SYS_MMC_ENV_DEV=0
CONFIG_SYS_I2C_SPEED=100000
CONFIG_SYS_BOOTM_LEN=0x1000000
CONFIG_SYS_TEXT_BASE=0x87800000
CONFIG_USB_FUNCTION_MASS_STORAGE=y
CONFIG_SYS_AUXCORE_BOOTDATA=0x7F8000
CONFIG_MXC_GPT_HCLK=y
CONFIG_MXC_UART=y
CONFIG_SPLASH_SCREEN=y
CONFIG_SYS_BARGSIZE=$(CONFIG_SYS_CBSIZE)
CONFIG_IS_BUILTIN(option)="config_enabled(CONFIG_VAL(option))"
CONFIG_VIDEO_BMP_RLE8=y
CONFIG_MXC_USB_PORTSC="(PORT_PTS_UTMI | PORT_PTS_PTW)"
CONFIG_DBG_MONITOR=y
CONFIG_SYS_FSL_JR0_ADDR="(CONFIG_SYS_FSL_SEC_ADDR + 0x1000)"
CONFIG_POWER_I2C=y
CONFIG_SYS_MAXARGS=32
CONFIG_BMP_16BPP=y
CONFIG_SYS_PBSIZE="(CONFIG_SYS_CBSIZE + 128)"
CONFIG_FEC_XCV_TYPE="RGMII"
CONFIG_MXC_GPIO=y
CONFIG_BOARDDIR="board/freescale/mx7dsabresd"
CONFIG_POWER=y
CONFIG_BOUNCE_BUFFER=y
CONFIG_OF_LIBFDT=y
CONFIG_IMX_FIXED_IVT_OFFSET=y
CONFIG_SYS_MAX_FLASH_SECT=512
CONFIG_PHYLIB=y
CONFIG_CMDLINE_EDITING=y
CONFIG_CMD_USB=y
CONFIG_VIDEO_LOGO=y
CONFIG_MFG_ENV_SETTINGS="mfgtool_args=setenv bootargs console=${console},${baudrate} rdinit=/linuxrc g_mass_storage.stall=0 g_mass_storage.removable=1 g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF g_mass_storage.iSerialNumber=" CONFIG_MFG_NAND_PARTITION "clk_ignore_unused 0initrd_addr=0x838000000initrd_high=0xffffffff0bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};0"
CONFIG_SYS_CONSOLE_IS_IN_ENV=y
CONFIG_CMD_EXT2=y
CONFIG_CMD_EXT4=y
CONFIG_USB_EHCI=y
CONFIG_DFU_ENV_SETTINGS="dfu_alt_info=image raw 0 0x800000;u-boot raw 0 0x4000;bootimg part 0 1;rootfs part 0 20"
CONFIG_CMD_DFU=y
CONFIG_G_DNL_PRODUCT_NUM=0xa4a5
CONFIG_SYS_MMC_MAX_BLK_COUNT=65535
CONFIG_ZLIB=y
CONFIG_LOADADDR=0x80800000
CONFIG_USB_GADGET_DUALSPEED=y
CONFIG_G_DNL_MANUFACTURER="FSL"
CONFIG_ETHPRIME="FEC"
CONFIG_MX7_SEC=y
CONFIG_CMD_BOOTZ=y
CONFIG_AUTO_COMPLETE=y
CONFIG_SYS_MMC_IMG_LOAD_PART=y
CONFIG_FSL_USDHC=y
CONFIG_ENV_IS_IN_MMC=y
CONFIG_FEC_ENET_DEV=0
CONFIG_DOS_PARTITION=y
CONFIG_GZIP=y
CONFIG_DFU_MMC=y
CONFIG_USB_FUNCTION_DFU=y
CONFIG_SC_TIMER_CLK=8000000
CONFIG_SYS_FSL_ESDHC_ADDR=0
CONFIG_SYS_INIT_RAM_SIZE="IRAM_SIZE"
CONFIG_IOMUX_LPSR=y
CONFIG_FEC_MXC_PHYADDR=0x0
CONFIG_SYS_BAUDRATE_TABLE="{ 9600, 19200, 38400, 57600, 115200 }"
CONFIG_VAL(option)="config_val(option)"
CONFIG_SUPPORT_EMMC_BOOT=y
CONFIG_SYS_HUSH_PARSER=y
CONFIG_VIDEO=y
CONFIG_MFG_NAND_PARTITION=y
CONFIG_SYS_SDRAM_BASE="PHYS_SDRAM"
CONFIG_IMAGE_FORMAT_LEGACY=y
CONFIG_CFB_CONSOLE=y
CONFIG_SYS_INIT_SP_OFFSET="(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)"
CONFIG_USB_ETHER_ASIX=y
CONFIG_GENERIC_MMC=y
CONFIG_FEC_MXC_MDIO_BASE="ENET_IPS_BASE_ADDR"
CONFIG_SYS_I2C=y
CONFIG_SYS_INIT_RAM_ADDR="IRAM_BASE_ADDR"
CONFIG_CI_UDC=y
CONFIG_EXTRA_ENV_SETTINGS="UPDATE_M4_ENV CONFIG_MFG_ENV_SETTINGS CONFIG_DFU_ENV_SETTINGS "script=boot.scr0image=zImage0console=ttymxc00fdt_high=0xffffffff0initrd_high=0xffffffff0fdt_file=imx7d-sdb.dtb0fdt_addr=0x830000000boot_fdt=try0ip_dyn=yes0panel=TFT43AB0mmcdev=__stringify(CONFIG_SYS_MMC_ENV_DEV)"0mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "0mmcroot=" CONFIG_MMCROOT " rootwait rw0mmcautodetect=yes0mmcargs=setenv bootargs console=${console},${baudrate} root=${mmcroot}0loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};0bootscript=echo Running bootscript from mmc ...; source0loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}0loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}0mmcboot=echo Booting from mmc ...; run mmcargs; if test ${boot_fdt} = yes || test ${boot_fdt} = try; then if run loadfdt; then bootz ${loadaddr} - ${fdt_addr}; else if test ${boot_fdt} = try; then bootz; else echo WARN: Cannot load the DT; fi; fi; else bootz; fi;0netargs=setenv bootargs console=${console},${baudrate} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp0netboot=echo Booting from net ...; run netargs; if test ${ip_dyn} = yes; then setenv get_cmd dhcp; else setenv get_cmd tftp; fi; ${get_cmd} ${image}; if test ${boot_fdt} = yes || test ${boot_fdt} = try; then if ${get_cmd} ${fdt_addr} ${fdt_file}; then bootz ${loadaddr} - ${fdt_addr}; else if test ${boot_fdt} = try; then bootz; else echo WARN: Cannot load the DT; fi; fi; else bootz; fi;0"
CONFIG_FSL_CLK=y
CONFIG_SYS_INIT_SP_ADDR="(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)"
CONFIG_FSL_ESDHC=y
CONFIG_IMX_THERMAL=y
CONFIG_BAUDRATE=115200
CONFIG_CMD_BMODE=y
CONFIG_MXC_UART_BASE="UART1_IPS_BASE_ADDR"
CONFIG_SPLASH_SCREEN_ALIGN=y
CONFIG_SYS_BOOT_RAMDISK_HIGH=y
CONFIG_USB_HOST_ETHER=y
CONFIG_SYS_FSL_USDHC_NUM=2
CONFIG_PARTITIONS=y
CONFIG_CMD_I2C=y
CONFIG_SYS_MMC_ENV_PART=0
CONFIG_FEC_MXC=y
CONFIG_USBD_HS=y
CONFIG_USB_MAX_CONTROLLER_COUNT=2
CONFIG_CMD_MEMTEST=y
CONFIG_SYS_DEF_EEPROM_ADDR=0
CONFIG_FS_EXT4=y
CONFIG_SYS_MEMTEST_END="(CONFIG_SYS_MEMTEST_START + 0x20000000)"
CONFIG_POWER_PFUZE3000=y
CONFIG_USB_STORAGE=y
CONFIG_USB_EHCI_MX7=y
CONFIG_DISPLAY_CPUINFO=y
CONFIG_VIDEO_SW_CURSOR=y
CONFIG_MMCROOT="/dev/mmcblk0p2"
CONFIG_DFU_RAM=y
CONFIG_EXT4_WRITE=y
CONFIG_SYS_MEMTEST_START=0x80000000
CONFIG_PHY_BROADCOM=y
CONFIG_CMD_EXT4_WRITE=y
CONFIG_CONS_INDEX=y
CONFIG_LMB=y
CONFIG_SYS_I2C_MXC=y
CONFIG_IS_ENABLED(option)="(config_enabled(CONFIG_VAL(option)) || config_enabled(CONFIG_VAL(option ##_MODULE)))"
CONFIG_SYS_I2C_MXC_I2C1=y
CONFIG_CMD_MII=y
CONFIG_USB_GADGET=y
CONFIG_CMD_BMP=y
CONFIG_USB_GADGET_VBUS_DRAW=2
CONFIG_CMD_MMC=y
