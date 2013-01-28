/*
 *  File Name       : arch/arm/mach-emxx/include/mach/gpio.h
 *  Function        : gpio
 *  Release Version : Ver 1.02
 *  Release Date    : 2012/12/24
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 *
 ****************************  History  ***************************
 *	Ver 1.01 2010/12/24	created by Renesas
 *	Ver 1.02 2012/12/24	modified by Channing for N307
 */
#ifndef __ASM_ARCH_EMXX_GPIO_H
#define __ASM_ARCH_EMXX_GPIO_H

#include <linux/io.h>
#include <linux/errno.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/pwc.h>
#include <mach/extio.h>

/* GPIO port */

/* GIO Registers */
#define VA_GIO	IO_ADDRESS(EMXX_GPIO_BASE)

#define	GIO_000_OFFSET	0x0000
#define	GIO_032_OFFSET	0x0080
#define	GIO_064_OFFSET	0x0100
#define	GIO_096_OFFSET	0x0180
#define	GIO_128_OFFSET	0x0200

#define GIO_OFFSET	0x80

#define PIN_INDEX(X)	((X) >> 5)


/* GPIO offset address */
#define GIO_E1		0x0000	/* set output */
#define GIO_E0		0x0004	/* set input */
#define GIO_EM		0x0004	/* in/out setting monitor */
#define GIO_OL		0x0008	/* data output low 16bit */
#define GIO_OH		0x000C	/* data output high 16bit */
#define GIO_I		0x0010	/* data intput */
#define GIO_IIA		0x0014	/* enable interrupt */
#define GIO_IEN		0x0018	/* interrupt unmask */
#define GIO_IDS		0x001C	/* interrupt mask */
#define GIO_IIM		0x001C	/* interrupt mask status */
#define GIO_RAW		0x0020	/* interrupt raw status */
#define GIO_MST		0x0024	/* interrupt masked status */
#define GIO_IIR		0x0028	/* interrupt status reset */
#define GIO_GSW		0x003C	/* set INT_FIQ */
#define GIO_IDT0	0x0040	/* interrupt detect control  0- 7 bit */
#define GIO_IDT1	0x0044	/* interrupt detect control  8-15 bit */
#define GIO_IDT2	0x0048	/* interrupt detect control 16-23 bit */
#define GIO_IDT3	0x004C	/* interrupt detect control 24-31 bit */
#define GIO_RAWBL	0x0050	/* interrupt raw status(double-edge) low */
#define GIO_RAWBH	0x0054	/* interrupt raw status(double-edge) high */
#define GIO_IRBL	0x0058	/* interrupt status reset(double-edge) low */
#define GIO_IRBH	0x005C	/* interrupt status reset(double-edge) high */

/* GPIO number */
#define GPIO_P0		0
#define GPIO_P1		1
#define GPIO_P2		2
#define GPIO_P3		3
#define GPIO_P4		4
#define GPIO_P5		5
#define GPIO_P6		6
#define GPIO_P7		7
#define GPIO_P8		8
#define GPIO_P9		9
#define GPIO_P10	10
#define GPIO_P11	11
#define GPIO_P12	12
#define GPIO_P13	13
#define GPIO_P14	14
#define GPIO_P15	15
#define GPIO_P16	16
#define GPIO_P17	17
#define GPIO_P18	18
#define GPIO_P19	19
#define GPIO_P20	20
#define GPIO_P21	21
#define GPIO_P22	22
#define GPIO_P23	23
#define GPIO_P24	24
#define GPIO_P25	25
#define GPIO_P26	26
#define GPIO_P27	27
#define GPIO_P28	28
#define GPIO_P29	29
#define GPIO_P30	30
#define GPIO_P31	31
#define GPIO_P32	32
#define GPIO_P33	33
#define GPIO_P34	34
#define GPIO_P35	35
#define GPIO_P36	36
#define GPIO_P37	37
#define GPIO_P38	38
#define GPIO_P39	39
#define GPIO_P40	40
#define GPIO_P41	41
#define GPIO_P42	42
#define GPIO_P43	43
#define GPIO_P44	44
#define GPIO_P45	45
#define GPIO_P46	46
#define GPIO_P47	47
#define GPIO_P48	48
#define GPIO_P49	49
#define GPIO_P50	50
#define GPIO_P51	51
#define GPIO_P52	52
#define GPIO_P53	53
#define GPIO_P54	54
#define GPIO_P55	55
#define GPIO_P56	56
#define GPIO_P57	57
#define GPIO_P58	58
#define GPIO_P59	59
#define GPIO_P60	60
#define GPIO_P61	61
#define GPIO_P62	62
#define GPIO_P63	63
#define GPIO_P64	64
#define GPIO_P65	65
#define GPIO_P66	66
#define GPIO_P67	67
#define GPIO_P68	68
#define GPIO_P69	69
#define GPIO_P70	70
#define GPIO_P71	71
#define GPIO_P72	72
#define GPIO_P73	73
#define GPIO_P74	74
#define GPIO_P75	75
#define GPIO_P76	76
#define GPIO_P77	77
#define GPIO_P78	78
#define GPIO_P79	79
#define GPIO_P80	80
#define GPIO_P81	81
#define GPIO_P82	82
#define GPIO_P83	83
#define GPIO_P84	84
#define GPIO_P85	85
#define GPIO_P86	86
#define GPIO_P87	87
#define GPIO_P88	88
#define GPIO_P89	89
#define GPIO_P90	90
#define GPIO_P91	91
#define GPIO_P92	92
#define GPIO_P93	93
#define GPIO_P94	94
#define GPIO_P95	95
#define GPIO_P96	96
#define GPIO_P97	97
#define GPIO_P98	98
#define GPIO_P99	99
#define GPIO_P100	100
#define GPIO_P101	101
#define GPIO_P102	102
#define GPIO_P103	103
#define GPIO_P104	104
#define GPIO_P105	105
#define GPIO_P106	106
#define GPIO_P107	107
#define GPIO_P108	108
#define GPIO_P109	109
#define GPIO_P110	110
#define GPIO_P111	111
#define GPIO_P112	112
#define GPIO_P113	113
#define GPIO_P114	114
#define GPIO_P115	115
#define GPIO_P116	116
#define GPIO_P117	117
#define GPIO_P118	118
#define GPIO_P119	119
#define GPIO_P120	120
#define GPIO_P121	121
#define GPIO_P122	122
#define GPIO_P123	123
#define GPIO_P124	124
#define GPIO_P125	125
#define GPIO_P126	126
#define GPIO_P127	127
#define GPIO_P128	128
#define GPIO_P129	129
#define GPIO_P130	130
#define GPIO_P131	131
#define GPIO_P132	132
#define GPIO_P133	133
#define GPIO_P134	134
#define GPIO_P135	135
#define GPIO_P136	136
#define GPIO_P137	137
#define GPIO_P138	138
#define GPIO_P139	139
#define GPIO_P140	140
#define GPIO_P141	141
#define GPIO_P142	142
#define GPIO_P143	143
#define GPIO_P144	144
#define GPIO_P145	145
#define GPIO_P146	146
#define GPIO_P147	147
#define GPIO_P148	148
#define GPIO_P149	149
#define GPIO_P150	150
#define GPIO_P151	151
#define GPIO_P152	152
#define GPIO_P153	153
#define GPIO_P154	154
#define GPIO_P155	155
#define GPIO_P156	156
#define GPIO_P157	157
#define GPIO_P158	158
#define GPIO_P159	159

#define GPIO_GPIO_LAST   GPIO_P159

#define GPIO_PWC_BASE	(GPIO_GPIO_LAST + 1)
#define GPIO_PWC_P0	(GPIO_PWC_BASE + 0)
#define GPIO_PWC_P1	(GPIO_PWC_BASE + 1)
#define GPIO_PWC_P2	(GPIO_PWC_BASE + 2)
#define GPIO_PWC_P3	(GPIO_PWC_BASE + 3)
#define GPIO_PWC_P4	(GPIO_PWC_BASE + 4)
#define GPIO_PWC_P5	(GPIO_PWC_BASE + 5)
#define GPIO_PWC_P6	(GPIO_PWC_BASE + 6)
#define GPIO_PWC_P7	(GPIO_PWC_BASE + 7)
#define GPIO_PWC_P8	(GPIO_PWC_BASE + 8)
#define GPIO_PWC_P9	(GPIO_PWC_BASE + 9)
#define GPIO_PWC_P10	(GPIO_PWC_BASE + 10)
#define GPIO_PWC_P11	(GPIO_PWC_BASE + 11)
#define GPIO_PWC_P12	(GPIO_PWC_BASE + 12)
#define GPIO_PWC_P13	(GPIO_PWC_BASE + 13)
#define GPIO_PWC_P14	(GPIO_PWC_BASE + 14)
#define GPIO_PWC_P15	(GPIO_PWC_BASE + 15)
#define GPIO_PWC_LAST	GPIO_PWC_P15

#define GPIO_EXT1_BASE	(GPIO_PWC_LAST + 1)
#define GPIO_EXT1_P0	(GPIO_EXT1_BASE + 0)
#define GPIO_EXT1_P1	(GPIO_EXT1_BASE + 1)
#define GPIO_EXT1_P2	(GPIO_EXT1_BASE + 2)
#define GPIO_EXT1_P3	(GPIO_EXT1_BASE + 3)
#define GPIO_EXT1_P4	(GPIO_EXT1_BASE + 4)
#define GPIO_EXT1_P5	(GPIO_EXT1_BASE + 5)
#define GPIO_EXT1_P6	(GPIO_EXT1_BASE + 6)
#define GPIO_EXT1_P7	(GPIO_EXT1_BASE + 7)
#define GPIO_EXT1_P8	(GPIO_EXT1_BASE + 8)
#define GPIO_EXT1_P9	(GPIO_EXT1_BASE + 9)
#define GPIO_EXT1_P10	(GPIO_EXT1_BASE + 10)
#define GPIO_EXT1_P11	(GPIO_EXT1_BASE + 11)
#define GPIO_EXT1_P12	(GPIO_EXT1_BASE + 12)
#define GPIO_EXT1_P13	(GPIO_EXT1_BASE + 13)
#define GPIO_EXT1_P14	(GPIO_EXT1_BASE + 14)
#define GPIO_EXT1_P15	(GPIO_EXT1_BASE + 15)
#define GPIO_EXT1_P16	(GPIO_EXT1_BASE + 16)
#define GPIO_EXT1_P17	(GPIO_EXT1_BASE + 17)
#define GPIO_EXT1_P18	(GPIO_EXT1_BASE + 18)
#define GPIO_EXT1_P19	(GPIO_EXT1_BASE + 19)
#define GPIO_EXT1_P20	(GPIO_EXT1_BASE + 20)
#define GPIO_EXT1_P21	(GPIO_EXT1_BASE + 21)
#define GPIO_EXT1_P22	(GPIO_EXT1_BASE + 22)
#define GPIO_EXT1_P23	(GPIO_EXT1_BASE + 23)
#define GPIO_EXT1_P24	(GPIO_EXT1_BASE + 24)
#define GPIO_EXT1_P25	(GPIO_EXT1_BASE + 25)
#define GPIO_EXT1_P26	(GPIO_EXT1_BASE + 26)
#define GPIO_EXT1_P27	(GPIO_EXT1_BASE + 27)
#define GPIO_EXT1_P28	(GPIO_EXT1_BASE + 28)
#define GPIO_EXT1_P29	(GPIO_EXT1_BASE + 29)
#define GPIO_EXT1_P30	(GPIO_EXT1_BASE + 30)
#define GPIO_EXT1_P31	(GPIO_EXT1_BASE + 31)
#define GPIO_EXT1_LAST	GPIO_EXT1_P31

#ifdef CONFIG_EMGR_TI_PMIC

#define GPIO_PWC_EXT_BASE	(GPIO_EXT1_LAST + 1)
#define GPIO_PWC_EXT_P0		(GPIO_PWC_EXT_BASE + 0)
#define GPIO_PWC_EXT_P1		(GPIO_PWC_EXT_BASE + 1)
#define GPIO_PWC_EXT_P2		(GPIO_PWC_EXT_BASE + 2)
#define GPIO_PWC_EXT_P3		(GPIO_PWC_EXT_BASE + 3)
#define GPIO_PWC_EXT_P4		(GPIO_PWC_EXT_BASE + 4)
#define GPIO_PWC_EXT_P5		(GPIO_PWC_EXT_BASE + 5)
#define GPIO_PWC_EXT_P6		(GPIO_PWC_EXT_BASE + 6)
#define GPIO_PWC_EXT_P7		(GPIO_PWC_EXT_BASE + 7)
#define GPIO_PWC_EXT_LAST	GPIO_PWC_EXT_P7

#define GPIO_LAST	GPIO_PWC_EXT_LAST

#else /* CONFIG_EMGR_TI_PMIC */
#define GPIO_LAST	GPIO_EXT1_LAST
#endif /* CONFIG_EMGR_TI_PMIC */

#define GPIO_PXXX    GPIO_P69    //AB_CSB0, not used port, for build purpose.

/* board specific */
#ifdef CONFIG_EMGR_TI_PMIC
#define GPIO_PWC	GPIO_PWC_EXT_P0	/* in  */
#define GPIO_PWC_EXT	GPIO_P0		/* in  */
#else /* CONFIG_EMGR_TI_PMIC */
#define GPIO_PWC	GPIO_P0		/* in  */     //defined but not used in this kernel. used in 307;
#endif /* CONFIG_EMGR_TI_PMIC */
#define GPIO_JT_SEL	GPIO_P2		/* in  */     //detect the pin configuration for JTAG ?
#ifdef CONFIG_EMEV_BOARD_EVA
//#define GPIO_ETH	GPIO_P1		/* in  */     //not used in 307 ---youyou
//#define GPIO_HDMI_INT	GPIO_P6		/* in  */     //not used in 307 ---youyou
#define GPIO_USB_PPON	GPIO_PXXX      //GPIO_P117 is LCM_SDO in 307, not used	/* out */ 
#define GPIO_USB_OCI	GPIO_PXXX      //GPIO_P118 is LCM_CS in 307, not used	/* in  */   
#elif defined(CONFIG_EMGR_BOARD_EVA)
#define GPIO_ETH	GPIO_P3		/* in  */
#define GPIO_HDMI_INT	GPIO_P10	/* in  */
#define GPIO_USB_OCI	GPIO_P11	/* in  */
#define GPIO_USB_PPON	GPIO_P12	/* out */
#endif
#define GPIO_SDC_CD	GPIO_P49	/* in  */
#define GPIO_VBUS	GPIO_P153	/* in  */

/* PWC */
#define GPIO_EXTIO1	GPIO_PWC_P0	/* in  */
#define GPIO_PWC_LED1	GPIO_PWC_P14	/* out */
#define GPIO_PWC_LED2	GPIO_PWC_P15	/* out */

/* EXTIO */
#define GPIO_KEY_OUT0	GPIO_EXT1_P0	/* out */
#define GPIO_KEY_OUT1	GPIO_EXT1_P1	/* out */
#define GPIO_KEY_OUT2	GPIO_EXT1_P2	/* out */
#define GPIO_KEY_OUT3	GPIO_EXT1_P3	/* out */
#define GPIO_KEY_OUT4	GPIO_EXT1_P4	/* out */
#define GPIO_SD_WP	GPIO_EXT1_P5	/* in  */
#define GPIO_SDI1_CD	GPIO_EXT1_P6	/* in  */
#define GPIO_SDI2_CD	GPIO_EXT1_P7	/* in  */
#define GPIO_KEY_DATA0	GPIO_EXT1_P8	/* in  */
#define GPIO_KEY_DATA1	GPIO_EXT1_P9	/* in  */
#define GPIO_KEY_DATA2	GPIO_EXT1_P10	/* in  */
#define GPIO_KEY_DATA3	GPIO_EXT1_P11	/* in  */
#define GPIO_DTV_GIO0	GPIO_EXT1_P12	/* out */
#define GPIO_DAIT_INT	GPIO_EXT1_P13	/* in  */
#define GPIO_NTSC_RST	GPIO_EXT1_P16	/* out */
#define GPIO_LCD_RST	GPIO_EXT1_P17	/* out */
#define GPIO_AUDIO_RST	GPIO_EXT1_P18	/* out */
#define GPIO_ETH_RST	GPIO_EXT1_P19	/* out */
#define GPIO_CAM_RST	GPIO_EXT1_P20	/* out */
#define GPIO_DTV_GIO1	GPIO_EXT1_P21	/* out */
#define GPIO_DAIT_RST	GPIO_EXT1_P22	/* out */
#define GPIO_NAND_WP	GPIO_EXT1_P24	/* out */
#define GPIO_XD_WP	GPIO_EXT1_P25	/* out */
#define GPIO_SDI1_WP	GPIO_EXT1_P26	/* in  */
#define GPIO_SDI2_WP	GPIO_EXT1_P27	/* in  */

/* Specific for broncho board */
/* GPIO hardware buttons */
#define GPIO_KEY_VOLUMEDOWN  	GPIO_P87  
#define GPIO_KEY_VOLUMEUP    	GPIO_P85  
#define GPIO_KEY_POWER       	GPIO_P150
#define GPIO_KEY_MENU		GPIO_P83
#define GPIO_KEY_BACK		GPIO_P82
#define GPIO_KEY_LIGHTUP	GPIO_P103
#define GPIO_KEY_LIGHTDOWN	GPIO_P104
#define GPIO_KEY_F1		GPIO_P110
#define GPIO_KEY_F2		GPIO_P112
#define GPIO_KEY_F3		GPIO_P86
#define GPIO_KEY_F4		GPIO_P93

/* WiFi & BT control */
#define GPIO_BCM_BT_RST       GPIO_P31    //GPIO_P114  is connect to TP_SDA in 307.  TODO:P31 is connected to HUB_EN in 307 board?
#define GPIO_BCM_WLAN_BT_EN   GPIO_P6     //GPIO_P115 is LCM_SCL in 307, not used
#define GPIO_BCM_WLAN_PDN     GPIO_PXXX   //GPIO_P116 is LCM_SDI in 307, not used.  defined but not used in kernel...
#define GPIO_BCM_WLAN_RST     GPIO_P7     //GPIO_P117
#define GPIO_WIFI_WAKE        GPIO_P9     //GPIO_P106
/* 3G module control */
#define GPIO_3G_RF_DISABLE_N              //GPIO_P12 is not used in 307
#define GPIO_3G_RST_N                     //GPIO_P25 is used as MESH_WAKEUP in 307
#define GPIO_3G_EN                        //GPIO_P24 is used as CMX_PWREN in 307
/* LCD control */
//#define SCREEN_RST 	                  //GPIO_P103 is key_light- in 307
#define SCREEN_DISP	       GPIO_P1      //GPIO_P99 is not used in 307
#define SCREEN_POERON	       GPIO_P10    //GPIO_P150 is key_power in 307

/*Touch Screen control*/
#define TP_INT_GPIO	GPIO_P15
#define I2C_SCL_GPIO	GPIO_P119	//i2c-gpio
#define I2C_SDA_GPIO	GPIO_P114

/*CMX*/
#define CMX_PWREN		GPIO_P24
#define CMX_IRQ			GPIO_P105
#define CMX_SYNC		GPIO_P106	

#define	BUCK_OFFRT_GPIO 	GPIO_P144
#define GPS_PWR_EN		GPIO_P90

/*GPIO related register control*/
#define FLAG_FUN_BIT		(31)	/*1 bits*/
#define FLAG_PULL_BIT		(30)	/*1 bits*/
#define FLAG_DRIVE_BIT		(29) 	/* 1 bits */
#define FLAG_MOD_BIT		(28) 	/* 1 bits */
#define FLAG_INPUT_BIT		(27) 	/* 1 bits */
#define MFP_FUN_BIT		(14)	/*1 bits*/
#define MFP_INPUT_BIT		(12) 	/* 1 bits */
#define MFP_MOD_BIT		(10) 	/* 2 bits */
#define MFP_PULL_BIT		(8)  	/* 2 bits */
#define MFP_PIN_BIT		(0)  	/* 8 bits */

#define MFP_FUN_FLAG(x)		((x)&(0x1<<FLAG_FUN_BIT))
#define MFP_PULL_FLAG(x)	((x)&(0x1<<FLAG_PULL_BIT))
#define MFP_DRIVE_FLAG(x)	((x)&(0x1<<FLAG_DRIVE_BIT))
#define MFP_MOD_FLAG(x)		((x)&(0x1<<FLAG_MOD_BIT))
#define MFP_INPUT_FLAG(x)	((x)&(0x1<<FLAG_INPUT_BIT))

#define MFP_PIN(x)		(((x)>>MFP_PIN_BIT)&0xff)
#define MFP_PULL(x)		(((x)>>MFP_PULL_BIT)&0x03)
#define MFP_MOD(x)		(((x)>>MFP_MOD_BIT)&0x03)
#define MFP_INPUT(x)		(((x)>>MFP_INPUT_BIT)&0x03)
#define MFP_FUN(x)		(((x)>>MFP_FUN_BIT)&0x01)

enum __MFP_PULL__{
	MFP_UART0 = GPIO_P159,
	MFP_SD_CMD,
	MFP_SD_DATA,
	MFP_USI0_CK,
	MFP_USI0_DI,
	MFP_USI0_DO,
	MFP_USI0_CS0,
	MFP_USI1_CK,
	MFP_USI1_CS0,
	MFP_JT,
	MFP_JT_RST,
	MFP_JT_DEN,
	MFP_JT_EN,
	MFP_NULL = 0xff,
};

enum __MFP_MOD__{
	MFP_MOD_LCD3_0 = GPIO_P159,
	MFP_MOD_LCD3_2,
	MFP_MOD_LCD3_4,
	MFP_MOD_LCD3_6,
	MFP_MOD_LCD3_8,
	MFP_MOD_LCD3_10,
	MFP_MOD_UART,
	MFP_MOD_I2C,
	MFP_MOD_SD_0,
	MFP_MOD_SD_2,
	MFP_MOD_AB_0,
	MFP_MOD_AB_2,
	MFP_MOD_AB_4,
	MFP_MOD_AB_6,
	MFP_MOD_AB_8,
	MFP_MOD_AB_10,
	MFP_MOD_AB_12,
	MFP_MOD_AB_14,
	MFP_MOD_USI_0,
	MFP_MOD_USI_2,
	MFP_MOD_USI_4,
	MFP_MOD_USI_6,
	MFP_MOD_USI_8,
	MFP_MOD_NTSC,
	MFP_MOD_CAM,
	MFP_MOD_HSI,
	MFP_MOD_MAX,
};

#define MFP_PULL_NONE		((0x0<<MFP_PULL_BIT)|(0x1<<FLAG_PULL_BIT))
#define MFP_PULL_UP		((0x1<<MFP_PULL_BIT)|(0x1<<FLAG_PULL_BIT))
#define MFP_PULL_DOWN		((0x3<<MFP_PULL_BIT)|(0x1<<FLAG_PULL_BIT))

#define MFP_FUN_GPIO		((0x1<<MFP_FUN_BIT)|(0x1<<FLAG_FUN_BIT))
#define MFP_FUN_OTHER		((0x0<<MFP_FUN_BIT)|(0x1<<FLAG_FUN_BIT))

#define MFP_FUN_MOD0		((0x0<<MFP_MOD_BIT)|(0x1<<FLAG_MOD_BIT))
#define MFP_FUN_MOD1		((0x1<<MFP_MOD_BIT)|(0x1<<FLAG_MOD_BIT))
#define MFP_FUN_MOD2		((0x2<<MFP_MOD_BIT)|(0x1<<FLAG_MOD_BIT))
#define MFP_FUN_MOD3		((0x3<<MFP_MOD_BIT)|(0x1<<FLAG_MOD_BIT))

#define MFP_INPUT_DISABLE	((0x0<<MFP_INPUT_BIT)|(0x1<<FLAG_INPUT_BIT))
#define MFP_INPUT_NORMAL	((0x1<<MFP_INPUT_BIT)|(0x1<<FLAG_INPUT_BIT))
#define MFP_INPUT_TRIGGER	((0x3<<MFP_INPUT_BIT)|(0x1<<FLAG_INPUT_BIT))

#define MFP_CFG(pin, mfp)	((pin<<MFP_PIN_BIT)|mfp)

extern void emxx_single_mfp_config(unsigned int mfp);
extern void emxx_mfps_config(unsigned int *pmfp, int num);

static inline int __gpio_set_direction(unsigned gpio, int is_input)
{
	unsigned int offset;

	offset = GIO_OFFSET * PIN_INDEX(gpio);
	offset += is_input ? (GIO_E0) : (GIO_E1);

	__raw_writel(1U << (gpio & 0x1f), VA_GIO + offset);
	return 0;
}

static inline int __gpio_get_direction(unsigned gpio)
{
	unsigned int offset;
	unsigned int val;

	offset = GIO_OFFSET * PIN_INDEX(gpio);
	offset += GIO_EM;

	val = __raw_readl(VA_GIO + offset);
	return (val & (1U << (gpio & 0x1f))) ? 1 : 0;
}

static inline void __gpio_set_value(unsigned gpio, int val)
{
	unsigned int offset;
	unsigned int goe, gos;

	offset = GIO_OFFSET * PIN_INDEX(gpio);
	offset += (gpio & 0x10) ? GIO_OH : GIO_OL;

	goe = (0x10000U << (gpio & 0xf));
	gos = val ? (1U << (gpio & 0xf)) : 0;

	__raw_writel(goe | gos, VA_GIO + offset);
}

static inline int __gpio_get_value(unsigned gpio)
{
	unsigned int val;
	unsigned int offset;

	offset = GIO_OFFSET * PIN_INDEX(gpio);

	if (__gpio_get_direction(gpio)) {
		offset += (gpio & 0x10) ? GIO_OH : GIO_OL;
		val = __raw_readl(VA_GIO + offset);
		return (val & (1U << (gpio & 0xf))) ? 1 : 0;
	} else {
		offset += GIO_I;
		val = __raw_readl(VA_GIO + offset);
		return (val & (1U << (gpio & 0x1f))) ? 1 : 0;
	}
}

static inline int gpio_direction_input(unsigned gpio)
{
	if (gpio <= GPIO_GPIO_LAST)
		return __gpio_set_direction(gpio, 1);
	else if (gpio <= GPIO_PWC_LAST)
		return pwc_set_direction(gpio, 1);
	else if (gpio <= GPIO_EXT1_LAST)
		return extio_set_direction(gpio, 1);
#ifdef CONFIG_EMGR_TI_PMIC
	else if (gpio <= GPIO_PWC_EXT_LAST)
		return pwc_ext_set_direction(gpio, 1);
#endif
	else
		return -EINVAL;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	int ret = -EINVAL;

	if (gpio <= GPIO_GPIO_LAST) {
		ret = __gpio_set_direction(gpio, 0);
		if (!ret)
			__gpio_set_value(gpio, value);
	} else if (gpio <= GPIO_PWC_LAST) {
		ret = pwc_set_direction(gpio, 0);
		if (!ret)
			pwc_set_value(gpio, value);
	} else if (gpio <= GPIO_EXT1_LAST) {
		ret = extio_set_direction(gpio, 0);
		if (!ret)
			extio_set_value(gpio, value);
#ifdef CONFIG_EMGR_TI_PMIC
	} else if (gpio <= GPIO_PWC_EXT_LAST) {
		ret = pwc_ext_set_direction(gpio, 0);
		if (!ret)
			pwc_ext_set_value(gpio, value);
#endif
	}
	return ret;
}

static inline int gpio_get_value(unsigned gpio)
{
	if (gpio <= GPIO_GPIO_LAST)
		return __gpio_get_value(gpio);
	else if (gpio <= GPIO_PWC_LAST)
		return pwc_get_value(gpio);
	else if (gpio <= GPIO_EXT1_LAST)
		return extio_get_value(gpio);
#ifdef CONFIG_EMGR_TI_PMIC
	else if (gpio <= GPIO_PWC_EXT_LAST)
		return pwc_ext_get_value(gpio);
#endif
	else
		return -EINVAL;
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (gpio <= GPIO_GPIO_LAST)
		__gpio_set_value(gpio, value);
	else if (gpio <= GPIO_PWC_LAST)
		pwc_set_value(gpio, value);
	else if (gpio <= GPIO_EXT1_LAST)
		extio_set_value(gpio, value);
#ifdef CONFIG_EMGR_TI_PMIC
	else if (gpio <= GPIO_PWC_EXT_LAST)
		pwc_ext_set_value(gpio, value);;
#endif
}

static inline int gpio_is_valid(int number)
{
	return number >= 0 && number <= GPIO_LAST;
}

static inline int gpio_cansleep(unsigned gpio)
{
	if (gpio <= GPIO_PWC_LAST)
		return 0;
	else if (gpio <= GPIO_EXT1_LAST)
		return 1;
	else
		return 0;
}

static inline int gpio_get_value_cansleep(unsigned gpio)
{
	might_sleep();
	return gpio_get_value(gpio);
}

static inline void gpio_set_value_cansleep(unsigned gpio, int value)
{
	might_sleep();
	gpio_set_value(gpio, value);
}

static inline int gpio_to_irq(unsigned gpio)
{
	if (gpio <= GPIO_GPIO_LAST)
		return gpio + INT_GPIO_BASE;
	else if (gpio <= GPIO_PWC_LAST)
		return (gpio - GPIO_PWC_BASE) + INT_PWC_MASK3_BASE;
	else
		return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	if ((irq >= INT_GPIO_BASE) && (irq <= INT_GPIO_LAST))
		return irq - INT_GPIO_BASE;
	else if ((irq >= INT_PWC_MASK3_BASE) && (irq <= INT_PWC_LAST))
		return (irq - INT_PWC_MASK3_BASE) + GPIO_PWC_BASE;
	else
		return -EINVAL;
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	if (gpio <= GPIO_LAST)
		return 0;
	else
		return -EINVAL;
}

static inline void gpio_free(unsigned gpio)
{
}

extern int emxx_gio_set_irq_type(unsigned int irq, unsigned int type);

#endif	/* __ASM_ARCH_EMXX_GPIO_H */
