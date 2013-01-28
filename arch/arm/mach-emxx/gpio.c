/*
 *  File Name       : arch/arm/mach-emxx/gpio.c
 *  Function        : gpio
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/02/05
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach/irq.h>
#include <mach/smu.h>

/*#define GPIO_DEBUG 1*/
#ifdef GPIO_DEBUG
#define DPRINT(FMT, ARGS...) printk(KERN_INFO "%s(): " FMT, __func__, ##ARGS)
#else
#define DPRINT(FMT, ARGS...)
#endif

#define PIN_MASK(X)	(1U << ((X) & 0x1f))	/* IIA/IEN/IDS/PLS/... */

static const unsigned char pull_reg_config[] = {
	GPIO_P18,	GPIO_P19,	GPIO_P20,	GPIO_P21,	GPIO_P22,	GPIO_P23,	GPIO_P24,	GPIO_P25,	/* CHG_PULL0 */
	GPIO_P26,	GPIO_P27,	GPIO_P28,	GPIO_P29,	GPIO_P30,	GPIO_P31,	GPIO_P32,	GPIO_P33,	/* CHG_PULL1 */
	GPIO_P34,	GPIO_P35,	GPIO_P36,	GPIO_P37,	GPIO_P38,	GPIO_P39,	GPIO_P40,	GPIO_P41,	/* CHG_PULL2 */
	GPIO_P42,	GPIO_P43,	MFP_NULL,	MFP_UART0,	GPIO_P44,	GPIO_P45,	GPIO_P46,	GPIO_P47,	/* CHG_PULL3 */
	GPIO_P48,	MFP_SD_CMD,	MFP_SD_DATA,GPIO_P49,	GPIO_P50,	GPIO_P51,	GPIO_P52,	MFP_NULL,	/* CHG_PULL4 */
	GPIO_P53,	GPIO_P54,	GPIO_P55,	GPIO_P56,	GPIO_P57,	GPIO_P58,	GPIO_P59,	GPIO_P60,	/* CHG_PULL5 */
	GPIO_P61,	GPIO_P62,	GPIO_P63,	GPIO_P64,	GPIO_P65,	GPIO_P66,	GPIO_P67,	MFP_NULL,	/* CHG_PULL6 */
	GPIO_P68,	GPIO_P69,	GPIO_P70,	GPIO_P71,	GPIO_P72,	GPIO_P73,	GPIO_P74,	GPIO_P75,	/* CHG_PULL7 */
	GPIO_P76,	GPIO_P77,	GPIO_P78,	GPIO_P79,	GPIO_P80,	GPIO_P81,	GPIO_P82,	GPIO_P83,	/* CHG_PULL8 */
	GPIO_P84,	GPIO_P85,	GPIO_P86,	GPIO_P87,	GPIO_P88,	GPIO_P89,	GPIO_P90,	GPIO_P91,	/* CHG_PULL9 */
	GPIO_P92,	GPIO_P93,	GPIO_P94,	GPIO_P95,	GPIO_P96,	GPIO_P97,	GPIO_P98,	GPIO_P99,	/* CHG_PULL10 */
	GPIO_P100,	GPIO_P101,	GPIO_P102,	GPIO_P103,	GPIO_P104,	MFP_NULL,	MFP_NULL,	MFP_NULL,	/* CHG_PULL11 */
	MFP_USI0_CK,MFP_USI0_DI,MFP_USI0_DO,MFP_USI0_CS0,GPIO_P105,GPIO_P106,	MFP_NULL,	MFP_NULL,	/* CHG_PULL12 */
	MFP_USI1_CK,GPIO_P107,	GPIO_P108,	MFP_USI1_CS0,GPIO_P6,	GPIO_P7,	GPIO_P8,	GPIO_P9,	/* CHG_PULL13 */
	GPIO_P10,	GPIO_P11,	GPIO_P12,	GPIO_P13,	GPIO_P14,	GPIO_P15,	GPIO_P16,	GPIO_P17,	/* CHG_PULL14 */
	GPIO_P109,	GPIO_P110,	GPIO_P111,	GPIO_P112,	GPIO_P113,	GPIO_P114,	MFP_NULL,	MFP_NULL,	/* CHG_PULL15 */
	GPIO_P115,	GPIO_P116,	GPIO_P117,	GPIO_P118,	GPIO_P119,	GPIO_P120,	GPIO_P121,	MFP_NULL,	/* CHG_PULL16 */
	GPIO_P122,	GPIO_P123,	GPIO_P124,	GPIO_P125,	GPIO_P126,	MFP_NULL,	MFP_NULL,	MFP_NULL,	/* CHG_PULL17 */
	GPIO_P127,	GPIO_P128,	GPIO_P129,	GPIO_P130,	GPIO_P131,	GPIO_P132,	GPIO_P133,	GPIO_P134,	/* CHG_PULL18 */
	GPIO_P135,	GPIO_P136,	GPIO_P137,	GPIO_P138,	GPIO_P139,	GPIO_P140,	GPIO_P141,	GPIO_P142,	/* CHG_PULL19 */
	GPIO_P0,	GPIO_P1,	GPIO_P2,	GPIO_P3,	GPIO_P4,	GPIO_P5,	GPIO_P154,	MFP_NULL,	/* CHG_PULL20 */
	GPIO_P143,	GPIO_P144,	GPIO_P145,	GPIO_P146,	GPIO_P147,	GPIO_P148,	GPIO_P149,	GPIO_P150,	/* CHG_PULL21 */
	MFP_NULL,	MFP_JT,		GPIO_P151,	MFP_JT_RST,	GPIO_P152,	MFP_JT_DEN,	MFP_JT_EN,	GPIO_P153,	/* CHG_PULL22 */
	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	/* CHG_PULL23 */
	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	/* CHG_PULL24 */
	GPIO_P155,	GPIO_P156,	GPIO_P157,	GPIO_P158,	MFP_NULL,	MFP_NULL,	MFP_NULL,	MFP_NULL,	/* CHG_PULL25 */
};

struct pinsel_config_data {
	unsigned long address;
	unsigned char pin[10];
};
static const struct pinsel_config_data pinsel_reg_config[] = {
	{ CHG_PINSEL_LCD3, {MFP_MOD_LCD3_0, MFP_MOD_LCD3_2, MFP_MOD_LCD3_4, MFP_MOD_LCD3_6, MFP_MOD_LCD3_8, MFP_MOD_LCD3_10},},
	{ CHG_PINSEL_UART, {MFP_MOD_UART},},
#ifdef CONFIG_MACH_EMEV
	{ CHG_PINSEL_IIC, {MFP_MOD_I2C},},
#endif
	{ CHG_PINSEL_SD, {MFP_MOD_SD_0, MFP_MOD_SD_2},},
	{ CHG_PINSEL_AB, {MFP_MOD_AB_0, MFP_MOD_SD_2, MFP_MOD_AB_4, MFP_MOD_AB_6, MFP_MOD_AB_8, MFP_MOD_AB_10, MFP_MOD_AB_12, MFP_MOD_AB_14},},
	{ CHG_PINSEL_USI, {MFP_MOD_USI_0, MFP_MOD_USI_2, MFP_MOD_USI_4, MFP_MOD_USI_6, MFP_MOD_USI_8},},
#ifdef CONFIG_MACH_EMEV
	{ CHG_PINSEL_NTSC, {MFP_MOD_NTSC},},
	{ CHG_PINSEL_CAM, {MFP_MOD_CAM},},
	{ CHG_PINSEL_HSI, {MFP_MOD_HSI},},
#endif
};

void emxx_single_mfp_config(unsigned int mfp)
{
	unsigned int pin = MFP_PIN(mfp);
	int offset, index, size;
	unsigned long regval;
	if (MFP_FUN_FLAG(mfp)) {
		if (pin > GPIO_GPIO_LAST) {
			printk(KERN_ERR "pin%d mfp error!\n", pin);
			return;
		}
		offset = (pin>>5)<<2;
		index = pin&0x1f;

		regval = readl(CHG_PINSEL_G000+offset);
		regval &= ~(0x1<<index);
		regval |= MFP_FUN(mfp)<<index;
		writel(regval, CHG_PINSEL_G000+offset);
		if(MFP_FUN(mfp))
		{
			if((pin > GPIO_P76) && (pin < GPIO_P93))
			{
				regval = readl(CHG_BUSHOLD);
				index = pin % GPIO_P77;
				regval &= ~(0x1<<index);
				writel(regval, CHG_BUSHOLD);	
			}
		}
	}

	if (MFP_PULL_FLAG(mfp) || MFP_INPUT_FLAG(mfp)) {
		unsigned char* ptr = (unsigned char*)pull_reg_config;
		size = sizeof(pull_reg_config);
		for(index=0; index<size; index++) {
			if (pin == ptr[index])
				break;
		}
		if ((index >= size) || (ptr[index]==MFP_NULL)) {
			printk(KERN_ERR "pin%d pull error!\n", pin);
			return;
		}
		offset = (index>>3)<<2;
		index = (index&0x7)<<2;

		regval = readl(CHG_PULL0+offset);
		if (MFP_PULL_FLAG(mfp)) {
			regval &= ~(0x3<<index);
			regval |= MFP_PULL(mfp)<<index;
		}
		if (MFP_INPUT_FLAG(mfp)) {
			regval &= ~(0x03<<(index+2));
			regval |= MFP_INPUT(mfp)<<(index+2);
		}
		writel(regval, CHG_PULL0+offset);
	}

	if (MFP_MOD_FLAG(mfp)) {
		struct pinsel_config_data* pinsel_config;
		for (size=0; size < ARRAY_SIZE(pinsel_reg_config); size++) {
			pinsel_config = (struct pinsel_config_data*)&pinsel_reg_config[size];
			for(index=0; index<ARRAY_SIZE(pinsel_config->pin); index++) {
				if (!pinsel_config->pin[index])
					break;
				if (pin == pinsel_config->pin[index])
					goto find_pinsel_ok;
			}
		}
		printk(KERN_ERR "pin%d mod error!\n", pin);
		return;

find_pinsel_ok:
		offset = pinsel_config->address;
		index = (index&0xf)<<1;
		regval = readl(offset);
		regval &= ~(0x3<<index);
		regval |= MFP_MOD(mfp)<<index;
		writel(regval, offset);
	}
}

void emxx_mfps_config(unsigned int *pmfp, int num)
{
	for (;num; num--) {
		emxx_single_mfp_config(*pmfp++);
	}
}

static DEFINE_SPINLOCK(emxx_gio_lock);

static void emxx_gio_irq_ack(unsigned int irq);
static void emxx_gio_irq_mask(unsigned int irq);
static void emxx_gio_irq_unmask(unsigned int irq);
/*static */int emxx_gio_set_irq_type(unsigned int irq, unsigned int type);

static struct irq_chip emxx_gio_chip_ack = {
	.name     = "GIO-edge",
	.ack      = emxx_gio_irq_ack,
	.mask     = emxx_gio_irq_mask,
	.unmask   = emxx_gio_irq_unmask,
	.set_type = emxx_gio_set_irq_type,
	.disable  = emxx_gio_irq_mask,
};

static struct irq_chip emxx_gio_chip = {
	.name     = "GIO-level",
	.ack      = emxx_gio_irq_mask,
	.mask     = emxx_gio_irq_mask,
	.unmask   = emxx_gio_irq_unmask,
	.set_type = emxx_gio_set_irq_type,
	.disable  = emxx_gio_irq_mask,
};

static uint32_t accept_bits[5] = { 0, 0, 0, 0, 0};


/* called from set_irq_type() */
/*static*/ int emxx_gio_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int pin = irq - INT_GPIO_BASE;
	unsigned int mode = 0;
	unsigned long flag;
	unsigned int oiia;
	unsigned int x;
	unsigned int mask;
	unsigned int idtshift;
	unsigned int idt, iia, iir;
	struct irq_desc *desc;

	DPRINT("Enter\n");

	if ((irq < INT_GPIO_BASE) || (INT_GPIO_LAST < irq))
		return -EINVAL;

	desc = irq_to_desc(irq);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		mode |= 0x00;
		desc->chip = &emxx_gio_chip_ack;
		desc->handle_irq = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode |= 0x01;
		desc->chip = &emxx_gio_chip_ack;
		desc->handle_irq = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		mode |= 0x02;
		desc->chip = &emxx_gio_chip;
		desc->handle_irq = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		mode |= 0x03;
		desc->chip = &emxx_gio_chip;
		desc->handle_irq = handle_level_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode |= 0x04;
		desc->chip = &emxx_gio_chip_ack;
		desc->handle_irq = handle_edge_irq;
		break;
	default:
		return -EINVAL;
	}

	mask = PIN_MASK(pin);
	idtshift = (pin & 0x7) << 2;	/* IDT shift bit */

	spin_lock_irqsave(&emxx_gio_lock, flag);

	idt = VA_GIO + (GIO_OFFSET * PIN_INDEX(pin)) +
			((pin >> 1) & 0xc) + GIO_IDT0;
	iia = VA_GIO + (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IIA;
	iir = VA_GIO + (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IIR;

	/* IIA enable -> disable */
	oiia = __raw_readl(iia);
	if ((oiia & mask) != 0)
		__raw_writel((oiia & ~(mask)), iia);

	/* set IDT */
	x = __raw_readl(idt);
	if ((x & (0xfU << idtshift)) != (mode << idtshift)) {
		x &= ~(0xfU << idtshift);
		x |= (mode << idtshift);
		__raw_writel(x, idt);
	}
	DPRINT("PIN %d Set IDT, addr=0x%08x, data=0x%08x\n",
					pin, idt, __raw_readl(idt));

	/* Interrupt clear */
	__raw_writel(mask, iir);

	/* Restore if changed */
	if ((oiia & mask) != 0)
		__raw_writel(oiia, iia);

	spin_unlock_irqrestore(&emxx_gio_lock, flag);

	DPRINT("Exit\n");

	return 0;
}

static void emxx_gio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct irq_desc *d;
	uint32_t ist, gio_irq, offset;

	DPRINT("Enter\n");

	if ((irq != INT_GIO0) && (irq != INT_GIO1) &&
		 (irq != INT_GIO2) && (irq != INT_GIO3) &&
		 (irq != INT_GIO4) && (irq != INT_GIO5) &&
		 (irq != INT_GIO6) && (irq != INT_GIO7) &&
		 (irq != INT_GIO8) && (irq != INT_GIO9)) {
		return;
	}

	desc->chip->ack(irq);
	offset = VA_GIO + GIO_MST;

	do {
		switch (irq) {
		case INT_GIO0:
			gio_irq = INT_GPIO_0;
			ist = __raw_readl(GIO_000_OFFSET + offset) & 0xffffU;
			break;
		case INT_GIO1:
			gio_irq = INT_GPIO_16;
			ist = __raw_readl(GIO_000_OFFSET + offset) >> 16;
			break;
		case INT_GIO2:
			gio_irq = INT_GPIO_32;
			ist = __raw_readl(GIO_032_OFFSET + offset) & 0xffffU;
			break;
		case INT_GIO3:
			gio_irq = INT_GPIO_48;
			ist = __raw_readl(GIO_032_OFFSET + offset) >> 16;
			break;
		case INT_GIO4:
			gio_irq = INT_GPIO_64;
			ist = __raw_readl(GIO_064_OFFSET + offset) & 0xffffU;
			break;
		case INT_GIO5:
			gio_irq = INT_GPIO_80;
			ist = __raw_readl(GIO_064_OFFSET + offset) >> 16;
			break;
		case INT_GIO6:
			gio_irq = INT_GPIO_96;
			ist = __raw_readl(GIO_096_OFFSET + offset) & 0xffffU;
			break;
		case INT_GIO7:
			gio_irq = INT_GPIO_112;
			ist = __raw_readl(GIO_096_OFFSET + offset) >> 16;
			break;
		case INT_GIO8:
			gio_irq = INT_GPIO_128;
			ist = __raw_readl(GIO_128_OFFSET + offset) & 0xffffU;
			break;
		case INT_GIO9:
			gio_irq = INT_GPIO_144;
			ist = __raw_readl(GIO_128_OFFSET + offset) >> 16;
			break;
		}

		if (ist == 0)
			break;

		while (ist) {
			if ((ist & 1) != 0) {
				DPRINT("PIN %d interrupt handler\n",
						gio_irq - INT_GPIO_0);
				d = irq_to_desc(gio_irq);
				d->handle_irq(gio_irq, d);
			}
			ist >>= 1;
			gio_irq++;
		}
	} while (1);

	desc->chip->unmask(irq);

	DPRINT("Exit\n");
}

static void emxx_gio_irq_ack(unsigned int irq)
{
	unsigned int pin = irq - INT_GPIO_BASE;
	unsigned int mask = PIN_MASK(pin);
	unsigned int offset;

	DPRINT("Enter\n");

	if ((irq < INT_GPIO_BASE) || (INT_GPIO_LAST < irq))
		return;

	offset = (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IIR;
	__raw_writel(mask, VA_GIO + offset);

	DPRINT("Exit\n");
}

static void emxx_gio_irq_mask(unsigned int irq)
{
	unsigned int pin = irq - INT_GPIO_BASE;
	unsigned int mask = PIN_MASK(pin);
	unsigned int offset;

	DPRINT("Enter\n");

	if ((irq < INT_GPIO_BASE) || (INT_GPIO_LAST < irq))
		return;

	/* disable */
	offset = (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IDS;
	__raw_writel(mask, VA_GIO + offset);

	DPRINT("Exit\n");
}

static void emxx_gio_irq_unmask(unsigned int irq)
{
	unsigned int iia;
	unsigned int pin = irq - INT_GPIO_BASE;
	unsigned int mask = PIN_MASK(pin);
	unsigned long flag;
	unsigned int iia_addr, ien_addr;
	uint32_t *accept;

	DPRINT("Enter\n");

	if ((irq < INT_GPIO_BASE) || (INT_GPIO_LAST < irq))
		return;

	/* IRQ enable */
	iia_addr = VA_GIO + (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IIA;
	ien_addr = VA_GIO + (GIO_OFFSET * PIN_INDEX(pin)) + GIO_IEN;
	accept = (accept_bits + PIN_INDEX(pin));

	spin_lock_irqsave(&emxx_gio_lock, flag);
	if ((*accept & mask) == 0) {
		iia = __raw_readl(iia_addr);
		__raw_writel((iia | mask), iia_addr);
		*accept |= mask;
	}
	spin_unlock_irqrestore(&emxx_gio_lock, flag);

	__raw_writel(mask, ien_addr);

	DPRINT("PIN %d irq unmask, addr=0x%08x, mask=0x%08x\n",
				pin, ien_addr, mask);

	DPRINT("Exit\n");
}

static int __init emxx_gio_init(void)
{
	unsigned int i, gpio_num = 0, level_irq, idt_val[20], idt;
	unsigned int offset;

	DPRINT("Enter\n");

	for (i = 0; i < 20; i += 4) {
		offset = (i / 4) * GIO_OFFSET;
		idt_val[i + 0] = __raw_readl(VA_GIO + offset + GIO_IDT0);
		idt_val[i + 1] = __raw_readl(VA_GIO + offset + GIO_IDT1);
		idt_val[i + 2] = __raw_readl(VA_GIO + offset + GIO_IDT2);
		idt_val[i + 3] = __raw_readl(VA_GIO + offset + GIO_IDT3);
	}

	/* setup default GIO Interrupt modes */
	for (i = INT_GPIO_BASE; i <= INT_GPIO_LAST; i++, gpio_num++) {
		level_irq = 0;
		idt = idt_val[gpio_num / 8];
		if (idt & (2 << (4 * (gpio_num & 0x7))))
			level_irq = 1;

		if (level_irq) {
			set_irq_chip(i, &emxx_gio_chip_ack);
			set_irq_chip(i, &emxx_gio_chip);
			set_irq_handler(i, handle_level_irq);
		} else {
			set_irq_chip(i, &emxx_gio_chip);
			set_irq_chip(i, &emxx_gio_chip_ack);
			set_irq_handler(i, handle_edge_irq);
		}
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(INT_GIO0, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO1, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO2, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO3, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO4, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO5, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO6, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO7, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO8, emxx_gio_irq_handler);
	set_irq_chained_handler(INT_GIO9, emxx_gio_irq_handler);

	DPRINT("Exit\n");

	return 0;
}

arch_initcall(emxx_gio_init);
