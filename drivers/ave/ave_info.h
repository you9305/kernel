/*
 *  File Name       : ave_info.h
 *  Function        : AVE information
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

#ifndef __ave_info_h__
#define __ave_info_h__

#include <linux/ave_common_info.h>

#define AVE_ADDRESS	EMXX_AVE_BASE

#define AVE_CODE_RUN	(IO_ADDRESS(AVE_ADDRESS) + 0x0000)
#define AVE_CODE_DL	(IO_ADDRESS(AVE_ADDRESS) + 0x0004)
#define AVE_HOST_INTRQ	(IO_ADDRESS(AVE_ADDRESS) + 0x0008)
#define AVE_BIT_INT_CLR	(IO_ADDRESS(AVE_ADDRESS) + 0x000C)
#define AVE_BIT_INT_STS	(IO_ADDRESS(AVE_ADDRESS) + 0x0010)
#define AVE_BIT_CUR_PC	(IO_ADDRESS(AVE_ADDRESS) + 0x0018)

#define AVE_CODE_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0100)
#define AVE_WORK_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0104)
#define AVE_PARA_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0108)
#define AVE_BITSTR_CTRL	(IO_ADDRESS(AVE_ADDRESS) + 0x010C)
#define AVE_FRAME_CTRL	(IO_ADDRESS(AVE_ADDRESS) + 0x0110)
#define AVE_DECFUN_CTRL	(IO_ADDRESS(AVE_ADDRESS) + 0x0114)
#define AVE_BITSTR_RP0	(IO_ADDRESS(AVE_ADDRESS) + 0x0120)
#define AVE_BITSTR_WP0	(IO_ADDRESS(AVE_ADDRESS) + 0x0124)
#define AVE_BITSTR_RP1	(IO_ADDRESS(AVE_ADDRESS) + 0x0128)
#define AVE_BITSTR_WP1	(IO_ADDRESS(AVE_ADDRESS) + 0x012C)
#define AVE_BITSTR_RP2	(IO_ADDRESS(AVE_ADDRESS) + 0x0130)
#define AVE_BITSTR_WP2	(IO_ADDRESS(AVE_ADDRESS) + 0x0134)
#define AVE_BITSTR_RP3	(IO_ADDRESS(AVE_ADDRESS) + 0x0138)
#define AVE_BITSTR_WP3	(IO_ADDRESS(AVE_ADDRESS) + 0x013C)
#define AVE_AXI_SDR_USE	(IO_ADDRESS(AVE_ADDRESS) + 0x0140)

#define AVE_BITF_DISF0	(IO_ADDRESS(AVE_ADDRESS) + 0x0150)
#define AVE_BITF_DISF1	(IO_ADDRESS(AVE_ADDRESS) + 0x0154)
#define AVE_BITF_DISF2	(IO_ADDRESS(AVE_ADDRESS) + 0x0158)
#define AVE_BITF_DISF3	(IO_ADDRESS(AVE_ADDRESS) + 0x015C)
#define AVE_BUSY_FLAG	(IO_ADDRESS(AVE_ADDRESS) + 0x0160)
#define AVE_RUNCMD	(IO_ADDRESS(AVE_ADDRESS) + 0x0164)
#define AVE_RUN_INDEX	(IO_ADDRESS(AVE_ADDRESS) + 0x0168)
#define AVE_RUN_CODSTD	(IO_ADDRESS(AVE_ADDRESS) + 0x016C)
#define AVE_INT_ENABLE	(IO_ADDRESS(AVE_ADDRESS) + 0x0170)
#define AVE_INT_REASON	(IO_ADDRESS(AVE_ADDRESS) + 0x0174)
#define AVE_RUNAUXSTD	(IO_ADDRESS(AVE_ADDRESS) + 0x0178)

#define AVE_SEQC_BB_START	(IO_ADDRESS(AVE_ADDRESS) + 0x0180)
#define AVE_SEQC_BB_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x0184)
#define AVE_SEQC_OPTION		(IO_ADDRESS(AVE_ADDRESS) + 0x0188)
#define AVE_SEQC_SRC_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x018C)
#define AVE_SEQC_START_BYTE	(IO_ADDRESS(AVE_ADDRESS) + 0x0190)
#define AVE_SEQC_PS_START	(IO_ADDRESS(AVE_ADDRESS) + 0x0194)
#define AVE_SEQC_PS_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x0198)
#define AVE_SEQC_MP4_CLASS	(IO_ADDRESS(AVE_ADDRESS) + 0x019C)
#define AVE_SEQC_VC1_FMT	(IO_ADDRESS(AVE_ADDRESS) + 0x019C)

#define AVE_SEQR_ASPECT		(IO_ADDRESS(AVE_ADDRESS) + 0x01B0)
#define AVE_SEQR_SUCCESS	(IO_ADDRESS(AVE_ADDRESS) + 0x01C0)
#define AVE_SEQR_SRC_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x01C4)
#define AVE_SEQR_SRC_FRATE	(IO_ADDRESS(AVE_ADDRESS) + 0x01C8)
#define AVE_SEQR_FRAME_NEED	(IO_ADDRESS(AVE_ADDRESS) + 0x01CC)
#define AVE_SEQR_FRAME_DLY	(IO_ADDRESS(AVE_ADDRESS) + 0x01D0)
#define AVE_SEQR_INFO		(IO_ADDRESS(AVE_ADDRESS) + 0x01D4)
#define AVE_SEQR_CROP_LR	(IO_ADDRESS(AVE_ADDRESS) + 0x01D8)
#define AVE_SEQR_CROP_TB	(IO_ADDRESS(AVE_ADDRESS) + 0x01DC)
#define AVE_SEQR_NUM_UNIT	(IO_ADDRESS(AVE_ADDRESS) + 0x01E4)
#define AVE_SEQR_TIME_SCALE	(IO_ADDRESS(AVE_ADDRESS) + 0x01E8)
#define AVE_SEQR_MP4_PAR	(IO_ADDRESS(AVE_ADDRESS) + 0x01E8)
#define AVE_SEQR_HDR_RPRT	(IO_ADDRESS(AVE_ADDRESS) + 0x01EC)

#define AVE_PICC_ROT_MODE	(IO_ADDRESS(AVE_ADDRESS) + 0x0180)
#define AVE_PICC_ROT_Y		(IO_ADDRESS(AVE_ADDRESS) + 0x0184)
#define AVE_PICC_ROT_CB		(IO_ADDRESS(AVE_ADDRESS) + 0x0188)
#define AVE_PICC_ROT_CR		(IO_ADDRESS(AVE_ADDRESS) + 0x018C)
#define AVE_PICC_ROT_STRD	(IO_ADDRESS(AVE_ADDRESS) + 0x0190)
#define AVE_PICC_OPTION		(IO_ADDRESS(AVE_ADDRESS) + 0x0194)
#define AVE_PICC_FSKIP_NUM	(IO_ADDRESS(AVE_ADDRESS) + 0x0198)
#define AVE_PICC_FILE_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x019C)
#define AVE_PICC_BB_START	(IO_ADDRESS(AVE_ADDRESS) + 0x01A0)
#define AVE_PICC_START_BYTE	(IO_ADDRESS(AVE_ADDRESS) + 0x01A4)
#define AVE_PICC_PARA_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x01A8)
#define AVE_PICC_USER_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x01AC)
#define AVE_PICC_USER_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x01B0)

#define AVE_PICR_SIZE		(IO_ADDRESS(AVE_ADDRESS) + 0x01BC)
#define AVE_PICR_FRAME_NUM	(IO_ADDRESS(AVE_ADDRESS) + 0x01C0)
#define AVE_PICR_IDX		(IO_ADDRESS(AVE_ADDRESS) + 0x01C4)
#define AVE_PICR_ERR_MB_NUM	(IO_ADDRESS(AVE_ADDRESS) + 0x01C8)
#define AVE_PICR_TYPE		(IO_ADDRESS(AVE_ADDRESS) + 0x01CC)
#define AVE_PICR_POST		(IO_ADDRESS(AVE_ADDRESS) + 0x01D0)
#define AVE_PICR_OPTION		(IO_ADDRESS(AVE_ADDRESS) + 0x01D4)
#define AVE_PICR_SUCCESS	(IO_ADDRESS(AVE_ADDRESS) + 0x01D8)
#define AVE_PICR_CUR_IDX	(IO_ADDRESS(AVE_ADDRESS) + 0x01DC)
#define AVE_PICR_CONSUMED	(IO_ADDRESS(AVE_ADDRESS) + 0x01F4)

#define AVE_FRMC_BUF_NUM	(IO_ADDRESS(AVE_ADDRESS) + 0x0180)
#define AVE_FRMC_BUF_STRIDE	(IO_ADDRESS(AVE_ADDRESS) + 0x0184)
#define AVE_FRMC_SLC_START	(IO_ADDRESS(AVE_ADDRESS) + 0x0188)
#define AVE_FRMC_SLC_SIZE	(IO_ADDRESS(AVE_ADDRESS) + 0x018C)
#define AVE_FRMC_AXI_B_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0190)
#define AVE_FRMC_AXI_A_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0194)
#define AVE_FRMC_AXI_DY_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x0198)
#define AVE_FRMC_AXI_DC_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x019C)
#define AVE_FRMC_AXI_O_ADDR	(IO_ADDRESS(AVE_ADDRESS) + 0x01A0)

#define AVE_PARC_TYPE		(IO_ADDRESS(AVE_ADDRESS) + 0x0180)
#define AVE_PARC_SIZE		(IO_ADDRESS(AVE_ADDRESS) + 0x0184)

#define AVE_FLASHC_TYPE		(IO_ADDRESS(AVE_ADDRESS) + 0x0180)
#define AVE_FLASHC_RDPTR       	(IO_ADDRESS(AVE_ADDRESS) + 0x0184)

#define AVE_VERC_NUM		(IO_ADDRESS(AVE_ADDRESS) + 0x01C0)

#define SEQ_INIT	0x01
#define SEQ_END		0x02
#define PICTURE_RUN	0x03
#define SET_FRAME_BUF	0x04
#define DEC_PARA_SET	0x07
#define DEC_BUF_FLUSH	0x08
#define CMD_GETVER	0x0F

#define INT_SEQ_INIT	0x0002
#define INT_PIC_RUN	0x0008
#define INT_BUF_EMP	0x4000

#define PARAM_BUFSIZE	512

#define PV_SEQ_BUSY	0x200
#define PV_SWON		0x1
#define PV_PDON		0x100

#define AVE_RETENTION	0
#define AVE_POWERDOWN	1

#define PV_POWER_WAIT	20000000

enum ave_state_t {
	state_initial,
	state_idle,
	state_decoding,
};

struct ave_info_t {
	enum ave_state_t state;
	struct ave_interrupt_t interrupt;
	unsigned int int_data;
	struct ave_common_info_t *common_info;
	dma_addr_t pa_common_info;
	dma_addr_t pa_code_addr;
	void *v_code_addr;
	unsigned int fw_code_ver;
};

extern struct ave_info_t *ave_info;

#endif /* __avc_info_h__ */
