/*
 *  include/linux/zt2092.h
 *
 *  Copyright (C) 2010 Ingenic Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_ZT2092_H
#define _LINUX_ZT2092_H

#define ZT2092_I2C_NAME	"zt2092_ts"

#define	ZOOM_FAST_MODE		/* enable zoom in and out more faster */

#define	X_AXIS_MAX	3780
#define	X_AXIS_MIN	1540
#define	Y_AXIS_MAX	3799
#define	Y_AXIS_MIN	280
#define	PRESSURE_MAX	2000
#define	PRESSURE_MIN	500

#define	DATA_SIZE	25

#define	SCREEN_MAXX	320
#define	SCREEN_MAXY	480
#define	PRESS_MAXZ      256

#define	RX_PLATE	1024	/* Rx plate 442 ohm */
#define	RY_PLATE	362	/* Ry plate 362 ohm */
#define	R_THRESHOLD_HIGH	2200	/* Rtouch (TP Pressure)	*/
#define	R_THRESHOLD_LOW		500	/* Rtouch * 0.5(0.6)	*/

#define	R_MAX	0x7FFF

// for two points coord computing.
#if 0
#define Max_X_ADC	3800	
#define Max_Y_ADC	3811	
#define Min_X_ADC	1686	
#define Min_Y_ADC	393	
#define Min_DX2_ADC	2712	
#define Min_DY2_ADC	2360	
#endif

#define	Diff_Xref	120
#define	Diff_Yref	250 

/*
 * DUAL_THRESHOLD is used to tell whether two fingers is pressed or not,
 * the original value is 10, if this value set too large, when the two 
 * fingers get too close, these two fingers will be considered as one 
 * finger 
 */
#define	DUAL_THRESHOLD	10


#define	DIFF_TH		55	/* move_diff_threshold for single touch */
#define	STH		8	/* shift value threshold which shift between
				   two limits for dual touch(for zoom in and
				   zoom out) */
#define	LINE_LENGTH_TH	700	/* for single touch */
#define	LINE_WIDTH_TH	300
#define	PAN_LENGTH_TH	700	/* for pan gesture for dual touch */
#define	PAN_WIDTH_TH	300	/* for pan gesture for dual touch */
#define	JUDGEDATAMAX	3	/* for tow touch number of data to judgement */

#define	TAP_TH		2

#if defined(_NFequalEight_)
#define	FILTER_CNT	8
#else
#define	FILTER_CNT	6
#endif

enum zt2092_regs {
	ZT2092_REG_RESET	= 0x00,
	ZT2092_REG_SETUP	= 0x01,
	ZT2092_REG_SEQUENCE	= 0x02,
	ZT2092_REG_STATUS	= 0x10,
	ZT2092_REG_DATA		= 0x11,
	ZT2092_REG_DATA_X1	= 0x11,
	ZT2092_REG_DATA_Y1	= 0x13,
	ZT2092_REG_DATA_Z1	= 0x15,
	ZT2092_REG_DATA_Z2	= 0x17,
	ZT2092_REG_DATA_X2	= 0x19,
	ZT2092_REG_DATA_Y2	= 0x1B,
};

/*  setup command configuration */
enum zt2092_mode {
	ZT2092_MODE_NORMAL	= 0x00,
	ZT2092_MODE_SLEEP1	= 0x10,	/* PENIRQ = H */
	ZT2092_MODE_SLEEP2	= 0x20,	/* PENIRQ = Hi-Z */
};

/* sequence command configuration */
enum zt2092_sample_count {
	ZT2092_ADC_6_TIMES	= 0x00,
	ZT2092_ADC_10_TIMES	= 0x08,
};

/* sampling interval times */
enum zt2092_sample_interval {
	ZT2092_INTERVAL_0US	= 0x00,	/* 0 us */	
	ZT2092_INTERVAL_5US	= 0x01,	
	ZT2092_INTERVAL_10US	= 0x02,	
	ZT2092_INTERVAL_20US	= 0x03,	
	ZT2092_INTERVAL_50US	= 0x04,	
	ZT2092_INTERVAL_100US	= 0x05,	
	ZT2092_INTERVAL_200US	= 0x06,	
	ZT2092_INTERVAL_500US	= 0x07,	
};

/* sequence mode */
enum {
	ZT2092_SEQM_XYZ		= 0x00,
	ZT2092_SEQM_XY		= 0x10,
	ZT2092_SEQM_X		= 0x20,
	ZT2092_SEQM_Y		= 0x30,
	ZT2092_SEQM_Z		= 0x40,
	ZT2092_SEQM_X2		= 0x50,
	ZT2092_SEQM_ALL		= 0x60,
	ZT2092_SEQM_Y2		= 0x70,
};

/* gestures */
enum {
	TAP		= 0x01,
	RHORIZONTAL	= 0x02,
	LHORIZONTAL	= 0x03,
	UVERTICAL	= 0x04,
	DVERTICAL	= 0x05,
	RPAN		= 0x06,	/* Right pan */
	LPAN		= 0x07,
	UPAN		= 0x08,
	DPAN		= 0x09,
	CLOCKWISE	= 0x10,
	CCLOCKWISE	= 0x11,	/* counter-clockwise */
	ZOOMIN		= 0x12,
	ZOOMOUT		= 0x13,
	UNKNOWN		= 0x7f,
};

enum {
	ZERO_TOUCH	= 0,
	ONE_TOUCH	= 1,
	DUAL_TOUCH	= 2,
	DUAL_TOUCH_SF	= 3,
};

struct zt2092_platform_data {
	int intr;
};

#endif /* _LINUX_ZT2092_H */
