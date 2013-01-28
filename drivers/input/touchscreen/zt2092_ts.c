/*
 *  drivers/input/touchscreens/zt2092_ts.c
 *
 *  ZillTek ZT2092 touchscreen module driver
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

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/zt2092.h>
#include <linux/earlysuspend.h>

//#include <asm/gpio.h>
//#include <asm/uaccess.h>
//#include <asm/jzsoc.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

static u8 current_code;
static s16 x_limits = 2650, y_limits = 2471;
static struct i2c_client *this_client;
static struct zt2092_platform_data *pdata;

//static u16 org_x1, org_y1, org_x2, org_y2;
//static u16 scale_x, scale_y, w_x_adc, w_y_adc;
static int counter = 0;

struct ts_event {
	u8  status;
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 pressure;
};

struct zt2092_data {
	struct	input_dev	*input_dev;
	struct	mutex		lock;
	struct	ts_event	event;
	struct	delayed_work	ts_reader;
	struct	work_struct	pen_event_work;
	struct	early_suspend	early_suspend;
	struct	workqueue_struct *ts_workq;
	u8	touch_mode;
	u8	irq_pending;
};

#if defined(CONFIG_ZT2092_GESTURE)
struct coord_struct {
	u8	touch_mode;
	s16	coordx;
	s16	coordy;
	s16	lx2;
	s16	ly2;
};

struct coord_struct scoord[DATA_SIZE];
s8 scoord_idx = 0;
#endif

/* filter */
#define	LongtapFilterBuffer	1
#define	TempCoordBuffer		5

#define	X_Offset		6
#define	Y_Offset		6

struct ext_coord_struct {
	u8	touch_mode;
	s16	coordx;
	s16	coordy;
	s16	lx2;
	s16	ly2;
	s16	status;
};

struct ext_coord_struct  FilterCoord[LongtapFilterBuffer];
struct ext_coord_struct  TempBufCoord[TempCoordBuffer];

s16 Index_LongtapFilter = 0;
/************************************/

static int zt2092_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		printk(KERN_ERR "%s: i2c transfer error %d\n", __func__, ret);

	return ret;
}

static int zt2092_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		printk(KERN_ERR "%s: i2c transfer error %d\n", __func__, ret);

	return ret;
}

static u16 transform_to_screen_x(u16 x)
{
	if (x < X_AXIS_MIN) x = X_AXIS_MIN;
	if (x > X_AXIS_MAX) x = X_AXIS_MAX;
#if defined(CONFIG_TOUCHSCREEN_X_FLIP)
	return (X_AXIS_MAX - x) * SCREEN_MAXX / (X_AXIS_MAX - X_AXIS_MIN);
#else
	return (x - X_AXIS_MIN) * SCREEN_MAXX / (X_AXIS_MAX - X_AXIS_MIN);
#endif
}

static u16 transform_to_screen_y(u16 y)
{
	if (y < Y_AXIS_MIN) y = Y_AXIS_MIN;
	if (y > Y_AXIS_MAX) y = Y_AXIS_MAX;

#if defined(CONFIG_TOUCHSCREEN_Y_FLIP)
	return (y - Y_AXIS_MIN) * SCREEN_MAXY / (Y_AXIS_MAX - Y_AXIS_MIN);
#else
	return (Y_AXIS_MAX - y) * SCREEN_MAXY / (Y_AXIS_MAX - Y_AXIS_MIN);
#endif
}

static u16 transform_to_screen_z(u16 z)
{
	if (z < PRESSURE_MIN) z = PRESSURE_MIN;
	if (z > PRESSURE_MAX) z = PRESSURE_MAX;

	return (PRESSURE_MAX - z) * PRESS_MAXZ / (PRESSURE_MAX - PRESSURE_MIN);
}

/*
 * Noise Filter
 * get 6 or 8 elements
 * Drop the max and min value, average the other 4 values
 */
static short noise_filter_fast(short *datatemp)
{
	u8 i;
	s16 swaptemp;

#if !defined(_NFequalEight_)	
	for (i = 0; i < 5; i++) {
		if (datatemp[i] > datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}

	for (i = 0; i < 4; i++) {	
		if (datatemp[i] < datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}
#else
	for (i = 0; i < 7; i++) {
		if (datatemp[i] > datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}

	for (i = 0; i < 6; i++) {
		if (datatemp[i] > datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}

	for (i = 0; i < 5; i++) {	
		if (datatemp[i] < datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}

	for (i = 0; i < 4; i++) {	
		if (datatemp[i] < datatemp[i+1]) {
			swaptemp = datatemp[i];
			datatemp[i] = datatemp[i+1];
			datatemp[i+1] = swaptemp;
		}
	}
#endif

	swaptemp = 0;
	for (i = 0; i < 4; i++) {	
		swaptemp = swaptemp + datatemp[i];
	}

	swaptemp = swaptemp>>2;

	return swaptemp;
}

static int zt2092_get_xylimits(u8 *buf, s16 *xy_limits)
{
	u8 i;
	s16 temp[FILTER_CNT];
	int ret;

	for (i = 0; i < FILTER_CNT; i++) {

		ret = zt2092_i2c_txdata(buf, 3);
		if (ret < 0)
			return ret;

		do {
			buf[0] = ZT2092_REG_STATUS;
			ret = zt2092_i2c_rxdata(buf, 1);
			if (ret < 0)
				return ret;
		} while (buf[0] != 0x00);


		buf[0] = ZT2092_REG_DATA;
		ret = zt2092_i2c_rxdata(buf, 2);
		if (ret < 0)
			return ret;

		dev_dbg(&this_client->dev, "%s %d %d %#x %#x\n", __func__, __LINE__,
					ret, buf[0], buf[1]);
		temp[i] = (((s16)buf[0])<<4 | ((s16)buf[1])>>4);

	}
	*xy_limits = noise_filter_fast(temp);

	return 0;
}

#if defined(CONFIG_ZT2092_GESTURE)
#if defined(ZOOM_FAST_MODE)
static void remove_ahead_items(s16 first_judge_index)
{
	s16 i, count = 0;
	if (scoord_idx < 3)
		return;

	for (i = first_judge_index; i < scoord_idx; i++) {
		if (scoord[i].touch_mode == DUAL_TOUCH || 
				scoord[i].touch_mode == DUAL_TOUCH_SF) {
			scoord[count].touch_mode = scoord[i].touch_mode;
			scoord[count].coordx = scoord[i].coordx;
			scoord[count].coordy = scoord[i].coordy;
			scoord[count].lx2 = scoord[i].lx2;
			scoord[count].ly2 = scoord[i].ly2;
			count++;
		}
	}
	scoord_idx = count;
}
#endif

static void data_struct_flush(void)
{
	memset(scoord, 0, sizeof(*scoord) * scoord_idx);
	scoord_idx = 0;
}

static void remove_struct_item(void)
{
	scoord_idx--;
	scoord[scoord_idx].touch_mode = ZERO_TOUCH;
}

static void push_scoord(u8 touch_mode, s16 *x, s16 *y, s16 lx, s16 ly)
{
	if (scoord_idx >= DATA_SIZE)
		return;

	//printk("x: %d, y: %d, idx: %d\n", *x, *y, scoord_idx);
	scoord[scoord_idx].touch_mode = touch_mode;
	scoord[scoord_idx].coordx = *x;
	scoord[scoord_idx].coordy = *y;
	scoord[scoord_idx].lx2 = lx;
	scoord[scoord_idx].ly2 = ly;
	scoord_idx++;
}

/*
 * in coord array, the data flow will be saved in array as below
 * 0 represents zero touch
 * 1 represents single touch
 * 2 represents dual touch
 * following policy are defined which one will be saved
 *
 * * 111.....
 * * 222....
 * * 221....
 * * 112.. invalid
 */
static void check_scoord(u8 touch_mode, s16 *x, s16 *y, s16 lx, s16 ly)
{
	//u8 index_last_two;

	if (touch_mode == ZERO_TOUCH)
		return;

	if (scoord_idx >= DATA_SIZE) {
		if (touch_mode == DUAL_TOUCH &&
				scoord[DATA_SIZE - 1].touch_mode == ONE_TOUCH) {
			data_struct_flush();
			push_scoord(touch_mode, x, y, lx, ly);
		} else if (touch_mode == ONE_TOUCH) {
			remove_struct_item();
			push_scoord(touch_mode, x, y, lx, ly);
		}

		return;
	}

	/* inside array, 1 = one touch, 2 = dual touch, 3 = two shift touch
	 * if ary = {1, 1} + 1 then ary = {1, 1, 1}
	 * if ary = {1, 1} + 2 then flush, ary = {2}
	 * if ary = {2, 2} + 1 then ary = {2, 2, 1}
	 * if ary = {2, 2} + 2 then ary = {2, 2, 2}
	 */

	if (scoord_idx == 0) {
		push_scoord(touch_mode, x, y, lx, ly);
	} else if (touch_mode == DUAL_TOUCH &&
			scoord[scoord_idx - 1].touch_mode == ONE_TOUCH) {
		data_struct_flush();
		push_scoord(touch_mode, x, y, lx, ly);
	} else if (touch_mode == ONE_TOUCH &&
			(scoord[scoord_idx - 1].touch_mode == DUAL_TOUCH ||
			 scoord[scoord_idx - 1].touch_mode == DUAL_TOUCH_SF)) {
		push_scoord(touch_mode, x, y, lx, ly);
	} else {
		if (touch_mode == ONE_TOUCH) {
			if (abs(*x - scoord[scoord_idx - 1].coordx) >= DIFF_TH ||
					abs(*y - scoord[scoord_idx - 1].coordy) >= DIFF_TH) {
				push_scoord(touch_mode, x, y, lx, ly);
			}
		} else if (touch_mode == DUAL_TOUCH) {
			if (abs(lx - scoord[scoord_idx - 1].lx2) >= STH ||
					abs(ly - scoord[scoord_idx - 1].ly2) >= STH)
				push_scoord(touch_mode, x, y, lx, ly);
			else if (abs(*x - scoord[scoord_idx - 1].coordx) >= DIFF_TH ||
					abs(*y - scoord[scoord_idx - 1].coordy) >= DIFF_TH)
				push_scoord(DUAL_TOUCH_SF, x, y, lx, ly);
		}
	}
}

/*
 * return the clockwise status of a curve through three points,
 * the cross product of vector AB and AC is positive indicates
 * counter clockwise, negetive indicates clockwise
 * vector AB: (dx1, dy1) = (x2 - x1, y2 - y1)
 * vector AC: (dx2, dy2) = (x3 - x1, y3 - y1)
 * |dx1 dy1|
 * |dx2 dy2|
 * i.e. dx1*dy2 - dy1*dx2
 */
static int zt2092_clockwise(struct coord_struct *p, u8 n)
{
	u8 i, j, k;
	s8 count = 0;
	int z;

	//for (i = 0; i < n; i++)
	//	printk("data[%d] x: %d, y: %d\n", i, p[i].coordx, p[i].coordy);

	if (n < 3)
		return 0;
	n -= 2;

	for (i = 0; i < n; i++) {
		j = i + 1;
		k = i + 2;
		z = (p[j].coordx - p[i].coordx) * (p[k].coordy - p[j].coordy);
		z -= (p[j].coordy - p[i].coordy) * (p[k].coordx - p[j].coordx);

		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}

	if (count > 0)
		return CCLOCKWISE;
		//printk("counter clockwise\n");
	else if (count < 0)
		return CLOCKWISE;
		//printk("clockwise\n");
	else
		return UNKNOWN;
		//printk("unknown gesture!\n");

	return 0;
}

static u8 zt2092_get_gesture(void)
{
	u8 i;
	u8 index;
	u8 max_xid = 0, min_xid = 0;
	u8 max_yid = 0, min_yid = 0;
	s16 dx, dy, dlx, dly, adlx, adly;

	/* first one should be one touch mode */
	if (scoord_idx == 0 || scoord[0].touch_mode != ONE_TOUCH)
		return UNKNOWN;

	/*
	 * find structs, if there are only one touch and then sort X/Y
	 * if not only one touch, return unknow
	 */
	for (i = 1; i < scoord_idx; i++) {
		if (scoord[i].touch_mode != ONE_TOUCH)
			return UNKNOWN;
	}

	if (scoord_idx < TAP_TH)
		return TAP;

	index = scoord_idx;

	for (i = 1; i < index; i++) {
		if (scoord[i].coordx > scoord[max_xid].coordx)
			max_xid = i;
		if (scoord[i].coordy > scoord[max_yid].coordy)
			max_yid = i;
		if (scoord[i].coordx < scoord[min_xid].coordx)
			min_xid = i;
		if (scoord[i].coordy < scoord[min_yid].coordy)
			min_yid = i;
	}

	if (index < 3)
		return UNKNOWN;

	dx = scoord[max_xid].coordx - scoord[min_xid].coordx;
	dy = scoord[max_yid].coordy - scoord[min_yid].coordy;
	dlx = scoord[0].coordx - scoord[index - 1].coordx;
	dly = scoord[0].coordy - scoord[index - 1].coordy;
	adlx = abs(dlx);
	adly = abs(dly);

	if (dx > LINE_LENGTH_TH && dy < LINE_WIDTH_TH
		&& adlx > LINE_LENGTH_TH && adly < LINE_WIDTH_TH) {
		if (dlx > 0)
			return LHORIZONTAL;
		else
			return RHORIZONTAL;
	} else if (dy > LINE_LENGTH_TH && dx < LINE_WIDTH_TH
			&& adly > LINE_LENGTH_TH && adlx < LINE_WIDTH_TH) {
		if (dly > 0)
			return DVERTICAL;
		else
			return UVERTICAL;
	}

	if (index >= 3)
		return zt2092_clockwise(scoord, index);

	return UNKNOWN;
}

static u8 zt2092_get_gesture_runtime(void)
{
	u16 i;
	u16 zoomin = 0, zoomout = 0;
	u16 index1, index2;
	s16 dx, dy, adx, ady;

#if defined(ZOOM_FAST_MODE)
	s8 judge_index, first_judge_flag;
#endif

	if (scoord_idx == 0)
		return UNKNOWN;
	if (scoord_idx < JUDGEDATAMAX || scoord[0].touch_mode == ONE_TOUCH)
		return UNKNOWN;

	/*
	 * find mode is two touch for coord data ignoring DUAL_TOUCH_SF mode
	 * if find another mode, just return UNKNOWN
	 */

	for ( i = 1; i < scoord_idx; i++) {
		if (scoord[i].touch_mode == DUAL_TOUCH ||
				scoord[i].touch_mode == DUAL_TOUCH_SF) {
			dx = scoord[i].lx2 - scoord[index1].lx2;
			dy = scoord[i].ly2 - scoord[index1].ly2;
			adx = scoord[index1].lx2 - scoord[i].lx2;
			ady = scoord[index1].ly2 - scoord[i].ly2;

			if (dx >= STH || dy >= STH) {
				zoomout++;
				index1 = i;
#if defined(ZOOM_FAST_MODE)
				if (first_judge_flag == 0) {
					judge_index = i;
					first_judge_flag = 1;
				}
#endif
			} else if (adx >= STH || ady >=STH) {
				zoomin++;
				index1 = i;
#if defined(ZOOM_FAST_MODE)
				if (first_judge_flag == 0) {
					judge_index = i;
					first_judge_flag = 1;
				}
#endif
			} else {
				break;
			}
		}
	} /* end of for */

#if defined(ZOOM_FAST_MODE)
	if (zoomin > zoomout && (zoomin -zoomout) >= (JUDGEDATAMAX - 1)) {
		remove_ahead_items(judge_index);
		first_judge_flag = 0;
		printk("zoom in\n");
		return ZOOMIN;
	} else if (zoomout > zoomin && (zoomout - zoomin) >= (JUDGEDATAMAX - 1)) {
		remove_ahead_items(judge_index);
		first_judge_flag = 0;
		printk("zoom out\n");
		return ZOOMOUT;
	}
#else
	if (zoomin > zoomout && (zoomin -zoomout) >= (JUDGEDATAMAX - 1)) {
		data_struct_flush();
		printk("zoom in\n");
		return ZOOMIN;
	} else if (zoomout > zoomin && (zoomout - zoomin) >= (JUDGEDATAMAX - 1)) {
		data_struct_flush();
		printk("zoom out\n");
		return ZOOMOUT;
	}
#endif

	/* find last DUAL_TOUCH_SF index */
	for (i = 1; i < scoord_idx; i++) {
		if (scoord[i].touch_mode == DUAL_TOUCH_SF)
			index2 = i;
	}

	dx = scoord[0].coordx - scoord[index2].coordx;
	dy = scoord[0].coordy - scoord[index2].coordy;
	adx = abs(dx);
	ady = abs(dy);

	if (adx > ady && adx > PAN_LENGTH_TH && ady < PAN_WIDTH_TH) {
		if (dx > 0)
			return LPAN;
		else
			return RPAN;
	} else if (ady > adx && ady > PAN_LENGTH_TH && adx < PAN_WIDTH_TH) {
		if (dy > 0)
			return DPAN;
		else
			return UPAN;
	}

	return UNKNOWN;
}
#else
static u8 zt2092_get_gesture_runtime(void)
{
	return 0;
}

static void Push_TempBufCoord(s8 touch_mode, s16 x, s16 y, s16 lx2, s16 ly2, s16 status)
{
	TempBufCoord[0].touch_mode = touch_mode;
	TempBufCoord[0].coordx = x;
	TempBufCoord[0].coordy = y;
	TempBufCoord[0].lx2 = lx2;
	TempBufCoord[0].ly2 = ly2;
	TempBufCoord[0].status = status;
}
static s8 GetLastTempBufTouchMode(void)
{
	return TempBufCoord[0].touch_mode;
}

static void GetLastTempBufData(s16 *x, s16 *y, s16 *lx2, s16 *ly2, s16 *status)
{
	*x = TempBufCoord[0].coordx;
	*y = TempBufCoord[0].coordy;
	*lx2 = TempBufCoord[0].lx2;
	*ly2 = TempBufCoord[0].ly2;
	*status = TempBufCoord[0].status;
}

static void Temp_Buffer_Struct_Flush(void)
{
	memset(TempBufCoord, 0, sizeof(struct ext_coord_struct) * 1);
	Index_LongtapFilter = 0;
}

#if 0 // can work right
static void GetTwoPoints( s16 *x1, s16 *y1,s16 *x2, s16 *y2, s16 x2_ref, s16 y2_ref)
{
		s16 org_x1, org_y1, org_x2, org_y2;
		s16 scale_x, scale_y;
		s16 half_width_x_adc, half_width_y_adc;
		s16 half_dx_axis, half_dy_axis;
		s16 center_x_adc, center_y_adc;
	s16 Min_DX2_ADC;
	s16 Min_DY2_ADC;
	
		printk("ref: %d %d\n", x2_ref, y2_ref);
		
		/***********************************************************************************/
		//X_AXIS_MAX	: max adc values of x axis.
		//Y_AXIS_MAX	: max adc values of y axis.
		//X_AXIS_MIN	: min adc values of x axis.
		//Y_AXIS_MIN	: min adc values of y axis.
		//Diff_Xref	: in dual-touch, differential between max x2 ref(xlimits) and min x2 ref.
		//Diff_Yref	: in dual-touch, differential between max y2 ref(xlimits) and min y2 ref.
		/******************************************************************************/
		half_dx_axis = (X_AXIS_MAX - X_AXIS_MIN)>>1;
		half_dy_axis = (Y_AXIS_MAX - Y_AXIS_MIN)>>1;
		scale_x = (half_dx_axis) * 10 / Diff_Xref;  
		scale_y = (half_dy_axis) * 10 / Diff_Yref;		
		//scale_x = ((Max_X_ADC - Min_X_ADC)>>1)*10/(x_limits - Min_DX2_ADC);
		//scale_y = ((Max_Y_ADC - Min_Y_ADC)>>1)*10/(y_limits - Min_DY2_ADC);
		/******************************************************************************/


		/******************************************************************************/
		//Min_DX2_ADC		: in dual-touch, values of min x2 ref.
		//Min_DY2_ADC		: in dual-touch, values of min y2 ref.
		Min_DX2_ADC = x_limits - Diff_Xref;
		Min_DY2_ADC = y_limits - Diff_Yref;
		/******************************************************************************/

		if (x2_ref < Min_DX2_ADC)
			half_width_x_adc = 0; //width of x adc
		else
			half_width_x_adc = (x2_ref - Min_DX2_ADC) * scale_x / 10; //width of x adc

		if (y2_ref < Min_DY2_ADC)
			half_width_y_adc = 0;//width of y adc
		else
			half_width_y_adc = (y2_ref - Min_DY2_ADC) * scale_y / 10; //width of y adc
			
		// may cause zoom-in, zoom-out reversed.
		org_x1 = half_width_x_adc + X_AXIS_MIN;
		org_y1 = half_width_y_adc + Y_AXIS_MIN;

		org_x2 = X_AXIS_MAX - half_width_x_adc;
		org_y2 = Y_AXIS_MAX - half_width_y_adc;


		dev_err(&this_client->dev, "-----------org_x1:%d  org_y1:%d\n", org_x1, org_y1);   
		dev_err(&this_client->dev, "-----------org_x2:%d  org_y2:%d\n", org_x2, org_y2);
	
		*x1 = org_x1;
		*y1 = org_y1;
		*x2 = org_x2;
		*y2 = org_y2;
}
#else

static void GetTwoPoints( s16 *x1, s16 *y1,s16 *x2, s16 *y2, s16 x2_ref, s16 y2_ref)
{
		s16 org_x1, org_y1, org_x2, org_y2;
		s16 scale_x, scale_y;
		s16 half_width_x_adc, half_width_y_adc;
		s16 half_dx_axis, half_dy_axis;
		//s16 center_x_adc, center_y_adc;
		s16 Min_DX2_ADC;
		s16 Min_DY2_ADC;
//****************************************************
		s16 tx1,ty1,tx2,ty2,status;
		//s16 half_dx_axis, half_dy_axis;
		s16 center_x_adc, center_y_adc;
//****************************************************

		//printk("ref: %d %d\n", x2_ref, y2_ref);
		
		/*******************************************************************************/
		//X_AXIS_MAX	: max adc values of x axis.
		//Y_AXIS_MAX	: max adc values of y axis.
		//X_AXIS_MIN	: min adc values of x axis.
		//Y_AXIS_MIN	: min adc values of y axis.
		//Diff_Xref	: in dual-touch, differential between max x2 ref(xlimits) and min x2 ref.
		//Diff_Yref	: in dual-touch, differential between max y2 ref(xlimits) and min y2 ref.
		/*******************************************************************************/
		half_dx_axis = (X_AXIS_MAX - X_AXIS_MIN)>>1;
		half_dy_axis = (Y_AXIS_MAX - Y_AXIS_MIN)>>1;
		scale_x = (half_dx_axis) * 10 / Diff_Xref;  
		scale_y = (half_dy_axis) * 10 / Diff_Yref;		
		//scale_x = ((Max_X_ADC - Min_X_ADC)>>1)*10/(x_limits - Min_DX2_ADC);
		//scale_y = ((Max_Y_ADC - Min_Y_ADC)>>1)*10/(y_limits - Min_DY2_ADC);
		/*******************************************************************************/


		/*******************************************************************************/
		//Min_DX2_ADC		: in dual-touch, values of min x2 ref.
		//Min_DY2_ADC		: in dual-touch, values of min y2 ref.
		Min_DX2_ADC = x_limits - Diff_Xref;
		Min_DY2_ADC = y_limits - Diff_Yref;
		/*******************************************************************************/

		if (x2_ref < Min_DX2_ADC)
			half_width_x_adc = 0; //width of x adc
		else
			half_width_x_adc = (x2_ref - Min_DX2_ADC) * scale_x / 10; //width of x adc

		if (y2_ref < Min_DY2_ADC)
			half_width_y_adc = 0;//width of y adc
		else
			half_width_y_adc = (y2_ref - Min_DY2_ADC) * scale_y / 10; //width of y adc

    // Touch Panel ADC coord
    //   +-------------------------------------+
    //   | max_x,max_y                         |             
    //   |                                     |
    //   |       (3)            (4)            |
    //   |                                     |
    //   |                + center x,y         |          y
    //   |                                     |          ^
    //   |       (2)            (1)            |          | 
    //   |                                     |          |   
    //   |                          min_x,min_y|          |   
    //   +-------------------------------------+     x<---+                         
    //
    //status
    // 1 : first point at min_x and min_y, second point at max_x and max_y.(\)
    // 2 : first point at max_x and min_y, second point at min_x and max_y.(/)
    // 3 : first point at max_x and max_y, second point at min_x and min_y.(\)
    // 4 : first point at min_x and max_y, second point at max_x and min_y.(/)

		GetLastTempBufData(&tx1,&ty1,&tx2,&ty2,&status);

		if (GetLastTempBufTouchMode() == ONE_TOUCH) {

			//half_dx_axis = (X_AXIS_MAX - X_AXIS_MIN)>>1;
			//half_dy_axis = (Y_AXIS_MAX - Y_AXIS_MIN)>>1;
			center_x_adc = half_dx_axis + X_AXIS_MIN;
			center_y_adc = half_dy_axis + Y_AXIS_MIN;
				
				if(tx1 <= center_x_adc) {
					if (ty1 <= center_y_adc) {
						status = 1;
					}else{
						status = 4;
					}
				}else{
					if (ty1 <= center_y_adc) {
						status = 2;
					} else {
						status = 3;
					}
				}//else-if( tx1 <= center_x_adc)
				//printk("^^^^^^^^^ status: %d \n", status);
		} else if (GetLastTempBufTouchMode() == 0) {
			status = 1;
		}

		switch(status) {
		case 1:
			org_x1 = half_width_x_adc + X_AXIS_MIN;
			org_y1 = half_width_y_adc + Y_AXIS_MIN;
			org_x2 = X_AXIS_MAX - half_width_x_adc;
			org_y2 = Y_AXIS_MAX - half_width_y_adc;
			break;
		case 2:
			org_x1 = X_AXIS_MAX - half_width_x_adc;
			org_y1 = half_width_y_adc + Y_AXIS_MIN;
			org_x2 = half_width_x_adc + X_AXIS_MIN;
			org_y2 = Y_AXIS_MAX - half_width_y_adc;
			break;
		case 3:
			org_x1 = X_AXIS_MAX - half_width_x_adc;
			org_y1 = Y_AXIS_MAX - half_width_y_adc;
			org_x2 = half_width_x_adc + X_AXIS_MIN;
			org_y2 = half_width_y_adc + Y_AXIS_MIN;
			break;
		case 4:
			org_x1 = half_width_x_adc + X_AXIS_MIN;
			org_y1 = Y_AXIS_MAX - half_width_y_adc;
			org_x2 = X_AXIS_MAX - half_width_x_adc;
			org_y2 = half_width_y_adc + Y_AXIS_MIN;
			break;
		}
		dev_dbg(&this_client->dev, "-----------org_x1:%d  org_y1:%d, org_x2:%d  org_y2:%d\n", org_x1, org_y1, org_x2, org_y2);   

#if 1
		if (GetLastTempBufTouchMode() != 0) {
			org_x2 = org_x2 - (org_x1 - tx1);
			org_y2 = org_y2 - (org_y1 - ty1);
			org_x1 = tx1;
			org_y1 = ty1;
			dev_dbg(&this_client->dev, "------ONE_TOUCH---org_x1:%d  org_y1:%d, org_x2:%d  org_y2:%d\n", org_x1, org_y1, org_x2, org_y2);   
		}
#endif

		Push_TempBufCoord(DUAL_TOUCH, org_x1, org_y1, org_x2, org_y2, status);

		*x1 = org_x1;
		*y1 = org_y1;
		*x2 = org_x2;
		*y2 = org_y2;
}
#endif

#if 0
static void Translate(s16 *x1, s16 *y1,s16 *x2, s16 *y2, s16 x2_ref, s16 y2_ref)
{
		s16 half_dx_axis, half_dy_axis;
		s16 center_x_adc, center_y_adc;

		s16 new_width_x;
		s16 new_width_y;
		s16 tx1,ty1,tx2,ty2,status;
		s16 twx,twy;
		
		GetLastTempBufData(&tx1,&ty1,&tx2,&ty2,&status);
		
		if(status == -1 )
		{
			return;
		}
	
		new_width_x = *x2 - *x1;
		new_width_y = *y2 - *y1;		
		new_width_x = (new_width_x < 0)? (- new_width_x) : new_width_x;
		new_width_y = (new_width_y < 0)? (- new_width_y) : new_width_y;
	
		
		
		twx = tx2 - tx1;
		twy = ty2 - ty1;		
		twx = (twx < 0)? (- twx) : twx;
		twy = (twy < 0)? (- twy) : twy;
		//if(tx1 == x1 && ty1 == y1 && tx2 == x2 && ty2 == y2){
		if(twx == new_width_x && twy == new_width_y){
			return;
		}

    // Touch Panel ADC coord
    //   +-------------------------------------+
    //   | max_x,max_y                         |             
    //   |                                     |
    //   |       (3)            (4)            |
    //   |                                     |
    //   |                + center x,y         |          y
    //   |                                     |          ^
    //   |       (2)            (1)            |          |  
    //   |                                     |          |   
    //   |                          min_x,min_y|          |   
    //   +-------------------------------------+     x<---+                         
    //
    //status
    // 1 : first point at min_x and min_y, second point at max_x and max_y.(\)
    // 2 : first point at max_x and min_y, second point at min_x and max_y.(/)
    // 3 : first point at max_x and max_y, second point at min_x and min_y.(\)
    // 4 : first point at min_x and max_y, second point at max_x and min_y.(/)
		
		if (GetLastTempBufTouchMode() == ONE_TOUCH) {


			half_dx_axis = (X_AXIS_MAX - X_AXIS_MIN)>>1;
			half_dy_axis = (Y_AXIS_MAX - Y_AXIS_MIN)>>1;
			center_x_adc = half_dx_axis + X_AXIS_MIN;
			center_y_adc = half_dy_axis + Y_AXIS_MIN;
				
				if( tx1 <= center_x_adc){
					tx2 = new_width_x + tx1;
					if (ty1 <= center_y_adc){
						status = 1;
						ty2 = new_width_y + ty1;
					}else{
						status = 4;
						ty2 = ty1 - new_width_y;
					}
				}else{
					tx2 = tx1 - new_width_x;
					if (ty1 <= center_y_adc){
						status = 2;
						ty2 = new_width_y + ty1;
					}else{
						status = 3;
						ty2 = ty1 - new_width_y;
					}
				}//else-if( tx1 <= center_x_adc)
		}else if (GetLastTempBufTouchMode() == DUAL_TOUCH){
			twx = tx1;
			twy = ty1;
			center_x_adc = tx2;
			center_y_adc = ty2;
			switch(status){
				case 1: 
						tx1 =  center_x_adc - (twx >>1);
						ty1 =  center_y_adc - (twy >>1);
						tx2 = twx + tx1;
						ty2 = twy + ty1;
						break;
				case 2:
						tx1 =  center_x_adc + (twx >>1);
						ty1 =  center_y_adc - (twy >>1);
						tx2 = tx1 - twx;
						ty2 = twy + ty1;
						break;
				case 3:
						tx1 =  center_x_adc + (twx >>1);
						ty1 =  center_y_adc + (twy >>1);
						tx2 = tx1 - twx;
						ty2 = ty1 - twy;
						break;
				case 4:
						tx1 =  center_x_adc - (twx >>1);
						ty1 =  center_y_adc + (twy >>1);
						tx2 = twx + tx1;
						ty2 = ty1 - twy;
						break;
				
			}//switch()
			
			
		}//else-if (GetLastTempBufTouchMode() == ONE_TOUCH)


		//Push_TempBufCoord(DUAL_TOUCH, tx1, ty1, tx2, ty2, status);
		Push_TempBufCoord(DUAL_TOUCH, new_width_x, new_width_y, center_x_adc, center_y_adc, status);
		
		*x1 = tx1;
		*y1 = ty1;
		*x2 = tx2;
		*y2 = ty2;
}
#endif

static void Push_FilterCoord(s16 *x, s16 *y)
{
	FilterCoord[0].coordx = *x;
	FilterCoord[0].coordy = *y;
	Index_LongtapFilter++;
}

static void Filter_Coord_Struct_Flush(void)
{
	memset(FilterCoord, 0, sizeof(struct ext_coord_struct) * 1);
	Index_LongtapFilter = 0;
}

static void Touch_Panel_Longtap_Filter(s16 *x, s16 *y)
{
	s16 x_diff, y_diff;

	if (Index_LongtapFilter == 0) {
		Push_FilterCoord(x, y);
		return;
	}

	//=== without using abs() function ====//	
	if (*x>FilterCoord[0].coordx)   
		x_diff = *x-FilterCoord[0].coordx;
	else
		x_diff = FilterCoord[0].coordx-*x;    

	if (*y>FilterCoord[0].coordy)    
		y_diff = *y-FilterCoord[0].coordy;
	else
		y_diff = FilterCoord[0].coordy-*y; 


	if (x_diff > X_Offset || y_diff > Y_Offset) {
		Filter_Coord_Struct_Flush();
		Push_FilterCoord(x, y);
	} else {
		*x = FilterCoord[0].coordx;
		*y = FilterCoord[0].coordy;
	}
}
#endif

static int zt2092_startup(void)
{
	u8 buf[3];
	s16 x, y;
	int ret;

	/* issue dummy command */
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	ret = zt2092_i2c_txdata(buf, 2);
	if (ret < 0)
		return ret;

	/* issue reset command */
	buf[0] = ZT2092_REG_RESET;
	buf[1] = 0x1;
	ret = zt2092_i2c_txdata(buf, 2);
	if (ret < 0)
		return ret;

	/*
	 * issue setup command
	 * zt2092 could be disable by sending 0x11 to SETUP register
	 */
	buf[0] = ZT2092_REG_SETUP;
	buf[1] = ZT2092_MODE_NORMAL;
	ret = zt2092_i2c_txdata(buf, 2);
	if (ret < 0)
		return ret;

	buf[0] = ZT2092_REG_SEQUENCE;
	buf[1] = ZT2092_SEQM_X2 | ZT2092_ADC_10_TIMES | ZT2092_INTERVAL_0US;
	buf[2] = 0x04;
	zt2092_get_xylimits(buf, &x);

	buf[0] = ZT2092_REG_SEQUENCE;
	buf[1] = ZT2092_SEQM_Y2 | ZT2092_ADC_10_TIMES | ZT2092_INTERVAL_0US;
	buf[2] = 0x04;
	zt2092_get_xylimits(buf, &y);

	dev_dbg(&this_client->dev, "xylimits: %d %d\n\n", x, y);
	if (x > x_limits || y > y_limits) {
		x_limits = x;
		y_limits = y;
	}

	return 0;
}

static int zt2092_get_touch_mode(s16 x2_ref, s16 y2_ref, s16 pressure)
{
	u8 touch_mode;

	if ((x_limits - x2_ref) >= DUAL_THRESHOLD || (y_limits - y2_ref) >= DUAL_THRESHOLD)
		touch_mode = DUAL_TOUCH;
	else if ((x_limits - x2_ref) < DUAL_THRESHOLD && (y_limits - y2_ref) < DUAL_THRESHOLD)
		touch_mode = ONE_TOUCH;
	else
		touch_mode = ONE_TOUCH;
		//touch_mode = ZERO_TOUCH;

	if (touch_mode == DUAL_TOUCH && pressure <= R_THRESHOLD_LOW)
		touch_mode = DUAL_TOUCH;
	else if (touch_mode == ONE_TOUCH &&  pressure < R_THRESHOLD_HIGH)
		touch_mode = ONE_TOUCH;
	else
		touch_mode = ZERO_TOUCH;

	if (pressure == 0 || pressure == R_MAX)
		touch_mode = ZERO_TOUCH;

	return touch_mode;
}

static int zt2092_read_data(short *rbuf)
{
  struct zt2092_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	s16 x, y, z1, z2, pressure;
	s16 x2_ref, y2_ref;
	u8 buf[12];

	int ret;
	s16 org_x1, org_y1, org_x2, org_y2;

	buf[0] = ZT2092_REG_SEQUENCE;
	buf[1] = ZT2092_SEQM_ALL | ZT2092_ADC_10_TIMES | ZT2092_INTERVAL_0US;
	buf[2] = 0x04;
	ret = zt2092_i2c_txdata(buf, 3);
	if (ret < 0)
		return ret;

	do {
		buf[0] = ZT2092_REG_STATUS;
		ret = zt2092_i2c_rxdata(buf, 1);
		if (ret < 0)
			return ret;
	} while (buf[0] != 0x00);

	buf[0] = ZT2092_REG_DATA;
	ret = zt2092_i2c_rxdata(buf, 12);
	if (ret < 0)
		return ret;

	x = (((s16)buf[0])<<4 | ((s16)buf[1])>>4);
	y = (((s16)buf[2])<<4 | ((s16)buf[3])>>4);
	z1 = (((s16)buf[4])<<4 | ((s16)buf[5])>>4);
	z2 = (((s16)buf[6])<<4 | ((s16)buf[7])>>4);
	
	x2_ref =(((s16)buf[8])<<4 | ((s16)buf[9])>>4);
	y2_ref =(((s16)buf[10])<<4 | ((s16)buf[11])>>4);

	if ((z1 > 0) && (z2 > z1)) {
		/* Rx*x/4096*(z2/z1 - 1), assume Rx=1024 */
		pressure = ((x)*(z2-z1)/z1) >> 2;
	} else {
		pressure = R_MAX;
	}

	if ((pressure > R_THRESHOLD_HIGH) || pressure == 0) {
		dev_dbg(&this_client->dev, "data invalid, pressure: %d\n", pressure);
		return -EIO;
	}
	event->pressure = transform_to_screen_z(pressure);

	data->touch_mode = zt2092_get_touch_mode(x2_ref, y2_ref, pressure);
	//printk("raw data: %d %d, ref: %d %d, mode: %d, pressure: %d\n",
	//		x, y, x2_ref, y2_ref, data->touch_mode, pressure);

#if defined(CONFIG_ZT2092_GESTURE)
	if (data->touch_mode != ZERO_TOUCH)
		check_scoord(data->touch_mode, &x, &y, x2_ref, y2_ref);
#endif

	if (data->touch_mode == ZERO_TOUCH) {
		return -EIO;
	} else if (data->touch_mode == ONE_TOUCH) {
		Push_TempBufCoord(data->touch_mode, x, y, x, y, 0);
		//if ((x > 300) && (x < 1300)) {
			event->x1 = transform_to_screen_x(x);
			event->y1 = transform_to_screen_y(y);
		//}
		//printk("x: %d, y: %d\n", event->x1, event->y1);
	} else if (data->touch_mode == DUAL_TOUCH) {
		if(Index_LongtapFilter == 0 && (counter++ < 1)) {
			//printk("############# counter: %d\n", counter);
			return -EIO;
		}

		Touch_Panel_Longtap_Filter(&x2_ref, &y2_ref);
		GetTwoPoints(&org_x1, &org_y1, &org_x2, &org_y2, x2_ref, y2_ref);
		//printk("filter ref: %d %d. org_x1:%d  org_y1:%d  org_x2:%d  org_y2:%d \n", x2_ref, y2_ref, org_x1, org_y1, org_x2, org_y2);

		//printk("translate  org_x1:%d org_y1:%d  org_x2:%d  org_y2:%d \n",  org_x1, org_y1, org_x2, org_y2);
		
/*
		printk("ref: %d %d\n", x2_ref, y2_ref);
		scale_x = ((X_AXIS_MAX - X_AXIS_MIN)>>1) * 10 / Diff_Xref;
		scale_y = ((Y_AXIS_MAX - Y_AXIS_MIN)>>1) * 10 / Diff_Yref;		

		Min_DX2_ADC = x_limits - Diff_Xref;
		Min_DY2_ADC = y_limits - Diff_Yref;

		if (x2_ref < Min_DX2_ADC)
			w_x_adc = 0; //width of x adc
		else
			w_x_adc = (x2_ref - Min_DX2_ADC) * scale_x / 10; //width of x adc

		if (y2_ref < Min_DY2_ADC)
			w_y_adc = 0;
		else
			w_y_adc = (y2_ref - Min_DY2_ADC) * scale_y / 10; //width of y adc

		org_x1 = w_x_adc + X_AXIS_MIN;
		org_y1 = w_y_adc + Y_AXIS_MIN;

		org_x2 = X_AXIS_MAX - w_x_adc;
		org_y2 = Y_AXIS_MAX - w_y_adc;

		dev_err(&this_client->dev, "-----------org_x1:%d  org_y1:%d\n", org_x1, org_y1);   
		dev_err(&this_client->dev, "-----------org_x2:%d  org_y2:%d\n", org_x2, org_y2);
*/

#if 1
		event->x1 = transform_to_screen_x((u16)org_x1);
		event->y1 = transform_to_screen_y((u16)org_y1);
		event->x2 = transform_to_screen_x((u16)org_x2);
		event->y2 = transform_to_screen_y((u16)org_y2);
#else
		event->x1 = (u16)org_x1;
		event->y1 = (u16)org_y1;
		event->x2 = (u16)org_x2;
		event->y2 = (u16)org_y2;
#endif
	}

	//printk(KERN_DEBUG "x1:%d y1:%d x2:%d y2:%d pressure: %d z1:%d z2:%d\n",
	//	event->x1, event->y1, event->x2, event->y2, event->pressure, z1, z2);

	return 0;
}

static void zt2092_ts_release(void)
{
        struct zt2092_data *data = i2c_get_clientdata(this_client);
	
#ifdef CONFIG_ZT2092_MULTITOUCH
	if (data->event.x1 == 0) {
		input_report_key(data->input_dev, current_code, 0);
	} else {
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(data->input_dev);
	}
#else
	if (data->event.x1 == 0) {
		input_report_key(data->input_dev, current_code, 0);
	} else {
		input_report_abs(data->input_dev, ABS_PRESSURE, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	}
#endif
	input_sync(data->input_dev);
}

static void zt2092_report_value(short *rbuf)
{
	struct zt2092_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;    
	u8 touch_mode = data->touch_mode;
	u16 x, y;

	x = event->x1;
	y = event->y1;

#ifdef CONFIG_ZT2092_MULTITOUCH
	if (x == 0) {
		/* menu, code 0x8b */
		if ((y > 12) && (y < 88)) {
			input_report_key(data->input_dev, KEY_MENU, 1);
			current_code = KEY_MENU;
		}

		/* home, code 0x66 */
		if ((y > 200) && (y < 270)) {
			input_report_key(data->input_dev, KEY_HOME, 1);
			current_code = KEY_HOME;
		}

		/* back, code 0x9e */
		if ((y > 385) && (y < 455)) {
			input_report_key(data->input_dev, KEY_BACK, 1);
			current_code = KEY_BACK;
		}
	} else {
		if (touch_mode == 1) {
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		} else if (touch_mode == 2) {
			dev_dbg(&this_client->dev, "the second finger\n");
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);	

			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);	
		}
	}
	input_sync(data->input_dev);
#else
	if (x == 0) {
		/* menu, code 0x8b */
		if ((y > 12) && (y < 88)) {
			input_report_key(data->input_dev, KEY_MENU, 1);
			current_code = KEY_MENU;
		}

		/* home, code 0x66 */
		if ((y > 200) && (y < 270)) {
			input_report_key(data->input_dev, KEY_HOME, 1);
			current_code = KEY_HOME;
		}

		/* back, code 0x9e */
		if ((y > 385) && (y < 455)) {
			input_report_key(data->input_dev, KEY_BACK, 1);
			current_code = KEY_BACK;
		}
	} else {
		if (touch_mode == 1) {
			input_report_abs(data->input_dev, ABS_X, event->x1);
			input_report_abs(data->input_dev, ABS_Y, event->y1);
			input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}
	input_sync(data->input_dev);
#endif /* CONFIG_ZT2092_MULTITOUCH */
}

static void zt2092_ts_reader(struct work_struct *work)
{
        struct zt2092_data *zt2092 = container_of(work, struct zt2092_data, ts_reader.work);
	int ret = -1;
	short buf[3];

	ret = zt2092_read_data(buf);
	if (ret < 0) {
		dev_dbg(&this_client->dev, "%s get data failed, error %d\n", __func__, ret);
	} else {
		zt2092_get_gesture_runtime();
		zt2092_report_value(buf);
	}
	//if (!__gpio_get_pin(pdata->intr))
        if (!gpio_get_value(pdata->intr)) {
		queue_delayed_work(zt2092->ts_workq, &zt2092->ts_reader, 1);
        }
}

static void zt2092_pen_irq_work(struct work_struct *work)
{
        struct zt2092_data *zt2092 = container_of(work, struct zt2092_data, pen_event_work);

	if (zt2092->irq_pending) {
		zt2092->irq_pending = 0;
		//enable_irq(this_client->irq);
	}

	//if (!__gpio_get_pin(pdata->intr)) {
        if (!gpio_get_value(pdata->intr)) {
		queue_delayed_work(zt2092->ts_workq, &zt2092->ts_reader, 0);
		//__gpio_as_irq_rise_edge(pdata->intr);
                emxx_gio_set_irq_type(this_client->irq, IRQ_TYPE_EDGE_RISING);
	} else {
		cancel_delayed_work_sync(&zt2092->ts_reader);
		zt2092_ts_release();
		//printk("xylimits: %d %d\n", x_limits, y_limits);
		counter = 0;

#if defined(CONFIG_ZT2092_GESTURE)
		zt2092_get_gesture();
		data_struct_flush();
#endif

		/*
		 * calculate xylimits, interrupt pin may change its state,
		 * therefore interrupt needs to be disabled
		 */
		disable_irq(this_client->irq);	/* __gpio_mask_irq(pdata->intr); */
		zt2092_startup();
		Filter_Coord_Struct_Flush();
		Temp_Buffer_Struct_Flush();
		//__gpio_as_irq_fall_edge(pdata->intr);
                emxx_gio_set_irq_type(this_client->irq, IRQ_TYPE_EDGE_FALLING);
		enable_irq(this_client->irq);	/* __gpio_unmask_irq(pdata->intr); */
	}
}

static irqreturn_t zt2092_interrupt(int irq, void *dev_id)
{
	struct zt2092_data *zt2092 = dev_id;

	//disable_irq(this_client->irq);

	if (!work_pending(&zt2092->pen_event_work)) {
		zt2092->irq_pending = 1;
		queue_work(zt2092->ts_workq, &zt2092->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zt2092_early_suspend(struct early_suspend *handler)
{
	struct zt2092_data *zt2092 = container_of(handler, struct zt2092_data, early_suspend);
	u8 buf[2];

	flush_scheduled_work();

	if (zt2092->input_dev->users)
		cancel_delayed_work_sync(&zt2092->ts_reader);

	/* issue sleep command */
	buf[0] = ZT2092_REG_SETUP;
	buf[1] = ZT2092_MODE_SLEEP2;
	zt2092_i2c_txdata(buf, 2);
}

static void zt2092_early_resume(struct early_suspend *handler)
{
	u8 buf[2];

	/* set zt2092 to normal mode */
	buf[0] = ZT2092_REG_SETUP;
	buf[1] = ZT2092_MODE_NORMAL;
	zt2092_i2c_txdata(buf, 2);
}
#endif

static int zt2092_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct zt2092_data *zt2092;
	int err = 0;

/*check if adapter supports everything we need*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_err(&client->dev,"%s: i2c check functionality failed.\n", __func__);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

/*allocate memory*/
	zt2092 = kzalloc(sizeof(*zt2092), GFP_KERNEL);
	if (!zt2092) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, zt2092);
	this_client = client;
	
	INIT_DELAYED_WORK(&zt2092->ts_reader, zt2092_ts_reader);
	INIT_WORK(&zt2092->pen_event_work, zt2092_pen_irq_work);

	zt2092->ts_workq = create_singlethread_workqueue("zt2092");
	if (!zt2092->ts_workq) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	mutex_init(&zt2092->lock);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}

#if 1
	/*
	 * test result shows that, for most touchscreen, the value
	 * of x_limits and y_limits makes no difference, so there is no
	 * reason to get these two values every time the driver loads
	 */
	err = zt2092_startup();
	if (err < 0) {
		dev_err(&client->dev, "%s: zt2092_init failed\n", __func__);
		goto exit_init_failed;
	}
#endif

	err = request_irq(client->irq, zt2092_interrupt, IRQF_DISABLED,
			"zt2092", zt2092);
	if (err < 0) {
		dev_err(&client->dev, "zt2092_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	//__gpio_as_irq_fall_edge(pdata->intr);
        emxx_gio_set_irq_type(this_client->irq, IRQ_TYPE_EDGE_FALLING);

	zt2092->input_dev = input_allocate_device();
	if (!zt2092->input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev,
		       "zt2092_probe: failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef CONFIG_ZT2092_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, zt2092->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, zt2092->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, zt2092->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, zt2092->input_dev->absbit);
	set_bit(KEY_HOME, zt2092->input_dev->keybit);
	set_bit(KEY_MENU, zt2092->input_dev->keybit);
	set_bit(KEY_BACK, zt2092->input_dev->keybit);

	input_set_abs_params(zt2092->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAXX+1, 0, 0);	
	input_set_abs_params(zt2092->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAXY+1, 0, 0);	
	input_set_abs_params(zt2092->input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAXZ+1, 0, 0);	
	input_set_abs_params(zt2092->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#else
	set_bit(ABS_X, zt2092->input_dev->absbit);
	set_bit(ABS_Y, zt2092->input_dev->absbit);
	set_bit(ABS_PRESSURE, zt2092->input_dev->absbit);
	set_bit(BTN_TOUCH, zt2092->input_dev->keybit);
	set_bit(KEY_HOME, zt2092->input_dev->keybit);
	set_bit(KEY_MENU, zt2092->input_dev->keybit);
	set_bit(KEY_BACK, zt2092->input_dev->keybit);

	input_set_abs_params(zt2092->input_dev, ABS_X, 0, SCREEN_MAXX+1, 0, 0);
	input_set_abs_params(zt2092->input_dev, ABS_Y, 0, SCREEN_MAXY+1, 0, 0);
	input_set_abs_params(zt2092->input_dev, ABS_PRESSURE, 0, PRESS_MAXZ+1, 0, 0);

#endif
	set_bit(EV_ABS, zt2092->input_dev->evbit);
	set_bit(EV_KEY, zt2092->input_dev->evbit);

	zt2092->input_dev->name = "zt2092";
	zt2092->input_dev->id.bustype = BUS_I2C;
	zt2092->input_dev->id.vendor = 0x0098;
	zt2092->input_dev->id.product = 0x0001;
	zt2092->input_dev->id.version = 0x1001;

	err = input_register_device(zt2092->input_dev);
	if (err) {
		dev_err(&client->dev,
		       "zt2092_probe: failed to register input device: %s\n",
		       zt2092->input_dev->name);
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	zt2092->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	zt2092->early_suspend.suspend = zt2092_early_suspend;
	zt2092->early_suspend.resume = zt2092_early_resume;
	register_early_suspend(&zt2092->early_suspend);
#endif

	return 0;

exit_input_register_device_failed:
	input_free_device(zt2092->input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, zt2092);
exit_irq_request_failed:
	dev_err(&client->dev, "%s %d\n", __func__, __LINE__);
exit_init_failed:
exit_platform_data_null:
	cancel_work_sync(&zt2092->pen_event_work);
	cancel_delayed_work_sync(&zt2092->ts_reader);
	destroy_workqueue(zt2092->ts_workq);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(zt2092);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit zt2092_remove(struct i2c_client *client)
{
	struct zt2092_data *zt2092 = i2c_get_clientdata(client);

	cancel_work_sync(&zt2092->pen_event_work);

	/* ts_reader rearms itself so we need to explicitly stop it
	 * before we destroy the workqueue.
	 */
	cancel_delayed_work_sync(&zt2092->ts_reader);
	destroy_workqueue(zt2092->ts_workq);
	input_unregister_device(zt2092->input_dev);
	free_irq(client->irq, zt2092);
	i2c_set_clientdata(client, NULL);
	kfree(zt2092);

	return 0;
}

static int zt2092_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

/* jz_battery_is_low: defined in pm.c, as a voltage low flag.*/
extern volatile int jz_battery_is_low;
static int zt2092_resume(struct i2c_client *client)
{
	struct zt2092_data *zt2092 = i2c_get_clientdata(client);
#if 0
	if (jz_battery_is_low == 1) {
		serial_puts("battery voltage is low!!!\n");
		input_report_key(zt2092->input_dev, KEY_MENU, 1);
		input_report_key(zt2092->input_dev, KEY_MENU, 0);
		jz_battery_is_low = 0;
	}
#endif
	return 0;
}

static const struct i2c_device_id zt2092_id[] = {
	{ ZT2092_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, zt2092_id);

static struct i2c_driver zt2092_driver = {
	.driver	= {
		.name	= ZT2092_I2C_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = zt2092_suspend,
	.resume	= zt2092_resume,
	.probe	= zt2092_probe,
	.remove	= __devexit_p(zt2092_remove),
	.id_table = zt2092_id,
};

static int __init zt2092_init(void)
{
	return i2c_add_driver(&zt2092_driver);
}

static void __exit zt2092_exit(void)
{
	i2c_del_driver(&zt2092_driver);
}

module_init(zt2092_init);
module_exit(zt2092_exit);

MODULE_AUTHOR("Ross Bai <fdbai@ingenic.cn");
MODULE_DESCRIPTION("ZillTek ZT2092 touchscreen module driver");
MODULE_LICENSE("GPL");
