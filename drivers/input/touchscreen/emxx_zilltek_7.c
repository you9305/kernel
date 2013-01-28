/**************************************************************/
//Author : Huang Hua Bin
//Date : 2011.05.19
//Function : Driver For 7inch resistance TP of ILLTEK
/*************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
/*add by youyou*/
#define DEBUG 1
/*end*/
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/delay.h>


#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include  <linux/timer.h>

#include <mach/gpio.h>
#include <mach/smu.h>
//#include <mach/gpio_define.h>

#ifdef CONFIG_TOUCHSCREEN_FILTER
#include <linux/ts_filter.h>
#include <linux/ts_filter_group.h>
#include <linux/ts_filter_linear.h>
#include <linux/ts_filter_mean.h>
#include <linux/ts_filter_median.h>
#endif

#define ZILLTEK_NAME "micco-ts"
#define XPT2046_TS_SCAN_TIME 10
#define	FENMU		(1)
#define FENZI		(1)

#ifdef CONFIG_BOARD_N960
	#define X_REG		(513)
	#define Y_REG		(310)
	#define X_DIV     	(5)
	#define Y_DIV     	(7)
	#define VIRT_MAX	(800)
	#define FAST_FILTER	(100)
	#define FILTER_TIMES (8)
	#define X_THREHOLD	(70)
	#define Y_THREHOLD	(120)
	#define	RTH			(8000)
	#define TIMER		(2)
	#define X_NEXT		(150)
	#define Y_NEXT		(120)
#else
	#define X_REG		(750)
	#define MAX_DIV     (8)
	#define VIRT_MAX	(800)
	#define FAST_FILTER	(60)
	#define FILTER_TIMES (8)
	#define X_THREHOLD	(100)
	#define Y_THREHOLD	(150)
	#define	RTH			(10000)
	#define TIMER		(2)
#endif

#define	MAX_12BIT	((1 << 12) - 1)
#define TTH			(12)
#define SSTH		(6)
#define ZOOMIN		(1)
#define ZOOMOUT		(1)
#define SLOWTIMES	(1)
#define CAL_RANGE	(50)
#define ZOOM_IN		(1)
#define ZOOM_OUT	(2)
#define DISCARDTIMES (1)
#define DISCARDTIMES_2 (7)

#define ZILLTEK_DEBUG
#ifdef ZILLTEK_DEBUG
	#define PDEBUG(fmt,args...) printk(fmt,##args)
#else
	#define PDEBUG(fmt,args...)
#endif 

#define PNORMAL(fmt,args...) printk("emxx_zilltek_7.c--%s:"fmt,__FUNCTION__,##args)

#define REG_RESET	0x00
#define	REG_SETUP	0x01
#define REG_SEQMOD	0x02

static struct i2c_client *zilltek_tp_client = NULL;
static struct input_dev *zilltek_tp_input = NULL;
static struct workqueue_struct *zilltek_workq = NULL;
static struct work_struct zilltek_worker;
static struct timer_list zilltek_tp_timer;
static int zilltek_tp_irq = 0;
static int pen_down = 0;
static struct platform_device *zilltek_tp_pdev = NULL;

static unsigned char lockflag = 0;
static unsigned char touchmode = 0,caltimes = 0,calflag = 0,slowtimes = 0,discardtimes = 0,discardtimes_2 = 0;
static unsigned int  x2_cal = 0,y2_cal = 0,x2_cal_old = 0,y2_cal_old = 0,x2_limit = 0,y2_limit = 0;
static int x_init ,y_init ,virt_y1 ,virt_y2,x_bk,y_bk;
static int sumx2,sumy2,pressure;

#ifdef CONFIG_TOUCHSCREEN_FILTER
static struct ts_filter_linear_configuration ts_linear_config = {
	.constants = {1, 0, 0, 0, 1, 0, 1},
	.coord0 = 0,
	.coord1 = 1,
};

static void *ts_filter_configs[] =
{
	[0] = &ts_linear_config,
	NULL,
};

static struct ts_filter *ts_filter;

static struct ts_filter_api *ts_filter_apis[]=
{
	[0] = &ts_filter_linear_api,
	NULL,
};
#endif


static int zilltek_hw_initial(void)
{
	u8	sendbuf[] = {0xff,0xff,REG_RESET,0x01,REG_SETUP,0x00};

	int ret;

	if((ret = i2c_master_send(zilltek_tp_client,&sendbuf[0],2)) < 0)
		return ret;
	if((ret = i2c_master_send(zilltek_tp_client,&sendbuf[2],2)) < 0)
		return ret;
	if((ret = i2c_master_send(zilltek_tp_client,&sendbuf[4],2)) < 0)
		return ret;
	return 0;
}

static irqreturn_t zilltek_irq_handler(int irq,void *dev_id)
{
	zilltek_tp_irq = irq;
	mod_timer(&zilltek_tp_timer,jiffies + TIMER);
	disable_irq_nosync(zilltek_tp_irq);
	return IRQ_HANDLED;	
}

static int i2c_read_zilltek(struct i2c_client *client,u8 * rec_buf)
{
	unsigned char buf[] = {0x11};
	struct i2c_msg msg[2];
	//write
	msg[0].addr = client->addr;
	msg[0].flags = 0 | I2C_ZILLTEK_RD;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	//read
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msg[1].len = 0x0c;
	msg[1].buf = rec_buf;

	return i2c_transfer(client->adapter,msg,2);
}

static void zilltek_cal(int x2,int y2)
{
    if(calflag == 0) {
		if(caltimes < 4) {
			if(((x2_cal_old + CAL_RANGE) > x2 ) && ((y2_cal_old + CAL_RANGE) > y2 )) {
				x2_cal += x2;
                y2_cal += y2;
                caltimes++;
            }
            else {
                caltimes = 0;
                x2_cal = 0;
                y2_cal = 0;
            }
            x2_cal_old = x2;
            y2_cal_old = y2;
            return ;
        }
        x2_cal = x2_cal / 4;
        y2_cal = y2_cal / 4;
        x2_cal_old = 0;
        y2_cal_old = 0;
		x2_limit = x2_cal;
		y2_limit = y2_cal;
        calflag = 1;
		caltimes = 0;
        PNORMAL("cal ok:x2_cal = %d,y2_cal = %d\n",x2_cal,y2_cal);
   }
}

static int zilltek_touch_mode(int x2,int y2)
{
#ifdef CONFIG_BOARD_N706 
	if(((x2 + TTH) <= x2_limit) || ((y2 + TTH) <= y2_limit)) {
		if(pressure > (RTH / 2))
			return -1;

		PDEBUG("Two touch!\n");
		touchmode = 2;
	}
	else {

	//	if(pressure > RTH) {
	//		input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 0);
	//		input_mt_sync(zilltek_tp_input);
	//		input_sync(zilltek_tp_input);
	//		return -1;
	//	}
		PDEBUG("One\n");
		zilltek_cal(x2,y2);
		if(touchmode == 2) {
			discardtimes_2 ++;
			if(discardtimes_2 < DISCARDTIMES_2) {
				return -1;
			}
		}
		discardtimes_2 = 0;
		touchmode = 1;
	}
#else
//	if(pressure > RTH) {
//		input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 0);
//		input_mt_sync(zilltek_tp_input);
//		input_sync(zilltek_tp_input);
//		return -1;
//	}
	PDEBUG("One\n");
	zilltek_cal(x2,y2);
	if(touchmode == 2) {
		discardtimes_2 ++;
		if(discardtimes_2 < DISCARDTIMES_2) {
			return -1;
		}
	}
	discardtimes_2 = 0;
	touchmode = 1;
#endif
	return 0;
}

static void zilltek_x2y2_filter(int *x2,int *y2)
{
	static int tmpx[8] = {0};
	static int tmpy[8] = {0};
	int i;
	static int x2_bk = 0,y2_bk = 0;

	sumx2 = 0;
	sumy2 = 0;

	if(pen_down == 0) {
		for(i = 0;i < FILTER_TIMES;i++) {
			tmpx[i] = *x2;
			tmpy[i] = *y2;
		}
	}
	else if(abs((x2_bk + y2_bk) - (*x2 + *y2)) > FAST_FILTER){
		for(i = 0;i < FILTER_TIMES;i++) {
			tmpx[i] = *x2;
			tmpy[i] = *y2;
		}
	}
	else {

		tmpx[7] = tmpx[6];	
		tmpy[7] = tmpy[6];

		tmpx[6] = tmpx[5];	
		tmpy[6] = tmpy[5];

		tmpx[5] = tmpx[4];	
		tmpy[5] = tmpy[4];

		tmpx[4] = tmpx[3];	
		tmpy[4] = tmpy[3];

		tmpx[3] = tmpx[2];	
		tmpy[3] = tmpy[2];

		tmpx[2] = tmpx[1];	
		tmpy[2] = tmpy[1];

		tmpx[1] = tmpx[0];	
		tmpy[1] = tmpy[0];

		tmpx[0] = *x2;	
		tmpy[0] = *y2;
	}

	for(i = 0;i < FILTER_TIMES;i++)
	{
		sumx2 += tmpx[i];
		sumy2 += tmpy[i];
	}

	*x2 = sumx2 / FILTER_TIMES;
	*y2 = sumy2 / FILTER_TIMES;
	x2_bk = *x2;
	y2_bk = *y2;
}

static int zilltek_touch_proc(int x,int y,int x2,int y2)
{
	int ret = 0,y2_final;
	static int x2_last = 0,y2_last = 0;

	if(touchmode == 2)
	{
		ret = -1;
//		zilltek_x2y2_filter(&x2,&y2);
//		if(discardtimes < DISCARDTIMES) {
//			discardtimes++;
//			return ret;
//		}

		slowtimes++;
		if(slowtimes >= SLOWTIMES)
		{
			slowtimes = 0;
			y2_final = (x2 + y2) - (x2_last + y2_last);
			PDEBUG("y2_final = %d\n",y2_final);
			x2_last = x2;
			y2_last = y2;
			if(lockflag == 0) {
				lockflag = 1;
				x_init = x;
				y_init = y;
				virt_y1 = y - 100;
				virt_y2 = y + 100;
				return ret;
			}

			if((virt_y1 + y2_final) <= 0)
				virt_y1 = 0;
			else if((virt_y1 + y2_final) >= y_init)
				virt_y1 = y_init;
			else
				virt_y1 = virt_y1 + ((y2_final * FENZI) / FENMU);


			if((virt_y2 - y2_final) <= y_init)
				virt_y2 = y_init;
			else if((virt_y2 - y2_final) >= VIRT_MAX)
				virt_y2 = VIRT_MAX;
			else
				virt_y2 = virt_y2 - ((y2_final * FENZI) / FENMU);
			
			input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(zilltek_tp_input, ABS_MT_POSITION_X, x_init);
			input_report_abs(zilltek_tp_input, ABS_MT_POSITION_Y, virt_y1);
			input_mt_sync(zilltek_tp_input);
			input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(zilltek_tp_input, ABS_MT_POSITION_X, x_init);
			input_report_abs(zilltek_tp_input, ABS_MT_POSITION_Y, virt_y2);
			input_mt_sync(zilltek_tp_input);
			input_sync(zilltek_tp_input);
			
			PDEBUG("(%d,%d)---(%d,%d)\n",x_init,virt_y1,x_init,virt_y2);
		}
	}
	return ret;
}

static void zilltek_do_work(struct work_struct *work)
{
	int x,y,x2,y2,z1,z2,tmpx[3],tmpy[3];
	int coords[2] = { 0 }; 
	u8	sendbuf[] = {0x02,0x64,0x04};
	u8	val[12];
	int ret;
	int tmp1,tmp2,tmp3 = 0;
	unsigned char stableflag = 0;
	
	if(!gpio_get_value(TP_INT_GPIO))
	{
    	do {
	       	ret = i2c_master_send(zilltek_tp_client,&sendbuf[0],3);
        	if(ret < 0) {
	           	PDEBUG("send error,ret = %d!\n",ret);
            	goto    exit;
	       	}
        	ret = i2c_read_zilltek(zilltek_tp_client,&val[0]);
        	if(ret < 0) {
            	PDEBUG("read error,ret = %d!\n",ret);
            	goto    exit;
        	}
        	if(gpio_get_value(TP_INT_GPIO)) {
            	PDEBUG("already up!\n");
            	goto    exit;
        	}

			x  = (val[0]  << 4) | (val[1]  >> 4);
			y  = (val[2]  << 4) | (val[3]  >> 4);
			z1 = (val[4]  << 4) | (val[5]  >> 4);
			z2 = (val[6]  << 4) | (val[7]  >> 4);
			x2 = (val[8]  << 4) | (val[9]  >> 4);
			y2 = (val[10] << 4) | (val[11] >> 4);
	
			if((x >= MAX_12BIT) || (y >= MAX_12BIT) || (z1 < 1) || (z2 >= MAX_12BIT) || (x2 >= MAX_12BIT) || (y2 >= MAX_12BIT)) {
				PDEBUG("Data Failed!\n");
				goto	exit;
			}

			tmpx[2] = tmpx[1];
			tmpy[2] = tmpy[1];
			tmpx[1] = tmpx[0];
			tmpy[1] = tmpy[0];
			tmpx[0] = x;
			tmpy[0] = y;
			tmp3++;
			if(tmp3 == 3) {
				if((abs(tmpx[0] - tmpx[1]) < X_THREHOLD) && (abs(tmpy[0] - tmpy[1]) < Y_THREHOLD)) {
					if((abs(tmpx[1] - tmpx[2]) < X_THREHOLD) && (abs(tmpy[1] - tmpy[2]) < Y_THREHOLD))
						if((abs(tmpx[0] - tmpx[2]) < X_THREHOLD) && (abs(tmpy[0] - tmpy[2]) < Y_THREHOLD)) {
							stableflag = 1;
						}
				}
				if(stableflag == 0) {
					tmp3--;
					#ifdef CONFIG_BOARD_N960
					tmp3 = 0;
					#endif
					PDEBUG("failed   x = %d,y = %d,z1 = %d,z2 = %d,x2 = %d,y2 = %d,pressure = %d\n",x,y,z1,z2,x2,y2,pressure);
				}
			}
    	}while(stableflag == 0);

#ifdef CONFIG_BOARD_N960
		tmp1 = (z2 -z1) << 4;
		pressure =  ((x >> 2) * (tmp1 / z1)) >> 4;
#else
		tmp1 = (X_REG * x) / 4096;
		tmp2 = ((z2 - z1) * 10) / z1;
		pressure = tmp1 * tmp2;
#endif

#ifdef CONFIG_BOARD_N960
		zilltek_x2y2_filter(&x,&y);
#else
		zilltek_x2y2_filter(&x2,&y2);
#endif
		if(zilltek_touch_mode(x2,y2) < 0)
			goto	exit;
		
		PDEBUG("x = %d,y = %d,z1 = %d,z2 = %d,x2 = %d,y2 = %d,pressure = %d\n",x,y,z1,z2,x2,y2,pressure);
#ifdef CONFIG_TOUCHSCREEN_FILTER
#ifdef CONFIG_BOARD_N960
		coords[0] = x/X_DIV;
	    coords[1] = y/Y_DIV;
#else
		coords[0] = y/MAX_DIV;
	    coords[1] = x/MAX_DIV;
#endif
		if(ts_filter->api->process(ts_filter,coords))
			ts_filter->api->scale(ts_filter, coords);
		x = coords[0];
        y = coords[1];
#endif
		pen_down = 1;
		if(zilltek_touch_proc(x,y,x2,y2) < 0)
			goto exit;
#ifdef CONFIG_BOARD_N960
		if((x > 800) || (y > 600))
			goto exit;
		if((x_bk > 0) && (y_bk > 0)) {
			if((abs(x - x_bk) > X_NEXT) || (abs(y - y_bk) > Y_NEXT))
				goto exit;
		}
		x_bk = x;
		y_bk = y;
#endif
		input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(zilltek_tp_input, ABS_MT_POSITION_X, x);
		input_report_abs(zilltek_tp_input, ABS_MT_POSITION_Y, y);
		input_mt_sync(zilltek_tp_input);
		PDEBUG("DOWN x %d, y %d\n", x, y);
		input_sync(zilltek_tp_input);
exit:
		mod_timer(&zilltek_tp_timer,jiffies + TIMER);
		return ;
	}

#ifdef CONFIG_TOUCHSCREEN_FILTER
		ts_filter->api->clear(ts_filter);
#endif
		if(pen_down == 1)
		{
			PDEBUG("UP\n");
			input_report_abs(zilltek_tp_input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(zilltek_tp_input);
			input_sync(zilltek_tp_input);
			pen_down = 0;
			lockflag = 0;
			slowtimes = 0;
			discardtimes = 0;
			discardtimes_2 = 0;
			x_bk = -1;
			y_bk = -1;
		}
		enable_irq(zilltek_tp_irq);
}

static void zilltek_tp_do_timer(unsigned long arg)
{
	queue_work(zilltek_workq,&zilltek_worker);
}

static int zilltek_tp_suspend(struct i2c_client *client,pm_message_t state)
{
	return 0;
}


static int zilltek_tp_resume(struct i2c_client *client)
{
	return 0;
}

/**********************************************************************************/

static ssize_t sample_show (struct device *dev, struct device_attribute *attr, unsigned char *buf)
{
	if(calflag)
		return snprintf(buf,11,"%d,%d\n",x2_cal,y2_cal);
	return 0;
}

static ssize_t sample_store (struct device *dev, struct device_attribute *attr, const unsigned char *buf, size_t size)
{
	char cmd[16] = {0};

	if (sscanf (buf, "%s", cmd) == 1)
	{
		if(strcmp(cmd, "start") == 0 )
		{
			PNORMAL("It is sample\n");
			calflag = 0;
		}	
	}			
	return size;
}

static ssize_t device_show (struct device *dev, struct device_attribute *attr, unsigned char *buf)
{
	return snprintf(buf,11,"%d,%d\n",x2_limit,y2_limit);
}

static ssize_t device_store (struct device *dev, struct device_attribute *attr, const unsigned char *buf, size_t size)
{
	int i,j = 0;
	unsigned int limit[2] = {0,0};

	for(i = 0;i < size;i++) {
		if(buf[i] == 0x2c) {
			j = 1;
			i++;
		}
		if(buf[i] == 0x0a)
			break;
		if(j == 0)
			limit[j] = (limit[j] * 10) + (buf[i] - 0x30);
		if(j == 1)
			limit[j] = (limit[j] * 10) + (buf[i] - 0x30);
	}
	x2_limit = limit[0];
	y2_limit = limit[1];
	calflag = 1;
	PNORMAL("x2_limit = %d,y2_limit = %d\n",x2_limit,y2_limit);
	return size;
}

static DEVICE_ATTR (sample, S_IRUGO | S_IWUGO, sample_show, sample_store);
static DEVICE_ATTR (device, S_IRUGO | S_IWUGO, device_show, device_store);

static struct class *class = NULL;
static struct device *device = NULL;
static int create_tp_calibrate_device (void)
{
	int ret = 0;
	
	PDEBUG("create tp calibrate device\n");

	class = class_create (THIS_MODULE, "tp_calibrate");
	if (IS_ERR (class))
	{
		ret = PTR_ERR (class);
		goto err_class_create;
	}
	
	device = device_create (class, NULL, MKDEV (0, 1), "%s", "tp_calibrate_class");
	if (IS_ERR (device))
	{
		ret = PTR_ERR (device);
		goto err_device_create;
	}
	
	ret = device_create_file (device, &dev_attr_sample);
	if (ret)
	{
		goto err_device_create_file;
	}

	ret = device_create_file (device, &dev_attr_device);
	if (ret)
	{
		goto err_device_create_file;
	}

	PDEBUG("create tp calibrate device success!\n");
	return 0;

err_device_create_file:	
	device_destroy (class, MKDEV (0, 1));

err_device_create:
	class_destroy (class);

err_class_create:
	PNORMAL("create tp calibrate device error!\n");
	return ret;
}

static void destroy_tp_calibrate_device (void)
{
	device_remove_file (device, &dev_attr_sample);
	device_remove_file (device, &dev_attr_device);
	device_destroy (class, MKDEV (0, 1));
	class_destroy (class);
}
/**********************************************************************************/

#ifdef CONFIG_TOUCHSCREEN_FILTER
static int zilltek_ts_probe(struct platform_device *pdev)
{
	PDEBUG("i'am probe!\n");
	zilltek_tp_pdev = pdev;
	ts_filter_create_chain(pdev, ts_filter_apis, ts_filter_configs, &ts_filter,2);
	ts_filter->api->clear(ts_filter);
	return 0;
}

static struct platform_driver zilltek_ts_driver = {
	.probe = zilltek_ts_probe,
	.driver = {
		.name = ZILLTEK_NAME,
		.owner = THIS_MODULE,
	},
};
#endif

static	int zilltek_tp_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret;

	PDEBUG("i2c_address = 0x%x\n",client->addr);

	zilltek_tp_client = client;

	ret = zilltek_hw_initial();
	if(ret < 0) {
		PNORMAL("zilltek hardware initial error!\n");
		return ret;
	}

	ret = gpio_request(TP_INT_GPIO,"zilltek_int_gpio");
	if(ret < 0) {
		PNORMAL("zilltek request gpio error!\n");
		return ret;
	}
	gpio_direction_input(TP_INT_GPIO);

	if(create_tp_calibrate_device()) {
		ret = -ENOMEM;
		PNORMAL("create tp calibrate device error!\n");
		goto	error1;
	}

	zilltek_tp_input = input_allocate_device();
	if(zilltek_tp_input == NULL) {
		PNORMAL("allocate input device error!\n");
		ret = -ENOMEM;
		goto error2;
	}
	
	set_bit(EV_ABS,zilltek_tp_input->evbit);
	set_bit(EV_SYN,zilltek_tp_input->evbit);
	set_bit(EV_KEY,zilltek_tp_input->evbit);
	set_bit(BTN_TOUCH,zilltek_tp_input->keybit);
	set_bit(ABS_MT_POSITION_X, zilltek_tp_input->absbit);
	set_bit(ABS_MT_POSITION_Y, zilltek_tp_input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, zilltek_tp_input->absbit);

	zilltek_tp_input->name = "zilltek";
	zilltek_tp_input->phys = "zilltek/input";
	zilltek_tp_input->dev.parent = &client->dev;

	ret = input_register_device(zilltek_tp_input);
	if(ret) {
		PNORMAL("input device register error!\n");
		goto	error3;
	}
#ifdef CONFIG_TOUCHSCREEN_FILTER
	ret = platform_driver_register(&zilltek_ts_driver);
	if(ret) {
		PNORMAL("platform driver register error!\n");
		goto	error3;
	}
#endif
	zilltek_workq = create_workqueue("zilltek");
	if(zilltek_workq == NULL) {
		ret = -ENOMEM;
		goto	error4;
	}
	INIT_WORK(&zilltek_worker,zilltek_do_work);	
	zilltek_tp_irq = gpio_to_irq(TP_INT_GPIO);
	ret = request_irq(gpio_to_irq(TP_INT_GPIO),zilltek_irq_handler,IRQF_TRIGGER_FALLING ,client->name,(void *)zilltek_tp_client);
	if(ret) {
		PNORMAL("%s:request irq error!\n",__FUNCTION__);
		goto error5;
	}

	init_timer(&zilltek_tp_timer);
	zilltek_tp_timer.function = zilltek_tp_do_timer;
	zilltek_tp_timer.data = 0;
	zilltek_tp_timer.expires = 0;

	x_bk = -1;
	y_bk = -1;
	PNORMAL("Zilltek Tp Dectected(%s).\n",client->name);

	return 0;
error5:
	destroy_workqueue(zilltek_workq);
error4:
#ifdef CONFIG_TOUCHSCREEN_FILTER
	platform_driver_unregister(&zilltek_ts_driver);
#endif
 	input_unregister_device(zilltek_tp_input);
error3:
	input_free_device(zilltek_tp_input);
	zilltek_tp_input = NULL;	
error2:
	zilltek_tp_client = NULL;
error1:
	destroy_tp_calibrate_device();
	gpio_free(TP_INT_GPIO);
	return ret;
}

static int zilltek_tp_remove (struct i2c_client *client)
{
	destroy_workqueue(zilltek_workq);
 	input_unregister_device(zilltek_tp_input);
	input_free_device(zilltek_tp_input);	
	zilltek_tp_client = NULL;
	zilltek_tp_input = NULL;
	destroy_tp_calibrate_device();
	free_irq(zilltek_tp_irq,(void *)zilltek_tp_client);
	gpio_free(TP_INT_GPIO);
#ifdef CONFIG_TOUCHSCREEN_FILTER
	platform_driver_unregister(&zilltek_ts_driver);
#endif
	return 0;
}


static struct i2c_device_id zilltek_i2c_id[] = {
	{"zilltek_7",0},
	{ },
};

static struct i2c_driver i2c_zilltek_driver = {
	.driver = {
		.name = "zilltek_7",
	},
	.probe 		= zilltek_tp_probe,
	.remove 	= zilltek_tp_remove,
	.id_table	= zilltek_i2c_id,
	.suspend	= zilltek_tp_suspend,
	.resume		= zilltek_tp_resume,
};

static __init int zilltek_tp_init(void)
{
	return i2c_add_driver(&i2c_zilltek_driver);
}


static __exit void zilltek_tp_exit(void)
{
	i2c_del_driver(&i2c_zilltek_driver);
}

module_init(zilltek_tp_init);
module_exit(zilltek_tp_exit);
MODULE_LICENSE("GPL");

