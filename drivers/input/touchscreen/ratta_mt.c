#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/proc_ratta.h>
#include <linux/input/ratta_touch.h>

#define RATTA_MT_DEBUG		1
#define RATTA_MT_NAME		"ratta-slide"
#define RATTA_MAX_FINGERS	10
#define RATTA_INVALID_VALUE	0x7FFFFFFF
#define RATTA_SLIDE_INTERVAL	1 /* time in seconds */
#define RATTA_SLIDE_U2D		KEY_F23
#define RATTA_SLIDE_D2U		KEY_REFRESH
#define RATTA_SLIDE_DROP	KEY_F24

#define RATTA_SN100_START	528 /* a5 start position */
#define RATTA_SN100_END		1340 /* a5 end position */
#define RATTA_SN100_OFFSET	325 /* a5 valid slied offset */
#define RATTA_SN078_START	402 /* a6 start position */
#define RATTA_SN078_END		1474 /* a6 end position */
#define RATTA_SN078_OFFSET	428 /* a6 valid slied offset */

#define RATTA_DROP_START	50
#define RATTA_DROP_WIDTH	100
#define RATTA_DROP_HEIGHT	100
#define RATTA_MIN_Y		3

static int RATTA_SLIDE_OFFSET, RATTA_SLIDE_XSTART, RATTA_SLIDE_XEND, RATTA_Y;

struct ratta_mt_record {
	/* start x and y position */
	int x, y;
	unsigned long jiffs;
	bool done;
};

struct ratta_mt_device {
	struct input_dev *input;
	struct ratta_mt_record history[RATTA_MAX_FINGERS];
};

static struct ratta_mt_device *ratta_device = NULL;
extern int ratta_touch_ic;
#define ratta_mt_debug(fmt, args...) do { \
		if (RATTA_MT_DEBUG) { \
			printk("[ratta_mt]%s:"fmt"\n", __func__, ##args); \
		} \
	} while (0);

static void ratta_report_slide(int code)
{
	input_report_key(ratta_device->input,
			 code, 1);
	input_report_key(ratta_device->input,
			 code, 0);
	input_sync(ratta_device->input);

	switch (code) {
	case RATTA_SLIDE_U2D:
			dev_info(&ratta_device->input->dev,
					 "report slide down.\n");
			break;
	case RATTA_SLIDE_D2U:
			dev_info(&ratta_device->input->dev,
					 "report slide up.\n");
			break;
	case RATTA_SLIDE_DROP:
			dev_info(&ratta_device->input->dev,
					 "report slide drop.\n");
			break;
	default:
			dev_info(&ratta_device->input->dev,
					 "report %d.\n", code);
			break;
	}
}

/*
 * @id: track id
 * @x: x position
 * @y: y position
 * @state: true for touch and false for leave
 */
int ratta_mt_record(int id, int x, int y, bool state)
{
	struct ratta_mt_record *pr;

	if (!ratta_device)
		return -ENODEV;

	if ((id < 0) || (id >= RATTA_MAX_FINGERS)) {
		return -EINVAL;
	}

	//if (ratta_get_board() == BOARD_A6X) {
	//	RATTA_SLIDE_XSTART = RATTA_SN078_START;
	//	RATTA_SLIDE_XEND = RATTA_SN078_END;
//		RATTA_SLIDE_OFFSET = RATTA_SN078_OFFSET;
	//} else {
		RATTA_SLIDE_XSTART = RATTA_SN100_START;
		RATTA_SLIDE_XEND = RATTA_SN100_END;
		RATTA_SLIDE_OFFSET = RATTA_SN100_OFFSET;
		RATTA_Y = RATTA_MIN_Y;
	//}
	switch (ratta_touch_ic){
		case RATTA_TOUCH_IC_ATMEL:
			if (ratta_get_board() == BOARD_A6X) {
				RATTA_SLIDE_XSTART = RATTA_SN078_START;
				RATTA_SLIDE_XEND = RATTA_SN078_END;
				RATTA_SLIDE_OFFSET = RATTA_SN078_OFFSET;
			} else {
				RATTA_SLIDE_XSTART = RATTA_SN100_START;
				RATTA_SLIDE_XEND = RATTA_SN100_END;
				RATTA_SLIDE_OFFSET = RATTA_SN100_OFFSET;
			}
			RATTA_Y = RATTA_MIN_Y;
			break;
		//case RATTA_TOUCH_IC_GOODIX:
		//	break;
	 	case RATTA_TOUCH_IC_CYTTSP5:
			RATTA_SLIDE_XSTART = RATTA_SN078_START;
			RATTA_SLIDE_XEND = RATTA_SN078_END;
			RATTA_SLIDE_OFFSET = RATTA_SN078_OFFSET;
			RATTA_Y = RATTA_MIN_Y;
			break;
		//case RATTA_TOUCH_IC_PARADE:
		//	break;
		case RATTA_TOUCH_IC_FT5XX:
			RATTA_SLIDE_XSTART = RATTA_SN100_START;
			RATTA_SLIDE_XEND = RATTA_SN100_END;
			RATTA_SLIDE_OFFSET = RATTA_SN100_OFFSET;
			RATTA_Y = 1999;
			break;
		default:
			RATTA_SLIDE_XSTART = RATTA_SN100_START;
			RATTA_SLIDE_XEND = RATTA_SN100_END;
			RATTA_SLIDE_OFFSET = RATTA_SN100_OFFSET;
			RATTA_Y = RATTA_MIN_Y;
			break;
	}
#if 0
	dev_info(&ratta_device->input->dev,
		 "%d,%d,%d,%d(%d,%d,%d)\n",
		 id, x, y, state,
		 RATTA_SLIDE_XSTART,
		 RATTA_SLIDE_XEND,
		 RATTA_SLIDE_OFFSET);
#endif

	pr = &ratta_device->history[id];
	if(ratta_touch_ic == RATTA_TOUCH_IC_FT5XX){
		if (state) {
			/* touch */
			if ((pr->x == RATTA_INVALID_VALUE) ||
			    (pr->y == RATTA_INVALID_VALUE)) {
				ratta_mt_debug("Start111: x=%d,y=%d,id=%d\n", x, y, id);
				pr->x = x;
				pr->y = y;
				pr->jiffs = jiffies;
				pr->done = false;
			} else if ((y < RATTA_Y) && (pr->y >= RATTA_Y)) {
				if (!pr->done)
					ratta_mt_debug("Invalid slide point: x=%d,y=%d,id=%d\n",
								   x, y, id);
				pr->x = RATTA_INVALID_VALUE;
				pr->y = RATTA_INVALID_VALUE;
				pr->done = false;
			} else if ((y > 0) && (abs(pr->y - y) > RATTA_DROP_WIDTH)) {
				if ((pr->y != RATTA_INVALID_VALUE) &&
					(pr->x <= RATTA_DROP_START) &&
					!pr->done)
					ratta_mt_debug("Invalid drag point: x=%d,y=%d,id=%d\n",
								   x, y, id);
				pr->x = RATTA_INVALID_VALUE;
				pr->y = RATTA_INVALID_VALUE;
				pr->done = false;
			} else if ((pr->y >= RATTA_Y) &&
				   (x >= RATTA_SLIDE_XSTART) &&
				   (x <= RATTA_SLIDE_XEND) &&
				   (((x > pr->x) &&
				     ((x - RATTA_SLIDE_XSTART) > RATTA_SLIDE_OFFSET)) ||
				    ((x < pr->x) &&
				     ((RATTA_SLIDE_XEND - x) > RATTA_SLIDE_OFFSET))) &&
				   (abs(x - pr->x) > RATTA_SLIDE_OFFSET)) {
				if (!pr->done) {
					ratta_mt_debug("Slide point: x=%d,y=%d,id=%d\n", x, y, id);
				    if ((jiffies_to_msecs(jiffies - pr->jiffs) / 1000) <=
						RATTA_SLIDE_INTERVAL) {
						ratta_report_slide((pr->x < x) ?
										   RATTA_SLIDE_U2D :
										   RATTA_SLIDE_D2U);
						pr->done = true;
				    }
				}
			} else if ((pr->x <= RATTA_DROP_START) &&
				   (pr->y > 0) && (x > pr->x) &&
				   (abs(pr->y - y) < RATTA_DROP_WIDTH) &&
				   (abs(pr->x - x) > RATTA_DROP_HEIGHT)) {
				if (!pr->done) {
					ratta_mt_debug("Drag point: x=%d,y=%d,id=%d\n", x, y, id);
					ratta_report_slide(RATTA_SLIDE_DROP);
					pr->done = true;
				}
			} else if ((y >= RATTA_Y) && (pr->y >= RATTA_Y) && !pr->done) {
				ratta_mt_debug("Slide point: x=%d,y=%d,id=%d\n", x, y, id);
			} else if (!pr->done &&
					   (pr->x <= (RATTA_DROP_START + RATTA_DROP_HEIGHT)) &&
					   ((pr->y > 0) &&
						(x > pr->x) &&
						(abs(pr->y - y) < RATTA_DROP_WIDTH))) {
				ratta_mt_debug("Drag point: x=%d,y=%d,id=%d\n", x, y, id);
			}
		} else {
			/* leave */
			ratta_mt_debug("Stop: x=%d,y=%d,id=%d\n", x, y, id);
			pr->x = RATTA_INVALID_VALUE;
			pr->y = RATTA_INVALID_VALUE;
			pr->done = false;
		}
	}else{
    	if (state) {
    		/* touch */
    		if ((pr->x == RATTA_INVALID_VALUE) ||
    		    (pr->y == RATTA_INVALID_VALUE)) {
    			ratta_mt_debug("Start: x=%d,y=%d,id=%d\n", x, y, id);
    			pr->x = x;
    			pr->y = y;
    			pr->jiffs = jiffies;
    			pr->done = false;
    		} else if ((y > RATTA_MIN_Y) && (pr->y <= RATTA_MIN_Y)) {
    			if (!pr->done)
    				ratta_mt_debug("Invalid slide point: x=%d,y=%d,id=%d\n",
    							   x, y, id);
    			pr->x = RATTA_INVALID_VALUE;
    			pr->y = RATTA_INVALID_VALUE;
    			pr->done = false;
    		} else if ((y > 0) && (abs(pr->y - y) > RATTA_DROP_WIDTH)) {
    			if ((pr->y != RATTA_INVALID_VALUE) &&
    				(pr->x <= RATTA_DROP_START) &&
    				!pr->done)
    				ratta_mt_debug("Invalid drag point: x=%d,y=%d,id=%d\n",
    							   x, y, id);
    			pr->x = RATTA_INVALID_VALUE;
    			pr->y = RATTA_INVALID_VALUE;
    			pr->done = false;
    		} else if ((pr->y <= RATTA_MIN_Y) &&
    			   (x >= RATTA_SLIDE_XSTART) &&
    			   (x <= RATTA_SLIDE_XEND) &&
    			   (((x > pr->x) &&
    			     ((x - RATTA_SLIDE_XSTART) > RATTA_SLIDE_OFFSET)) ||
    			    ((x < pr->x) &&
    			     ((RATTA_SLIDE_XEND - x) > RATTA_SLIDE_OFFSET))) &&
    			   (abs(x - pr->x) > RATTA_SLIDE_OFFSET)) {
    			if (!pr->done) {
    				ratta_mt_debug("Slide point: x=%d,y=%d,id=%d\n", x, y, id);
    			    if ((jiffies_to_msecs(jiffies - pr->jiffs) / 1000) <=
    					RATTA_SLIDE_INTERVAL) {
    					ratta_report_slide((pr->x < x) ?
    									   RATTA_SLIDE_U2D :
    									   RATTA_SLIDE_D2U);
    					pr->done = true;
    			    }
    			}
    		} else if ((pr->x <= RATTA_DROP_START) &&
    			   (pr->y > 0) && (x > pr->x) &&
    			   (abs(pr->y - y) < RATTA_DROP_WIDTH) &&
    			   (abs(pr->x - x) > RATTA_DROP_HEIGHT)) {
    			if (!pr->done) {
    				ratta_mt_debug("Drag point: x=%d,y=%d,id=%d\n", x, y, id);
    				ratta_report_slide(RATTA_SLIDE_DROP);
    				pr->done = true;
    			}
    		} else if ((y <= RATTA_MIN_Y) && (pr->y <= RATTA_MIN_Y) && !pr->done) {
    			ratta_mt_debug("Slide point: x=%d,y=%d,id=%d\n", x, y, id);
    		} else if (!pr->done &&
    				   (pr->x <= (RATTA_DROP_START + RATTA_DROP_HEIGHT)) &&
    				   ((pr->y > 0) &&
    					(x > pr->x) &&
    					(abs(pr->y - y) < RATTA_DROP_WIDTH))) {
    			ratta_mt_debug("Drag point: x=%d,y=%d,id=%d\n", x, y, id);
    		}
    	} else {
    		/* leave */
    		ratta_mt_debug("Stop: x=%d,y=%d,id=%d\n", x, y, id);
    		pr->x = RATTA_INVALID_VALUE;
    		pr->y = RATTA_INVALID_VALUE;
    		pr->done = false;
    	}
    }
	return 0;
}

int ratta_mt_probe(struct device *dev)
{
	int i, ret;

	if (ratta_device) {
		return -EEXIST;
	}

	ratta_device = (typeof(ratta_device))kzalloc(sizeof(*ratta_device),
						     GFP_KERNEL);
	if (!ratta_device) {
		return -ENOMEM;
	}

	ratta_device->input = devm_input_allocate_device(dev);
	if (!ratta_device->input) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < RATTA_MAX_FINGERS; i++) {
		ratta_device->history[i].x = RATTA_INVALID_VALUE;
		ratta_device->history[i].y = RATTA_INVALID_VALUE;
		ratta_device->history[i].jiffs = 0;
		ratta_device->history[i].done = false;
	}

	ratta_device->input->name = RATTA_MT_NAME;
	ratta_device->input->id.bustype = BUS_I2C;

	// 20210717: 如果没有 VID/PID，则默认会通过 名字来匹配。
	//ratta_device->input->id.vendor = 0x0777;
	//ratta_device->input->id.product = 0x0007;
	// 20210717: 如果增加了 version，那么 kl 文件的后缀也要增加version。看
	// frameworks/native/libs/input/InputDevice.cpp 文件。
	//ratta_device->input->id.version = 0x0100;
	
	__set_bit(EV_KEY, ratta_device->input->evbit);
	__set_bit(RATTA_SLIDE_U2D, ratta_device->input->keybit);
	__set_bit(RATTA_SLIDE_D2U, ratta_device->input->keybit);
	__set_bit(RATTA_SLIDE_DROP, ratta_device->input->keybit);

	ret = input_register_device(ratta_device->input);
	if (ret < 0) {
		goto err;
	}

	return 0;
err:
	kfree(ratta_device);
	ratta_device = NULL;

	return ret;
}

void ratta_mt_remove(void)
{
	input_unregister_device(ratta_device->input);
	kfree(ratta_device);
	ratta_device = NULL;
}
