#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>

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

static int RATTA_SLIDE_OFFSET, RATTA_SLIDE_XSTART, RATTA_SLIDE_XEND;

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

static void ratta_report_slide(int code)
{
	input_report_key(ratta_device->input,
			 code, 1);
	input_report_key(ratta_device->input,
			 code, 0);
	input_sync(ratta_device->input);
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

	if (true) {
		RATTA_SLIDE_XSTART = RATTA_SN078_START;
		RATTA_SLIDE_XEND = RATTA_SN078_END;
		RATTA_SLIDE_OFFSET = RATTA_SN078_OFFSET;
	} else {
		RATTA_SLIDE_XSTART = RATTA_SN100_START;
		RATTA_SLIDE_XEND = RATTA_SN100_END;
		RATTA_SLIDE_OFFSET = RATTA_SN100_OFFSET;
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
	if (state) {
		/* touch */
		if ((pr->x == RATTA_INVALID_VALUE) ||
		    (pr->y == RATTA_INVALID_VALUE)) {
			pr->x = x;
			pr->y = y;
			pr->jiffs = jiffies;
			pr->done = false;
		} else if (((y > 0) && (pr->y <= 0)) ||
			   ((y > 0) &&
			    (abs(pr->y - y) > RATTA_DROP_WIDTH))) {
			pr->x = RATTA_INVALID_VALUE;
			pr->y = RATTA_INVALID_VALUE;
			pr->done = false;
		} else if ((pr->y <= 0) &&
			   (x >= RATTA_SLIDE_XSTART) &&
			   (x <= RATTA_SLIDE_XEND) &&
			   (((x > pr->x) &&
			     ((x - RATTA_SLIDE_XSTART) > RATTA_SLIDE_OFFSET)) ||
			    ((x < pr->x) &&
			     ((RATTA_SLIDE_XEND - x) > RATTA_SLIDE_OFFSET))) &&
			   (abs(x - pr->x) > RATTA_SLIDE_OFFSET)) {
			if (!pr->done) {
			    if ((jiffies_to_msecs(jiffies - pr->jiffs) /
			     1000) <= RATTA_SLIDE_INTERVAL) {
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
				ratta_report_slide(RATTA_SLIDE_DROP);
				pr->done = true;
			}
		}
	} else {
		/* leave */
		pr->x = RATTA_INVALID_VALUE;
		pr->y = RATTA_INVALID_VALUE;
		pr->done = false;
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
