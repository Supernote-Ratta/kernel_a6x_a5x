#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>

#include <linux/input/ratta_touch.h>

int volatile ratta_touch_ic = RATTA_TOUCH_IC_NONE;

bool ratta_touch_exist(int ic)
{
	return (ratta_touch_ic > RATTA_TOUCH_IC_NONE) && (ratta_touch_ic != ic);
}

void ratta_touch_set_ic(int ic)
{
	ratta_touch_ic = ic;
}

static int ratta_touch_probe(struct platform_device *pdev)
{
	return 0;
}

static int ratta_touch_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ratta_touch_of_match[] = {
	{ .compatible = "ratta,touch", },
	{},
};
MODULE_DEVICE_TABLE(of, ratta_touch_of_match);

static struct platform_driver ratta_touch_driver = {
	.driver = {
		.name = "ratta_touch",
		.owner = THIS_MODULE,
		.of_match_table = ratta_touch_of_match,
	},
	.probe = ratta_touch_probe,
	.remove = ratta_touch_remove,
};

static int __init ratta_touch_init(void)
{
	return platform_driver_register(&ratta_touch_driver);
}

fs_initcall(ratta_touch_init);
