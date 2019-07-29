#ifndef _EINK_TCON_H_
#define _EINK_TCON_H_

#include "panel.h"

struct eink_tcon {
	struct device *dev;
	void __iomem *regs;
	unsigned int *regcache;	/* register cache */
	unsigned int len;
	int irq;
	int id;
	struct clk *hclk;
	struct clk *dclk;
	struct clk *aclk;
	struct regmap *grf;
	struct phy *phy;

	struct eink_buffer *buf;
	void (*flush)(struct eink_tcon *tcon, struct eink_buffer *buf);
	int (*enable)(struct eink_tcon *tcon, struct eink_panel *panel);
	void (*disable)(struct eink_tcon *tcon);
};

static inline int eink_tcon_enable(struct eink_tcon *tcon,
				   struct eink_panel *panel)
{
	return tcon->enable(tcon, panel);
}

static inline void eink_tcon_disable(struct eink_tcon *tcon)
{
	tcon->disable(tcon);
}

static inline void eink_tcon_flush(struct eink_tcon *tcon,
				   struct eink_buffer *buf)
{
	tcon->flush(tcon, buf);
}

#endif	/* _EINK_TCON_H_ */
