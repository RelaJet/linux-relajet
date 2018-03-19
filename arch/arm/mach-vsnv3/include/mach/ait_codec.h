#ifndef __MCRV2_AFE_
#define __MCRV2_AFE_

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>

struct ait_codec{
	struct platform_device *pdev;
	struct snd_soc_codec *codec;
	u32 sysclk;
};

typedef struct ait_codec_data_{
	/* Device data */
	struct device *dev;
	struct platform_device *pdev;
	struct clk *clk;

	/* Memory resources */
	void __iomem *base;
	resource_size_t pbase;
	size_t base_size;

	/* Client devices */
	struct ait_codec aitaud;
	u8	mute;
	u16 digital_gain;
}ait_codec_data;



/* Register access macros */

#endif

