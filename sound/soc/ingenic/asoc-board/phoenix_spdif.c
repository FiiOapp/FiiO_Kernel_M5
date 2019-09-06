 /*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author: cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>

void phoenix_spk_sdown(struct snd_pcm_substream *sps){

	return;
}

int phoenix_spk_sup(struct snd_pcm_substream *sps){
	
	return 0;
}

int phoenix_i2s_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params) {
	
	return 0;
};

int phoenix_i2s_hw_free(struct snd_pcm_substream *substream)
{
	/*notify release pll*/
	return 0;
};

static struct snd_soc_ops phoenix_i2s_ops = {
	.startup = phoenix_spk_sup,
	.shutdown = phoenix_spk_sdown,
	.hw_params = phoenix_i2s_hw_params,
	.hw_free = phoenix_i2s_hw_free,

};

static struct snd_soc_dai_link phoenix_dais[] = {
	[0] = {
		.name = "PHOENIX ICDC",
		.stream_name = "PHOENIX ICDC",
		.platform_name = "jz-asoc-aic-dma",
		.cpu_dai_name = "jz-asoc-aic-spdif",
		.codec_dai_name = "spdif-dump-dai",
		.codec_name = "spdif-dump",
		.ops = &phoenix_i2s_ops,
	},
	#ifdef CONFIG_SND_ASOC_JZ_PCM_V13
	[1] = {
		.name = "PHOENIX PCMBT",
		.stream_name = "PHOENIX PCMBT",
		.platform_name = "jz-asoc-pcm-dma",
		.cpu_dai_name = "jz-asoc-pcm",
		.codec_dai_name = "pcm dump dai",
		.codec_name = "pcm dump",
	},
	#endif
	#if CONFIG_SND_ASOC_JZ_DMIC_V13
	[2] = {
		.name = "PHOENIX DMIC",
		.stream_name = "PHOENIX DMIC",
		.platform_name = "jz-asoc-dmic-dma",
		.cpu_dai_name = "jz-asoc-dmic",
		.codec_dai_name = "dmic dump dai",
		.codec_name = "dmic dump",
	},
	#endif
};

static struct snd_soc_card phoenix = {
	.name = "phoenix",
	.owner = THIS_MODULE,
	.dai_link = phoenix_dais,
	.num_links = ARRAY_SIZE(phoenix_dais),
};

static int snd_phoenix_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk(">>>>File:%s Function:%s\n",__FILE__,__func__);
	phoenix.dev = &pdev->dev;
	ret = snd_soc_register_card(&phoenix);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
	return ret;
}

static int snd_phoenix_remove(struct platform_device *pdev)
{
	printk(">>>>Begin:File:%s Function:%s\n",__FILE__,__func__);
	snd_soc_unregister_card(&phoenix);
	platform_set_drvdata(pdev, NULL);
	printk(">>>>End:File:%s Function:%s\n",__FILE__,__func__);
	return 0;
}

static struct platform_driver snd_phoenix_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ingenic-alsa",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_phoenix_probe,
	.remove = snd_phoenix_remove,
};
//module_platform_driver(snd_phoenix_driver);

#ifdef CONFIG_FIIO_SOUND_BUILD_MODULE
//define
#include <soc/base.h>
#include <soc/irq.h>

#include <mach/platform.h>
#include <mach/jzdma.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>


static void snd_alsa_release(struct device *dev)
{
	printk("releasing '%s'\n", dev_name(dev));
}

struct platform_device snd_alsa_device = {
	.name = "ingenic-alsa",
	.dev = {
		.release = snd_alsa_release,
	},
};


#endif

static int jz_snd_phoenix_init(void)
{
	int err = 0;
	
	#ifdef CONFIG_FIIO_SOUND_BUILD_MODULE
		printk(">>>>File:%s Function:%s mode\n",__FILE__,__func__);
		err = platform_driver_register(&snd_phoenix_driver);
		if (err)
			return err;

		/* Register snd_uac2 device */
		err = platform_device_register(&snd_alsa_device);
		if (err) {
			platform_driver_unregister(&snd_phoenix_driver);
			return err;
		}
			
		return 0;
	#else
		printk(">>>>File:%s Function:%s\n",__FILE__,__func__);
		return platform_driver_register(&snd_phoenix_driver);
	#endif
}
module_init(jz_snd_phoenix_init);

static void jz_snd_phoenix_exit(void)
{
	printk(">>>>File:%s Function:%s\n",__FILE__,__func__);
	
	#ifdef CONFIG_FIIO_SOUND_BUILD_MODULE
		platform_driver_unregister(&snd_phoenix_driver);
		platform_device_unregister(&snd_alsa_device);
	#else
		platform_driver_unregister(&snd_phoenix_driver);
	#endif
}
module_exit(jz_snd_phoenix_exit);


MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC phoenix Snd Card");
MODULE_LICENSE("GPL");
