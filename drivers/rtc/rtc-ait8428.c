/*
 * PS3 RTC Driver
 *
 * Copyright 2009 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <mach/mmpf_rtc.h>

static u64 read_rtc(void)
{
	int result;
	u64 tb_val;
	u64 rtc_val = 0;
	//result = lv1_get_rtc(&rtc_val, &tb_val);
	//BUG_ON(result);

	return rtc_val;
}

static int mcrv2_get_time(struct device *dev, struct rtc_time *tm)
{
	//rtc_time_to_tm(read_rtc() + ps3_os_area_get_rtc_diff(), tm);
/*	
	MMPF_RTC_TIME_FORMAT time;
	MMP_ERR MMPF_RTC_GetTime(&time)
	tm->tm_hour = time.usHour;
	tm->tm_year = time.usYear;
	tm->tm_mon = time.usMonth;
	tm->tm_yday = time.usDay;
	tm->tm_wday = time.usDayInWeek;
	tm->tm_sec = time.usSecond;
	tm->tm_min = time.usMinute;
	tm->tm_mday = 2;
	tm->tm_isdst = 0;
*/
	MMP_ULONG64 sec;
	sec = MMPF_RTC_GetTime_InSeconds();
	rtc_time_to_tm((unsigned long)sec,tm);
	return rtc_valid_tm(tm);
}

static int mcrv2_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long now;
	
	//rtc_tm_to_time(tm, &now);
	//ps3_os_area_set_rtc_diff(now - read_rtc());
	rtc_tm_to_time(tm,&now);
	MMPF_RTC_SetTimeInSec(now);
	return 0;
}

static const struct rtc_class_ops mcrv2_rtc_ops = {
	.read_time = mcrv2_get_time,
	.set_time = mcrv2_set_time,
};

static int __init mcrv2_rtc_probe(struct platform_device *dev)
{
	struct rtc_device *rtc;
	MMP_ERR res;

	res = MMPF_RTC_Initialize();
	if (res != MMP_ERR_NONE) {
		printk(KERN_ERR "[rtc-ait8428] Failed to initialize RTC: status=%d\n", res);
		return -EIO;
	}

	rtc = rtc_device_register("rtc-ait8428", &dev->dev, &mcrv2_rtc_ops,
				  THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(dev, rtc);
	return 0;
}

static int __exit mcrv2_rtc_remove(struct platform_device *dev)
{
	rtc_device_unregister(platform_get_drvdata(dev));
	return 0;
}

static struct platform_driver mcrv2_rtc_driver = {
	.driver = {
		.name = "rtc-ait8428",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(mcrv2_rtc_remove),
};

static int __init mcrv2_rtc_init(void)
{
	return platform_driver_probe(&mcrv2_rtc_driver, mcrv2_rtc_probe);
}

static void __exit mcrv2_rtc_fini(void)
{
	platform_driver_unregister(&mcrv2_rtc_driver);
}

module_init(mcrv2_rtc_init);
module_exit(mcrv2_rtc_fini);

MODULE_AUTHOR("A-I-T");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AIT8428 RTC driver");
MODULE_ALIAS("platform:AIT8428");
