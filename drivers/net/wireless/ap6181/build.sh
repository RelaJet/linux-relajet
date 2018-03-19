#!/bin/bash
export CROSS_COMPILE=/opt/buildroot-vsnv3_2014q1/bin/arm-buildroot-linux-uclibcgnueabi-
export LINUX_SRC=/home/vin/customer/linux/svn_working2/branches/linux-3.2.7_SkypeCam
export CONFIG_BCMDHD=m
export ARCH=arm

make  O=/home/vin/customer/linux/svn_working2/branches/obj/
cp bcm_wlan.ko bcm_wlan_ap6181.ko
${CROSS_COMPILE}strip bcm_wlan_ap6181.ko --strip-unneeded 

