#export LD_LIBRARY_PATH=/opt/buildroot-ait84_201402v1/lib
#/;$(LD_LIBRARY_PATH)
export CROSS_COMPILE=/opt/buildroot-2014.11/bin/arm-buildroot-linux-uclibcgnueabi-
#export CROSS_COMPILE=/opt/arm-2008q3/bin/arm-none-linux-gnueabi-
#export ARCH=arm
make -C ~/linux/linux-3.2.7 M=$PWD
#/opt/buildroot-2014.11/bin/arm-linux-strip --strip-unneed ait-cam-codec.ko


