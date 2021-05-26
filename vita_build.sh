#!/bin/sh

set -xe

TOOLCHAIN=/opt/armv7-eabihf--glibc--bleeding-edge-2020.08-1/bin/arm-buildroot-linux-gnueabihf-

#cp arch/arm/configs/nintendo3ds_defconfig .config

make ARCH=arm CROSS_COMPILE=$TOOLCHAIN -j8
make ARCH=arm CROSS_COMPILE=$TOOLCHAIN vita1000.dtb vita2000.dtb pstv.dtb

echo "Output file: ./arch/arm/boot/zImage"
echo "Output DTB: ./arch/arm/boot/dts/"
