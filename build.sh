#!/bin/sh

export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm

make M=drivers/media/platform/s5p-tv modules
scp drivers/media/platform/s5p-tv/s5p-cec.ko root@x2_wifi:/root
