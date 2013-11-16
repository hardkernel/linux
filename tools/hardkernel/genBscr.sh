#!/bin/bash

HDTV_TYPE="dvi hdmi"
HDTV_FORMAT="480p60hz 576p50hz 720p60hz 720p50hz 1080p60hz 1080i60hz 1080i50hz 1080p50hz 1080p30hz 1080p25hz 1080p24hz"

RAM=" mem=2047M" # should be mem=1023M for ODROID-X

for hdtv_type in `echo $HDTV_TYPE`; do
	for hdtv_format in `echo $HDTV_FORMAT`; do
		echo "setenv initrd_high \"0xffffffff\"" >> ./boot.tmp
		echo "setenv fdt_high \"0xffffffff\"" >> ./boot.tmp
		echo "setenv hdtv_type \"$hdtv_type\"" >> ./boot.tmp
		echo "setenv hdtv_format \"$hdtv_format\"" >> ./boot.tmp
		echo "setenv bootcmd \"fatload mmc 0:1 0x40008000 zImage; fatload mmc 0:1 0x42000000 uInitrd; bootm 0x40008000 0x42000000\"" >> ./boot.tmp
		echo "setenv bootargs \"console=tty1 console=ttySAC1,115200n8 hdtv_type=\${hdtv_type} hdtv_format=\${hdtv_format} root=/dev/mmcblk0p2 rootwait ro $RAM\"" >> ./boot.tmp
		echo "boot" >> ./boot.tmp
		mkimage -A arm -T script -C none -n "Boot.scr for $hdtv_type at $hdtv_format" -d ./boot.tmp ./boot-$hdtv_type-$hdtv_format.scr
		rm -rf boot.tmp
	done
done

