/*
   800x480.S: EDID data set for 800x480 on XU4

   Copyright (C) 2015 Mauro Ribeiro <mauro.ribeiro@hardkernel.com>

	based on the original 1920x1080 file.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
*/

/* EDID */
#define VERSION 1
#define REVISION 3

/* Display */
#define CLOCK 27000 /* kHz */
#define XPIX 800
#define YPIX 480
#define XY_RATIO XY_RATIO_16_9
#define XBLANK 192
#define YBLANK 19
#define XOFFSET 24
#define XPULSE 72
#define YOFFSET (63+3)
#define YPULSE (63+10)
#define DPI 96
#define VFREQ 57 /* Hz */
#define TIMING_NAME "ODROID v1"
#define ESTABLISHED_TIMINGS_BITS 0x00 /* none */
#define HSYNC_POL 1
#define VSYNC_POL 1
#define CRC 0xdf

#include "edid.S"
