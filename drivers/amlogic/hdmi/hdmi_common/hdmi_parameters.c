/*
 * drivers/amlogic/hdmi/hdmi_common/hdmi_parameters.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <linux/kernel.h>
#include <linux/amlogic/hdmi_tx/hdmi_common.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

static struct hdmi_format_para fmt_para_1920x1080p60_16x9 = {
	.vic = HDMI_1920x1080p60_16x9,
	.name = "1920x1080p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 148500,
	.timing = {
		.pixel_freq = 148500,
		.h_freq = 67500,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2200,
		.h_blank = 280,
		.h_front = 88,
		.h_sync = 44,
		.h_back = 148,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank = 45,
		.v_front = 4,
		.v_sync = 5,
		.v_back = 36,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1920x1080p50_16x9 = {
	.vic = HDMI_1920x1080p50_16x9,
	.name = "1920x1080p50hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 148500,
	.timing = {
		.pixel_freq = 148500,
		.h_freq = 56250,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2640,
		.h_blank = 720,
		.h_front = 528,
		.h_sync = 44,
		.h_back = 148,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank = 45,
		.v_front = 4,
		.v_sync = 5,
		.v_back = 36,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1920x1080p24_16x9 = {
	.vic = HDMI_1920x1080p24_16x9,
	.name = "1920x1080p24hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 74250,
	.timing = {
		.pixel_freq = 74250,
		.h_freq = 27000,
		.v_freq = 24000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2750,
		.h_blank = 830,
		.h_front = 638,
		.h_sync = 44,
		.h_back = 148,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank = 45,
		.v_front = 4,
		.v_sync = 5,
		.v_back = 36,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p30_16x9 = {
	.vic = HDMI_3840x2160p30_16x9,
	.name = "3840x2160p30hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 297000,
	.timing = {
		.pixel_freq = 297000,
		.h_freq = 67500,
		.v_freq = 30000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 4400,
		.h_blank = 560,
		.h_front = 176,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p60_16x9 = {
	.vic = HDMI_3840x2160p60_16x9,
	.name = "3840x2160p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 135000,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 4400,
		.h_blank = 560,
		.h_front = 176,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p50_16x9 = {
	.vic = HDMI_3840x2160p50_16x9,
	.name = "3840x2160p50hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 112500,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5280,
		.h_blank = 1440,
		.h_front = 1056,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p25_16x9 = {
	.vic = HDMI_3840x2160p25_16x9,
	.name = "3840x2160p25hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 297000,
	.timing = {
		.pixel_freq = 297000,
		.h_freq = 56250,
		.v_freq = 25000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5280,
		.h_blank = 1440,
		.h_front = 1056,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p24_16x9 = {
	.vic = HDMI_3840x2160p24_16x9,
	.name = "3840x2160p24hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 297000,
	.timing = {
		.pixel_freq = 297000,
		.h_freq = 54000,
		.v_freq = 24000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5500,
		.h_blank = 1660,
		.h_front = 1276,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_4096x2160p24_256x135 = {
	.vic = HDMI_4096x2160p24_256x135,
	.name = "4096x2160p24hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 297000,
	.timing = {
		.pixel_freq = 297000,
		.h_freq = 54000,
		.v_freq = 24000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 4096,
		.h_total = 5500,
		.h_blank = 1404,
		.h_front = 1020,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1920x1080i60_16x9 = {
	.vic = HDMI_1920x1080i60_16x9,
	.name = "1920x1080i60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 0,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 74250,
	.timing = {
		.pixel_freq = 74250,
		.h_freq = 33750,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2200,
		.h_blank = 280,
		.h_front = 88,
		.h_sync = 44,
		.h_back = 148,
		.v_active = 1080/2,
		.v_total = 1125,
		.v_blank = 45/2,
		.v_front = 2,
		.v_sync = 5,
		.v_back = 15,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1920x1080i50_16x9 = {
	.vic = HDMI_1920x1080i50_16x9,
	.name = "1920x1080i50hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 0,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 74250,
	.timing = {
		.pixel_freq = 74250,
		.h_freq = 28125,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2640,
		.h_blank = 720,
		.h_front = 528,
		.h_sync = 44,
		.h_back = 148,
		.v_active = 1080/2,
		.v_total = 1125,
		.v_blank = 45/2,
		.v_front = 2,
		.v_sync = 5,
		.v_back = 15,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1280x720p60_16x9 = {
	.vic = HDMI_1280x720p60_16x9,
	.name = "1280x720p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 74250,
	.timing = {
		.pixel_freq = 74250,
		.h_freq = 45000,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1280,
		.h_total = 1650,
		.h_blank = 370,
		.h_front = 110,
		.h_sync = 40,
		.h_back = 220,
		.v_active = 720,
		.v_total = 750,
		.v_blank = 30,
		.v_front = 5,
		.v_sync = 5,
		.v_back = 20,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_1280x720p50_16x9 = {
	.vic = HDMI_1280x720p50_16x9,
	.name = "1280x720p50hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 74250,
	.timing = {
		.pixel_freq = 74250,
		.h_freq = 37500,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1280,
		.h_total = 1980,
		.h_blank = 700,
		.h_front = 440,
		.h_sync = 40,
		.h_back = 220,
		.v_active = 720,
		.v_total = 750,
		.v_blank = 30,
		.v_front = 5,
		.v_sync = 5,
		.v_back = 20,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_720x480p60_16x9 = {
	.vic = HDMI_720x480p60_16x9,
	.name = "720x480p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 27000,
	.timing = {
		.pixel_freq = 27000,
		.h_freq = 31469,
		.v_freq = 59940,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 720,
		.h_total = 858,
		.h_blank = 138,
		.h_front = 16,
		.h_sync = 62,
		.h_back = 60,
		.v_active = 480,
		.v_total = 525,
		.v_blank = 45,
		.v_front = 9,
		.v_sync = 6,
		.v_back = 30,
		.v_sync_ln = 7,
	},
};

static struct hdmi_format_para fmt_para_720x480i60_16x9 = {
	.vic = HDMI_720x480i60_16x9,
	.name = "720x480i60hz",
	.pixel_repetition_factor = 1,
	.progress_mode = 0,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 27000,
	.timing = {
		.pixel_freq = 27000,
		.h_freq = 15734,
		.v_freq = 59940,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 1440,
		.h_total = 1716,
		.h_blank = 276,
		.h_front = 38,
		.h_sync = 124,
		.h_back = 114,
		.v_active = 480/2,
		.v_total = 525,
		.v_blank = 45/2,
		.v_front = 4,
		.v_sync = 3,
		.v_back = 15,
		.v_sync_ln = 4,
	},
};

static struct hdmi_format_para fmt_para_720x576p50_16x9 = {
	.vic = HDMI_720x576p50_16x9,
	.name = "720x576p50hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 27000,
	.timing = {
		.pixel_freq = 27000,
		.h_freq = 31250,
		.v_freq = 50000,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 720,
		.h_total = 864,
		.h_blank = 144,
		.h_front = 12,
		.h_sync = 64,
		.h_back = 68,
		.v_active = 576,
		.v_total = 625,
		.v_blank = 49,
		.v_front = 5,
		.v_sync = 5,
		.v_back = 39,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_720x576i50_16x9 = {
	.vic = HDMI_720x576i50_16x9,
	.name = "720x576i50hz",
	.pixel_repetition_factor = 1,
	.progress_mode = 0,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 27000,
	.timing = {
		.pixel_freq = 27000,
		.h_freq = 15625,
		.v_freq = 50000,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 1440,
		.h_total = 1728,
		.h_blank = 288,
		.h_front = 24,
		.h_sync = 126,
		.h_back = 138,
		.v_active = 576/2,
		.v_total = 625,
		.v_blank = 49/2,
		.v_front = 2,
		.v_sync = 3,
		.v_back = 19,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x1080p120_16x9 = {
	.vic = HDMI_3840x1080p120hz,
	.name = "3840x1080p120hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 135000,
		.v_freq = 120000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 4400,
		.h_blank = 560,
		.h_front = 176,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank = 45,
		.v_front = 4,
		.v_sync = 5,
		.v_back = 36,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x1080p100_16x9 = {
	.vic = HDMI_3840x1080p100hz,
	.name = "3840x1080p100hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 112500,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5280,
		.h_blank = 1440,
		.h_front = 1056,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank = 45,
		.v_front = 4,
		.v_sync = 5,
		.v_back = 36,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x540p240_16x9 = {
	.vic = HDMI_3840x540p240hz,
	.name = "3840x540p240hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 135000,
		.v_freq = 120000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 4400,
		.h_blank = 560,
		.h_front = 176,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 540,
		.v_total = 562,
		.v_blank = 22,
		.v_front = 2,
		.v_sync = 2,
		.v_back = 18,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x540p200_16x9 = {
	.vic = HDMI_3840x1080p100hz,
	.name = "3840x1080p200hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 112500,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5280,
		.h_blank = 1440,
		.h_front = 1056,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 540,
		.v_total = 562,
		.v_blank = 22,
		.v_front = 2,
		.v_sync = 2,
		.v_back = 18,
		.v_sync_ln = 1,
	},
};

/* the following are for Y420 mode*/
static struct hdmi_format_para fmt_para_3840x2160p50_16x9_y420 = {
	.vic = HDMI_3840x2160p50_16x9_Y420,
	.name = "3840x2160p50hz420",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 112500,
		.v_freq = 50000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 5280,
		.h_blank = 1440,
		.h_front = 1056,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_3840x2160p60_16x9_y420 = {
	.vic = HDMI_3840x2160p60_16x9_Y420,
	.name = "3840x2160p60hz420",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 1,
	.tmds_clk_div40 = 1,
	.tmds_clk = 594000,
	.timing = {
		.pixel_freq = 594000,
		.h_freq = 135000,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 3840,
		.h_total = 4400,
		.h_blank = 560,
		.h_front = 176,
		.h_sync = 88,
		.h_back = 296,
		.v_active = 2160,
		.v_total = 2250,
		.v_blank = 90,
		.v_front = 8,
		.v_sync = 10,
		.v_back = 72,
		.v_sync_ln = 1,
	},
};

/* end of Y420 modes*/

static struct hdmi_format_para fmt_para_vesa_640x480p60_4x3 = {
	.vic = HDMIV_640x480p60hz,
	.name = "640x480p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 25175,
	.timing = {
		.pixel_freq = 25175,
		.h_freq = 26218,
		.v_freq = 59940,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 640,
		.h_total = 800,
		.h_blank = 160,
		.h_front = 16,
		.h_sync = 96,
		.h_back = 48,
		.v_active = 480,
		.v_total = 525,
		.v_blank = 45,
		.v_front = 10,
		.v_sync = 2,
		.v_back = 33,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_800x480p60_4x3 = {
	.vic = HDMIV_800x480p60hz,
	.name = "800x480p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 29760,
	.timing = {
		.pixel_freq = 29760,
		.h_freq = 30000,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 800,
		.h_total = 992,
		.h_blank = 192,
		.h_front = 24,
		.h_sync = 72,
		.h_back = 96,
		.v_active = 480,
		.v_total = 500,
		.v_blank = 20,
		.v_front = 3,
		.v_sync = 7,
		.v_back = 10,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_480x800p60_4x3 = {
	.vic = HDMIV_480x800p60hz,
	.name = "480x800p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 32000,
	.timing = {
		.pixel_freq = 32000,
		.h_freq = 52600,
		.v_freq = 62300,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 480,
		.h_total = 608,
		.h_blank = 128,
		.h_front = 40,
		.h_sync = 48,
		.h_back = 40,
		.v_active = 800,
		.v_total = 845,
		.v_blank = 45,
		.v_front = 13,
		.v_sync = 3,
		.v_back = 29,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_800x600p60_4x3 = {
	.vic = HDMIV_800x600p60hz,
	.name = "800x600p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 40000,
	.timing = {
		.pixel_freq = 66666,
		.h_freq = 37879,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 800,
		.h_total = 1056,
		.h_blank = 256,
		.h_front = 40,
		.h_sync = 128,
		.h_back = 88,
		.v_active = 600,
		.v_total = 628,
		.v_blank = 28,
		.v_front = 1,
		.v_sync = 4,
		.v_back = 23,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1024x600p60_17x10 = {
	.vic = HDMIV_1024x600p60hz,
	.name = "1024x600p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 50400,
	.timing = {
		.pixel_freq = 50400,
		.h_freq = 38280,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1024,
		.h_total = 1344,
		.h_blank = 320,
		.h_front = 24,
		.h_sync = 136,
		.h_back = 160,
		.v_active = 600,
		.v_total = 638,
		.v_blank = 38,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 29,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1024x768p60_4x3 = {
	.vic = HDMIV_1024x768p60hz,
	.name = "1024x768p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 79500,
	.timing = {
		.pixel_freq = 79500,
		.h_freq = 48360,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1024,
		.h_total = 1344,
		.h_blank = 320,
		.h_front = 24,
		.h_sync = 136,
		.h_back = 160,
		.v_active = 768,
		.v_total = 806,
		.v_blank = 38,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 29,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1280x800p60_8x5 = {
	.vic = HDMIV_1280x800p60hz,
	.name = "1280x800p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 83500,
	.timing = {
		.pixel_freq = 83500,
		.h_freq = 49380,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1280,
		.h_total = 1440,
		.h_blank = 160,
		.h_front = 48,
		.h_sync = 32,
		.h_back = 80,
		.v_active = 800,
		.v_total = 823,
		.v_blank = 23,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 14,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1280x1024p60_5x4 = {
	.vic = HDMIV_1280x1024p60hz,
	.name = "1280x1024p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 108000,
	.timing = {
		.pixel_freq = 108000,
		.h_freq = 64080,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1280,
		.h_total = 1688,
		.h_blank = 408,
		.h_front = 48,
		.h_sync = 112,
		.h_back = 248,
		.v_active = 1024,
		.v_total = 1068,
		.v_blank = 42,
		.v_front = 1,
		.v_sync = 3,
		.v_back = 38,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1360x768p60_16x9 = {
	.vic = HDMIV_1360x768p60hz,
	.name = "1360x768p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 855000,
	.timing = {
		.pixel_freq = 855000,
		.h_freq = 47700,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1360,
		.h_total = 1792,
		.h_blank = 432,
		.h_front = 64,
		.h_sync = 112,
		.h_back = 256,
		.v_active = 768,
		.v_total = 795,
		.v_blank = 27,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 18,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1366x768p60_16x9 = {
	.vic = HDMIV_1366x768p60hz,
	.name = "1366x768p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 85500,
	.timing = {
		.pixel_freq = 85500,
		.h_freq = 47880,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1366,
		.h_total = 1792,
		.h_blank = 426,
		.h_front = 70,
		.h_sync = 143,
		.h_back = 213,
		.v_active = 768,
		.v_total = 798,
		.v_blank = 30,
		.v_front = 3,
		.v_sync = 3,
		.v_back = 24,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1440x900p60_8x5 = {
	.vic = HDMIV_1440x900p60hz,
	.name = "1440x900p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 106500,
	.timing = {
		.pixel_freq = 106500,
		.h_freq = 56040,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1440,
		.h_total = 1904,
		.h_blank = 464,
		.h_front = 80,
		.h_sync = 152,
		.h_back = 232,
		.v_active = 900,
		.v_total = 934,
		.v_blank = 34,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 25,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1600x900p60_16x9 = {
	.vic = HDMIV_1600x900p60hz,
	.name = "1600x900p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 108000,
	.timing = {
		.pixel_freq = 108000,
		.h_freq = 60000,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1600,
		.h_total = 1800,
		.h_blank = 200,
		.h_front = 24,
		.h_sync = 80,
		.h_back = 96,
		.v_active = 900,
		.v_total = 1000,
		.v_blank = 100,
		.v_front = 1,
		.v_sync = 3,
		.v_back = 96,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1600x1200p60_4x3 = {
	.vic = HDMIV_1600x1200p60hz,
	.name = "1600x1200p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 156000,
	.timing = {
		.pixel_freq = 156000,
		.h_freq = 76200,
		.v_freq = 60000,
		.vsync_polarity = 0,
		.hsync_polarity = 0,
		.h_active = 1600,
		.h_total = 2048,
		.h_blank = 448,
		.h_front = 32,
		.h_sync = 160,
		.h_back = 256,
		.v_active = 1200,
		.v_total = 1270,
		.v_blank = 70,
		.v_front = 10,
		.v_sync = 8,
		.v_back = 52,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1680x1050p60_8x5 = {
	.vic = HDMIV_1680x1050p60hz,
	.name = "1680x1050p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 146250,
	.timing = {
		.pixel_freq = 146250,
		.h_freq = 65340,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 1680,
		.h_total = 2240,
		.h_blank = 560,
		.h_front = 104,
		.h_sync = 176,
		.h_back = 280,
		.v_active = 1050,
		.v_total = 1089,
		.v_blank = 39,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 30,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_1920x1200p60_8x5 = {
	.vic = HDMIV_1920x1200p60hz,
	.name = "1920x1200p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 154000,
	.timing = {
		.pixel_freq = 154000,
		.h_freq = 74040,
		.v_freq = 60000,
		.vsync_polarity = 0,
		.hsync_polarity = 1,
		.h_active = 1920,
		.h_total = 2080,
		.h_blank = 160,
		.h_front = 48,
		.h_sync = 32,
		.h_back = 80,
		.v_active = 1200,
		.v_total = 1235,
		.v_blank = 35,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 26,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_2560x1440p60_16x9 = {
	.vic = HDMIV_2560x1440p60hz,
	.name = "2560x1440p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 241500,
	.timing = {
		.pixel_freq = 241500,
		.h_freq = 88800,
		.v_freq = 60000,
		.vsync_polarity = 1,
		.hsync_polarity = 1,
		.h_active = 2560,
		.h_total = 2720,
		.h_blank = 160,
		.h_front = 48,
		.h_sync = 32,
		.h_back = 80,
		.v_active = 1440,
		.v_total = 1481,
		.v_blank = 41,
		.v_front = 2,
		.v_sync = 5,
		.v_back = 34,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_2560x1600p60_8x5 = {
	.vic = HDMIV_2560x1600p60hz,
	.name = "2560x1600p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 268500,
	.timing = {
		.pixel_freq = 268500,
		.h_freq = 98700,
		.v_freq = 60000,
		.vsync_polarity = 0, /* -VSync */
		.hsync_polarity = 1, /* +HSync */
		.h_active = 2560,
		.h_total = 2720,
		.h_blank = 160,
		.h_front = 48,
		.h_sync = 32,
		.h_back = 80,
		.v_active = 1600,
		.v_total = 1646,
		.v_blank = 46,
		.v_front = 3,
		.v_sync = 6,
		.v_back = 38,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_2560x1080p60_128x59 = {
	.vic = HDMIV_2560x1080p60hz,
	.name = "2560x1080p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 185580,
	.timing = {
		.pixel_freq = 185580,
		.h_freq = 66659,
		.v_freq = 60000,
		.vsync_polarity = 0, /* -VSync */
		.hsync_polarity = 1, /* +HSync */
		.h_active = 2560,
		.h_total = 2784,
		.h_blank = 224,
		.h_front = 64,
		.h_sync = 64,
		.h_back = 96,
		.v_active = 1080,
		.v_total = 1111,
		.v_blank = 31,
		.v_front = 3,
		.v_sync = 10,
		.v_back = 18,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_3440x1440p60_43x18 = {
	.vic = HDMIV_3440x1440p60hz,
	.name = "3440x1440p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 319750,
	.timing = {
		.pixel_freq = 319750,
		.h_freq = 88819,
		.v_freq = 60000,
		.vsync_polarity = 0, /* -VSync */
		.hsync_polarity = 1, /* +HSync */
		.h_active = 3440,
		.h_total = 3600,
		.h_blank = 160,
		.h_front = 48,
		.h_sync = 32,
		.h_back = 80,
		.v_active = 1440,
		.v_total = 1481,
		.v_blank = 41,
		.v_front = 3,
		.v_sync = 10,
		.v_back = 28,
		.v_sync_ln = 1,
	},
};

static struct hdmi_format_para fmt_para_vesa_480x320p60_3x2 = {
	.vic = HDMIV_480x320p60hz,
	.name = "480x320p60hz",
	.pixel_repetition_factor = 0,
	.progress_mode = 1,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
	.tmds_clk = 25200,
	.timing = {
		.pixel_freq = 25200,
		.h_freq = 31500,
		.v_freq = 60000,
		.vsync_polarity = 0, /* -VSync */
		.hsync_polarity = 0, /* -HSync */
		.h_active = 480,
		.h_total = 800,
		.h_blank = 320,
		.h_front = 120,
		.h_sync = 100,
		.h_back = 100,
		.v_active = 320,
		.v_total = 525,
		.v_blank = 205,
		.v_front = 8,
		.v_sync = 8,
		.v_back = 189,
		.v_sync_ln = 1,
	},
};

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
static struct hdmi_format_para fmt_para_custombuilt = {
	.vic = HDMIV_CUSTOMBUILT,
	.name = "custombuilt",
	.pixel_repetition_factor = 0,
	.scrambler_en = 0,
	.tmds_clk_div40 = 0,
};

struct modeline_table {
	/* resolutions */
	unsigned int horpixels;
	unsigned int verpixels;
	/* clock and frequency */
	unsigned int pixel_clock;
	unsigned int hor_freq;
	unsigned int ver_freq;
	/* htimings */
	unsigned int hdisp;
	unsigned int hsyncstart;
	unsigned int hsyncend;
	unsigned int htotal;
	/* vtiminigs */
	unsigned int vdisp;
	unsigned int vsyncstart;
	unsigned int vsyncend;
	unsigned int vtotal;
	/* polarity and scan mode */
	unsigned int hsync_polarity; /* 1:+hsync,0:-hsync */
	unsigned int vsync_polarity; /* 1:+vsync,0:-vsync */
	unsigned int progress_mode; /* 1: progress, 0: interlaced */
};

void debug_modeline(struct modeline_table tbl)
{
	pr_info("modeline - horpixels %d\n", tbl.horpixels);
	pr_info("modeline - verpixels %d\n", tbl.verpixels);
	pr_info("modeline - pixel_clock %d\n", tbl.pixel_clock);
	pr_info("modeline - hor_freq %d\n", tbl.hor_freq);
	pr_info("modeline - ver_freq %d\n", tbl.ver_freq);
	pr_info("modeline - hdisp %d\n", tbl.hdisp);
	pr_info("modeline - hsyncstart %d\n", tbl.hsyncstart);
	pr_info("modeline - hsyncend %d\n", tbl.hsyncend);
	pr_info("modeline - htotal %d\n", tbl.htotal);
	pr_info("modeline - vdisp %d\n", tbl.vdisp);
	pr_info("modeline - vsyncstart %d\n", tbl.vsyncstart);
	pr_info("modeline - vsyncend %d\n", tbl.vsyncend);
	pr_info("modeline - vtotal %d\n", tbl.vtotal);
	pr_info("modeline - hsync_polarity %d\n", tbl.hsync_polarity);
	pr_info("modeline - vsync_polarity %d\n", tbl.vsync_polarity);
	pr_info("modeline - progress_mode %d\n", tbl.progress_mode);
}

void debug_hdmi_fmt_param(struct hdmi_format_para param)
{
	pr_info("fmt_para - vic %d\n", param.vic);
	pr_info("fmt_para - name %s\n", param.name);
	pr_info("fmt_para - pixel_repetition_factor %d\n",
		param.pixel_repetition_factor);
	pr_info("fmt_para - progress mode %d\n", param.progress_mode);
	pr_info("fmt_para - scrambler_en %d\n", param.scrambler_en);
	pr_info("fmt_para - tmds_clk_div40 %d\n", param.tmds_clk_div40);
	pr_info("fmt_para - tmds_clk %d\n", param.tmds_clk);
	pr_info("fmt_para - pixel_freq %d\n", param.timing.pixel_freq);
	pr_info("fmt_para - h_freq %d\n", param.timing.h_freq);
	pr_info("fmt_para - v_freq %d\n", param.timing.v_freq);
	pr_info("fmt_para - hsync_polarity %d\n", param.timing.hsync_polarity);
	pr_info("fmt_para - vsync_polarity %d\n", param.timing.vsync_polarity);
	pr_info("fmt_para - h_active %d\n", param.timing.h_active);
	pr_info("fmt_para - h_total %d\n", param.timing.h_total);
	pr_info("fmt_para - h_blank %d\n", param.timing.h_blank);
	pr_info("fmt_para - h_front %d\n", param.timing.h_front);
	pr_info("fmt_para - h_sync %d\n", param.timing.h_sync);
	pr_info("fmt_para - h_back %d\n", param.timing.h_back);
	pr_info("fmt_para - v_active %d\n", param.timing.v_active);
	pr_info("fmt_para - v_total %d\n", param.timing.v_total);
	pr_info("fmt_para - v_blank %d\n", param.timing.v_blank);
	pr_info("fmt_para - v_front %d\n", param.timing.v_front);
	pr_info("fmt_para - v_sync %d\n", param.timing.v_sync);
	pr_info("fmt_para - v_back %d\n", param.timing.v_back);
}

/*
 * assuming modeline information from command line is as following.
 * setenv modeline
 * "horpixels,verpixels,pixel_clock,hor_freq,ver_freq
 * ,hdisp,hsyncstart,hsyncend,htotal,vdisp,vsyncstart,vsyncend,vtotal
 * ,hsync_polarity,vsync_polarity,progress_mode"
 */
static int __init setup_modeline(char *s)
{
	struct hdmi_cea_timing *custom_timing;
	struct modeline_table tbl;
	unsigned int *buf;
	char *item = NULL;
	unsigned long temp = 0;
	int ret;
	int i = 0;

	/* 1. parsing modeline information from command line */
	buf = (unsigned int *)&(tbl.horpixels);

	while (s != NULL) {
		item = strsep(&s, ",");
		ret = kstrtoul(item, 0, &temp);
		*(buf + i) = temp;
		i++;
		pr_info("modeline parsing - %ld\n", temp);
	}

	/* check parameters */
	debug_modeline(tbl);

	/* 2. build hdmi_format_para */
	fmt_para_custombuilt.progress_mode = tbl.progress_mode;

	fmt_para_custombuilt.tmds_clk = tbl.pixel_clock;
	fmt_para_custombuilt.timing.pixel_freq = tbl.pixel_clock;

	fmt_para_custombuilt.timing.h_freq = tbl.hor_freq;
	fmt_para_custombuilt.timing.v_freq = (tbl.ver_freq * 1000);

	/* check prototype of command line */
	fmt_para_custombuilt.timing.hsync_polarity = tbl.hsync_polarity;
	fmt_para_custombuilt.timing.vsync_polarity = tbl.vsync_polarity;

	/* h_active = hdisp */
	fmt_para_custombuilt.timing.h_active = tbl.hdisp;
	/* h_total = htotal */
	fmt_para_custombuilt.timing.h_total =  tbl.htotal;
	/* h_blank = htotal - hdisp */
	fmt_para_custombuilt.timing.h_blank = tbl.htotal - tbl.hdisp;
	/* h_front = hsyncstart - hdisp */
	fmt_para_custombuilt.timing.h_front = tbl.hsyncstart - tbl.hdisp;
	/* h_sync = hsyncend - hsyncstart */
	fmt_para_custombuilt.timing.h_sync = tbl.hsyncend - tbl.hsyncstart;
	/* h_back = (h_blank - (h_front + h_sync))*/
	fmt_para_custombuilt.timing.h_back
		= fmt_para_custombuilt.timing.h_blank
		- fmt_para_custombuilt.timing.h_front
		- fmt_para_custombuilt.timing.h_sync;

	/* v_active = vdisp */
	fmt_para_custombuilt.timing.v_active = tbl.vdisp;
	/* v_total = vtotal */
	fmt_para_custombuilt.timing.v_total = tbl.vtotal;
	/* v_blank = vtotal - vdisp */
	fmt_para_custombuilt.timing.v_blank = tbl.vtotal - tbl.vdisp;
	/* v_front = vsyncstart - vdisp */
	fmt_para_custombuilt.timing.v_front = tbl.vsyncstart - tbl.vdisp;
	/* v_sync = vsyncend - vsyncstart */
	fmt_para_custombuilt.timing.v_sync = tbl.vsyncend - tbl.vsyncstart;
	/* v_back = (v_blank - (v_front + v_sync)) */
	fmt_para_custombuilt.timing.v_back
		= fmt_para_custombuilt.timing.v_blank
		- fmt_para_custombuilt.timing.v_front
		- fmt_para_custombuilt.timing.v_sync;

	/* check parameters */
	debug_hdmi_fmt_param(fmt_para_custombuilt);

	/* 3. copy custom-built timing information for backup */
	custom_timing = get_custom_timing();
	memcpy(custom_timing, &fmt_para_custombuilt.timing,
		sizeof(fmt_para_custombuilt.timing));

	return 0;
}
__setup("modeline=", setup_modeline);
#endif /* CONFIG_ARCH_MESON64_ODROIDC2 */

static struct hdmi_format_para *all_fmt_paras[] = {
	&fmt_para_3840x2160p60_16x9,
	&fmt_para_3840x2160p50_16x9,
	&fmt_para_3840x2160p30_16x9,
	&fmt_para_3840x2160p25_16x9,
	&fmt_para_3840x2160p24_16x9,
	&fmt_para_4096x2160p24_256x135,
	&fmt_para_1920x1080p50_16x9,
	&fmt_para_1920x1080p60_16x9,
	&fmt_para_1920x1080p24_16x9,
	&fmt_para_1920x1080i60_16x9,
	&fmt_para_1920x1080i50_16x9,
	&fmt_para_1280x720p60_16x9,
	&fmt_para_1280x720p50_16x9,
	&fmt_para_720x480p60_16x9,
	&fmt_para_720x480i60_16x9,
	&fmt_para_720x576p50_16x9,
	&fmt_para_720x576i50_16x9,
	&fmt_para_3840x1080p100_16x9,
	&fmt_para_3840x1080p120_16x9,
	&fmt_para_3840x540p200_16x9,
	&fmt_para_3840x540p240_16x9,
	&fmt_para_3840x2160p60_16x9_y420,
	&fmt_para_3840x2160p50_16x9_y420,
	&fmt_para_vesa_640x480p60_4x3,
	&fmt_para_vesa_800x480p60_4x3,
	&fmt_para_vesa_480x800p60_4x3,
	&fmt_para_vesa_800x600p60_4x3,
	&fmt_para_vesa_1024x600p60_17x10,
	&fmt_para_vesa_1024x768p60_4x3,
	&fmt_para_vesa_1280x800p60_8x5,
	&fmt_para_vesa_1280x1024p60_5x4,
	&fmt_para_vesa_1360x768p60_16x9,
	&fmt_para_vesa_1366x768p60_16x9,
	&fmt_para_vesa_1440x900p60_8x5,
	&fmt_para_vesa_1600x900p60_16x9,
	&fmt_para_vesa_1600x1200p60_4x3,
	&fmt_para_vesa_1680x1050p60_8x5,
	&fmt_para_vesa_1920x1200p60_8x5,
	&fmt_para_vesa_2560x1440p60_16x9,
	&fmt_para_vesa_2560x1600p60_8x5,
	&fmt_para_vesa_2560x1080p60_128x59,
	&fmt_para_vesa_3440x1440p60_43x18,
	&fmt_para_vesa_480x320p60_3x2,
	&fmt_para_custombuilt,
	NULL,
};

struct hdmi_format_para *hdmi_get_fmt_paras(enum hdmi_vic vic)
{
	int i;
	for (i = 0; all_fmt_paras[i] != NULL; i++) {
		if (vic == all_fmt_paras[i]->vic)
			return all_fmt_paras[i];
	}
	return NULL;
}

/* For check all format parameters only */
void check_detail_fmt(void)
{
	int i;
	struct hdmi_format_para *p;
	struct hdmi_cea_timing *t;
	pr_warn("VIC Hactive Vactive I/P Htotal Hblank Vtotal Vblank Hfreq Vfreq Pfreq\n");
	for (i = 0; all_fmt_paras[i] != NULL; i++) {
		p = all_fmt_paras[i];
		t = &p->timing;
		pr_warn("%s[%d] %d %d %c %d %d %d %d %d %d %d\n",
			all_fmt_paras[i]->name, all_fmt_paras[i]->vic,
			t->h_active, t->v_active,
			(p->progress_mode) ? 'P' : 'I',
			t->h_total, t->h_blank, t->v_total, t->v_blank,
			t->h_freq, t->v_freq, t->pixel_freq);
	}

	pr_warn("\nVIC Hfront Hsync Hback Hpol Vfront Vsync Vback Vpol Ln\n");
	for (i = 0; all_fmt_paras[i] != NULL; i++) {
		p = all_fmt_paras[i];
		t = &p->timing;
	pr_warn("%s[%d] %d %d %d %c %d %d %d %c %d\n",
		all_fmt_paras[i]->name, all_fmt_paras[i]->vic,
		t->h_front, t->h_sync, t->h_back,
		(t->hsync_polarity) ? 'P' : 'N',
		t->v_front, t->v_sync, t->v_back,
		(t->vsync_polarity) ? 'P' : 'N',
		t->v_sync_ln);
	}

	pr_warn("\nCheck Horizon parameter\n");
	for (i = 0; all_fmt_paras[i] != NULL; i++) {
		p = all_fmt_paras[i];
		t = &p->timing;
	if (t->h_total != (t->h_active + t->h_blank))
		pr_warn("VIC[%d] Ht[%d] != (Ha[%d] + Hb[%d])\n",
		all_fmt_paras[i]->vic, t->h_total, t->h_active,
		t->h_blank);
	if (t->h_blank != (t->h_front + t->h_sync + t->h_back))
		pr_warn("VIC[%d] Hb[%d] != (Hf[%d] + Hs[%d] + Hb[%d])\n",
			all_fmt_paras[i]->vic, t->h_blank,
			t->h_front, t->h_sync, t->h_back);
	}

	pr_warn("\nCheck Vertical parameter\n");
	for (i = 0; all_fmt_paras[i] != NULL; i++) {
		p = all_fmt_paras[i];
		t = &p->timing;
		if (t->v_total != (t->v_active + t->v_blank))
			pr_warn("VIC[%d] Vt[%d] != (Va[%d] + Vb[%d]\n",
				all_fmt_paras[i]->vic, t->v_total, t->v_active,
				t->v_blank);
	if ((t->v_blank != (t->v_front + t->v_sync + t->v_back))
		& (p->progress_mode == 1))
		pr_warn("VIC[%d] Vb[%d] != (Vf[%d] + Vs[%d] + Vb[%d])\n",
			all_fmt_paras[i]->vic, t->v_blank,
			t->v_front, t->v_sync, t->v_back);
	if ((t->v_blank/2 != (t->v_front + t->v_sync + t->v_back))
		& (p->progress_mode == 0))
		pr_warn("VIC[%d] Vb[%d] != (Vf[%d] + Vs[%d] + Vb[%d])\n",
			all_fmt_paras[i]->vic, t->v_blank, t->v_front,
			t->v_sync, t->v_back);
	}
}

struct hdmi_audio_fs_ncts aud_32k_para = {
	.array[0] = {
	.tmds_clk = 25174,
	.n = 4576,
	.cts = 28125,
	},
	.array[1] = {
	.tmds_clk = 74176,
	.n = 11648,
	.cts = 210937,
	},
	.array[2] = {
	.tmds_clk = 148352,
	.n = 11648,
	.cts = 421875,
	},
	.array[3] = {
	.tmds_clk = 296703,
	.n = 5824,
	.cts = 421875,
	},
	.array[4] = {
	.tmds_clk = 297000,
	.n = 3072,
	.cts = 222750,
	},
	.def_n = 4096,
};

static struct hdmi_audio_fs_ncts *all_aud_paras[] = {
	NULL,
	&aud_32k_para,
};

unsigned int hdmi_get_aud_n_paras(enum hdmi_audio_fs fs, unsigned int tmds_clk)
{
	struct hdmi_audio_fs_ncts *p = NULL;
	unsigned int i;

	p = all_aud_paras[fs];
	for (i = 0; i < AUDIO_PARA_MAX_NUM; i++) {
		if (tmds_clk == p->array[i].tmds_clk)
			break;
	}

	if ((i < AUDIO_PARA_MAX_NUM) && (p->array[i].n))
		return p->array[i].n;
	else
		return p->def_n;
}
/*--------------------------------------------------------------*/
/* for csc coef */

static unsigned char coef_yc444_rgb_24bit_601[] = {
	0x20, 0x00, 0x69, 0x26, 0x74, 0xfd, 0x01, 0x0e,
	0x20, 0x00, 0x2c, 0xdd, 0x00, 0x00, 0x7e, 0x9a,
	0x20, 0x00, 0x00, 0x00, 0x38, 0xb4, 0x7e, 0x3b
};

static unsigned char coef_yc444_rgb_24bit_709[] = {
	0x20, 0x00, 0x71, 0x06, 0x7a, 0x02, 0x00, 0xa7,
	0x20, 0x00, 0x32, 0x64, 0x00, 0x00, 0x7e, 0x6d,
	0x20, 0x00, 0x00, 0x00, 0x3b, 0x61, 0x7e, 0x25
};


static struct hdmi_csc_coef_table hdmi_csc_coef[] = {
	{hdmi_color_format_444, hdmi_color_format_RGB, hdmi_color_depth_24B, 0,
		sizeof(coef_yc444_rgb_24bit_601), coef_yc444_rgb_24bit_601},
	{hdmi_color_format_444, hdmi_color_format_RGB, hdmi_color_depth_24B, 1,
		sizeof(coef_yc444_rgb_24bit_709), coef_yc444_rgb_24bit_709},
};

unsigned int hdmi_get_csc_coef(
	unsigned int input_format, unsigned int output_format,
	unsigned int color_depth, unsigned int color_format,
	unsigned char **coef_array, unsigned int *coef_length)
{
	unsigned int i = 0, max = 0;

	max = sizeof(hdmi_csc_coef)/sizeof(struct hdmi_csc_coef_table);

	for (i = 0; i < max; i++) {
		if ((input_format == hdmi_csc_coef[i].input_format) &&
			(output_format == hdmi_csc_coef[i].output_format) &&
			(color_depth == hdmi_csc_coef[i].color_depth) &&
			(color_format == hdmi_csc_coef[i].color_format)) {
			*coef_array = hdmi_csc_coef[i].coef;
			*coef_length = hdmi_csc_coef[i].coef_length;
			return 0;
		}
	}

	coef_array = NULL;
	*coef_length = 0;

	return 1;
}

