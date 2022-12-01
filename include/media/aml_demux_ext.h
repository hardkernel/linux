/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef AML_DEMUX_EXT_H_
#define AML_DEMUX_EXT_H_

#include <linux/types.h>
#include <media/demux.h>

struct dmx_demux_ext {
	struct dmx_demux dmx;
	int (*set_input)(struct dmx_demux *demux, int source);
	int (*get_ts_mem_info)(struct dmx_demux *demux,
			void *feed,
			void *info);
	int (*get_sec_mem_info)(struct dmx_demux *demux,
			void *feed,
			void *info);
	int (*set_hw_source)(struct dmx_demux *demux, int hw_source);
	int (*get_hw_source)(struct dmx_demux *demux, int *hw_source);
	int (*get_dmx_mem_info)(struct dmx_demux *demux,
			void *info);
	int (*set_sec_mem)(struct dmx_demux *demux,
			void *sec_mem);
	int (*get_dvr_mem)(struct dmx_demux *demux,
			void *info);
	int (*remap_pid)(struct dmx_demux *demux,
			u16 pids[2]);
	int (*decode_info)(struct dmx_demux *demux,
		void *info);
};

#endif
