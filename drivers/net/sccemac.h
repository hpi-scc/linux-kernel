/*******************************************************************************

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.

  Contact Information:
  Jan-Michael Brummer <jan-michael.brummer@intel.com>
  Intel Braunschweig

*******************************************************************************/

#ifndef EMAC_H
#define EMAC_H

#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>

#include <asm/cacheflush.h>
#include <asm/apic.h>

#include <asm/lapic.h>

#include <linux/sccsys.h>

#define DEBUG_EMAC

#ifdef DEBUG_EMAC
#define EPRINTK(_level, x...) do {\
	if (_level & debug_level) {\
			printk(KERN_INFO "%s(): ", __FUNCTION__);\
			printk(x);\
		}\
	} while ( 0 )
#else
#define EPRINTK(_level, x...) //
#endif

/* Debugging */
#define DEBUG_READ								0x01
#define DEBUG_WRITE								0x02
#define DEBUG_INFO								0x04
#define DEBUG_TXBUFFER							0x100

/* Read 16bit from buffer */
//#define U16(_addr)								(256 * (*((volatile u8*)
//							(_addr + 1))) + (*((volatile u8*)(_addr))))

#define U16(_addr)								(256 * (*((u8*)\
							(_addr + 1))) + (*((u8*)(_addr))))

/* MAC Address */
#define MAC_ADDRESS								0x00454D414331ULL
#define MAC_HI(_x)								((((_x) >> 32)) & 0xFFFF)
#define MAC_LO(_x)								(((_x) ) & 0xFFFFFFFF)

/* Cache line wrappers */
#define CLINE_SHIFT								5
#define CLINE_SIZE								(1UL << CLINE_SHIFT)
#define CLINE_MASK								(~(CLINE_SIZE - 1))
#define CLINE_ALIGN(_x)							(((_x) + CLINE_SIZE - 1) & CLINE_MASK)
#define CLINE_PACKETS(_x)						(CLINE_ALIGN(_x) >> CLINE_SHIFT)

/* Flush */
#define CL1FLUSH								__asm__ volatile (".byte 0x0F; .byte 0x0A;\n")

/* Limits */
/* Minimum buffer size must be 48 + 1! in order to handle a maximum ethernet frame */
/** Order of 9 = 2MB, order of 4 = 64k */
#define BUFFER_ORDER							9
#define BUFFER_NUM								(1 << BUFFER_ORDER)
#define BUFFER_SIZE								(BUFFER_NUM * PAGE_SIZE)

/* Mapping */
#define RA(_x, _y)								(sccsys_get_grb() + (_x) + (_y) * 4)

/* Interrupt configuration */
#ifdef USE_LVT0
	#define EMAC_LVT							APIC_LVT0
	#define EMAC_IRQ_MASK						0x00000002
	#define EMAC_IRQ_NR							4
	#define EMAC_IRQ_CONFIG						0
#else
	#define EMAC_LVT							APIC_LVT1
	#define EMAC_IRQ_MASK						0x00000001
	#define EMAC_IRQ_NR							3
	#define EMAC_IRQ_CONFIG						1
#endif

#define EMAC0									0x01
#define EMAC1									0x02
#define EMAC2									0x04
#define EMAC3									0x08

#define EMAC0_BASE								0x9000
#define EMAC1_BASE								0xA000
#define EMAC2_BASE								0xB000
#define EMAC3_BASE								0xC000

/* NAPI weight */
#define EMAC_NAPI_WEIGHT						(64)

/* EMAC RX */
#define EMAC_RX_BUFFER_START_ADDRESS			0x0000
#define EMAC_RX_BUFFER_READ_OFFSET				0x0100
#define EMAC_RX_BUFFER_WRITE_OFFSET				0x0200
#define EMAC_RX_BUFFER_SIZE						0x0300
#define EMAC_RX_BUFFER_THRESHOLD				0x0400
#define EMAC_RX_MODE							0x0500
#define EMAC_RX_NETWORK_PORT_MAC_ADDRESS_HI		0x0600
#define EMAC_RX_NETWORK_PORT_MAC_ADDRESS_LO		0x0700
#define EMAC_RX_NETWORK_PORT_ENABLE				0x0800

/* EMAC TX */
#define EMAC_TX_BUFFER_START_ADDRESS			0x0900
#define EMAC_TX_BUFFER_READ_OFFSET				0x0A00
#define EMAC_TX_BUFFER_WRITE_OFFSET				0x0B00
#define EMAC_TX_BUFFER_SIZE						0x0C00
#define EMAC_TX_MODE							0x0D00
#define EMAC_TX_NETWORK_PORT_ENABLE				0x0E00

/* Xilinx IP configuration - base address */
#define XILINX_EMAC0_BASE						0x3200
#define XILINX_EMAC1_BASE						0x4200
#define XILINX_EMAC2_BASE						0x5200
#define XILINX_EMAC3_BASE						0x6200

/* Xilinx IP configuration - offsets */
#define CONFIG_FLOW_CONTROL_ADD					0xC0
#define TRANSMITTER_ADDRESS						0x80
#define RECEIVER1_ADDRESS						0x40
#define CONFIG_ADD								0x100
#define ADD_FILTER_MOD							0x190

/* Xilinx ethernet statistic */
#define STAT0_TRBYTE							0x00004400
#define STAT0_REBYTE							0x00004408
#define STAT0_UFREC								0x00004410
#define STAT0_FRFRREC							0x00004418
#define STAT0_64BREC							0x00004420
#define STAT0_127BREC							0x00004428
#define STAT0_255BREC							0x00004430
#define STAT0_511BREC							0x00004438
#define STAT0_1023BREC							0x00004440
#define STAT0_MAXBREC							0x00004448
#define STAT0_OVFROK							0x00004450
#define STAT0_64BTRA							0x00004458
#define STAT0_127BTRA							0x00004460
#define STAT0_255BTRA							0x00004468
#define STAT0_511BTRA							0x00004470
#define STAT0_1023BTRA							0x00004478
#define STAT0_MAXBTRA							0x00004480
#define STAT0_OVSZTX							0x00004488
#define STAT0_FRRXOK							0x00004490
#define STAT0_FRCHERR							0x00004498
#define STAT0_BCFRRXOK							0x000044a0
#define STAT0_MCFRRXOK							0x000044a8
#define STAT0_CTFRRXOK							0x000044b0
#define STAT0_LGOUTRG							0x000044b8
#define STAT0_VLFRRXOK							0x000044c0
#define STAT0_PFRRXOK							0x000044c8
#define STAT0_CTRRXBAD							0x000044d0
#define STAT0_FRTRANOK							0x000044d8
#define STAT0_BCFRTXOK							0x000044e0
#define STAT0_MCFRTXOK							0x000044e8
#define STAT0_UNDERR							0x000044f0
#define STAT0_CTFRTXOK							0x000044f8
#define STAT0_VLFRTXOK							0x00004500
#define STAT0_PSFRTXOK							0x00004508
#define STAT0_SGLCOLFR							0x00004510
#define STAT0_MLTCOLFR							0x00004518
#define STAT0_DEFTRANS							0x00004520
#define STAT0_LATCOLL							0x00004528
#define STAT0_EXCCOLL							0x00004530
#define STAT0_FRWEXCD							0x00004538
#define STAT0_FRRXAERR							0x00004540
#define STAT0_UNDCOUNT							0x00004548

/** private network information */
struct emac_priv {
	/** network statistic */
	struct net_device_stats stats;
	/** indicates which device port is in use */
	u8 device;
	/* register base address */
	u32 base;
	/** flag for polling shutdown */
	u8 shutdown;
	/** own core id */
	u8 pid;
	/** rx ring buffer */
	u8 *rx_buffer;
	/** tx ring buffer */
	u8 *tx_buffer;
	/** maximum rx buffer level */
	u32 rx_buffer_max;
	/** current rx buffer level */
	u32 rx_read_offset;
	/** maximum tx buffer level */
	u32 tx_buffer_max;
	/** current tx buffer level */
	u32 tx_write_offset;
	/** napi structure */
	struct napi_struct napi;
	/** network device */
	struct net_device *net_device;
};

#endif
