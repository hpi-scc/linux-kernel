/*
 *  Copyright 2011 Jan-Arne Sobania <jan-arne.sobania@hpi.uni-potsdam.de>, HPI
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_SCCSYS_H__
#define __LINUX_SCCSYS_H__

/* Query bus frequency at boot time. */
extern unsigned long scc_get_boot_busclock(void);

/* Check whether we are running on the lowest layer on SCC hardware */
extern int scc_bare_metal(void);

/* SCC-specific constants
 */

/* Number of cores on a SCC chip */
#define SCC_CORECOUNT		48
/* Number of tiles on a SCC chip */
#define SCC_TILECOUNT		24
/* Number of bytes in a cache line */
#define SCC_CLINE_SIZE		32
/* Raw size of the message passing buffer (bytes per core) */
#define SCC_MPB_SIZE		8192
/* Raw size of the configuration register bank (bytes per tile) */
#define SCC_CRB_SIZE		0x2000
/* Raw size of the global register bank */
#define SCC_GRB_SIZE		0x10000
/* Size of a SCC memory tile (16MB) */
#define SCC_TILE_SIZE		0x01000000

/* Stride between adjacent LUT entries */
#define SCC_LUT_STRIDE		8

/* Register offsets */
#define SCC_GLCFG0		0x0010
#define SCC_GLCFG1		0x0018
#define SCC_L2CFG0		0x0020
#define SCC_L2CFG1		0x0028
#define SCC_SENSOR		0x0040
#define SCC_GCBCFG		0x0080
#define SCC_TILEID		0x0100
#define SCC_LOCK0		0x0200
#define SCC_LOCK1		0x0400
#define SCC_LUT0		0x0800
#define SCC_LUT1		0x1000


/* Mask of the interrupt bits */
#define SCC_INTR_MASK		0x00000002
#define SCC_NMI_MASK		0x00000001


/* Global register offsets */
/* Please note the GRB space is particularly nasty, as accessing an undefined
 * register result in undefined behaviour, up to and including the whole SCC
 * locking up. */
#define SCCGRB_EMAC_MACBASE_HI	0x7E00
#define SCCGRB_EMAC_MACBASE_LO	0x7E04
#define SCCGRB_EMAC_IP_START	0x7E08
#define SCCGRB_EMAC_HOST_IP	0x7E0C
#define SCCGRB_EMAC_GW_IP	0x7E10

#define SCCGRB_FPGA_CONFIG	0x822C
#define SCCGRB_CLKFREQ		0x8230

#define SCCGRB_PRIVATE_SLOTS	0x8244


/* Valid sub destination IDs */
#define SCC_SUBDEST_CORE0	0
#define SCC_SUBDEST_CORE1	1
#define SCC_SUBDEST_CRB		2
#define SCC_SUBDEST_MPB		3
#define SCC_SUBDEST_PERIE	4
#define SCC_SUBDEST_PERIS	5
#define SCC_SUBDEST_PERIW	6
#define SCC_SUBDEST_PERIN	7

static const char* sccsys_subdest_names[] __attribute__((unused)) = {
	"CORE0", "CORE1", "-CRB-", "-MPB-", "PERIE", "PERIS", "PERIW", "PERIN"
};

/* The new instruction to flush message buffer content from L1 */
#define CL1FLUSHMB __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" )

typedef unsigned char scc_pid_t;
#define SCC_NO_PID ((scc_pid_t)(-1))

/* Perform hex dump */
static void __attribute__((unused)) sccutil_dump_buffer(unsigned char* buffer, unsigned len)
{
	char linebuf[16*3 + 10 + 64], *wptr;
	int li, ci;
	int lc = (len + 15) / 16;

	printk(KERN_INFO "<-- buffer @ %p, len = %x\n", buffer, len);
	for (li = 0; li < lc; li++) {
		int cc;

		wptr = linebuf;
		wptr += sprintf(wptr, "%04x: ", li * 16);

		cc = ((li + 1 == lc) && (len % 16)) ? (len % 16) : 16;

		for (ci = 0; ci < cc; ci++) {
			wptr += sprintf(wptr, " %02x", buffer[li*16+ci]);
		}
		for (; ci < 16; ci++) {
			wptr += sprintf(wptr, "   ");
		}
		wptr += sprintf(wptr, "  ");

		for (ci = 0; ci < cc; ci++) {
			char c = buffer[li*16+ci];
			if (c < 0x20 || c >= 0x7F) c = '.';
			wptr += sprintf(wptr, "%c", c);
		}
		for (; ci < 16; ci++) {
			wptr += sprintf(wptr, " ");
		}

		printk(KERN_CONT "%s\n", linebuf);
	}
	printk(KERN_CONT "-- end -->\n");
}


/* LookUp Table Entry */
typedef union scc_lut {
	struct {
		unsigned int address : 10;
		unsigned int subdest :  3;
		unsigned int x       :  4;
		unsigned int y       :  4;
		unsigned int bypass  :  1;
	};
	unsigned int raw;
} scc_lut_t;

#define sccsys_get_route(t)		(((t).y << 4) | (t).x)

/* Type for PFNs */
typedef unsigned long pfn_t;

#if PAGE_SHIFT != 12
#error SCCSYS expects a PAGE_SHIFT of 12.
#endif

/******************************************************************************/
/* Explicit cache control routines
 *
 * These routines are meant to allow direct control of the processor's caches
 * in regards to a specific page. */

/* Perform write-back/invalidate for all cachelines holding the specified pfn */
extern void scc_cop_wbinv_pfn(pfn_t pfn);

/* Perform write-back for all cachelines holding the specified pfn */
/* NOTE: This call must only be used if no invalidation is needed. However, it
 *       may be possible that invalidation is still performed, depending on
 *       whether the caches support issuing write-back only. */
#define scc_cop_wb_pfn(pfn)	scc_cop_wbinv_pfn(pfn)

/* Perform invalidate for all cachelines holding the specified pfn */
/* NOTE: This call may perform a writeback if required by the implementation.
 *       The caller merely provides a hint that a writeback is not needed. */
#define scc_cop_inv_pfn(pfn)	scc_cop_wbinv_pfn(pfn)

/* System address. */
typedef union scc_addr {
	struct {
		unsigned long long offset  : 24;
		unsigned long long address : 10;
		unsigned long long subdest :  3;
		unsigned long long x       :  4;
		unsigned long long y       :  4;
		unsigned long long bypass  :  1;
	};
	unsigned long long raw;
} scc_addr_t;

/* Convert a node-local physical address into a system address */
extern scc_addr_t sccsys_physical_to_system(scc_pid_t pid, unsigned long pa);

#define sccsys_local_physical_to_system(pa) \
	sccsys_physical_to_system(sccsys_get_pid(), pa);

/* System page frame number. */
typedef union scc_syspfn {
	struct {
		unsigned long long offset  : 12;
		unsigned long long address : 10;
		unsigned long long subdest :  3;
		unsigned long long x       :  4;
		unsigned long long y       :  4;
		unsigned long long bypass  :  1;
	};
	unsigned long long raw;
} scc_syspfn_t;

/* Get system pfn from system address */
static inline scc_syspfn_t sccsys_get_syspfn(scc_addr_t addr)
{
	scc_syspfn_t pfn;
	pfn.offset = addr.offset >> PAGE_SHIFT;
	pfn.address = addr.address;
	pfn.subdest = addr.subdest;
	pfn.x = addr.x;
	pfn.y = addr.y;
	pfn.bypass = addr.bypass;
	return pfn;
}

/*
 * Coordinates of a single component on the SCC die
 * This structure has specially been crafted s.t. its raw fields has the same
 * format as the TILEID configuration register.
 */
typedef union scc_coord {
	struct {
		int z : 3;
		int x : 4;
		int y : 4;
	};
	int raw;
} scc_coord_t;

/* Convert tileid into decoded coordinate structure. */
static inline scc_coord_t scc_tileid_to_coord(int tileid)
{
	scc_coord_t coord;
	coord.raw = tileid;
	return coord;
}

/* Convert decoded coordinate structure into tileid. */
static inline int scc_coord_to_tileid(scc_coord_t coord)
{
	return coord.raw;
}

/* Convert decoded coordinate structure into processor id. */
static inline scc_pid_t scc_coord_to_pid(scc_coord_t coord)
{
	scc_pid_t pid;
	pid=( ( coord.x + ( 6 * coord.y ) ) * 2 ) + coord.z;
	return pid;
}

/* Convert processor id to decoded coordinate structure. */
static inline scc_coord_t scc_pid_to_coord(scc_pid_t pid)
{
	scc_coord_t coord = { .raw = 0 };
	coord.z = (pid % 2);
	coord.y = (pid / 2) / 6;
	coord.x = (pid / 2) % 6;
	return coord;
}

/* Convert tileid to processor id. */
static inline scc_pid_t scc_tileid_to_pid(int tileid)
{
	return scc_coord_to_pid(scc_tileid_to_coord(tileid));
}

/* Convert processor id to tileid. */
static inline int scc_pid_to_tileid(scc_pid_t pid)
{
	return scc_coord_to_tileid(scc_pid_to_coord(pid));
}

/*
 * Global Clock Unit (GCU) Configuration Register
 */
typedef union scc_gckcfg {
	struct {
		unsigned long RESC0 : 1;
		unsigned long RESC1 : 1;
		unsigned long RESL20 : 1;
		unsigned long RESL21 : 1;
		unsigned long SREC0 : 1;
		unsigned long SREC1 : 1;
		unsigned long SREL21 : 1;
		unsigned long SREL20 : 1;
		unsigned long divider : 4;
		unsigned long ratio : 7;
		unsigned long router : 7;
		unsigned long reserved : 6;
	};
	unsigned long raw;
} scc_gckcfg_t;

/* Get value of own tileid. The format is 0...0_00000yyy_yxxxxzzz (in bits).
 * Tile IDs are not consecutive; if a consecutive number is needed, consider
 * using the PID instead.
 */
extern int sccsys_get_tileid(void);

/* Get own processor id. This is a consecutive number from 0 to SCC_CORECOUNT-1.
 */
extern scc_pid_t sccsys_get_pid(void);

/* Get own coordinates. This is the decoded version of the TILEID returned by
 * sccsys_get_tileid. */
extern scc_coord_t sccsys_get_coord(void);

/* Get logically next processor id. This is a consecutive number from 0 to
 * SCC_CORECOUNT-1, but is not neccessarily ((pid+1) % SCC_CORECOUNT). The
 * number is usually chosen to reflect the adjacent core having the shortest
 * distance, although it guarantees that the cycle closes after exactly
 * SCC_CORECOUNT invocations.
 */
extern scc_pid_t sccsys_get_next_pid(scc_pid_t pid);

/* Acquire the test&set register of the specified PID. This call returns 1 if
 * the lock has successfully been acquired, or 0 otherwise.
 *
 * In the current implementation, the call does only fail if the PID is invalid
 * (i.e., outside of the range from 0 to SCC_CORECOUNT-1) or mapping of the
 * configuration registers has failed.
 */
extern int sccsys_acquire_pid_lock(scc_pid_t pid);

/* Release the test&set register of the specified PID. This call returns 1 if
 * the lock has successfully been released, or 0 otherwise.
 *
 * In the current implementation, the call does only fail if the PID is invalid
 * (i.e., outside of the range from 0 to SCC_CORECOUNT-1) or mapping of the
 * configuration registers has failed.
 */
extern int sccsys_release_pid_lock(scc_pid_t pid);

/* Set a bit in a core's configuration register */
extern int sccsys_set_config_bits(scc_pid_t pid, unsigned int bits);

/* Clear a bit for a core's configuration register */
extern int sccsys_clear_config_bits(scc_pid_t pid, unsigned int bits);

/* Trigger an interrupt to a core (directly via the configuration register) */
extern int sccsys_trigger_irq_direct(scc_pid_t pid, unsigned int bits, int edgeIrq);

/* Clear an interrupt request bit for a processor identified by pid. */
extern int sccsys_clear_irq_direct(scc_pid_t pid, unsigned int bits);

/* Read LUT entry */
extern scc_lut_t sccsys_read_lut_entry(scc_pid_t pid, unsigned int index);

/* Write LUT entry */
extern int sccsys_write_lut_entry(scc_pid_t pid, unsigned int index, scc_lut_t lut);

/* Read Global Clock Unit (GCU) configuration register */
extern scc_gckcfg_t sccsys_read_gcbcfg(scc_pid_t pid);

/* Write Global Clock Unit (GCU) configuration register */
extern int sccsys_write_gcbcfg(scc_pid_t pid, scc_gckcfg_t cfg);

/* Get address of mapped global configuration register bank */
extern void* sccsys_get_grb(void);

/* Read global configuration register */
extern unsigned sccsys_read_grb_entry(unsigned int offset);

/* Write global configuration register */
extern void sccsys_write_grb_entry(unsigned int offset, unsigned value);

/* Read frequency of the fast clock from the global configuration register bank */
extern unsigned short sccsys_read_grb_fastclock(void);

#endif /* __LINUX_SCCSYS_H__ */
