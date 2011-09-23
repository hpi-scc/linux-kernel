/*
 *  linux/drivers/scc/sccsys.c
 *
 *  SCC system driver, partially based on the original sccmem.c driver
 *  distributed as part of SCC Linux.
 */

#include <linux/module.h>
#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <asm/io.h>

#include <linux/sccsys.h>

/* DEBUG messages */
#define DEBUG_MSG 1
#define PRINTD(format, args...) if (DEBUG_MSG) { printk(format, ##args); }

/* Symbols */
#define FIRST_MPB   0xc0000000
#define MPBADDRBITS 13
#define MPBSIZE     (1<<MPBADDRBITS)


/* Module parameters */
static int crb_offset = 0xE0000000;
module_param(crb_offset, int, 0644);
MODULE_PARM_DESC(crb_offset, "Physical start address of the register bank memory range");

static int grb_offset = 0xF9000000;
module_param(grb_offset, int, 0644);
MODULE_PARM_DESC(grb_offset, "Start address of the global register bank");

static int local_crb_offset = 0xF8000000;
module_param(local_crb_offset, int, 0644);
MODULE_PARM_DESC(local_crb_offset, "Start address of the local register bank");

static int disable_locking = 0;
module_param(disable_locking, int, 0644);
MODULE_PARM_DESC(disable_locking, "Enable/disable use of the test&set bits");

/* SCC System Context */
struct sccsys {
	void*			grb;
	void*			own_crb;
	int			own_tileid;
	scc_coord_t		own_coord;
	int			own_pid;
	void*			crb[SCC_TILECOUNT];
#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	spinlock_t		lock[SCC_CORECOUNT];
	unsigned int		lock_count[SCC_CORECOUNT];
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK
};

static struct sccsys		sccsys_buffer;
static struct sccsys*		sccsys = &sccsys_buffer;


#ifdef CONFIG_SCCSYS_FPGA_CLOCK
/* clocksource */
static struct clocksource clocksource_scc;

static cycle_t sccsys_read_clock(struct clocksource *cs)
{
	unsigned long lo, hi;

	/* Read global timestamp counter from GRB address space. We read the two
	 * halves separately, then check whether the high word has changed in
	 * between: if it has, the low word has overflowed, so we retry the
	 * operation. Otherwise, we are guaranteed to get a consistent pair of
	 * low and high word, and can construct a cycle_t from it. */
	do {
		hi = sccsys_read_grb_entry(0x8228);
		lo = sccsys_read_grb_entry(0x8224);
	} while (hi != sccsys_read_grb_entry(0x8228));

	return (cycle_t)(((unsigned long long)hi << 32) | lo);
}

static void sccsys_resume_clock(struct clocksource *cs)
{
	clocksource_scc.cycle_last = 0;
}

static struct clocksource clocksource_scc = {
	.name                   = "scc",
	.rating                 = 500,
	.read                   = sccsys_read_clock,
	.resume			= sccsys_resume_clock,
	.mask                   = CLOCKSOURCE_MASK(64),
	.flags                  = CLOCK_SOURCE_IS_CONTINUOUS,
};
#endif

/* cleanup */
static void sccsys_cleanup(void) {
	int i;

	for (i = 0; i < SCC_TILECOUNT; i++) {
		if (sccsys->crb[i]) {
			iounmap(sccsys->crb[i]);
			sccsys->crb[i] = NULL;
		}
	}
	if (sccsys->own_crb) {
		iounmap(sccsys->own_crb);
		sccsys->own_crb = NULL;
	}
	if (sccsys->grb) {
		iounmap(sccsys->grb);
		sccsys->grb = NULL;
	}
}

/* module initialization - called at module load time */
static int __init sccsys_init(void) {
	int i;
	void* own_mpb;
	int loopMPB;

	PRINTD(KERN_INFO "Starting up SCC system driver...\n");

	/* Zero out the sccsys structure */
	memset(sccsys, 0, sizeof(struct sccsys));

#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	for (i = 0; i < SCC_CORECOUNT; i++) {
		spin_lock_init(&sccsys->lock[i]);
	}
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK


	/* Begin with some sanity checks.
	 * *) We need to be running on a GaussLake CPU, which looks like a
	 *    regular P54C by its CPUID.
	 * *) The kernel must not have enabled PSE, as our MB bit shares its
	 *    location in the page tables with the PS bit. Enabling both
	 *    architectural extensions at the same time is not possible, neither
	 *    is to tell the kernel that PSE is not usable afterwards.
	 * *) We are running in a bare-metal environment, not paravirtualized
	 */
	if (!(boot_cpu_data.x86 == 5 && boot_cpu_data.x86_model == 2)) {
		printk(KERN_ERR "sccsys: Unknown CPU (%d:%d). This driver can only be used on GaussLake cores.\n",
			boot_cpu_data.x86,
			boot_cpu_data.x86_model);
		return -EINVAL;
	}
	if (read_cr4() & X86_CR4_PSE) {
		printk(KERN_ERR "sccsys: Unable to activate GaussLake extensions because the kernel has set CR4.PSE.\n");
		return -EINVAL;
	}
	if (!scc_bare_metal()) {
		printk(KERN_INFO "sccsys: Running on paravirtualized kernel.\n");
		return -EINVAL;
	}

	/* Map our own configuration registers and read the tileid */
	sccsys->own_crb = ioremap_nocache(local_crb_offset, SCC_CRB_SIZE);
	if (!sccsys->own_crb) {
		printk(KERN_ERR "sccsys: unable to map own CRB @%08x\n", local_crb_offset);
		return -ENOMEM;
	}

	sccsys->own_tileid = readl(sccsys->own_crb + SCC_TILEID);
	sccsys->own_coord  = scc_tileid_to_coord(sccsys->own_tileid);
	sccsys->own_pid    = scc_tileid_to_pid(sccsys->own_tileid);
	PRINTD(KERN_INFO "sccsys_init: starting on pid %02d (x=%d, y=%d, z=%d)\n",
		sccsys->own_pid,
		sccsys->own_coord.x, sccsys->own_coord.y, sccsys->own_coord.z);

	/* Map configuration registers of other tiles */
	for (i = 0; i < SCC_TILECOUNT; i++) {
		int crb_address = crb_offset + SCC_TILE_SIZE*i;
		sccsys->crb[i] = ioremap_nocache(crb_address, SCC_CRB_SIZE);
		if (!sccsys->crb[i]) {
			printk(KERN_ERR "sccsys: unable to map CRB of tile %d @%08x\n", i, crb_address);
			sccsys_cleanup();
			return -ENOMEM;
		}
	}

	/* Map global configuration registers */
	sccsys->grb = ioremap_nocache(grb_offset, SCC_GRB_SIZE);
	if (!sccsys->grb) {
		printk(KERN_ERR "sccsys: unable to map GRB @%08x\n", grb_offset);
		sccsys_cleanup();
		return -ENOMEM;
	}

	/* Initialize own message passing buffer */
	own_mpb = ioremap_nocache(FIRST_MPB+(sccsys->own_coord.x*0x01000000)+(sccsys->own_coord.y*0x06000000), 2*MPBSIZE);
	if (!own_mpb) {
		printk(KERN_ERR "sccsys: unable to map own MPB. GaussLake extensions will NOT be enabled on this core.\n");
		sccsys_cleanup();
		return -ENOMEM;
	}

	for (loopMPB = sccsys->own_coord.z*MPBSIZE; loopMPB < ((sccsys->own_coord.z+1)*MPBSIZE); loopMPB += 4) {
		writel(0, own_mpb + loopMPB);
	}
	iounmap(own_mpb);

	/* Enable GaussLake extensions. */
	PRINTD(KERN_INFO "sccsys_init: about to set CR4.MPE (%08x)\n", X86_CR4_MPE);
	set_in_cr4(X86_CR4_MPE);
	//__supported_pte_mask |= _PAGE_PMB;
	printk(KERN_INFO "sccsys_init: GaussLake extensions enabled.\n");

#ifdef CONFIG_SCCSYS_FPGA_CLOCK
	/* Register SCC clocksource */
	clocksource_register_khz(&clocksource_scc, 125000);
#endif

	return 0;
}

/* module unload */
static void __exit sccsys_exit(void)
{
	PRINTD(KERN_INFO "sccsys_exit: Shutting down SCC system driver...\n");
	sccsys_cleanup();
}


/*
 * Functions exported to other modules
 */

/* Get address of configuration register for a tile identified by a pid.
 * This routine is intended for tile-global registers that do not depend on
 * the target processor number. If a per-cpu configuration register is needed,
 * you can use sccsys_get_crb_entry_for_pid instead.
 */
static void* sccsys_get_crb_entry_for_tile_of_pid(scc_pid_t pid, int offset)
{
	void* address;

	if (pid < 0 || pid >= SCC_CORECOUNT) {
		printk(KERN_ERR "sccsys: get_crb_entry: invalid pid %02d\n", pid);
		BUG_ON(1);
		return NULL;
	}

	address = sccsys->crb[pid / 2];
	if (!address) {
		printk(KERN_ERR "sccsys: get_crb_entry: crb of pid %02d not mapped\n", pid);
		BUG_ON(1);
		return NULL;
	}

	return address + offset;
}

/* Get address of configuration register for the specified processor.
 * This routine is intended for per-cpu registers, so it takes two offsets.
 * cpu0_offset is used for the first processor of the tile (z=0),
 * cpu1_offset is used for the second one.
 */
static void* sccsys_get_crb_entry_for_pid(scc_pid_t pid, int cpu0_offset, int cpu1_offset)
{
	void* address;

	address = sccsys_get_crb_entry_for_tile_of_pid(pid, 0);
	if (!address) {
		return NULL;
	}

	if ((pid % 2) == 0) {
		return address + cpu0_offset;
	} else {
		return address + cpu1_offset;
	}
}


/* Get value of own tileid. The format is 0...0_00000yyy_yxxxxzzz (in bits).
 * Tile IDs are not consecutive; if a consecutive number is needed, consider
 * using the PID instead.
 */
int sccsys_get_tileid(void)
{
	return sccsys->own_tileid;
}
EXPORT_SYMBOL(sccsys_get_tileid);

/* Get own processor id. This is a consecutive number from 0 to SCC_CORECOUNT-1.
 */
scc_pid_t sccsys_get_pid(void)
{
	return sccsys->own_pid;
}
EXPORT_SYMBOL(sccsys_get_pid);

/* Get own coordinates. This is the decoded version of the TILEID returned by
 * sccsys_get_tileid. */
scc_coord_t sccsys_get_coord(void)
{
	return sccsys->own_coord;
}
EXPORT_SYMBOL(sccsys_get_coord);

/* Get logically next processor id. This is a consecutive number from 0 to
 * SCC_CORECOUNT-1, but is not neccessarily ((pid+1) % SCC_CORECOUNT). The
 * number is usually chosen to reflect the adjacent core having the shortest
 * distance, although it guarantees that the cycle closes after exactly
 * SCC_CORECOUNT invocations.
 */
scc_pid_t sccsys_get_next_pid(scc_pid_t pid)
{
#if SCC_CORECOUNT == 48
	if (pid >= 0 && pid < 11) {
		return pid + 1;
	} else if (pid == 11) {
		return 22;
	} else if ((pid == 12) || (pid >= 14 && pid <= 23)) {
		return (pid % 2) ? (pid - 3) : (pid + 1);
	} else if (pid == 13) {
		return 24;
	} else if (pid >= 24 && pid < 35) {
		return pid + 1;
	} else if (pid == 35) {
		return 46;
	} else if ((pid == 36) || (pid >= 38 && pid <= 47)) {
		return (pid % 2) ? (pid - 3) : (pid + 1);
	} else if (pid == 37) {
		return 0;
	} else {
		BUG();
	}
#else
	#error Unknown value for SCC_CORECOUNT.
#endif
}
EXPORT_SYMBOL(sccsys_get_next_pid);

/* Acquire the test&set register of the specified PID. This call returns 1 if
 * the lock has successfully been acquired, or 0 otherwise.
 *
 * In the current implementation, the call does only fail if the PID is invalid
 * (i.e., outside of the range from 0 to SCC_CORECOUNT-1) or mapping of the
 * configuration registers has failed.
 */
int sccsys_acquire_pid_lock(scc_pid_t pid)
{
#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	unsigned long flags;
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	void* lock_address;

	/* Simply ignore the test&set register if locking is disabled */
	if (disable_locking) {
		return 1;
	}
  
	/* Get address of test&set register for target  */
	lock_address = sccsys_get_crb_entry_for_pid(pid, SCC_LOCK0, SCC_LOCK1);
	if (!lock_address) {
		return 0;
	}

#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	spin_lock_irqsave(&sccsys->lock[pid], flags);

	/* Check for recursive acquire */
	if (++sccsys->lock_count[pid] != 1) {
		spin_unlock_irqrestore(&sccsys->lock[pid], flags);
		return 1;
	}
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	/* The LOCK bit is clear-on-read i.e. we have exclusive access when
	 * reading a '1'.
	 */
	while (!(readb((void*)lock_address) & 0x1)) cpu_relax();

#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	/* Release the spinlock after acquiring the PID lock, to guard against
	 * simultaneous calls from different threads. */
	spin_unlock_irqrestore(&sccsys->lock[pid], flags);
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	return 1;
}
EXPORT_SYMBOL(sccsys_acquire_pid_lock);

/* Release the test&set register of the specified PID. This call returns 1 if
 * the lock has successfully been released, or 0 otherwise.
 *
 * In the current implementation, the call does only fail if the PID is invalid
 * (i.e., outside of the range from 0 to SCC_CORECOUNT-1) or mapping of the
 * configuration registers has failed.
 */
int sccsys_release_pid_lock(scc_pid_t pid)
{
#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	unsigned long flags;
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	void* lock_address;

	/* Simply ignore the test&set register if locking is disabled */
	if (disable_locking) {
		return 1;
	}
  
	/* Get address of test&set register for target  */
	lock_address = sccsys_get_crb_entry_for_pid(pid, SCC_LOCK0, SCC_LOCK1);
	if (!lock_address) {
		return 0;
	}

#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	spin_lock_irqsave(&sccsys->lock[pid], flags);

	/* Check for recursive release */
	if (--sccsys->lock_count[pid] != 0) {
		spin_unlock_irqrestore(&sccsys->lock[pid], flags);
		return 1;
	}
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	/* The LOCK bit can be set by writing an arbitrary value to the
	   register.
	 */
	writeb(0, lock_address);

#ifdef CONFIG_SCCSYS_RECURSIVE_PID_LOCK
	/* Release the spinlock after releasing the PID lock, to guard against
	 * simultaneous calls from different threads. */
	spin_unlock_irqrestore(&sccsys->lock[pid], flags);
#endif // CONFIG_SCCSYS_RECURSIVE_PID_LOCK

	return 1;
}
EXPORT_SYMBOL(sccsys_release_pid_lock);

#define SCCSYS_TRIGGER_MODE_SET		0
#define SCCSYS_TRIGGER_MODE_EDGE_IRQ	1
#define SCCSYS_TRIGGER_MODE_LEVEL_IRQ	2

/* Trigger a bit in a core's configuration register */
static int sccsys_trigger_config_bits(scc_pid_t pid, unsigned int bits, unsigned int irq_clear_mask, int mode)
{
	void* conf_address;
	unsigned int value;

	/* Get address of per-cpu configuration register */
	conf_address = sccsys_get_crb_entry_for_pid(pid, SCC_GLCFG0, SCC_GLCFG1);
	if (!conf_address) {
		return 0;
	}

	/* Acquire the lock */
	if (!sccsys_acquire_pid_lock(pid)) {
		return 0;
	}

	value = readl(conf_address);
	if ((mode == SCCSYS_TRIGGER_MODE_LEVEL_IRQ) && (value & bits)) {
		value &= ~irq_clear_mask;
		writel(value, conf_address);
	}

	value |= bits;
	writel(value, conf_address);

	if (mode == SCCSYS_TRIGGER_MODE_EDGE_IRQ) {
		value &= ~irq_clear_mask;
		writel(value, conf_address);
	}

	/* Release the lock */
	sccsys_release_pid_lock(pid);

	return 1;
}

/* Set a bit in a core's configuration register */
int sccsys_set_config_bits(scc_pid_t pid, unsigned int bits)
{
	return sccsys_trigger_config_bits(pid, bits, 0, SCCSYS_TRIGGER_MODE_SET);
}
EXPORT_SYMBOL(sccsys_set_config_bits);

/* Trigger an interrupt to a core (directly via the configuration register) */
int sccsys_trigger_irq_direct(scc_pid_t pid, unsigned int bits, int edgeIrq)
{
	return sccsys_trigger_config_bits(pid, bits,
		SCC_INTR_MASK | SCC_NMI_MASK,
		edgeIrq ? SCCSYS_TRIGGER_MODE_EDGE_IRQ :
			  SCCSYS_TRIGGER_MODE_LEVEL_IRQ);
}
EXPORT_SYMBOL(sccsys_trigger_irq_direct);

/* Clear a bit for a core's configuration register */
int sccsys_clear_config_bits(scc_pid_t pid, unsigned int bits)
{
	void* conf_address;

	/* Get address of per-cpu configuration register */
	conf_address = sccsys_get_crb_entry_for_pid(pid, SCC_GLCFG0, SCC_GLCFG1);
	if (!conf_address) {
		return 0;
	}

	/* Acquire the lock */
	if (!sccsys_acquire_pid_lock(pid)) {
		return 0;
	}

	/* Clear the bits */
	writel(readl(conf_address) & ~bits, conf_address);

	/* Release the lock */
	sccsys_release_pid_lock(pid);

	return 1;
}
EXPORT_SYMBOL(sccsys_clear_config_bits);

/* Clear an interrupt request bit for a processor identified by pid. */
int sccsys_clear_irq_direct(scc_pid_t pid, unsigned int bits)
{
	return sccsys_clear_config_bits(pid, bits);
}
EXPORT_SYMBOL(sccsys_clear_irq_direct);

/* Read LUT entry */
scc_lut_t sccsys_read_lut_entry(scc_pid_t pid, unsigned int index)
{
	void* lut_address;
	scc_lut_t lut;

	lut.raw = 0;
	if (index > 255) {
		return lut;
	}

	/* Get address of per-cpu configuration register */
	lut_address = sccsys_get_crb_entry_for_pid(pid, SCC_LUT0, SCC_LUT1);
	if (!lut_address) {
		return lut;
	}

	lut.raw = readl(lut_address + SCC_LUT_STRIDE * index);
	return lut;
}
EXPORT_SYMBOL(sccsys_read_lut_entry);

/* Write LUT entry */
int sccsys_write_lut_entry(scc_pid_t pid, unsigned int index, scc_lut_t lut)
{
	void* lut_address;

	if (index > 255) {
		return 0;
	}

	/* Get address of per-cpu configuration register */
	lut_address = sccsys_get_crb_entry_for_pid(pid, SCC_LUT0, SCC_LUT1);
	if (!lut_address) {
		return 0;
	}

	writel(lut.raw, lut_address + SCC_LUT_STRIDE * index);
	return 1;
}
EXPORT_SYMBOL(sccsys_write_lut_entry);

/* Get address of mapped global configuration register bank */
void* sccsys_get_grb(void)
{
	return sccsys->grb;
}
EXPORT_SYMBOL(sccsys_get_grb);

/* Read global configuration register */
unsigned sccsys_read_grb_entry(unsigned int offset)
{
	return readl(sccsys->grb + offset);
}
EXPORT_SYMBOL(sccsys_read_grb_entry);

/* Convert a node-local physical address into a system address */
scc_addr_t sccsys_physical_to_system(scc_pid_t pid, unsigned long pa)
{
	unsigned int lut_index = (pa >> 24) & 0xFF;
	scc_lut_t lut = sccsys_read_lut_entry(pid, lut_index);
	scc_addr_t sccaddr;

	sccaddr.bypass = lut.bypass;
	sccaddr.y = lut.y;
	sccaddr.x = lut.x;
	sccaddr.subdest = lut.subdest;
	sccaddr.address = lut.address;
	sccaddr.offset = pa & 0x00FFFFFF;

	return sccaddr;
}
EXPORT_SYMBOL(sccsys_physical_to_system);

module_init(sccsys_init);
module_exit(sccsys_exit);
MODULE_DESCRIPTION("SCC system driver");
MODULE_AUTHOR("Jan-Arne Sobania <jan-arne.sobania@hpi.uni-potsdam.de>");

