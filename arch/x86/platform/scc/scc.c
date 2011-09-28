/*
 * scc.c: SCC platform specific setup code
 *
 * (C) Copyright 2011 Jan-Arne Sobania, Hasso-Plattner-Institut
 * Author: Jan-Arne Sobania (jan-arne.sobania@hpi.uni-potsdam.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mm_types.h>

#include <asm/setup.h>
#include <asm/apic.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/e820.h>
#include <asm/paravirt.h>

#include <linux/sccsys.h>	/* for CRB/GRB definitions only */


static int scc_subarch_enabled = 0;
static unsigned long scc_boot_busclock = CONFIG_SCC_BUSCLOCK;

/*
 * Here we do all the Local APIC initialization and stuff required to get the wormhole
 * IRQ delivered through LINT0 / 1
 *
 * LINT 0 services the UART at address 0x3f8 using IRQ4,
 * LINT 1 services the UART at address 0x2f8 using IRQ3.
 */

struct sccirq_entry {
	u32 reg;
	const char* name;
};

struct sccirq_entry sccirqs[] = {
	{ APIC_LVT1, "LINT1" },
	{ APIC_LVT0, "LINT0" }
};

#define SCCIRQ_FIRST_VECTOR	3
#define SCCIRQ_LAST_VECTOR	4

#define LVT0_IRQ 4
#define LVT1_IRQ 3

void scc_mask_lapic_lvt(u32 reg)
{
	apic_write(reg, apic_read(reg) | APIC_LVT_MASKED);
}

void scc_unmask_lapic_lvt(u32 reg)
{
	apic_write(reg, apic_read(reg) & ~APIC_LVT_MASKED);
}

static struct sccirq_entry* sccirq_get_entry(int irq)
{
	if (irq >= SCCIRQ_FIRST_VECTOR && irq <= SCCIRQ_LAST_VECTOR) {
		return &sccirqs[irq - SCCIRQ_FIRST_VECTOR];
	} else {
		panic("SCC: unable to map irq %d to LVT register.\n", irq);
	}
}

static void sccirq_mask_lapic(struct irq_data *data)
{
	scc_mask_lapic_lvt(sccirq_get_entry(data->irq)->reg);
}

static void sccirq_unmask_lapic(struct irq_data *data)
{
	scc_unmask_lapic_lvt(sccirq_get_entry(data->irq)->reg);
}

static void sccirq_ack_lapic(struct irq_data *data)
{
	ack_APIC_irq();
}

static struct irq_chip sccirq_lapic_chip __read_mostly = {
	.name		= "LAPIC-WORMHOLE",
	.irq_mask	= sccirq_mask_lapic,
	.irq_unmask	= sccirq_unmask_lapic,
	.irq_ack	= sccirq_ack_lapic,
	.irq_disable    = sccirq_mask_lapic,
};


void __init scc_setup_local_APIC_LINT(void)
{
	// set up LVT0 and LVT1 to point to IRQ4/3 vector
	// set up LINT 0 and LINT 1 IRQ handling through LAPIC
	apic_write(APIC_LVT0, (IRQ0_VECTOR + LVT0_IRQ) | APIC_LVT_MASKED | (APIC_MODE_FIXED << 8));
	apic_write(APIC_LVT1, (IRQ0_VECTOR + LVT1_IRQ) | APIC_LVT_MASKED | (APIC_MODE_FIXED << 8));
}

/*
 * Read a single unsigned int from a physical address. This routine maps the
 * page of physical memory using a non-caching version of early_ioremap, so it
 * should be called sparingly, and only until the full ioremap is available. */
static unsigned int __init scc_early_readl_phys(unsigned int phys)
{
	unsigned int phys_page = phys & ~(PAGE_SIZE - 1);
	unsigned int phys_offset = phys - phys_page;
	unsigned int value;
	void* base;

	base = early_ioremap_nocache(phys_page, PAGE_SIZE);
	value = readl(base + phys_offset);
	early_iounmap(base, PAGE_SIZE);

	return value;
}

/*
 * Query tile clock. This code is an exact duplicate of the one from sccfreq,
 * but we need to include it here because the driver is not initialized at this
 * point, or may never be loaded at all.
 */
static unsigned int __init scc_early_query_tile_frequency(void)
{
	unsigned int fastclock = 0;
	unsigned int divider = 0;
	unsigned int freq = 0;

	/* Get fastclock */
	fastclock = scc_early_readl_phys(0xF9000000 + SCCGRB_CLKFREQ) & 0xFFFF;

	/* Get core divider */
	divider = scc_early_readl_phys(0xF8000000 + SCC_GCBCFG);
	divider = (divider >> 8) & 0xF;
	divider++;

	/* Fallback to 533MHz */
	if (!divider || !fastclock) {
		freq = 533000;
	} else {
		/* Calculate new frequency */
		freq = fastclock / divider * 1000;
	}

	/* return frequency */
	return freq;
}

/*
 * SCC architecture initialization.
 */
static void __init scc_arch_setup(void)
{
	unsigned long real_busclock_khz;

	if (boot_cpu_data.x86 == 5 && boot_cpu_data.x86_model == 2) {
		printk(KERN_NOTICE "SCC: Intel GaussLake/P54C identified\n");

		/* Read busclock from FPGA */
		real_busclock_khz = scc_early_query_tile_frequency();

		if (scc_boot_busclock == real_busclock_khz * 1000) {
			printk(KERN_NOTICE "     Configured busclock of %lu.%03lu mHz matches detected busclock.\n",
				scc_boot_busclock / 1000000,
				(scc_boot_busclock / 1000) % 1000);
		} else {
			printk(KERN_NOTICE "     Overwriting configured busclock of %lu.%03lu mHz with detected busclock of %lu.%03lu mHz.\n",
				scc_boot_busclock / 1000000,
				(scc_boot_busclock / 1000) % 1000,
				real_busclock_khz / 1000,
				real_busclock_khz % 1000);
			scc_boot_busclock = real_busclock_khz * 1000;
		}
	} else
		printk(KERN_NOTICE "SCC: Unknown CPU (%d:%d)\n",
			boot_cpu_data.x86,
			boot_cpu_data.x86_model);
}

/* Get bus frequency at boot time. */
unsigned long scc_get_boot_busclock(void)
{
	return scc_boot_busclock;
}
EXPORT_SYMBOL_GPL(scc_get_boot_busclock);

/*
 * There is no legacy interrupt controller in the system, so there is also no
 * need to perform initialization of ISA interrupts.
 */
static void __init scc_pre_vector_init(void)
{
	int irq;

	/* Register wormhole interrupts. */
	for (irq = SCCIRQ_FIRST_VECTOR; irq <= SCCIRQ_LAST_VECTOR; irq++) {
		/* Add vector to IRQ mapping */
		per_cpu(vector_irq, 0)[IRQ0_VECTOR + irq] = irq;

		/* Set flags and install handler */
		irq_clear_status_flags(irq, IRQ_LEVEL);
		irq_set_chip_and_handler_name(irq, &sccirq_lapic_chip,
			handle_edge_irq, sccirq_get_entry(irq)->name);
	}
}

/*
 * The per-cpu clock source is the LAPIC clock. However, this routine also gets
 * called just in time to complete initialization of the wormhole interrupts.
 */
static void __init scc_setup_boot_cpu_clockev(void)
{
	scc_setup_local_APIC_LINT();
	setup_boot_APIC_clock();
}

/*
 * SCC systems don't have a secondary clock source, so we cannot perform the
 * default calibration protocol for the TSC. Instead, we rely on the kernel
 * configuration specifying the real bus block.
 */
static unsigned long __init scc_calibrate_tsc(void)
{
	unsigned long tsc_khz = scc_boot_busclock / 1000; /* in kHz */

	printk(KERN_INFO "SCC: TSC calibration skipped. Returning pre-configured BUSCLOCK value of %lu.%03lu mHz.\n",
		tsc_khz / 1000, tsc_khz % 1000);

	return tsc_khz;
}

/* SCC systems don't have an i8042 controller */
static int scc_i8042_detect(void)
{
	return 0;
}

/* Check whether we are running on the lowest layer on SCC hardware */
int scc_bare_metal(void)
{
	return scc_subarch_enabled && !paravirt_enabled();
}
EXPORT_SYMBOL(scc_bare_metal);

/*
 * SCC specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_scc_early_setup(void)
{
	/* We are about to use the SCC sub-architecture */
	scc_subarch_enabled = 1;

	/* Architecture initialization */
	x86_init.oem.arch_setup = scc_arch_setup;

	/* Static configuration of interrupt vectors */
	x86_init.irqs.pre_vector_init = scc_pre_vector_init;
	legacy_pic = &null_legacy_pic;

	/* Calibrate timestamp counter */
	x86_platform.calibrate_tsc = scc_calibrate_tsc;

	/* Timer. NOPed to prevent the legacy PIT from being initialized.
	 * Please note that legacy_pic->nr_legacy_itqs must be equal to 0,
	 * or io_apic.c!print_PIC will try to access PIT registers anyway.
	 */
	x86_init.timers.timer_init = x86_init_noop;

	/* Per-CPU clock source. This eventually delegates to the default
	 * routine of setup_boot_APIC_clock, but we need to complete LAPIC
	 * initialization first by registering the wormhole interrupts.
	 */
	x86_init.timers.setup_percpu_clockev = scc_setup_boot_cpu_clockev;

	/* Detect 8042 (keyboard controller). NOPed. */
	x86_platform.i8042_detect = scc_i8042_detect;

	/* Avoid searching for BIOS MP tables */
	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;

	/* Avoid searching for extension ROMs */
	x86_init.resources.probe_roms = x86_init_noop;

	/* Avoid reserving legacy PC resources */
	x86_init.resources.reserve_resources = x86_init_noop;
}
