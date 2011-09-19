/*
 *  linux/drivers/scc/sccperf.c
 *
 * SCC performance meter driver, based on fs/proc/stat.c and the
 * modified CPUUTIL program distributed as part of SCC Linux.
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/time.h>
#include <asm/cputime.h>
#include <asm/io.h>
#include <linux/sccsys.h>

#ifndef arch_irq_stat_cpu
#define arch_irq_stat_cpu(cpu) 0
#endif
#ifndef arch_irq_stat
#define arch_irq_stat() 0
#endif
#ifndef arch_idle_time
#define arch_idle_time(cpu) 0
#endif

static struct timer_list sccperf_timer;
static volatile char* loadRegs = NULL;

/* Timer callback. This routine calculates CPU utilization by reading kernel
 * tick counters directly, then writes the result into shared memory where the
 * MCPC can read it.
 *
 * Code to read the timer ticks has been copied from the proc filesystem driver
 * (/fs/proc/stat.c!show_stat). The calculation of CPU utilization is based on
 * the modified CPUUTIL program distributed as part of SCC Linux by Intel.
 */
static void sccperf_timer_function(unsigned long data)
{
	static cputime64_t last_user, last_nice, last_system, last_idle;
	static cputime64_t last_iowait, last_irq, last_softirq, last_steal;
	static cputime64_t last_guest, last_guest_nice;
	static int first_callback = 1;

	if (!loadRegs) {
		printk(KERN_ERR "sccmem_cpuutil: unable to map SHM_X0_Y0\n");
	} else {
		int i;

		/* Get ticks, just like /fs/proc/stat.c */
		cputime64_t user, nice, system, idle, iowait, irq, softirq, steal;
		cputime64_t guest, guest_nice;

		user = nice = system = idle = iowait =
			irq = softirq = steal = cputime64_zero;
		guest = guest_nice = cputime64_zero;

		for_each_possible_cpu(i) {
			user = cputime64_add(user, kstat_cpu(i).cpustat.user);
			nice = cputime64_add(nice, kstat_cpu(i).cpustat.nice);
			system = cputime64_add(system, kstat_cpu(i).cpustat.system);
			idle = cputime64_add(idle, kstat_cpu(i).cpustat.idle);
			idle = cputime64_add(idle, arch_idle_time(i));
			iowait = cputime64_add(iowait, kstat_cpu(i).cpustat.iowait);
			irq = cputime64_add(irq, kstat_cpu(i).cpustat.irq);
			softirq = cputime64_add(softirq, kstat_cpu(i).cpustat.softirq);
			steal = cputime64_add(steal, kstat_cpu(i).cpustat.steal);
			guest = cputime64_add(guest, kstat_cpu(i).cpustat.guest);
			guest_nice = cputime64_add(guest_nice,
				kstat_cpu(i).cpustat.guest_nice);
		}

		/* Calculate differences to last iteration */
		if (!first_callback) {
			char utilization = 0;

			cputime64_t total;
			cputime64_t diff_user, diff_nice, diff_system;
			cputime64_t diff_idle, diff_iowait, diff_irq;
			cputime64_t diff_softirq, diff_steal, diff_guest;
			cputime64_t diff_guest_nice;

			diff_user = cputime64_sub(user, last_user);
			total = diff_user;
			diff_nice = cputime64_sub(nice, last_nice);
			total = cputime64_add(total, diff_nice);
			diff_system = cputime64_sub(system, last_system);
			total = cputime64_add(total, diff_system);
			diff_idle = cputime64_sub(idle, last_idle);
			total = cputime64_add(total, diff_idle);
			diff_iowait = cputime64_sub(iowait, last_iowait);
			total = cputime64_add(total, diff_iowait);
			diff_irq = cputime64_sub(irq, last_irq);
			total = cputime64_add(total, diff_irq);
			diff_softirq = cputime64_sub(softirq, last_softirq);
			total = cputime64_add(total, diff_softirq);
			diff_steal = cputime64_sub(steal, last_steal);
			total = cputime64_add(total, diff_steal);
			diff_guest = cputime64_sub(guest, last_guest);
			total = cputime64_add(total, diff_guest);
			diff_guest_nice = cputime64_sub(guest_nice, last_guest_nice);
			total = cputime64_add(total, diff_guest_nice);

			/* Calculate local CPU utilization */
			if ((unsigned long)total == 0) {
				utilization = -1;
			} else {
				/* Division of unsigned long long seems not be
				 * available, so unsigned long is the best we
				 * can use for this. If the timer period is not
				 * too long, this should work...
				 */
				utilization = (char)((unsigned long)(
					diff_user + diff_nice + diff_system
					) * 100 / (unsigned long)total);
			}

			*(loadRegs + sccsys_get_pid()) = utilization;
		}

		/* Save current values for next iteration */
		last_user = user;
		last_nice = nice;
		last_system = system;
		last_idle = idle;
		last_iowait = iowait;
		last_irq = irq;
		last_softirq = softirq;
		last_steal = steal;
		last_guest = guest;
		last_guest_nice = guest_nice;
		first_callback = 0;
	}

	mod_timer(&sccperf_timer, jiffies + HZ);
}

/* module initialization - called at module load time */
static int __init sccperf_init(void) {
	/* This driver does only work in a bare-metal environment */
	if (!scc_bare_metal()) {
		printk(KERN_INFO "sccperf: startup in non-SCC or paravirtualized environment.\n");
		return -EINVAL;
	}

	/* Map shared memory section belonging to the performance meter */
	loadRegs = ioremap_nocache(/*SHM_X0_Y0*/ 0x80000000 + 0x900, 0x1000);
	if (!loadRegs) {
		printk(KERN_ERR "sccperf_init: Could not map shared memory section\n");
		return -ENOMEM;
	}

	/* Start polling timer */
	init_timer(&sccperf_timer);
	sccperf_timer.function = sccperf_timer_function;
	mod_timer(&sccperf_timer, jiffies + HZ);

	return 0;
}

/* module unload */
static void __exit sccperf_exit(void)
{
	del_timer_sync(&sccperf_timer);
	iounmap(loadRegs);
}

module_init(sccperf_init);
module_exit(sccperf_exit);
MODULE_DESCRIPTION("SCC performance meter driver...");
MODULE_AUTHOR("Jan-Arne Sobania <jan-arne.sobania@hpi.uni-potsdam.de");

