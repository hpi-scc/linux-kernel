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

#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <linux/cpufreq.h>
#include <linux/sccsys.h>

#include <asm/io.h>

#define MODVERSION	"0.2b"

/** Frequency mapping table */
/* The index member encodes the tile clock divisor for GCBCFG */
static struct cpufreq_frequency_table scc_freq_table[] = {
	{0x01, 800000},
	{0x02, 533000},
	{0x03, 400000},
	{0x04, 320000},
	{0x05, 266000},
	{0x06, 228000},
	{0x07, 200000},
	{0x08, 178000},
	{0x09, 160000},
	{0x0A, 145000},
	{0x0B, 133000},
	{0x0C, 123000},
	{0x0D, 114000},
	{0x0E, 106000},
	{0x0F, 100000},
	{0, CPUFREQ_TABLE_END},
};

/**
 * \brief Get current cpu frequency
 * \param cpu selected cpu (0)
 * \return frequency in khz
 */
static unsigned int scc_freq_get_cpu_frequency(unsigned int cpu) {
	unsigned int fastclock = 0;
	unsigned int divider = 0;
	unsigned int freq = 0;

	/* Get fastclock */
	fastclock = sccsys_read_grb_fastclock();

	/* Get core divider */
	divider = sccsys_read_gcbcfg(sccsys_get_pid()).divider;
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

/**
 * \brief Set current cpu state
 * \param state transition state
 */
static void scc_freq_set_cpu_state(unsigned int state) {
	struct cpufreq_freqs freqs;
	scc_gckcfg_t gcbcfg;
	unsigned int divider;

	/* Save old data and setup new values */
	freqs.old = scc_freq_get_cpu_frequency(0);
	freqs.new = scc_freq_table[state].frequency;
	freqs.cpu = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	printk(KERN_INFO "attempting to set frequency to %i kHz\n",
		scc_freq_table[state].frequency);

	local_irq_disable();

	/* Calculate new gcbcfg value and set it */
	gcbcfg = sccsys_read_gcbcfg(sccsys_get_pid());

	/* Get new divider value from frequency table. If it is out of range
	 * OR indicates a tile clock of 1600MHz, we force it down to 800MHz,
	 * although that should never happen unless we have serious memory
	 * corruption. */
	divider = (unsigned int)scc_freq_table[state].index;
	if (!divider || divider > 15) {
		divider = 2;
	}

	gcbcfg.divider = divider;
	gcbcfg.router = 7 * (divider + 1);

	sccsys_write_gcbcfg(sccsys_get_pid(), gcbcfg);

	local_irq_enable();

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
}

/**
 * \brief Verify selected frequency
 * \param policy pointer to cpufreq policy
 * \return result
 */
static int scc_freq_verify(struct cpufreq_policy *policy) {
	return cpufreq_frequency_table_verify(policy, &scc_freq_table[0]);
}

/**
 * \brief Get frequency target
 * \param policy pointer to cpufreq policy
 * \param target_freq target frequency
 * \param relation relation
 * \return 0 on success, otherwise error
 */
static int scc_freq_target(struct cpufreq_policy *policy, unsigned int target_freq, unsigned int relation) {
	unsigned int newstate = 0;

	/* On fault return -EINVAL */
	if (cpufreq_frequency_table_target(policy, scc_freq_table, target_freq, relation, &newstate)) {
		return -EINVAL;
	}

	scc_freq_set_cpu_state(newstate);

	return 0;
}

/**
 * \brief Initialize cpu frequency policy
 * \param policy pointer to cpufreq policy
 * \return 0 on success, otherwise error
 */
static int scc_freq_cpu_init(struct cpufreq_policy *policy) {
	struct cpuinfo_x86 *c = &cpu_data(0);
	int result;

	if (c->x86_vendor != 0 || c->x86 != 5 || c->x86_model != 2) {
		return -EINVAL;
	}

	/* cpuinfo and default policy values */
	policy->cpuinfo.transition_latency = 10000;
	policy->cur = scc_freq_get_cpu_frequency(0);

	result = cpufreq_frequency_table_cpuinfo(policy, scc_freq_table);
	if (result) {
		return result;
	}

	cpufreq_frequency_table_get_attr(scc_freq_table, policy->cpu);

	return 0;
}

/**
 * \brief Exit cpu frequency policy
 * \param policy pointer to cpufreq policy
 * \return 0
 */
static int scc_freq_cpu_exit(struct cpufreq_policy *policy) {
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

/** Frequency attribute table */
static struct freq_attr *scc_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

/** CPU Frequency driver structure */
static struct cpufreq_driver scc_freq_driver = {
	.get = scc_freq_get_cpu_frequency,
	.verify = scc_freq_verify,
	.target = scc_freq_target,
	.init = scc_freq_cpu_init,
	.exit = scc_freq_cpu_exit,
	.name = "scc_freq",
	.owner = THIS_MODULE,
	.attr = scc_freq_attr,
};

/**
 * \brief Initialze cpu frequency module
 * \return error code
 */
static int __init scc_freq_init(void) {
	struct cpuinfo_x86 *c = &cpu_data(0);

	if (c->x86_vendor != 0 || c->x86 != 5 || c->x86_model != 2) {
		return -EINVAL;
	}

	return cpufreq_register_driver(&scc_freq_driver);
}

/**
 * \brief Exit cpu frequency module
 */
static void __exit scc_freq_exit(void) {
	cpufreq_unregister_driver(&scc_freq_driver);
}

module_init(scc_freq_init);
module_exit(scc_freq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jan-Michael Brummer");
MODULE_VERSION(MODVERSION);
MODULE_DESCRIPTION("scc cpufreq driver");
