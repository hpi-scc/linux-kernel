/*
 *  linux/drivers/char/sccproc/proc.c
 *
 *  proc filesystem extension for SCC hardware
 */

#include "sccproc.h"

/* cleanup /proc/scc directory */
static void sccproc_cleanup(void)
{
	sccproc_cleanup_core();
	sccproc_cleanup_net();
	sccproc_cleanup_base();
}

/* module initialization - called at module load time */
static int __init sccproc_init(void)
{
	int err;

	/* This driver does only work in a bare-metal environment */
	if (!scc_bare_metal()) {
		printk(KERN_INFO "sccproc: startup in non-SCC or paravirtualized environment.\n");
		return -EINVAL;
	}

	/* Init /proc/scc/ */
	err = sccproc_init_base();
	if (err < 0) {
		sccproc_cleanup();
		return err;
	}

	/* Init /proc/scc/core/ */
	err = sccproc_init_core();
	if (err < 0) {
		sccproc_cleanup();
		return err;
	}

	/* Init /proc/scc/net/ */
	err = sccproc_init_net();
	if (err < 0) {
		sccproc_cleanup();
		return err;
	}

	return 0;
}

/* module unload */
static void __exit sccproc_exit(void)
{
	sccproc_cleanup();
}

module_init(sccproc_init);
module_exit(sccproc_exit);
MODULE_DESCRIPTION("SCC procfs driver");
MODULE_AUTHOR("Jan-Arne Sobania <jan-arne.sobania@hpi.uni-potsdam.de>");

