/*
 *  linux/drivers/char/sccproc/core.c
 *
 *  proc filesystem extension for SCC hardware
 */

#include "sccproc.h"

/* Root directory for SCC core entries. /proc/scc/core/ */
static struct proc_dir_entry *sccproc_core_dir;

struct sccproc_core_entry {
	struct proc_dir_entry* core_dir;
	struct proc_dir_entry* lutinfo;
	struct proc_dir_entry* lut_dir;
	struct proc_dir_entry* tileid;
	struct proc_dir_entry* lut[256];
};

struct proc_dir_entry* sccproc_core_self_entry;
struct sccproc_core_entry sccproc_core_entries[SCC_CORECOUNT];

/* /proc/scc/core/<pid>/lutinfo */
static void sccproc_print_lut(struct seq_file *m, scc_lut_t lut)
{
	int has_error = 0;

#define SET_LUT_ERROR(d...)	\
do { \
	if (!has_error) { \
		seq_printf(m, "    !" d); has_error = 1; \
	} else { \
		seq_printf(m, d); \
	} \
} while(0);

	/* Decode LUT entry */
	seq_printf(m, "%d_0x%x%x_%d(%s)_0x%03x",
		lut.bypass, lut.y, lut.x, lut.subdest,
		sccsys_subdest_names[lut.subdest], lut.address);

	/* Mark common configuration errors */

	/* Bypass does not work. MARC BUG #46 */
	if (lut.bypass) SET_LUT_ERROR("B");

	/* Tile coordinates: X = 0..5, Y = 0..3*/
	if ((lut.y > 3) || (lut.x > 5))  SET_LUT_ERROR("T");

	/*
	 * CORE0+CORE1 cannot really be accessed from scc
	 * hardware because they do not generate response packets, thus hanging
	 * the requesting core; note that the MCPC *CAN* access these, as it can
	 * just ignore that a response is missing.
	 */

	if ((lut.subdest == SCC_SUBDEST_CORE0) ||
	    (lut.subdest == SCC_SUBDEST_CORE1)) SET_LUT_ERROR("C");

	/* Nothing is connected to PERIN on any router */
	if ((lut.subdest == SCC_SUBDEST_PERIN) ||

	/* PERIS connects the VRC and SysIF (YX=(0,0) and (0,3), respectively) */
	    ((lut.subdest == SCC_SUBDEST_PERIS) && !(
		(lut.y == 0) && ((lut.x == 0) || (lut.x == 3))
	    )) ||

	/* PERIE is for memory controllers (YX=(0,5) and (2,5) only) */
	    ((lut.subdest == SCC_SUBDEST_PERIE) && !(
		((lut.y == 0) && (lut.x == 5)) || ((lut.y == 2) && (lut.x == 5))
	    )) ||

	/* PERIW is for memory controllers (YX=(0,0) and (2,0) only) */
	    ((lut.subdest == SCC_SUBDEST_PERIW) && !(
		((lut.y == 0) && (lut.x == 0)) || ((lut.y == 2) && (lut.x == 0))
	    )) ||

	/* That's it. MPB and CRB are available on all tiles. */
	    0) SET_LUT_ERROR("S");

	seq_printf(m, "\n");

#undef SET_LUT_ERROR
}

static int sccproc_lutinfo_show(struct seq_file *m, void *v)
{
	int pid = (int)m->private;
	unsigned int i;
	scc_lut_t lut;
	
	for (i = 0; i < 256; i++) {
		lut = sccsys_read_lut_entry(pid, i);

		seq_printf(m, "0x%02x: ", i);
		sccproc_print_lut(m, lut);
	}

	return 0;
}

static int sccproc_lutinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_lutinfo_show, PDE(inode)->data);
}

static const struct file_operations sccproc_lutinfo_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_lutinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* /proc/scc/core/<pid>/lut/<index> */
static int sccproc_lutentry_show(struct seq_file *m, void *v)
{
	int pid = ((int)m->private >> 8);
	int i = (int)m->private & 0xFF;
	scc_lut_t lut;
	
	lut = sccsys_read_lut_entry(pid, i);

	sccproc_print_lut(m, lut);

	return 0;
}

static int sccproc_lutentry_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_lutentry_show, PDE(inode)->data);
}

static const struct file_operations sccproc_lutentry_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_lutentry_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* /proc/scc/core/ * */
static int sccproc_create_core_entries(int core_index)
{
	struct sccproc_core_entry* core = &sccproc_core_entries[core_index];
	char dname[16];
	int i;

	/* /proc/scc/core/<pid>/ */
	snprintf(dname, sizeof(dname), "%02d", core_index);

	core->core_dir = proc_mkdir(dname, sccproc_core_dir);
	if (!core->core_dir) {
		return -ENOMEM;
	}

	/* /proc/scc/core/<pid>/tileid */
	core->tileid = proc_create_data("tileid", S_IFREG | S_IRUGO,
		core->core_dir, &sccproc_hex_fops, (void*)
		scc_pid_to_tileid(core_index));
	if (!core->tileid) {
		return -ENOMEM;
	}

	/* /proc/scc/core/<pid>/lutinfo */
	core->lutinfo = proc_create_data("lutinfo", S_IFREG | S_IRUGO,
		core->core_dir, &sccproc_lutinfo_fops, (void*)core_index);
	if (!core->lutinfo) {
		return -ENOMEM;
	}

	/* /proc/scc/core/<pid>/lut/ */
	core->lut_dir = proc_mkdir("lut", core->core_dir);
	if (!core->lut_dir) {
		return -ENOMEM;
	}

	/* /proc/scc/core/<pid>/lut/<lut_index> */
	for (i = 0; i < 256; i++) {
		snprintf(dname, sizeof(dname), "%02x", i);
		core->lut[i] = proc_create_data(dname, S_IFREG | S_IRUGO,
			core->lut_dir, &sccproc_lutentry_fops, (void*)(
			(core_index << 8) | i));
		if (!core->lut[i]) {
			return -ENOMEM;
		}
	}

	return 0;
}

static void sccproc_delete_core_entries(int core_index)
{
	int i;
	struct sccproc_core_entry* core = &sccproc_core_entries[core_index];

	if (core->lut_dir != NULL) {
		for (i = 0; i < 256; i++) {
			SCCPROC_DELETE(core->lut[i]);
		}
		SCCPROC_DELETE(core->lut_dir);
	}
	SCCPROC_DELETE(core->lutinfo);
	SCCPROC_DELETE(core->tileid);
	SCCPROC_DELETE(core->core_dir);
}

int sccproc_init_core(void)
{
	int i;
	char dname[16];

	/* Create /proc/scc/core/ directory */
	if (!(sccproc_core_dir = proc_mkdir("core", sccproc_root_dir))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/core directory.\n");
		return -ENOMEM;
	}

	/* Create symlink /proc/scc/core/self  --->  [/proc/scc/core/]<pid>/ */
	snprintf(dname, sizeof(dname), "%02d", sccsys_get_pid());
	if (!(sccproc_core_self_entry = proc_symlink("self", sccproc_core_dir, dname))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/core/self symlink to %s.\n", dname);
		return -ENOMEM;
	}

	/* Create per-core entries */
	for (i = 0; i < SCC_CORECOUNT; i++) {
		int err = sccproc_create_core_entries(i);

		if (err < 0) {
			return err;
		}
	}

	return 0;
}

void sccproc_cleanup_core(void)
{
	int i;

	for (i = 0; i < SCC_CORECOUNT; i++) {
		sccproc_delete_core_entries(i);
	}

	SCCPROC_DELETE(sccproc_core_self_entry);
	SCCPROC_DELETE(sccproc_core_dir);
}
