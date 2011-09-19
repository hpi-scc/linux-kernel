/*
 *  linux/drivers/char/sccproc/base.c
 *
 *  proc filesystem extension for SCC hardware
 */

#include "sccproc.h"

/* Root directory for SCC /proc entries. /proc/scc/ */
struct proc_dir_entry* sccproc_root_dir;
static struct proc_dir_entry* sccproc_pid_entry;
static struct proc_dir_entry* sccproc_tileid_entry;

/* Get root directory SCC /proc entries: /proc/scc/ */
struct proc_dir_entry* sccproc_get_root_dir(void)
{
	return sccproc_root_dir;
}
EXPORT_SYMBOL(sccproc_get_root_dir);

/* /proc/scc/pid */
static int sccproc_pid_show(struct seq_file *m, void *v)
{
	int pid;

	pid = sccsys_get_pid();

	seq_printf(m, "%02d", pid);

	return 0;
}

static int sccproc_pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_pid_show, NULL);
}

static const struct file_operations sccproc_pid_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_pid_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* /proc/scc/[core/<pid>/]tileid */
static int sccproc_hex_show(struct seq_file *m, void *v)
{
	int value = (int)m->private;

	seq_printf(m, "0x%08x", value);

	return 0;
}

static int sccproc_hex_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_hex_show, PDE(inode)->data);
}

const struct file_operations sccproc_hex_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_hex_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int sccproc_init_base(void)
{
	/* Create /proc/scc/ directory */
	sccproc_root_dir = proc_mkdir("scc", NULL);
	if (!sccproc_root_dir) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/ directory.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/pid file */
	if (!(sccproc_pid_entry = proc_create_data("pid", S_IFREG | S_IRUGO,
				sccproc_root_dir, &sccproc_pid_fops, NULL))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/pid file.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/tileid file */
	if (!(sccproc_tileid_entry = proc_create_data("tileid", S_IFREG | S_IRUGO,
				sccproc_root_dir, &sccproc_hex_fops, (void*)sccsys_get_tileid()))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/tileid file.\n");
		return -ENOMEM;
	}

	return 0;
}

void sccproc_cleanup_base(void)
{
	SCCPROC_DELETE(sccproc_pid_entry);
	SCCPROC_DELETE(sccproc_tileid_entry);
	SCCPROC_DELETE(sccproc_root_dir);
}
