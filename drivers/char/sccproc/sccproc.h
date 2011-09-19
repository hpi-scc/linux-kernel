#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sccsys.h>

extern const struct file_operations sccproc_hex_fops;
extern struct proc_dir_entry* sccproc_root_dir;

#define SCCPROC_DELETE(entry) \
	if (entry) { \
		remove_proc_entry((entry)->name, (entry)->parent); \
		entry = NULL; \
	}

extern int sccproc_init_base(void);
extern void sccproc_cleanup_base(void);

extern int sccproc_init_core(void);
extern void sccproc_cleanup_core(void);

extern int sccproc_init_net(void);
extern void sccproc_cleanup_net(void);
