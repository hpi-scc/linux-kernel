/*
 *  linux/drivers/char/sccproc/net.c
 *
 *  proc filesystem extension for SCC hardware
 */

#include "sccproc.h"

/* /proc/scc/net/emac<idx>/ * */
static struct proc_dir_entry* sccproc_net_dir;
static struct proc_dir_entry* sccproc_base_ip_entry;
static struct proc_dir_entry* sccproc_host_entry;
static struct proc_dir_entry* sccproc_gw_entry;
static struct proc_dir_entry* sccproc_emac_ports_entry;

struct sccproc_emac_entry {
	struct proc_dir_entry* net_dir;
	struct proc_dir_entry* stat_entry;
};

static struct sccproc_emac_entry sccproc_emac_entries[4];

/* /proc/scc/net/[base_ip|host|gw] */
static int sccproc_ip_show(struct seq_file *m, void *v)
{
	unsigned ip = (unsigned)m->private;

	seq_printf(m, "%d.%d.%d.%d", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
		(ip >> 8) & 0xFF, ip & 0xFF);

	return 0;
}

static int sccproc_ip_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_ip_show, PDE(inode)->data);
}

static const struct file_operations sccproc_ip_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_ip_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* EMAC statistic names, copied from emactool.c */
static const char *emac_stat_name[46]= {
	"TRBYTE",
	"REBYTE",
	"UFREC",
	"FRFRREC",
	"64BREC",
	"127BREC",
	"255BREC",
	"511BREC",
	"1023BREC",
	"MAXBREC",
	"OVFROK",
	"64BTRA",
	"127BTRA",
	"255BTRA",
	"511BTRA",
	"1023BTRA",
	"MAXBTRA",
	"OVSZTX",
	"FRRXOK",
	"FRCHERR",
	"BCFRRXOK",
	"MCFRRXOK",
	"CTFRRXOK",
	"LGOUTRG",
	"VLFRRXOK",
	"PFFRRXOK",
	"CTRRXBAD",
	"LGOUTRG",
	"VLFFRXOK",
	"PFRRXOK",
	"CTRRXBAD",
	"FRTRANOK",
	"BCFRTXOK",
	"MCFRTXOK",
	"UNDERR",
	"CTFRTXOK",
	"VLFRTXOK",
	"PSFRTXOK",
	"SGLCOLFR",
	"MLTCOLFR",
	"DEFTRANS",
	"LATCOLL",
	"EXCOLL",
	"FRWEXCD",
	"FRRXAERR",
	"UNDCOUNT"
};

/* Base of EMAC statistics in GRB, copied from emactool.h */
#define STAT0_TRBYTE							0x00003400
#define STAT1_TRBYTE							0x00004400
#define STAT2_TRBYTE							0x00005400
#define STAT3_TRBYTE							0x00006400

static int sccproc_emac_stat_show(struct seq_file *m, void *v)
{
	int emac_index = (int)m->private, i;
	unsigned base;

	switch (emac_index) {
	case 0:
		base = STAT0_TRBYTE;
		break;
	case 1:
		base = STAT1_TRBYTE;
		break;
	case 2:
		base = STAT2_TRBYTE;
		break;
	case 3:
		base = STAT3_TRBYTE;
		break;
	default:
		seq_printf(m, "Unexpected call for emac%d\n", emac_index);
		return 0;
	}

	seq_printf(m, "Ethernet statistic for emac%d: \n", emac_index);
	seq_printf(m, "----------------------------\n");

	for (i = 0; i < 46; i++) {
		unsigned value = sccsys_read_grb_entry(base + i * 8);
		seq_printf(m, "%8.8s (0x%4x)  - %10d\n", emac_stat_name[i], base + i * 8, value);
	}
	seq_printf(m, "----------------------------\n");

	return 0;
}

static int sccproc_emac_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, sccproc_emac_stat_show, PDE(inode)->data);
}

static const struct file_operations sccproc_emac_stat_fops = {
	.owner		= THIS_MODULE,
	.open		= sccproc_emac_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sccproc_create_emac_entries(int emac_index)
{
	struct sccproc_emac_entry* net = &sccproc_emac_entries[emac_index];
	char dname[16];

	/* Make sure we only activate EMACs that are present in the bitstream. */
	if (!((sccsys_read_grb_entry(SCCGRB_FPGA_CONFIG) >> 9) &
		(1 << emac_index))) {
		memset(net, 0, sizeof(struct sccproc_emac_entry));
		return 0;
	}

	/* /proc/scc/net/emac<idx>/ */
	snprintf(dname, sizeof(dname), "emac%d", emac_index);

	net->net_dir = proc_mkdir(dname, sccproc_net_dir);
	if (!net->net_dir) {
		return -ENOMEM;
	}

	/* /proc/scc/net/emac<idx>/stat */
	net->stat_entry = proc_create_data("stat", S_IFREG | S_IRUGO,
		net->net_dir, &sccproc_emac_stat_fops, (void*)emac_index);
	if (!net->stat_entry) {
		return -ENOMEM;
	}

	return 0;
}

static void sccproc_delete_emac_entries(int emac_index)
{
	struct sccproc_emac_entry* net = &sccproc_emac_entries[emac_index];

	SCCPROC_DELETE(net->stat_entry);
	SCCPROC_DELETE(net->net_dir);
}

int sccproc_init_net(void)
{
	int i;

	/* Create /proc/scc/net/ directory */
	if (!(sccproc_net_dir = proc_mkdir("net", sccproc_root_dir))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/net directory.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/net/base_ip file */
	if (!(sccproc_base_ip_entry = proc_create_data("base_ip", S_IFREG | S_IRUGO,
				sccproc_net_dir, &sccproc_ip_fops, (void*)sccsys_read_grb_entry(SCCGRB_EMAC_IP_START)))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/net/base_ip file.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/net/gw file */
	if (!(sccproc_gw_entry = proc_create_data("gw", S_IFREG | S_IRUGO,
				sccproc_net_dir, &sccproc_ip_fops, (void*)sccsys_read_grb_entry(SCCGRB_EMAC_GW_IP)))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/net/gw file.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/net/host file */
	if (!(sccproc_host_entry = proc_create_data("host", S_IFREG | S_IRUGO,
				sccproc_net_dir, &sccproc_ip_fops, (void*)sccsys_read_grb_entry(SCCGRB_EMAC_HOST_IP)))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/net/host file.\n");
		return -ENOMEM;
	}

	/* Create /proc/scc/net/emac_ports file */
	if (!(sccproc_emac_ports_entry = proc_create_data("emac_ports", S_IFREG | S_IRUGO,
				sccproc_net_dir, &sccproc_hex_fops, (void*)((sccsys_read_grb_entry(SCCGRB_FPGA_CONFIG) >> 16) >> 9 & 0xF)))) {
		printk(KERN_ERR "sccproc_init: unable to create /proc/scc/net/emac_ports file.\n");
		return -ENOMEM;
	}

	/* Create individual emac entries */
	for (i = 0; i < 4; i++) {
		int err = sccproc_create_emac_entries(i);

		if (err < 0) {
			return err;
		}
	}

	return 0;
}

void sccproc_cleanup_net(void)
{
	int i;

	/* Delete entries from /proc/scc/net directory */
	for (i = 0; i < 4; i++) {
		sccproc_delete_emac_entries(i);
	}

	SCCPROC_DELETE(sccproc_base_ip_entry);
	SCCPROC_DELETE(sccproc_host_entry);
	SCCPROC_DELETE(sccproc_gw_entry);
	SCCPROC_DELETE(sccproc_emac_ports_entry);

	SCCPROC_DELETE(sccproc_net_dir);
}
