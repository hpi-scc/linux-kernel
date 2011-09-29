/*
 *  linux/drivers/char/sccmem.c
 *  
 *  -> derived from:
 *  linux/drivers/char/mem.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Added devfs support. 
 *    Jan-11-1998, C. Scott Ananian <cananian@alumni.princeton.edu>
 *  Shared /dev/zero mmaping support, Feb 2000, Kanoj Sarcar <kanoj@sgi.com>
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/highmem.h>

#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif
#include <asm/io.h>

#include <linux/sccsys.h>

/* DEBUG messages */
#define DEBUG_MSG 0
#define MAJOR_ADDR 222
#define PRINTD(format, args...) if (DEBUG_MSG) { printk(format, ##args); }

/* Symbols */
#define OWN_MPB     0xd8000000

#define MPBADDRBITS 13
#define MPBSIZE     (1<<MPBADDRBITS)

/* PSE bit for Pentium+ equals MPE (message buffer enable) flag in SCC! So, use it to create _PAGE_MPB symbol... */
#define _PAGE_MPE _PAGE_PSE

/* In order to force write-backs of modified data from the L2 cache we have
 * to read new lines into every way. At 256kB, 4-way, 32B/line there are
 * 2048 sets, i.e. the distances between addresses in a pair of ways is
 * 2048*32B = 64kB.
 */
#define L2_LINESIZE 32UL
#define L2_WAYS     4UL
#define L2_CAPACITY (256*1024UL)
#define L2_WBSTRIDE (L2_CAPACITY/L2_WAYS)

/* Enable this to have the L2 flush routine execute a 'wbinvd'
 * instruction before doing the actual flush. */
// #define FLUSH_USES_WBINVD


/* Virtual address and current offset in our flush-data block,
 * which will be in an unused MPB address range */
static size_t OWN_MPB_vaddr = 0;
static size_t OWN_MPB_offset = 0;


/* Methods of the character device */
static int sccmem_open(struct inode *inode, struct file *filp);
static int sccmem_release(struct inode *inode, struct file *filp);
static int sccdcm_mmap(struct file *filp, struct vm_area_struct *vma);
static int sccncm_mmap(struct file *filp, struct vm_area_struct *vma);
static int sccmpb_mmap(struct file *filp, struct vm_area_struct *vma);
/* For simplicity reasons, exising file operations are overloaded with
 * new functionality:
 * -read()  returns the physical address of the buffer
 * -write() force write-backs of modified data from the caches
 */
static ssize_t sccdcm_read(struct file *filp,
                           char __user *buf, 
                           size_t count, 
                           loff_t *ppos);
static ssize_t sccdcm_write(struct file *filp, 
                            const char __user *buf, 
                            size_t count, 
                            loff_t *ppos);

/* The file operations, i.e. all character device methods */
static struct file_operations sccdcm_fops = {
	.open = sccmem_open, 
        .release = sccmem_release, 
        .mmap = sccdcm_mmap, 
        .read = sccdcm_read,
        .write = sccdcm_write,
        .owner = THIS_MODULE
};

static struct file_operations sccncm_fops = {
	.open = sccmem_open, 
        .release = sccmem_release, 
        .mmap = sccncm_mmap, 
        .owner = THIS_MODULE
};

static struct file_operations sccmpb_fops = {
	.open = sccmem_open, 
        .release = sccmem_release, 
        .mmap = sccmpb_mmap, 
        .owner = THIS_MODULE
};

/* character device open method */
static int sccmem_open(struct inode *inode, struct file *filp) {
  return 0;
}

/* character device last close method */
static int sccmem_release(struct inode *inode, struct file *filp) {
  return 0;
}

/* Character device mmap method for noncachable memory mapped registers like CRB->GLCFG0 */
static int sccncm_mmap(struct file *filp, struct vm_area_struct *vma) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  size_t size = vma->vm_end - vma->vm_start;

  PRINTD(KERN_DEBUG "sccncm_mmap: Device 0x%08lx (%d:%d),\n", (unsigned long)filp->f_dentry->d_inode->i_rdev, major, minor);
  PRINTD(KERN_DEBUG "             VM Start: 0x%08lx, size: 0x%08lx, offset: 0x%08lx\n", (unsigned long) vma->vm_start, (unsigned long) size, (unsigned long) vma->vm_pgoff);

  /* Mark the page protection value as "uncacheable" */
  vma->vm_page_prot = __pgprot(pgprot_val(vma->vm_page_prot) | _PAGE_PCD | _PAGE_PWT);
  PRINTD(KERN_DEBUG "             VM pgprot value is 0x%02lx!\n", pgprot_val(vma->vm_page_prot));

  /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
  if (remap_pfn_range(vma,  vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot)) return -EAGAIN;

  return 0;
}

/* Character device mmap method for cachable regular memory */
static int sccdcm_mmap(struct file *filp, struct vm_area_struct *vma) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  size_t size = vma->vm_end - vma->vm_start;

  PRINTD(KERN_DEBUG "sccdcm_mmap: Device 0x%08lx (%d:%d),\n", (unsigned long)filp->f_dentry->d_inode->i_rdev, major, minor);
  PRINTD(KERN_DEBUG "             VM Start: 0x%08lx, size: 0x%08lx, offset: 0x%08lx\n", (unsigned long) vma->vm_start, (unsigned long) size, (unsigned long) vma->vm_pgoff);

  /* Mark the page protection value as "cacheable" */
  vma->vm_page_prot = __pgprot((pgprot_val(vma->vm_page_prot) & ~(_PAGE_PCD | _PAGE_PWT)) );
  PRINTD(KERN_DEBUG "             VM pgprot value is 0x%02lx!\n", pgprot_val(vma->vm_page_prot));

  /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
  if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot)) return -EAGAIN;

  return 0;
}

/* Character device mmap method for cachable MPB-type memory */
static int sccmpb_mmap(struct file *filp, struct vm_area_struct *vma) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  size_t size = vma->vm_end - vma->vm_start;

  PRINTD(KERN_DEBUG "sccmpb_mmap: Device 0x%08lx (%d:%d),\n", (unsigned long)filp->f_dentry->d_inode->i_rdev, major, minor);
  PRINTD(KERN_DEBUG "             VM Start: 0x%08lx, size: 0x%08lx, offset: 0x%08lx\n", (unsigned long) vma->vm_start, (unsigned long) size, (unsigned long) vma->vm_pgoff);

  /* Mark the page protection value as "cacheable" MPB type */
  vma->vm_page_prot = __pgprot((pgprot_val(vma->vm_page_prot) & ~(_PAGE_PCD | _PAGE_PWT)) | _PAGE_MPE);
  PRINTD(KERN_DEBUG "             VM pgprot value is 0x%02lx!\n", pgprot_val(vma->vm_page_prot));

  /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
  if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot)) return -EAGAIN;

  return 0;
}



/* Character device read method to return the physical address
 */
static ssize_t sccdcm_read(struct file *filp, 
                           char __user *buf, 
                           size_t count, 
                           loff_t *ppos)
{
  struct task_struct  *tsk;
  struct mm_struct    *mm;
  
  pgd_t *pgd;
  pud_t *pud;
  pmd_t *pmd;
  pte_t *pte;
  
  unsigned long physAddress;
  unsigned long address = (unsigned long)buf;
  
  /* First get our process and memory information */
  pte = 0;
  tsk = current;
  mm  = tsk->mm;
    
  pgd = pgd_offset(mm, address);
  if (pgd_none(*pgd) ) return -EADDRNOTAVAIL; 
  pud = pud_offset(pgd, address);
  if (pud_none(*pud) ) return -EADDRNOTAVAIL;
  pmd = pmd_offset(pud, address);
  if (pmd_none(*pmd) ) return -EADDRNOTAVAIL;
  pte = pte_offset_map(pmd, address);

  if (pte_none(*pte) )
  {
    pte_unmap(pte);
    return -EADDRNOTAVAIL;
  }

  physAddress = pte_val(*pte);
  physAddress &= PAGE_MASK;
  physAddress |= (address & ~PAGE_MASK);
  
  pte_unmap(pte);
  
  return physAddress;
}


/* Helper function to purge a specific set from the caches
 * by reading invalid data into all ways
 */
__attribute__((always_inline)) static inline void sccdcm_purgeSet(unsigned long set)
{
  register char   tmp;

  /* Translate the set to a kernel space virtual address */
  const volatile char*  dummyData = (volatile char*)set;
  
  /* Now read new data into all four ways */
  tmp = *dummyData;
  tmp = *(dummyData + L2_WBSTRIDE);
  tmp = *(dummyData + L2_WBSTRIDE * 2);
  tmp = *(dummyData + L2_WBSTRIDE * 3);
}


/* Character device write method to write back modified data from the caches
 * to main memory (cacheable memory, only)
 */
static ssize_t sccdcm_write(struct file *filp, 
                            const char __user *buf, 
                            size_t count, 
                            loff_t *ppos)
{
  struct task_struct  *tsk;
  struct mm_struct    *mm;

  pgd_t *pgd;
  pud_t *pud;
  pmd_t *pmd;
  pte_t *pte;

  size_t        pos         = 0;
  size_t        purgebase   = 0;
  unsigned long physAddress = 0;
  unsigned long address     = (unsigned long)buf;
  unsigned long flags;


  PRINTD(KERN_INFO "sccdcm_write(): Flushing %d bytes from 0x%08lX\n",
                    count, address);

  /* First disable interrupts to be sure nobody else accesses the cache
   * in parallel and messes up the LRU state bits.
   */
  local_irq_save(flags);

  /* Flip-flop between two parts of the flush area */
  if(OWN_MPB_offset == 0)
    OWN_MPB_offset = L2_CAPACITY;
  else
    OWN_MPB_offset = 0;

  /* Set basepointer to flush area we will use in this flush */
  purgebase = OWN_MPB_vaddr + OWN_MPB_offset;

#ifdef FLUSH_USES_WBINVD
  /* Now write-back and invalidate the L1 content so subsequent reads
   * cannot trigger evictions which could again mess up L2 LRU state.
   * It is not clear that this is required - disabling it saves about 10K cycles */
  __asm__ volatile ( "wbinvd;\n\t" );
#endif

  /* In order to improve performance, check whether the entire cache has to
   * be flushed first; use count=0 as shortcut.
   */
  if ((count==0) || (count>=L2_WBSTRIDE))
  {
    for (physAddress=purgebase; physAddress<purgebase+L2_WBSTRIDE; physAddress+=L2_LINESIZE)
      sccdcm_purgeSet(physAddress);
  }
  else
  {
    /* First get our process and memory information */
    pte = 0;
    tsk = current;
    mm  = tsk->mm;

    /* Align the address to cacheline boundaries */
    address &= ~(L2_LINESIZE-1);

    /* Loop over the entire range which shall be purged from the caches */
    pos = 0;
    while (pos < count)
    {
      /* Get the page table information so we can calculate the physical
       * address. This is only necessary when moving to a new page.
       */
      if (pos==0 || (address+pos)%PAGE_SIZE==0)
      {
        /* Enable interrupts for the address lookup in order to handle
         * potential faults and errors.
         */
        local_irq_restore(flags);

        pgd = pgd_offset(mm, address);
        if (pgd_none(*pgd) ) return -EADDRNOTAVAIL;
        pud = pud_offset(pgd, address);
        if (pud_none(*pud) ) return -EADDRNOTAVAIL;
        pmd = pmd_offset(pud, address);
        if (pmd_none(*pmd) ) return -EADDRNOTAVAIL;
        pte = pte_offset_map(pmd, address);
              if (pte_none(*pte) )
        {
          pte_unmap(pte);
          return -EADDRNOTAVAIL;
        }
        physAddress = pte_val(*pte);
        pte_unmap(pte);

        /* Disable interrupts again while purging the data from the cache */
        local_irq_save(flags);
      }

      /* Determine the physical address by taking the MSBs from the PTE
       * and the offset within the page from the virtual space address.
       */
      physAddress &= PAGE_MASK;
      physAddress += ((address+pos) & ~PAGE_MASK);

      sccdcm_purgeSet((physAddress % L2_WBSTRIDE) + purgebase);

      /* We only have to read once per cacheline */
      pos += L2_LINESIZE;
    }
  }


  /* Enable normal operation again */
  local_irq_restore(flags);

  return 0;
}



static struct miscdevice sccncm_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rckncm",
	.fops = &sccncm_fops,
};

static struct miscdevice sccmpb_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rckmpb",
	.fops = &sccmpb_fops,
};

static struct miscdevice sccdcm_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rckdcm",
	.fops = &sccdcm_fops,
};

/* module initialization - called at module load time */
static int __init sccmem_init(void) {
  int ret = 0;

  PRINTD(KERN_INFO "Starting up SCC memory driver...\n");
  /* This driver does only work in a bare-metal environment */
  if (!scc_bare_metal()) {
    printk(KERN_INFO "sccmem: startup in non-SCC or paravirtualized environment.\n");
    return -EINVAL;
  }

  ret = misc_register(&sccncm_dev);
  if (ret != 0) {
    printk(KERN_ERR "sccmem_init: could not register sccncm device. err = %d\n",
	ret);
    goto out;
  }

  ret = misc_register(&sccmpb_dev);
  if (ret != 0) {
    printk(KERN_ERR "sccmem_init: could not register sccmpb device. err = %d\n",
	ret);
    goto out_dereg_ncm;
  }

  ret = misc_register(&sccdcm_dev);
  if (ret != 0) {
    printk(KERN_ERR "sccmem_init: could not register sccdcm device. err = %d\n",
	ret);
    goto out_dereg_mpb;
  }

  /* Make a memory mapping for invalid MPB data to flush the L2 cache with,
     this region starts at one L2 size beyond the MPB */
  OWN_MPB_vaddr = (size_t) ioremap_prot(OWN_MPB + L2_CAPACITY, L2_CAPACITY * 2, 0);
  OWN_MPB_offset = 0;

  if(!OWN_MPB_vaddr)
    goto out_dereg_dcm;

  printk(KERN_INFO "sccmem_init(): Mapped MPB at %x for flushing to %x\n", (OWN_MPB + MPBSIZE), OWN_MPB_vaddr);

  return 0;

out_dereg_dcm:
  misc_deregister(&sccdcm_dev);
out_dereg_mpb:
  misc_deregister(&sccmpb_dev);
out_dereg_ncm:
  misc_deregister(&sccncm_dev);
out:
  return ret;
}

/* module unload */
static void __exit sccmem_exit(void)
{
  PRINTD(KERN_INFO "sccmem_init(): Shutting down SCC memory driver...\n");

  /* remove the misc devices */
  misc_deregister(&sccncm_dev);
  misc_deregister(&sccmpb_dev);
  misc_deregister(&sccdcm_dev);

  /* Unmap invalid MPB memory used by L2 flush */
  iounmap((void*)OWN_MPB_vaddr);
}

/* Perform write-back/invalidate for all cachelines holding the specified pfn */
void scc_cop_wbinv_pfn(pfn_t pfn)
{
	unsigned long purgebase, physAddress, offset;
	unsigned long flags;

	/* First disable interrupts to be sure nobody else accesses the cache
	 * in parallel and messes up the LRU state bits. */
	local_irq_save(flags);

	/* Flip-flop between two parts of the flush area */
	if (OWN_MPB_offset == 0) {
		OWN_MPB_offset = L2_CAPACITY;
	} else {
		OWN_MPB_offset = 0;
	}

	/* Set basepointer to flush area we will use in this flush */
	purgebase = OWN_MPB_vaddr + OWN_MPB_offset;

#ifdef FLUSH_USES_WBINVD
	/* Begin by flushing the entire L1. We don't want any stale data in it,
	 * as the reads issued by sccdcm_purgeAddress need to go to L2. */
	__asm__ __volatile__ ("wbinvd;\n\t");
#endif

	/* Now flush the entire pfn out of the cache. It is no longer in L1 by
	 * definition, so all we need to do now is kick it out of L2...
	 * Beware of race conditions here, as no other thread may access the
	 * page afterwards or cachelines seem to reappear from nowhere. The
	 * safest way is to unmap all page table entries referring to the page.
	 * Alternatively, just disable interrupts around the current method. */
	physAddress = pfn << PAGE_SHIFT;

	for (offset = 0; offset < (1 << PAGE_SHIFT); offset += L2_LINESIZE) {
		sccdcm_purgeSet(purgebase + ((physAddress + offset) % L2_WBSTRIDE));
	}

	/* Re-enable interrupts */
	local_irq_restore(flags);
}
EXPORT_SYMBOL(scc_cop_wbinv_pfn);

module_init(sccmem_init);
module_exit(sccmem_exit);
MODULE_DESCRIPTION("SCC system memory driver...");
MODULE_AUTHOR("Michael Riepen <michael.riepen@intel.com>");

