/*
 * sccmb.c -- SCC message buffer driver
 *
 * Portions Copyright (C) 2009 Intel Corp.
 *
 * The code is based on sccmb.c from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.
 */

/* Notes:
 * - This driver assumes that the SCC system memory driver is also
 *   loaded as it performs the appropriate initialisation
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/interrupt.h>    /* mark_bh */

#include <linux/in.h>
#include <linux/netdevice.h>    /* struct device, and other headers */
#include <linux/etherdevice.h>  /* eth_type_trans */
#include <linux/ip.h>           /* struct iphdr */
#include <linux/tcp.h>          /* struct tcphdr */
#include <linux/skbuff.h>

#include <linux/in6.h>
#include <asm/checksum.h>

#include <asm/io.h>             /* ioremap() & friends */
#include <asm/pgtable.h>        /* page protection bits */
#include <asm/apic.h>           /* apic_read() & apic_write() */
#include <asm/lapic.h>          /* (un)set_lapic_mask */
#include <linux/kallsyms.h>	/* print_symbol */

#include <linux/sccsys.h>
#include "sccmbx.h"

MODULE_AUTHOR("Werner Haas");
MODULE_LICENSE("GPL");


/* DEBUG messages */
#define DEBUG_MSG 0
#define PRINTD(format, args...) if (DEBUG_MSG) { printk(format, ##args); }


/*
 * use the below define to aid in debugging
 */
/* #define DBG_INTERRUPT */

/*
 * runs better without NAPI poll methodology
 */
/* #define SCCMB_NO_NAPI*/



/*
 * Module parameters
 */
static int timeout = 5;
module_param(timeout, int, 0644);
MODULE_PARM_DESC(timeout, "Timeout period in jiffies of the NETDEV watchdog");

static int mpb_size = SCC_MPB_SIZE;
module_param(mpb_size, int, 0644);
MODULE_PARM_DESC(mpb_size, "Available MPB space for the network layer");

static int mpb_stride = SCC_TILE_SIZE;
module_param(mpb_stride, int, 0644);
MODULE_PARM_DESC(mpb_stride, "Stride between adjacent MPBs");

static int mpb_offset = 0xC0000000;
module_param(mpb_offset, int, 0644);
MODULE_PARM_DESC(mpb_offset, "Start address of the on-die SRAM memory range");

/* Optionally allow up to 4 packets in flight per destination */
static int multiPacket = 0;
module_param(multiPacket, int, 0644);
MODULE_PARM_DESC(multiPacket, "Enable/disable multiple packets in flight");

/* When interrupts are disabled the NAPI poll function is added to the queue 
 * at startup never returns 0.
 */
static int noIrq = 0;
module_param(noIrq, int, 0644);
MODULE_PARM_DESC(noIrq, "Do not use interrupts to trigger receiver");

/* Trigger mode of the receiver interrupt:
 * level = sender sets IRQ bit, cleared by receiver
 * edge  = sender sets and clears IRQ bit
 * Note that the pulse generation may not work reliably on SCC!
 */
static int edgeIrq = 1;
module_param(edgeIrq, int, 0644);
MODULE_PARM_DESC(edgeIrq, "Generate an IRQ edge, i.e. set&clear status bit");

/* Default: do not use the SCC test&set register so the driver does not 
 * interfere with other software such as RCCE
 */
static int disable_locking = 0;
module_param(disable_locking, int, 0644);
MODULE_PARM_DESC(disable_locking, "Enable/disable use of the test&set bits");

/* IP address of the local network the router core to enable packet forwarding.
 * Note that the parameter format is octet0|octet1|octet2|octet3.
 */
static unsigned int ownIpAddress = 0xC0A80000;       /* 192.168.0.0 */
module_param(ownIpAddress, uint, 0644);
MODULE_PARM_DESC(ownIpAddress, "IP address of the local network");

static unsigned int routerIpAddress = 0xC0A80001;    /* 192.168.0.1 */
module_param(routerIpAddress, uint, 0644);
MODULE_PARM_DESC(routerIpAddress, "IP address of a router core");

static int retriggerInt = 1;
#ifdef DBG_INTERRUPT
module_param(retriggerInt, int, 0644);
MODULE_PARM_DESC(retriggerInt, "Control retriggering of interrupt in case of timeout");


static int pingPongTarget = 0;
module_param(pingPongTarget, int, 0644);
MODULE_PARM_DESC(pingPongTarget, "Ping Pong Target Core number (1 - 48)");
#endif




/*
 * The SCC message buffer device
 */
struct net_device* sccmb_dev;

struct sccmb_priv {
  struct net_device_stats     stats;
  struct napi_struct          napi;
  spinlock_t                  lock;
    
  /* Device-specific constants:
   * - the local IP address
   * - the size of the data FIFO (in cache lines)
   */
  u8                          localIp;
  u8                          mpb_buffersize;
  /* SCC specific physical memory addresses are mapped into kernel space 
   */
  /* mpb = Message Passing Buffer */
  void*                       mpb[SCC_CORECOUNT];
  /* The FIFO data structure for the data transfer algorithm 
   * - pointer to the next cache line in the circular buffer
   * - busy flag for all peers
   * - map indicating which peer might be accessing each cache line
   */
  u8                          mpb_next;
  int                         mpb_busy[4*SCC_CORECOUNT];
  u8                          mpb_map[SCC_MPB_SIZE/SCC_CLINE_SIZE];
  u8                          mpb_nextDesc[SCC_CORECOUNT];
  u8                          mpb_rxDesc[SCC_CORECOUNT];
#ifdef DBG_PACKET_TRACE
void*     packetTrace;
unsigned  txTracePtr;
unsigned  rxTracePtr;
#endif

#ifdef DBG_INTERRUPT
unsigned txIntCount[SCC_CORECOUNT];
unsigned txRetriggerCount[SCC_CORECOUNT];
unsigned rxIntCount;
#endif
};



/*
 * SCC specific helper functions
 */


/* ip2phys calculates the physical address of the MPB for the
 * core ID
 */
int sccmb_id2phys(u8 id)
{
  int tile_number = (id/2);
  int core_number = (id%2);
  int result;
  
  /* Only accept IDs for the available cores */
  if (id >= SCC_CORECOUNT) return 0;
  
    result  = mpb_offset;
    result += mpb_stride*tile_number;
    result += SCC_MPB_SIZE*core_number;

  return PAGE_ALIGN(result);
}


/* Calculate the descriptor address for a packet transfer from the given
 * sender IP address. Note that the corresponding descriptor data is stored
 * in the receiver's MPB.
 */
int sccmb_get_desc_address(struct net_device* dev, u8 sender_ip, u8 receiver_ip)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         address;
  
  /* The packet descriptors are stored at the receiver side i.e. start with
   * the message passing buffer of the destination
   */
  address  = (int)(priv->mpb[receiver_ip-1]);
  /* The descriptor area is located at the top i.e. skip the data range */
  address += (SCC_CLINE_SIZE * priv->mpb_buffersize);
  /* Evaluate the offset corresponding to the sender IP */
  address += sender_ip-1;

  return address;
}


/* lock/unlock access the LOCK bit in the core register bank of the
 * given IP address
 */
void sccmb_lock(struct net_device* dev, u8 ip_address)
{
	int pid = ip_address - 1;

	if (disable_locking) return;

	sccsys_acquire_pid_lock(pid);
}

void sccmb_unlock(struct net_device* dev, u8 ip_address)
{
	int pid = ip_address - 1;

	if (disable_locking) return;

	sccsys_release_pid_lock(pid);
}


/* trigger_irq/clear_irq access the interrupt bit in the core register bank 
 * of the given IP address which is connected to the processor's INTR pin
 */
void sccmb_trigger_irq(struct net_device* dev, u8 ip_address)
{
	int pid = ip_address - 1;
	sccsys_trigger_irq_direct(pid, SCC_INTR_MASK, edgeIrq);
}


/* reset interrupt bit for core 
 */
void sccmb_clear_irq(struct net_device* dev, u8 ip_address)
{
	int pid = ip_address - 1;
	sccsys_clear_irq_direct(pid, SCC_INTR_MASK);
}


/* Flush the write-combining buffer
 */
void sccmb_flush_wcbuffer(struct net_device* dev)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         address;
  int                         value;
  int                         i;
  
  /* Read & write back our own message buffer to flush the write-combining
   * data. The area at the beginning of the MPB is read-only for other cores,
   * i.e. we can safely access these cache lines. By reading/writing data
   * from the first 2 cache lines we have at least 1 address change and thus
   * the data is forwarded to its destination.
   */
  address = (int)priv->mpb[priv->localIp-1];
  for (i=0; i<2; i++) {
    value = *((volatile int*)address);
    CL1FLUSHMB;
    *((volatile int*)address) = value;
    
    address += SCC_CLINE_SIZE;
  }
}



/* tx_pending returns the current status of send operations to the specified IP
 * In case the local busy flag is set, the remote message passing buffer is
 * queried to check whether the operation is still pending
 */
int sccmb_tx_pending(struct net_device* dev, u8 ip_address, int slot)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         desc_address;
  int                         i;
  int 			      timesent;
  static int 		      timeout_count = 0;

  /* 0 is not a valid IP address and marks unused circular buffer slots */
  if (ip_address == 0) return 0;
  
  /* Select the next Tx descriptor slot if none is specified */
  if (slot < 0) slot = priv->mpb_nextDesc[ip_address-1];
  
  /* Check whether there is an undelivered packet */
  if (priv->mpb_busy[slot*SCC_CORECOUNT + ip_address-1]) {
    /* Read the descriptor from the destination to see whether it is still
     * valid. First we have to flush potentially stale data, though.
     */
    CL1FLUSHMB;
    
    /* Calculate the descriptor address for the packet transfer from this
     * node to the given IP address.
     */
    desc_address = sccmb_get_desc_address(dev, priv->localIp, ip_address)
                 + SCC_CORECOUNT * slot;
    
    /* Read the packet offset
     * 0xFF is illegal because the descriptors are located at the top i.e.
     * this value signals transfer completed.
     */
    if (*((u8*)desc_address) != 0xFF) {
      /* Check the age of the transmission and delete the packet if it is
       * too old.
       */
      timesent =  priv->mpb_busy[slot*SCC_CORECOUNT+ip_address-1];
      if (jiffies > timesent + timeout)
      {
        timeout_count++;
	if (timeout_count == 1000) {
	  printk(KERN_DEBUG "sccmb_tx_pending(): Timeout at destination %d\n", 
		 ip_address);
	  timeout_count = 0;
	}
      
        /* Free all cache lines holding data for that destination */
        for (i=0; i<priv->mpb_buffersize; i++) {
          if ((priv->mpb_map[i]) == ((slot<<6)+ip_address) ) 
            priv->mpb_map[i] = 0;
        }
      }
      /* Else signal that the transmission is still pending and retrigger the target with another interrupt */
      else {
        if  (retriggerInt && (jiffies > timesent)) {
          sccmb_trigger_irq(dev, ip_address);
#ifdef DBG_INTERRUPT
	  priv->txRetriggerCount[ip_address - 1]++;
#endif
	}
	return 1;
      }
    }
    
    /* Clear the busy flag */
    priv->mpb_busy[slot*SCC_CORECOUNT + ip_address-1] = 0;
  }
  
  return 0;
}


/* set_descriptor updates the start of packet pointer corresponding to our
 * own ID at the given destination
 */
void sccmb_set_descriptor(struct net_device* dev, u8 ip_address, u8 offset)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         desc_address;
  
  /* Make sure that the write misses in the L1 */
  CL1FLUSHMB;
  
  /* Calculate the descriptor address for the packet transfer from this
   * node to the given IP address. In case multiple packets may be in
   * flight, the address has to be adjusted according to the current
   * descriptor slot.
   */
  desc_address = sccmb_get_desc_address(dev, priv->localIp, ip_address)
               + SCC_CORECOUNT * priv->mpb_nextDesc[ip_address-1];
    
  /* Write the packet offset for the new transfer */
  sccmb_lock(dev, ip_address);
  (*((u8*)desc_address) ) = offset;
  sccmb_flush_wcbuffer(dev);
  sccmb_unlock(dev, ip_address);
}


/* clear_descriptor invalidates the packet offset in our own descriptor area,
 * i.e. marks the data transfer from the specified IP address as completed.
 */
void sccmb_clear_descriptor(struct net_device* dev, u8 ip_address)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         desc_address;
    
  /* Make sure that the write misses in the L1 */
  CL1FLUSHMB;
  
  /* Calculate the descriptor address for the packet transfer from the
   * given IP address to our node and adjust it according to the descriptor
   * slot where we found the packet.
   */
  desc_address = sccmb_get_desc_address(dev, ip_address, priv->localIp)
               + SCC_CORECOUNT * priv->mpb_rxDesc[ip_address-1];

  /* Write 0xFF to enable message transfers again */
  sccmb_lock(dev, priv->localIp);
  (*((u8*)desc_address) ) = 0xFF;
  sccmb_flush_wcbuffer(dev);
  sccmb_unlock(dev, priv->localIp);
}



/*
 * Open and close
 * These functions are called when an interface is activated/stopped. Thus,
 * any system resources should be registered and the device itself should
 * be initialized.
 */
static irqreturn_t sccmb_interrupt(int, void*);

void sccmb_unmap(struct net_device* dev)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         i;

  for (i=0; i<SCC_CORECOUNT; i++) {
    iounmap(priv->mpb[i]);
  }
}

int sccmb_open(struct net_device* dev)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  int                         status = 0;
  int                         i;
  int                         address;
  int                         tmp;
  int                         pid;

  /* MPB initialisation */
  
  /* Calculate how many cache lines are available for the circular buffer
   * We need 1 status byte per core for message descriptors; in case
   * support for multiple packets in-flight to the same destination is
   * enabled, the descriptor range is expanded accordingly.
   */
  i = SCC_CORECOUNT/SCC_CLINE_SIZE;
  if (multiPacket) i = 4*SCC_CORECOUNT/SCC_CLINE_SIZE;
  if (i%SCC_CLINE_SIZE) i++;
  priv->mpb_buffersize = (mpb_size/SCC_CLINE_SIZE) - i;
      
  /* Map all message buffers */
  for (i=0; i<SCC_CORECOUNT; i++) {
    /* Map the address directly, setting the PMB bit so the memory is indeed
     * treated as message buffer by the core. Also mark it as Write-Through
     * in order to avoid flushing the L1 cache to get the data into the
     * message buffer.
     */
    address = sccmb_id2phys(i);
    priv->mpb[i] = ioremap_mpbt(address, mpb_size);
    if (!priv->mpb[i]) status = -EIO;
  }

  if (status) {
    printk(KERN_WARNING "sccmb_open(): Can't map message passing buffer\n");
    sccmb_unmap(dev);
    return status;
  }

  pid = sccsys_get_pid();
  /* Add 1 to the processor ID to avoid *.*.*.0 IP addresses */
  priv->localIp = pid + 1;

#ifdef DBG_PACKET_TRACE
/* Map 2*1MB DDR3 memory per core as packet trace buffer */
priv->packetTrace = ioremap_nocache(0x80800000ul + 0x400000ul*x + 0x200000ul*z, 2*1024*1024);
memset(priv->packetTrace, 0, 2*1024*1024);
#endif
  PRINTD(KERN_INFO "sccmb_open(): sccmb local IP = %d\n", priv->localIp);

  /* Initialize the descriptor area located at the top of the message buffer */
  address  = (int)(priv->mpb[priv->localIp-1]);
  address += (SCC_CLINE_SIZE * priv->mpb_buffersize);
  tmp = SCC_CORECOUNT;
  if (multiPacket) tmp *= 4;
  for (i=0; i<tmp; i++) *((u8*)(address+i)) = 0xFF;
  sccmb_flush_wcbuffer(dev);
    
  /* Configure interrupt handling */
  status = request_irq(dev->irq, &sccmb_interrupt, IRQF_SHARED, "sccmb", dev);
  if (status) {
    printk(KERN_WARNING "Can't get interrupt #%d\n", dev->irq);
    sccmb_unmap(dev);
    return status;
  }

  /* Assign the hardware address of the board (6 octets for Ethernet)
   * Note that the first octet of ethernet multicast addresses is odd
   */
  memcpy(dev->dev_addr, "\0MPB_0", ETH_ALEN);
    
  netif_start_queue(dev);
  napi_enable(&priv->napi);

  /* If interrupts are not used we immediately add the polling function
   * to the queue which would otherwise be done through the IRQ handler.
   */
  if (noIrq) napi_schedule(&priv->napi);

  return 0;
}


int sccmb_close(struct net_device* dev)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  napi_disable(&priv->napi);

  /* Unmap the SCC resources */
  free_irq(dev->irq, dev);
  sccmb_unmap(dev);
  
  netif_stop_queue(dev);
  return 0;
}



/*!
 * Configuration changes (passed on by ifconfig)
 */
int sccmb_config(struct net_device* dev, struct ifmap* map)
{	
#ifdef DBG_INTERRUPT
	int i;
	struct sccmb_priv*          priv = netdev_priv(dev);
#endif

/* Debugging printout trigger */
#ifdef DBG_INTERRUPT
	if (map->base_addr == 0xdeb9) {
	printk(KERN_INFO "sccmb_config(): got debug trigger! (0x%08x)\n", map->base_addr);

	printk(KERN_DEBUG "APIC_ID  : %lx\n", apic_read(APIC_ID));
	printk(KERN_DEBUG "APIC_LVT0: %lx\n", apic_read(APIC_LVT0));
	printk(KERN_DEBUG "APIC_LVT1: %lx\n", apic_read(APIC_LVT1));
	printk(KERN_DEBUG "APIC_ESR : %lx\n", apic_read(APIC_ESR));
	for (i=0; i < 8; i++) {
		printk(KERN_DEBUG "APIC_ISR%i: %lx\n", i, apic_read(APIC_ISR + i*0x10));
	}
	for (i=0; i < 8; i++) {
		printk(KERN_DEBUG "APIC_TMR%i: %lx\n", i, apic_read(APIC_TMR + i*0x10));
	}
	for (i=0; i < 8; i++) {
		printk(KERN_DEBUG "APIC_IRR%i: %lx\n", i, apic_read(APIC_IRR + i*0x10));
	}

	return 0;
	}
#endif

#ifdef DBG_INTERRUPT
	if (map->base_addr == 0xdeba) {
	for (i=0; i < 48; i++) {
		printk(KERN_DEBUG "TX int count to core %i: %i (ReTrig: %i)\n", i, priv->txIntCount[i], priv->txRetriggerCount[i]);
		priv->txIntCount[i] = 0;
		priv->txRetriggerCount[i] = 0;
	}
	printk(KERN_DEBUG "RX int count: %i\n", priv->rxIntCount);
	priv->rxIntCount = 0;
	
	return 0;
	}
#endif

#ifdef DBG_INTERRUPT
	if (map->base_addr == 0xdebb) {
	  if (pingPongTarget > 0) {
	    printk(KERN_DEBUG "Triggering Ping Pong to core %i\n", pingPongTarget);
	    sccmb_trigger_irq(dev, pingPongTarget);
	  }

	return 0;
	}
#endif


  /* Can't act on a running interface */
  if (dev->flags & IFF_UP) return -EBUSY;

  /* Don't allow changing the I/O address */
  if (map->base_addr != dev->base_addr) {
    printk(KERN_WARNING "sccmb: Can't change I/O address\n");
    return -EOPNOTSUPP;
  }

  /* Allow changing the IRQ */
  if (map->irq != dev->irq) {
    dev->irq = map->irq;
    /* request_irq() is delayed to open-time */
  }

  /* ignore other fields */
  return 0;
}


struct sccmbx_rx {
	struct net_device* dev;
	u8 sourceIp;
	u8 offset;
	int len;
	int cleared;
	void* mpb_address;
	void* mpb_start;
};

/* Clear receive descriptor */
void sccmbx_clear_rx(struct sccmbx_rx* rx)
{
  if (!rx->cleared) {
    sccmb_clear_descriptor(rx->dev, rx->sourceIp);
    rx->cleared = 1;
  }
}
EXPORT_SYMBOL(sccmbx_clear_rx)

/* Get pid of source node of message */
int sccmbx_get_rx_source(struct sccmbx_rx* rx)
{
  return rx->sourceIp - 1;
}
EXPORT_SYMBOL(sccmbx_get_rx_source)

/* Get pointer(s) to body of received packet. This routine can optionally
 * allocate an sk_buff if the caller needs to have the packet data in virtually-
 * continuous storage.
 *
 * If need_skb is set to 0, the pointers to the packet's contents are simply
 * stored in the sccmbx_bufx; the packet is not acknowledged, and no additional
 * memory allocation is needed. The routine is guaranteed to return 1 (SUCCESS)
 * in this case.
 * This mode is useful when receiving packets must not fail due to low-memory
 * situations, but the caller needs to be able to parse packets that may be
 * broken into two fragments (in case they overlap the end of our ring buffer).
 *
 * If need_skb is set to anything else, an sk_buff is allocated, the packet's
 * contents are copied, and the packet is acknowledged. If memory allocation
 * fails, the routine returns 0, and the packet is not acknowledged.
 *
 * Irrespective of the value of need_skb, caller should always invoke
 * sccmbx_retrieve_packet_body_cleanup when they are done with the packet.
 *
 * Please note that sccmbx_retrieve_packet_body[_cleanup] do not mark the packet
 * as received at the sending node if allocating an sk_buff fails. If the caller
 * wants to drop the packet completely without having the kernel try to re-
 * deliver it, the caller needs to call sccmbx_clear_rx explicitly if the call
 * to sccmbx_retrieve_packet_body fails.
 */
int sccmbx_retrieve_packet_body(struct sccmbx_rx* rx, struct sccmbx_bufx* bufx, int need_skb)
{
  int                         len = rx->len;
  void*                       mpb_address = rx->mpb_address;
  void*                       mpb_start = rx->mpb_start;
  struct sccmb_priv*          priv = netdev_priv(rx->dev);
  struct sk_buff*             skb = NULL;
  int                         headroom;
  int                         byteCount;
  unsigned char*              local_mem;

  /* Clear the returned sccmbx_bufx structure */
  memset(bufx, 0, sizeof(struct sccmbx_bufx));

  /* First calculate how much data we can fetch before the wrap-around
   * occurs so we can determine how much to copy in the first run. Note
   * that the headroom includes the header bytes as the entire packet
   * is cacheline aligned!
   */
  headroom = (priv->mpb_buffersize - rx->offset) * SCC_CLINE_SIZE - SCCMBX_HLEN;
  byteCount = len;
  if (headroom < byteCount) {
    byteCount = headroom;
  }

  /* Save pointers into the message buffer in our sccmbx_bufx structure */
  bufx->len = len;
  bufx->f[0].p = mpb_address;
  bufx->f[0].len = byteCount;
  if (byteCount < len) {
    bufx->f[1].p = mpb_start;
    bufx->f[1].len = len - byteCount;
  }

  /* If the caller does not need an skb, we are done now. */
  if (!need_skb) {
    return 1;
  }

  /* Build a skb for the packet data so upper layers can handle it
   * Note that IP headers should be aligned on 16B boundaries!
   */
  skb = dev_alloc_skb(len);
  if (!skb) {
    return 0;
  }

  /* IP headers should be aligned on 16B boundaries, i.e. we just reserve the
   * payload area and do not prepend the header bytes which are not used
   * anymore.
   */
  local_mem = skb_put(skb, len);

  /* Copy the packet data (without the header) into the buffer. Because 
   * of the circular buffer implementation we may have to do it in 2 steps.
   */
  memcpy(skb->data, (void*)mpb_address, byteCount);

#ifdef DBG_PACKET_TRACE
/* Store the entire packet (including header bytes) also in the trace buffer */
memcpy((void*)(priv->packetTrace + 1024*1024 + priv->rxTracePtr), (void*)(mpb_address-SCCMBX_HLEN), byteCount+SCCMBX_HLEN);
priv->rxTracePtr += byteCount+SCCMBX_HLEN;
if (priv->rxTracePtr > 1024*1024-2048) priv->rxTracePtr = 0;
#endif
  
  /* Finish with the rest if we could not copy the entire packet */
  if (byteCount < len) {
    memcpy(skb->data+byteCount, mpb_start, len-byteCount);
#ifdef DBG_PACKET_TRACE
memcpy((void*)(priv->packetTrace + 1024*1024 + priv->rxTracePtr), mpb_start, len-byteCount);
priv->rxTracePtr += len-byteCount;
if (priv->rxTracePtr > 1024*1024-2048) priv->rxTracePtr = 0;
#endif
  }

#ifdef DBG_PACKET_TRACE
/* Align the rx trace pointer to cacheline boundaries */
while (priv->rxTracePtr % SCC_CLINE_SIZE) priv->rxTracePtr++;
#endif

  /* Clear the descriptor as we no longer need the message buffer content */
  sccmbx_clear_rx(rx);

  /* Write metadata, and then pass to the receive level */
  skb->dev        = rx->dev;
  skb_set_transport_header(skb, 0);
  skb_set_network_header(skb, 0);
  skb_set_mac_header(skb, 0);
  skb->protocol   = htons(ETH_P_IP);
  skb->pkt_type   = PACKET_HOST;
  skb->ip_summed  = CHECKSUM_UNNECESSARY;

  bufx->skb = skb;

  return 1;
}
EXPORT_SYMBOL(sccmbx_retrieve_packet_body)

/* Mark the skb referred to by the sccmbx_bufx structure as consumed. After this
 * call, sccmbx_retrieve_packet_body_cleanup will not free the skb.
 */
void sccmbx_retrieve_skb_consumed(struct sccmbx_bufx* bufx)
{
  bufx->skb = NULL;
}
EXPORT_SYMBOL(sccmbx_retrieve_skb_consumed)

/* Cleanup packet state after sccmbx_retrieve_packet_body has successfully read
 * a packet. This routine should be invoked with the same values for rx and bufx
 * than sccmbx_retrieve_packet_body.
 *
 * Please note that sccmbx_retrieve_packet_body[_cleanup] do not mark the packet
 * as received at the sending node if allocating an sk_buff fails. If the caller
 * wants to drop the packet completely without having the kernel try to re-
 * deliver it, the caller needs to call sccmbx_clear_rx explicitly if the call
 * to sccmbx_retrieve_packet_body fails.
 */
void sccmbx_retrieve_packet_body_cleanup(struct sccmbx_rx* rx, struct sccmbx_bufx* bufx)
{
  /* If we did not use an sk_buff for this packet, the send descriptor has not
   * been cleared yet because the message buffer content was still needed. In
   * this case, clear the descriptor now. sccmbx_clear_rx makes sure not to
   * clear more than once, so this call is safe even if an sk_buff was used. */
  sccmbx_clear_rx(rx);

  /* If there was an sk_buff allocated, free it now */
  if (bufx->skb != NULL) {
    dev_kfree_skb_any(bufx->skb);
    bufx->skb = NULL;
  }
}
EXPORT_SYMBOL(sccmbx_retrieve_packet_body_cleanup)

/*
 * Receive a packet for the NET subsystem
 */
static void sccmbx_rx_net(struct sccmbx_rx* rx, struct sccmbx_hdr* header, int len)
{
  struct sccmb_priv*          priv = netdev_priv(rx->dev);
  struct sccmbx_bufx          bufx;

  /* Check for reasonable sizes before processing the data to
   * prevent nasty memory overflow errors.
   */
  if (len < sizeof(struct iphdr)) {
    /* Simply drop the packet */
    sccmbx_clear_rx(rx);
    
    printk(KERN_NOTICE "sccmb_rx(): illegal packet length %d => no IP hdr\n", len);
    priv->stats.rx_dropped++;
    return;
  }

  /* Get packet */
  if (!sccmbx_retrieve_packet_body(rx, &bufx, 1)) {
    if (printk_ratelimit() )
      printk(KERN_NOTICE "sccmb rx: low on mem - packet dropped\n");

    /* Note: since we do not clear the offset descriptor we do not trigger
     * a retransmission and the packet will eventually be processed.
     */
    priv->stats.rx_dropped++;
    return;
  }

  /* Update the interface statistics (also count the header bytes) */
  priv->stats.rx_packets++;
  priv->stats.rx_bytes += SCCMBX_HLEN+len;
  rx->dev->last_rx = jiffies;

  /* Deliver packet to upper layer */
#ifdef SCCMB_NO_NAPI
    netif_rx(bufx.skb);
#else
    netif_receive_skb(bufx.skb);
#endif
  sccmbx_retrieve_skb_consumed(&bufx);

  /* Cleanup. This really is a NOP, as the data will always be copied into an
   * sk_buff for the NET subsystem. */
  sccmbx_retrieve_packet_body_cleanup(rx, &bufx);
}

static sccmbx_rx_proc sccmbx_rx_procs[256] = { 0 };

static void __sccmbx_set_rx_proc(u8 subsys, sccmbx_rx_proc rx_proc)
{
	char subsys_name[16];
	switch (subsys) {
	case SCCMBX_SUBSYS_NET:
		strcpy(subsys_name, "NET");
		break;
	case SCCMBX_SUBSYS_SHM:
		strcpy(subsys_name, "SHM");
		break;
	default:
		sprintf(subsys_name, "%d", subsys);
		break;
	}
	printk(KERN_INFO "sccmbx: setting rx_proc for subsystem %s", subsys_name);
	print_symbol(" to %s\n", (unsigned long)rx_proc);

	sccmbx_rx_procs[subsys] = rx_proc;
}

/* Set receive callback for messages to a specific subsystem ID */
int sccmbx_set_rx_proc(u8 subsys, sccmbx_rx_proc rx_proc)
{
	if (unlikely(subsys == SCCMBX_SUBSYS_NET)) {
		return -EINVAL;
	}

	__sccmbx_set_rx_proc(subsys, rx_proc);
	return 0;
}
EXPORT_SYMBOL(sccmbx_set_rx_proc)

/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */
void sccmb_rx(struct net_device* dev, u8 sourceIp, u8 offset)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  void*                       mpb_start;
  void*                       mpb_address;
  int                         len, subsys;

  struct sccmbx_hdr*          header;
  struct sccmbx_rx            rx;
  sccmbx_rx_proc              rx_proc;

  /* Get address of message buffer of sending node */
  mpb_start = priv->mpb[sourceIp-1];

  /* Calculate the address of the packet */
  mpb_address  = mpb_start + offset*SCC_CLINE_SIZE;
  
  /* The packet length is stored in the first 2 bytes but does not include
   * the header.
   */
  header = (struct sccmbx_hdr*)mpb_address;
  len = 256*( header->len[0] )
      +     ( header->len[1] );
  subsys = header->subsys;
  mpb_address += SCCMBX_HLEN;

  /* Check for reasonable sizes before processing the data to
   * prevent nasty memory overflow errors.
   */
  if (len > dev->mtu) {
    /* Simply drop the packet */
    sccmb_clear_descriptor(dev, sourceIp);
    
    printk(KERN_NOTICE "sccmb_rx(): illegal packet length %d => too long\n", len);
    priv->stats.rx_dropped++;
    return;
  }
  
  /* We need to hand the packet over to the responsible subsystem now. Check
   * that the subsystem ID is in the allowed range and that the receive function
   * has been registered. */
  if (unlikely((subsys < 0) || (subsys >= sizeof(sccmbx_rx_procs) / sizeof(sccmbx_rx_procs[0])))) {
    sccmb_clear_descriptor(dev, sourceIp);

    printk(KERN_NOTICE "sccmbx_rx: invalid target subsystem id %d.\n", subsys);
    priv->stats.rx_dropped++;
    return;
  }

  rx_proc = sccmbx_rx_procs[subsys];
  if (unlikely(!rx_proc)) {
    sccmb_clear_descriptor(dev, sourceIp);

    printk(KERN_NOTICE "sccmbx_rx: subsystem id %d not registered.\n", subsys);
    priv->stats.rx_dropped++;
    return;
  }

  /* Build external receive descriptor and invoke the callback */
  memset(&rx, 0, sizeof(rx));
  rx.dev = dev;
  rx.sourceIp = sourceIp;
  rx.offset = offset;
  rx.len = len;
  rx.mpb_address = mpb_address;
  rx.mpb_start = mpb_start;

#ifdef CONFIG_SCCMBX_DUMP_NONNET_PACKETS
  if (subsys != SCCMBX_SUBSYS_NET) {
    printk(KERN_INFO "sccmbx_rx: received packet for subsystem %d, len=%x\n", subsys, len);
  }
#endif
  rx_proc(&rx, header, len);
}



/*
 * Check the receive descriptors and return the IP address if a valid
 * packet offset is found. Note that a round robin scheme is used to
 * ensure fairness.
 */
static int sccmb_nextRxPacket(struct net_device* dev, u8* offset)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  static u8                   lastSource = 0;
  u8                          source;
  u8                          i;
  u8                          nrOfDescriptors;
  int                         desc_address;
  
  nrOfDescriptors = multiPacket ? 4*SCC_CORECOUNT : SCC_CORECOUNT;
  
  *offset = 0xFF;
  for (i=0; i<nrOfDescriptors; i++) {
    /* Calculate the descriptor address for the packet transfer from the
     * given IP address to our node. Add 1 to the last found packet source
     * so it gets processed with least priority.
     */
    source = (1+lastSource + i) % nrOfDescriptors;
    desc_address = sccmb_get_desc_address(dev, 1+source, priv->localIp);
    
    /* Check for a valid receive descriptor */
    *offset = *((u8*)desc_address);
    if (*offset < 0xFF) {
      /* Store the descriptor slot where we found the packet */
      priv->mpb_rxDesc[source%SCC_CORECOUNT] = source/SCC_CORECOUNT;
      
      /* Remember the core number and return the IP address = 1+core number */
      lastSource = source;
      return 1 + source%SCC_CORECOUNT;
    }
  }
  
  /* Return 0 if no packet was found */
  return 0;
}


/*
 * The poll implementation processing all incoming packets
 */
static int sccmb_poll(struct napi_struct* napi, int to_do)
{
  struct sccmb_priv*          priv = container_of(napi, struct sccmb_priv, napi);
  struct net_device*          dev = napi->dev;
  int                         npackets = 0;
  int                         quota = to_do;
  u8                          sourceIp;
  u8                          offset = 0;

  /* Process up to quota packets from the receive queue */
  while (npackets<quota) {
    /* Flush the message buffer so we get up to date descriptor info */
    CL1FLUSHMB;
    sourceIp = sccmb_nextRxPacket(dev, &offset);
    
    /* Call the standard receive handler if there is a packet */
    if (sourceIp) {
      /* In case of level-triggered interrupts clear it now */
      if (!edgeIrq) sccmb_clear_irq(dev, priv->localIp);
      
      sccmb_rx(dev, sourceIp, offset);
      npackets++;
    }
    /* Else perform a clean exit */
    else {

      /* If interrupts are disabled we hae to remain in the polling function,
       * i.e. we may signal that we are done with packet processing but must
       * not remove us from the queueing list via netif_rx_complete().
       */
      if (noIrq) {
        break;
      }
      
      /* Tell the system we are done with polling */
      napi_complete(&priv->napi);

      /* Do not use enable_irq(dev->irq); since we might loose enables --> INT locked up */      
      unset_lapic_mask(APIC_LVT0, dev->irq);

      /* The interrupt was cleared before the Rx packet was processed,
       * i.e. it will be re-set if another packet arrived since the last
       * check. Thus the polling function will be scheduled again as
       * soon as we enable interrupts again so we can safely return.
       */
      break;
    }
  }
  
  return npackets;
}



/*
 * A NAPI interrupt handler since we can receive multiple packets 
 * (from different sources) with a single interrupt.
 */
static irqreturn_t sccmb_interrupt(int irq, void* dev_id)
{
  struct net_device*          dev = (struct net_device*)dev_id;
  struct sccmb_priv*          priv = netdev_priv(dev);

#ifdef SCCMB_NO_NAPI
  u8                          sourceIp = 0; 
  u8                          offset = 0; 
#endif

  /* Paranoid */
  if (!dev) {
    printk(KERN_DEBUG "sccmb interrupt %d for unknown device\n", irq);
    return IRQ_NONE;
  }

#ifdef DBG_INTERRUPT
  priv->rxIntCount++;
  
  if (pingPongTarget > 0) {
    sccmb_trigger_irq(dev, pingPongTarget);
    return IRQ_HANDLED;
  }
#endif

#ifdef SCCMB_NO_NAPI
  CL1FLUSHMB;
  sourceIp = sccmb_nextRxPacket(dev, &offset);
  while (sourceIp) {
    /* Call the standard receive handler if there is a packet */
    sccmb_rx(dev, sourceIp, offset);
    /* Flush the message buffer so we get up to date descriptor info */
    CL1FLUSHMB;
    sourceIp = sccmb_nextRxPacket(dev, &offset);
  }
  return IRQ_HANDLED;
#else

  /* Disable further interrupts and start the polling process. 
   * We have to use the nosync version since we are inside the interrupt
   * service routine!
   * Do not use disable_irq_nosync(dev->irq); since we might loose enables --> INT locked up
   */

  set_lapic_mask(APIC_LVT0, dev->irq);

  napi_schedule(&priv->napi);
  return IRQ_HANDLED;
#endif
}




/*
 * Transmit a packet (low level interface)
 * This function deals with HW details, i.e. it writes the packet
 * into the message buffer and informs the destination.
 */
int sccmb_get_destination(struct net_device* dev, struct sccmbx_bufx* bufx, u8* destIp)
{
  struct iphdr*               ipHeader;
  char                        packet_buf[SCCMBX_HLEN > sizeof(struct iphdr) ? SCCMBX_HLEN : sizeof(struct iphdr)];
  u32*                        destAddr;
  u8                          destCore;
  u8                          netAddr[4];
  int                         i;
  struct sccmbx_hdr*          header;

  *destIp = -1;

  /* Verify that the packet contains at least our hardware header */
  if (bufx->len < dev->hard_header_len) {
    return 0;
  }

  /* Check whether we have an explicit destination */
  header = (struct sccmbx_hdr*)packet_buf;
  sccmbx_copy_from_bufx(header, bufx, 0, sizeof(struct sccmbx_hdr));
  if (header->subsys != SCCMBX_SUBSYS_NET) {
#ifdef CONFIG_SCCMBX_DUMP_NONNET_PACKETS
    printk(KERN_INFO "sccmbx: transferring packet for subsystem %d, len = %x...\n", header->subsys, bufx->len);
    sccutil_dump_buffer(bufx->f[0].p, bufx->f[0].len);
    if (bufx->f[1].len > 0) {
      sccutil_dump_buffer(bufx->f[1].p, bufx->f[1].len);
    }
#endif
    destCore = header->dest;
    if ((destCore < 1) || (destCore > 48)) {
      printk(KERN_ERR "sccmbx: packet for subsystem %d specifies invalid destination %d\n",
		header->subsys, destCore);
      destCore = 1;
    }
    *destIp = destCore;
    return 1;
  }

  /* This is a network packet, so we assume it contains an IP header */
  if (bufx->len < dev->hard_header_len+sizeof(struct iphdr)) {
    return 0;
  }

  /* Extract the destination address from the IP header.
   * The last (4th) octet is interpreted as core ID.
   */
  ipHeader  = (struct iphdr*)packet_buf;
  sccmbx_copy_from_bufx(ipHeader, bufx, dev->hard_header_len, sizeof(struct iphdr));
  destAddr  = &(ipHeader->daddr);
  destCore  = ((u8*)destAddr)[3];

  /* Compare the network part to check if we have a local destination.
   * Note that the module parameter is expected in A.B.C.D format to make
   * it easier to modify so the values have to be reversed.
   * In case a remote network is detected, the core number of the router
   * is used as destination.
   */
  for (i=0; i<4; i++) netAddr[3-i] = (ownIpAddress >> 8*i) & 0xFF;
  for (i=0; i<3; i++) {
    if (((u8*)destAddr)[i] != netAddr[i]) destCore = routerIpAddress & 0xFF;
  }
  
  /* Make sure that only valid local IP addresses are returned */
  if ((destCore < 1) || (destCore > 48)) destCore = 1;
  *destIp = destCore;

  return 1;
}

static int sccmb_hw_tx(struct net_device* dev, struct sccmbx_bufx* bufx, u8 destIp)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  u8                          clen;
  int                         i;
  int                         ptr;
  int                         owner;
  int                         slot;

  /* Determine how many cache lines we need to store the message buffer data */
  clen = (bufx->len / SCC_CLINE_SIZE);
  if (bufx->len % SCC_CLINE_SIZE) clen++;
  
  /* Check whether there is enough space in the circular buffer */
  for (i=0; i<clen; i++) {
    ptr = (priv->mpb_next + i) % priv->mpb_buffersize;
    owner = priv->mpb_map[ptr] & 0x3F;
    slot  = priv->mpb_map[ptr] >> 6;
    if (sccmb_tx_pending(dev, owner, slot) ) return 1;
    
    /* This slot will be used for the new packet transfer - also mark the
     * the descriptor slot that is being used.
     */
    priv->mpb_map[ptr] = (priv->mpb_nextDesc[destIp-1] << 6) + destIp;
  }
  
  /* Copy the data into the circular buffer */
  ptr  = (int)(priv->mpb[priv->localIp-1]);
  ptr += (priv->mpb_next * SCC_CLINE_SIZE);
  if (clen + priv->mpb_next > priv->mpb_buffersize) {
    /* First fill up the remaining space to the top of the buffer */
    i = (priv->mpb_buffersize - priv->mpb_next) * SCC_CLINE_SIZE;
    sccmbx_copy_from_bufx((void*)ptr, bufx, 0, i);
    
    /* Copy the remaining bytes to the start */
    ptr  = (int)(priv->mpb[priv->localIp-1]);
    sccmbx_copy_from_bufx((void*)ptr, bufx, i, bufx->len-i);
  }
  else {
    /* Simply copy all the data since there is enough space without
     * pointer roll-over.
     */
    sccmbx_copy_from_bufx((void*)ptr, bufx, 0, bufx->len);
  }

#ifdef DBG_PACKET_TRACE
/* Store the packet in the trace buffer */
sccmbx_copy_from_bufx((void*)(priv->packetTrace + priv->txTracePtr), bufx, 0, bufx->len);
priv->txTracePtr += clen*SCC_CLINE_SIZE;
if (priv->txTracePtr > 1024*1024-2048) priv->txTracePtr = 0;
#endif    
  /* Set the descriptor on the receiver side
   * Since different addresses are used, this automatically flushes the
   * data inside the write-combining buffer so the receiver sees up to
   * date message buffer content.
   */
  sccmb_set_descriptor(dev, destIp, priv->mpb_next);
  
  /* Update the pointer to the next circular buffer slot and flag the
   * pending transmission.
   */
  slot = priv->mpb_nextDesc[destIp-1];
  priv->mpb_next = (priv->mpb_next + clen) % priv->mpb_buffersize;
  priv->mpb_busy[slot*SCC_CORECOUNT + destIp-1] = jiffies;

  /* Interrupt the receiver so it starts processing the packet */
  sccmb_trigger_irq(dev, destIp);
  
  /* Increment the next descriptor slot for the next transmission in case
   * multiple packets may be in flight.
   */
  if (multiPacket)
    priv->mpb_nextDesc[destIp-1] = (priv->mpb_nextDesc[destIp-1] + 1) % 4;
      
  /* Transmission succeeded so save the timestamp of the transmission and
   * update the statistics
   */
  dev->trans_start = jiffies;

  priv->stats.tx_packets++;
  priv->stats.tx_bytes += bufx->len;
  
  return 0;
}



static void sccmbx_init_header(struct sccmbx_hdrbuf* buf, u8 subsys, u8 dest,
			       unsigned int len)
{
  struct sccmbx_hdr* header = (struct sccmbx_hdr*)buf;

  memset(header, 0, SCCMBX_HLEN);

  /* Store the length starting with the MSBs */
  header->len[0] = len/256;
  header->len[1] = len%256;

  /* Store other information */
  header->subsys = subsys;
  header->dest = dest;
  header->src = sccsys_get_pid();
}

/* Initialize struct bufx with data from struct sk_buff */
static void sccmbx_init_transmit_bufx_from_skb(struct sccmbx_bufx* bufx, struct sk_buff* skb)
{
	memset(bufx, 0, sizeof(struct sccmbx_bufx));
	bufx->len = skb->len;
	bufx->f[0].p = skb->data;
	bufx->f[0].len = skb->len;
}

/* Initialize struct bufx for transmitting a raw packet */
void sccmbx_init_transmit_bufx(struct sccmbx_bufx* bufx, struct sccmbx_hdrbuf* hdr, u8 subsys, u8 dest, void* data, int len)
{
	memset(bufx, 0, sizeof(struct sccmbx_bufx));
	bufx->len = SCCMBX_HLEN + len;
	bufx->f[0].p = hdr;
	bufx->f[0].len = SCCMBX_HLEN;
	bufx->f[1].p = data;
	bufx->f[1].len = len;
	sccmbx_init_header(hdr, subsys, dest + 1, len);
}
EXPORT_SYMBOL(sccmbx_init_transmit_bufx);


/* Transmit a packet described by a struct sccmbx_bufx */
static int sccmb_tx_bufx(struct net_device* dev, struct sccmbx_bufx* bufx)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  u8                          destIp;
  u8                          slot = priv->mpb_nextDesc[destIp-1];
  u8                          i;
  
  //printk(KERN_DEBUG "sccmb_tx_bufx(): start\n");

  /* Perform a sanity check on the packet data, i.e. silently drop packets
   * that are either too short or send to an illegal IP address by indicating
   * success without executing any data transfer operations.
   */
  if (!sccmb_get_destination(dev, bufx, &destIp)) {
    printk(KERN_NOTICE "sccmb_tx(): Illegal packet (%i octets to IP %d)\n", 
                       bufx->len, destIp);
    priv->stats.tx_errors++;
    return 0;
  }

  /* Check if another transmission to the same destination is pending */
  if (sccmb_tx_pending(dev, destIp, -1) ) {
    /* If too much time elapsed since the previous transmission the receiver
     * is probably dead and we drop the packet
     */
    if (jiffies > priv->mpb_busy[slot*SCC_CORECOUNT + destIp-1] + timeout) {
      printk(KERN_NOTICE "sccmb_tx(): Timeout at destination %d\n", destIp);
      
      /* Also free all cache lines holding data for that destination */
      for (i=0; i<priv->mpb_buffersize; i++) {
        if ((priv->mpb_map[i]) == ((slot<<6)+destIp) )
          priv->mpb_map[i] = 0;
      }
      
      priv->stats.tx_errors++;
      return 0;
    }
    /* Else signal the kernel that the transmission failed so it can 
     * reschedule 
     */
    return 1;
  }
  
  /* Now perform the low level data transfer and return the result. If it
   * failed, e.g. because there is not enough space in the MPB, the kernel
   * will retry transmission.
   */
  return sccmb_hw_tx(dev, bufx, destIp);
}

/*
 * Transmit a packet (called by the kernel)
 */
int sccmb_tx(struct sk_buff* skb, struct net_device* dev)
{
  int                         res;
  struct sccmbx_bufx          bufx;
  
  //printk(KERN_DEBUG "sccmb_tx(): start\n");
  
  /* Convert the socket buffer to a struct sccmbx_bufx */
  sccmbx_init_transmit_bufx_from_skb(&bufx, skb);

  /* Try to transfer packet */
  res = sccmb_tx_bufx(dev, &bufx);
  
  /* Free socket buffer only if transmission was successful */
  if (res == 0) {
    dev_kfree_skb_any(skb);
  }
  return res;
}

/* Transmit a raw packet described by a struct bufx */
int sccmbx_transmit_bufx(struct sccmbx_bufx* bufx)
{
  return -EINVAL;
}
EXPORT_SYMBOL(sccmbx_transmit_bufx);


/*
 * Deal with a transmit timeout.
 */
void sccmb_tx_timeout(struct net_device* dev)
{
  /* A timeout occurs if the queue was stopped because too many packet
   * transfers were pending. Let's simply resume regular packet processing
   * in the hope that the receiver have cleared their backlog.
   */
  //printk(KERN_DEBUG "sccmb_tx_timeout()\n");
  
  netif_wake_queue(dev);
  return;
}



/*
 * ioctl commands
 */
int sccmb_ioctl(struct net_device* dev, struct ifreq* rq, int cmd)
{
  /*
   * Custom commands
   */
  return 0;
}



/*
 * Return statistics to the caller
 */
struct net_device_stats* sccmb_stats(struct net_device* dev)
{
  struct sccmb_priv* priv = netdev_priv(dev);
  
  return &(priv->stats);
}



/*
 * The "change_mtu" method is usually not needed.
 * If you need it, it must be like this.
 */
int sccmb_change_mtu(struct net_device* dev, int new_mtu)
{
  struct sccmb_priv*          priv = netdev_priv(dev);
  unsigned long               flags;
  spinlock_t*                 lock = &(priv->lock);

  /* check ranges */
  if ((new_mtu < 68) || (new_mtu > 1500)) return -EINVAL;

  /*
   * Simply accept the value
   */
  spin_lock_irqsave(lock, flags);
  dev->mtu = new_mtu;
  spin_unlock_irqrestore(lock, flags);
  
  return 0;
}



/*
 * This function is called to fill up an eth header, since ARP is not
 * available on the interface.
 */
int sccmb_rebuild_header(struct sk_buff* skb)
{
  printk(KERN_WARNING "sccmb_rebuild_header() called - ignoring\n");

  return 0;
}

static void sccmbx_create_header(struct sk_buff* skb, struct net_device* dev,
				 u8 subsys, u8 dest, unsigned int len)
{
  struct sccmbx_hdr* header;

  /* Prepend the message header */
  header = (struct sccmbx_hdr*)skb_push(skb, SCCMBX_HLEN);
  sccmbx_init_header((struct sccmbx_hdrbuf*)header, subsys, dest, len);
}

int sccmb_header(struct sk_buff* skb, struct net_device* dev,
                 unsigned short type, const void* daddr, const void* saddr,
                 unsigned int len)
{
  /* Currently, we do not support anything but IP */
  if (type != ETH_P_IP) {
    return -dev->hard_header_len;
  }

  /* Create header. We always use SCCMBX_SUBSYS_NET when going through the
   * header_ops callback. */
  sccmbx_create_header(skb, dev, SCCMBX_SUBSYS_NET, 0, len);
      
  return (dev->hard_header_len);
}

/* Allocate a packet */
struct sk_buff* sccmbx_allocate_packet(u8 subsys, u8 dest, int len, void* *body)
{
	struct sk_buff* skb;
	struct net_device* dev = sccmb_dev;

	/* Verify parameters */
	if (!dev || (subsys == SCCMBX_SUBSYS_NET) || (len < 0) || (len>dev->mtu)) {
		return NULL;
	}

	/* Allocate packet */
	skb = alloc_skb(len + LL_ALLOCATED_SPACE(dev) + 15, GFP_KERNEL);
	if (!skb) {
		return NULL;
	}
	skb_reserve(skb, LL_RESERVED_SPACE(dev));
	*body = skb_put(skb, len);

	/* Prepare header */
	sccmbx_create_header(skb, dev, subsys, dest + 1, len);

	skb->dev = dev;
	skb->protocol = 0;

	return skb;
}
EXPORT_SYMBOL(sccmbx_allocate_packet)

static const struct net_device_ops sccmb_netdev_ops = {
	.ndo_open		= sccmb_open,
	.ndo_stop		= sccmb_close,
	.ndo_set_config		= sccmb_config,
	.ndo_start_xmit		= sccmb_tx,
	.ndo_do_ioctl	        = sccmb_ioctl,
	.ndo_get_stats		= sccmb_stats,
	.ndo_change_mtu		= sccmb_change_mtu,
	.ndo_tx_timeout		= sccmb_tx_timeout,
};

static const struct header_ops sccmb_header_ops = {
	.create			= sccmb_header,
	.rebuild		= sccmb_rebuild_header,
};

/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
void sccmb_init(struct net_device* dev)
{
  struct sccmb_priv* priv;

  /*
   * Then initialize the priv field. This encloses the statistics
   * and a few private fields.
   */
  priv = netdev_priv(dev);
  memset(priv, 0, sizeof(struct sccmb_priv));
  
  spin_lock_init(&priv->lock);

  /* Get meaningful default values */
  ether_setup(dev);

  /* Set the correct function pointers */
  dev->netdev_ops = &sccmb_netdev_ops;
  dev->header_ops = &sccmb_header_ops;
  
  dev->watchdog_timeo   = timeout;

  /* Configure NAPI interrupt handling because we may receive multiple
   * packets per interrupt. Note that Lehnix/MCEMU set LVT0's vector to 4.
   */
  netif_napi_add(dev, &priv->napi, sccmb_poll, 16);
  dev->irq              = 4;

  /* Keep the default flags; just add NOARP */
  dev->flags           |= IFF_NOARP;
  /* Checksum checks are not required */
  // dev->features        |= NETIF_F_NO_CSUM;
  /* Disable caching of (nonexistent) ARP replies */
  //dev->hard_header_cache = NULL;
  /* Change the hardware header as there is no need for an Ethernet format */
  dev->hard_header_len    = SCCMBX_HLEN;
  dev->addr_len           = 0;
}



/*
 * Finally, the module stuff
 */
void sccmb_cleanup(void)
{
  if (sccmb_dev) {
    unregister_netdev(sccmb_dev);
    free_netdev(sccmb_dev);
    sccmb_dev = NULL;
  }
}



int sccmb_init_module(void)
{
  int result;

  /* This driver does only work in a bare-metal environment */
  if (!scc_bare_metal()) {
    printk(KERN_INFO "sccmbx: startup in non-SCC or paravirtualized environment.\n");
    return -EINVAL;
  }

  __sccmbx_set_rx_proc(SCCMBX_SUBSYS_NET, sccmbx_rx_net);

  /* Allocate the devices */
  sccmb_dev = alloc_netdev(sizeof(struct sccmb_priv), "mb0", sccmb_init);
  if (!sccmb_dev) return -ENOMEM;

  result = register_netdev(sccmb_dev);
  if (result) {
    printk(KERN_WARNING "sccmb: error %i registering device \"%s\"\n", 
                        result, sccmb_dev->name);
    free_netdev(sccmb_dev);
    return -ENODEV;
  }

  return 0;
}



module_init(sccmb_init_module);
module_exit(sccmb_cleanup);
