/*
 * sccpc.c -- SCC PC network driver
 *
 * Portions Copyright (C) 2009 Intel Corp.
 *
 * The code is based on snull.c from the book "Linux Device
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

#include <linux/sccsys.h>

#include <linux/in6.h>
#include <asm/checksum.h>

#include <asm/io.h>             /* ioremap() & friends */
#include <asm/pgtable.h>        /* page protection bits */
#include <asm/apic.h>           /* apic_read() & apic_write() */

MODULE_AUTHOR("Werner Haas");
MODULE_LICENSE("GPL");

/* DEBUG messages */
#define DEBUG_MSG 0
#define PRINTD(format, args...) if (DEBUG_MSG) { printk(format, ##args); }


/* SCC specific constants
 * Note that values that can be changed through LUT configuration are made
 * available as module parameters so it is not necessary to recompile the
 * driver if a non-default setup is used.
 */
/* Maximum size of data transfers */
#define MAX_PACKET_SIZE     4096
/* Number of bytes in a cache line */
#define SCC_CLINE_SIZE      32
/* Size of a SCC memory tile (16MB) */
#define SCC_TILE_SIZE       0x01000000

/* Interrupt configuration (register+mask bit) */
#define SCCPC_LVT           APIC_LVT1
#define SCCPC_IRQ_MASK      0x00000001

/* The new instruction to flush message buffer content from L1 */
#define CL1FLUSHMB __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" )

/* Register offsets of the host mailbox registers
 * Note that we hae to use different cachelines to use the write
 * combining buffer (WCB) efficiently.
 */
#define MBX_NULL            0x00 /* Dummy address to flush WCB */
#define MBX_CONFIG          0x20
#define MBX_PACKETSTART     0x40
#define MBX_PACKETDATA      0x60
#define MBX_RXDONE          0x80



/*
 * Module parameters
 */
/* Make the number of packets in flight configurable. Note that an unsigend
 * variable is used to store the status at the host, i.e. the value should
 * be <= 32!
 */
static int rxPacketSlots = 16;
module_param(rxPacketSlots, int, 0644);
MODULE_PARM_DESC(rxPacketSlots, "Number of packet slots in DDR3 memory");

/* Start address of the network buffer in the shared memory space. It is
 * assumed that 4 consecutive memory tiles are resered for the 4 quadrants
 * and that each core uses txPacketSlots*MAX_PACKET_SIZE bytes.
 * In the default memory layout the shared region starts at 0x8000.0000 but
 * we hae to leave space for the shared memory TTY.
 */
static int rxBaseAddress = 0x80200000;
module_param(rxBaseAddress, int, 0644);
MODULE_PARM_DESC(rxBaseAddress, "Start address of the packet space at the MC");

/* Address of the Tx mailbox. This address is mapped to the host instead
 * of DDR3 in the LUTs.
 */
static int txMailbox = 0xFA000000;
module_param(txMailbox, int, 0644);
MODULE_PARM_DESC(txMailbox, "Start address of the Tx mailbox at the host");

/* We need the local CRB registers so we can determine our core ID and clear
 * the interrupt.
 */
static int local_crb_offset = 0xF8000000;
module_param(local_crb_offset, int, 0644);
MODULE_PARM_DESC(local_crb_offset, "Start address of the local register bank");

/* When interrupts are disabled the NAPI poll function is added to the queue
 * at startup never returns 0.
 */
static int noIrq = 0;
module_param(noIrq, int, 0644);
MODULE_PARM_DESC(noIrq, "Do not use interrupts to trigger receiver");



/*
 * The SCC message buffer device
 */
struct net_device* sccpc_dev;

struct sccpc_priv {
  struct net_device_stats     stats;
  struct napi_struct          napi;
  spinlock_t                  lock;
  u8                          shutdown;

  /* Device-specific constants: */
  u8                          currentSlot;
  void*                       irqAddress;
  /* SCC specific physical memory addresses are mapped into
   * kernel space:  Core Register Bank
   *                Reiceive Buffer
   *                Host mailbox
   */
  void*                       crb;
  void*                       rxb;
  void*                       mailbox;
};



/*
 * Open and close
 * These functions are called when an interface is activated/stopped. Thus,
 * any system resources should be registered and the device itself should
 * be initialized.
 */
static irqreturn_t sccpc_interrupt(int, void*);

void sccpc_unmap(struct net_device* dev)
{
  struct sccpc_priv*          priv = netdev_priv(dev);


  iounmap(priv->crb);
  iounmap(priv->rxb);
  iounmap(priv->mailbox);
}

int sccpc_open(struct net_device* dev)
{
  struct sccpc_priv*          priv = netdev_priv(dev);
  unsigned                    address;
  int                         tmp;
  int                         status = 0;
  int                         x, y, z;
  int                         quadrant = 0;
  int                         position = 0;

  /* First map the local CRB register space so we can determine our ID */
  priv->crb = ioremap_nocache(local_crb_offset, PAGE_SIZE);
  if (!priv->crb) {
    printk(KERN_WARNING "sccpc_open: Failed to map register bank\n");
    return -EIO;
  }

  /* Determine our location from the tile ID register */
  tmp = readl((void*)(priv->crb + SCC_TILEID));
  x = (tmp>>3) & 0x0f; /* bits 06:03 */
  y = (tmp>>7) & 0x0f; /* bits 10:07 */
  z = (tmp   ) & 0x07; /* bits 02:00 */

  /* Set the interrupt register address accordingly */
  if (z == 0) priv->irqAddress = priv->crb + SCC_GLCFG0;
  else        priv->irqAddress = priv->crb + SCC_GLCFG1;


  /* Calculate the quadrant and the location within. Note that rows 1/3
   * correspond to positions 6..12!
   */
  position = 6*y + 2*x + z;
  if (x > 2) {
    quadrant += 1;
    position -= 6;
  }
  if (y > 1) {
    quadrant += 2;
    position -= 12;
  }
  /* Map space for rxPacketSlots and mark it as message passing buffer so it
   * can be easily invalidated. The write-through flag is also set so every
   * write is immediately propagated to main memory - note that the Rx buffer
   * is mostly read-only.
   */
  address = PAGE_ALIGN(rxBaseAddress
                       + quadrant*SCC_TILE_SIZE
                       + position*rxPacketSlots*MAX_PACKET_SIZE);
  priv->rxb = ioremap_mpbt(address, rxPacketSlots*MAX_PACKET_SIZE);
  if (!priv->rxb) {
    printk(KERN_WARNING "sccpc_open: Failed to map receive buffer\n");
    sccpc_unmap(dev);
    return -EIO;
  }
  PRINTD(KERN_INFO "sccpc: core %i in quadrant %i (0x%08X)\n",
                   position, quadrant, address);

  /* Finally map the host mailbox */
  priv->mailbox = ioremap_mpbt(txMailbox, PAGE_SIZE);
  if (!priv->mailbox) {
    printk(KERN_WARNING "sccpc_open: Failed to map host mailbox\n");
    sccpc_unmap(dev);
    return -EIO;
  }
  PRINTD(KERN_INFO "sccpc: rxb = %p, mailbox = %p\n", priv->rxb, priv->mailbox);

  /* Configure interrupt handling */
  status = request_irq(dev->irq, &sccpc_interrupt, IRQF_SHARED, "sccpc", dev);
  if (status) {
    printk(KERN_WARNING "Can't get interrupt #%d\n", dev->irq);
    sccpc_unmap(dev);
    return status;
  }

  /* Initialize the receive slots */
  for (position=0; position<rxPacketSlots; position++) {
    *((unsigned*)(priv->rxb + position*MAX_PACKET_SIZE)) = 0;
  }
  /* Tell the peer side we will start receiving at slot 0 and flush the
   * write-combining buffer so all data gets out to its destination
   */
  priv->currentSlot = 0;
  *((unsigned*)(priv->mailbox+MBX_CONFIG))  = 0;
  *((unsigned*)(priv->mailbox+MBX_NULL))    = 0;


  /* Assign the hardware address of the board (6 octets for Ethernet)
   * Note that the first octet of ethernet multicast addresses is odd
   */
  memcpy(dev->dev_addr, "\0HOST0", ETH_ALEN);

  netif_start_queue(dev);
  napi_enable(&priv->napi);

  PRINTD(KERN_NOTICE "sccpc_init: napi = %p, priv = %p, dev = %p\n", &priv->napi, priv, dev);

  /* If interrupts are not used we immediately add the polling function
   * to the queue which would otherwise be done through the IRQ handler.
   */
  if (noIrq) napi_schedule(&priv->napi);

  return 0;
}


int sccpc_close(struct net_device* dev)
{
  struct sccpc_priv*          priv = netdev_priv(dev);
  napi_disable(&priv->napi);

  /* Unmap/release the SCC resources */
  free_irq(dev->irq, dev);
  sccpc_unmap(dev);

  netif_stop_queue(dev);
  return 0;
}



/*
 * Configuration changes (passed on by ifconfig)
 */
int sccpc_config(struct net_device* dev, struct ifmap* map)
{
  /* Can't act on a running interface */
  if (dev->flags & IFF_UP) return -EBUSY;

  /* Don't allow changing the I/O address */
  if (map->base_addr != dev->base_addr) {
    printk(KERN_WARNING "sccpc: Can't change I/O address\n");
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



/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */
void sccpc_rx(struct net_device* dev, u8 slot, unsigned len)
{
  struct sccpc_priv*          priv = netdev_priv(dev);
  void*                       address = priv->rxb + slot*MAX_PACKET_SIZE;
  struct sk_buff*             skb;

#ifdef CONFIG_SCCPC_DUMP_PACKETS
  printk(KERN_NOTICE "sccpc_rx(): slot = %d, len = %u\n", slot, len);
#endif

  if (len < sizeof(struct iphdr) || len > dev->mtu) {
    /* Simply drop the packet */
    printk(KERN_NOTICE "sccpc_rx(): illegal packet length %d => drop\n", len);
    priv->stats.rx_errors++;
    priv->stats.rx_dropped++;

    goto rxDone;
  }

  /* Build a skb for the packet data so upper layers can handle it
   * Note that IP headers should be aligned on 16B boundaries, i.e. skip
   * the first 2 header bytes.
   */
  skb = dev_alloc_skb(len);
  if (!skb) {
    if (printk_ratelimit() )
      printk(KERN_NOTICE "sccpc rx: low on mem - packet dropped\n");

    /* Note: since we do not clear the offset descriptor we do not trigger
     * a retransmission and the packet will eventually be processed.
     */
    priv->stats.rx_dropped++;
    return;
  }

  /* IP headers should be aligned on 16B boundaries, i.e. we just reserve the
   * payload area and do not prepend the header bytes which are not used
   * anymore.
   */
  skb_put(skb, len);
  memcpy(skb->data, address+2, len);

#ifdef CONFIG_SCCPC_DUMP_PACKETS
  printk(KERN_NOTICE "sccpc: received packet, len = %u\n", len);
  sccutil_dump_buffer(skb->data, len);
#endif

  /* Update the interface statistics (also count the header bytes) */
  priv->stats.rx_packets++;
  priv->stats.rx_bytes += 2+len;
  dev->last_rx = jiffies;

  /* Write metadata, and then pass to the receive level */
  skb->dev        = dev;
  skb_set_transport_header(skb, 0);
  skb_set_network_header(skb, 0);
  skb_set_mac_header(skb, 0);
  skb->protocol   = htons(ETH_P_IP);
  skb->pkt_type   = PACKET_HOST;
  skb->ip_summed  = CHECKSUM_UNNECESSARY;

  netif_rx(skb);

rxDone:
  /* Invalidate the packet length so we do not process the data again */
  *((unsigned*)(priv->rxb + slot*MAX_PACKET_SIZE)) = 0;

  /* Tell the host we are done with the slot and move on to the next */
  *((unsigned*)(priv->mailbox+MBX_RXDONE)) = slot;
  *((unsigned*)(priv->mailbox+MBX_NULL)) = 0;
  priv->currentSlot = (priv->currentSlot + 1) % rxPacketSlots;

  return;
}



/*
 * The poll function looks for valid packets in the receive buffer
 */
static int sccpc_poll(struct napi_struct* napi, int to_do)
{
  struct sccpc_priv*          priv = container_of(napi, struct sccpc_priv, napi);
  struct net_device*          dev = napi->dev;
  int                         npackets = 0;
  int                         quota = to_do;
  void*                       address;
  unsigned                    len;
  unsigned                    tmp;

#ifdef CONFIG_SCCPC_DUMP_PACKETS
  printk(KERN_NOTICE "sccpc_poll: entry. napi = %p, priv = %p, dev = %p, quota = %d\n", napi, priv, dev, quota);
#endif

  /* Process up to quota packets from the receive queue */
  while (npackets<quota) {
    /* Quit polling when told to do so */
    if (priv->shutdown) {
      napi_complete(napi);
      return 0;
    }
    
    
    /* Flush the message buffer so we get up to date message buffer data */
    CL1FLUSHMB;

    /* Check for valid packet lengths */
    address = priv->rxb + priv->currentSlot*MAX_PACKET_SIZE;
    len = 256*( *((u8*)(address  )) )
        +     ( *((u8*)(address+1)) );

#ifdef CONFIG_SCCPC_DUMP_PACKETS
    printk(KERN_NOTICE "sccpc_poll: currentSlot = %d, len = %u\n", priv->currentSlot, len);
#endif

    /* Call the standard receive handler if there is a packet */
    if (len) {
      /* Clear the interrupt bit */
      tmp  = readl((void*)priv->irqAddress);
      tmp &= (~SCCPC_IRQ_MASK);
      writel(tmp, (void*)priv->irqAddress);

      /* Fetch the packet data */
      sccpc_rx(dev, priv->currentSlot, len);
      npackets++;
    }
    /* Else perform a clean exit */
    else {
      unsigned long v;

      /* If interrupts are disabled we have to remain in the polling function,
       * i.e. we may signal that we are done with packet processing but must
       * not remove us from the queueing list via netif_rx_complete().
       */
      if (noIrq) {
        break;
      }

      /* Tell the system we are done with polling */
      napi_complete(napi);
      
      v = apic_read(SCCPC_LVT);
      apic_write(SCCPC_LVT, v & ~APIC_LVT_MASKED);

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
 * with a single interrupt.
 */
static irqreturn_t sccpc_interrupt(int irq, void* dev_id)
{
  struct net_device*          dev = (struct net_device*)dev_id;
  struct sccpc_priv*          priv = netdev_priv(dev);

  unsigned long               v;
  /* Paranoid */
  if (!priv) {
    printk(KERN_DEBUG "sccpc interrupt %d for unknown device\n", irq);
    return IRQ_NONE;
  }

#ifdef CONFIG_SCCPC_DUMP_PACKETS
  printk("sccpc_interrupt: irq = %d, napi = %p, priv = %p, dev = %p\n", irq, &priv->napi, priv, dev);
#endif

  /* Mask further interrupts and start the polling process */
  v = apic_read(SCCPC_LVT);
  apic_write(SCCPC_LVT, v | APIC_LVT_MASKED);

  napi_schedule(&priv->napi);
  
  return IRQ_HANDLED;
}



/*
 * Transmit a packet (called by the kernel)
 */
int sccpc_tx(struct sk_buff* skb, struct net_device* dev)
{
  struct sccpc_priv*          priv = netdev_priv(dev);
  unsigned                    bytesToCopy;
  unsigned                    bytesTransferred  = 0;
  unsigned                    bytesRemaining    = 0;
  unsigned long               flags;
  spinlock_t*                 lock = &(priv->lock);

  spin_lock_irqsave(lock, flags);

#ifdef CONFIG_SCCPC_DUMP_PACKETS
  printk(KERN_NOTICE "sccpc_tx: sending packet. len = %u\n", skb->len);
  sccpc_dump_buffer(skb->data, skb->len);
#endif

  /* Send the packet data to the host. Note that the first cacheline
   * has its own mailbox address.
   */
  bytesToCopy = (skb->len > SCC_CLINE_SIZE) ? SCC_CLINE_SIZE : skb->len;
  memcpy(priv->mailbox+MBX_PACKETSTART, skb->data, bytesToCopy);
  bytesTransferred += bytesToCopy;
  
  /* Copy all remaining data to the PACKETDATA mailbox address */
  while (bytesTransferred < skb->len) {
    bytesRemaining = skb->len - bytesTransferred;
    bytesToCopy = (bytesRemaining > SCC_CLINE_SIZE) ? SCC_CLINE_SIZE 
                                                    : bytesRemaining;
    
    memcpy(priv->mailbox+MBX_PACKETDATA, 
           skb->data+bytesTransferred, bytesToCopy);
    bytesTransferred += bytesToCopy;
  }
  /* If the last memcpy did not fill the entire cache line we have to flush
   * the write combining buffer.
   */
  if (bytesToCopy < SCC_CLINE_SIZE) {
    /* Write to a different address if the partial data fits into a single
     * packet.
     */
    if (bytesToCopy <= 8) *((unsigned*)(priv->mailbox+MBX_NULL)) = 0;
    /* Else fill up the cacheline with dummy data */
    else memcpy(priv->mailbox+MBX_PACKETDATA+bytesToCopy, skb->data, 
                SCC_CLINE_SIZE-bytesToCopy);
  }
  spin_unlock_irqrestore(lock, flags);


  /* Transmission always succeeds so save the timestamp of the transmission,
   * update the statistics and free the socket buffer
   */
  dev->trans_start = jiffies;

  priv->stats.tx_packets++;
  priv->stats.tx_bytes += skb->len;
  dev_kfree_skb_any(skb);

  return NETDEV_TX_OK;
}



/*
 * Deal with a transmit timeout.
 */
void sccpc_tx_timeout(struct net_device* dev)
{
  /* A timeout should never occur as we send all data to the host. Wake the
   * queue in case it was stopped by someone.
   */
  //printk(KERN_NOTICE "sccpc_tx_timeout()\n");

  netif_wake_queue(dev);
  return;
}



/*
 * ioctl commands
 */
int sccpc_ioctl(struct net_device* dev, struct ifreq* rq, int cmd)
{
  /*
   * Custom commands
   */
  return 0;
}



/*
 * Return statistics to the caller
 */
struct net_device_stats* sccpc_stats(struct net_device* dev)
{
  struct sccpc_priv* priv = netdev_priv(dev);

  return &(priv->stats);
}



/*
 * The "change_mtu" method is usually not needed.
 */
int sccpc_change_mtu(struct net_device* dev, int new_mtu)
{
  struct sccpc_priv*          priv = netdev_priv(dev);
  unsigned long               flags;
  spinlock_t*                 lock = &(priv->lock);

  /* Check ranges, i.e. make sure that we send at least the IP header and
   * do not cross the maximum packet size (note the 2 header bytes).
   */
  if ((new_mtu < sizeof(struct iphdr)) || (new_mtu > MAX_PACKET_SIZE-2))
    return -EINVAL;

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
int sccpc_rebuild_header(struct sk_buff* skb)
{
  printk(KERN_WARNING "sccpc_rebuild_header() called - ignoring\n");

  return 0;
}

int sccpc_header(struct sk_buff* skb, struct net_device* dev,
                 unsigned short type, const void* daddr, const void* saddr,
                 unsigned len)
{
  /* Prepend 2 header bytes containing the packet length */
  u8* header = skb_push(skb, 2);

  /* Store the length, starting with the MSBs */
  header[0] = len/256;
  header[1] = len%256;

  return (dev->hard_header_len);
}


static const struct net_device_ops sccpc_netdev_ops = {
	.ndo_open		= sccpc_open,
	.ndo_stop		= sccpc_close,
	.ndo_set_config		= sccpc_config,
	.ndo_start_xmit		= sccpc_tx,
	.ndo_do_ioctl		= sccpc_ioctl,
	.ndo_get_stats		= sccpc_stats,
	.ndo_change_mtu		= sccpc_change_mtu,
	.ndo_tx_timeout		= sccpc_tx_timeout,
};

static const struct header_ops sccpc_header_ops = {
	.create			= sccpc_header,
	.rebuild		= sccpc_rebuild_header,
};

/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
void sccpc_init(struct net_device* dev)
{
  struct sccpc_priv* priv;

  /*
   * Then initialize the priv field. This encloses the statistics
   * and a few private fields.
   */
  priv = netdev_priv(dev);
  memset(priv, 0, sizeof(struct sccpc_priv));

  spin_lock_init(&priv->lock);

  /* Get meaningful default values */
  ether_setup(dev);

  /* Set the correct function pointers */
  dev->netdev_ops = &sccpc_netdev_ops;
  dev->header_ops = &sccpc_header_ops;

  dev->watchdog_timeo     = 10;

  /* Configure NAPI interrupt handling because we may receive multiple
   * packets per interrupt. Note that Lehnix/MCEMU set LVT1's vector to 3.
   */
  netif_napi_add(dev, &priv->napi, sccpc_poll, 16);
  dev->irq                = 3;

  /* Only set the NOARP flag as there is no broad-/multicast support */
  dev->flags              = IFF_NOARP;
  /* Disable caching of (nonexistent) ARP replies */
  //dev->hard_header_cache  = NULL;
  /* Change the hardware header as there is no need for an Ethernet format */
  dev->hard_header_len    = 2;
  dev->addr_len           = 0;
}



/*
 * Finally, the module stuff
 */
int sccpc_dev_event_handler(struct notifier_block* self, 
                            unsigned long event, void* data)
{
  struct net_device* dev  = (struct net_device*)data;
  struct sccpc_priv* priv = netdev_priv(dev);

    
  /* Watch out for the interface going down so we can stop the polling
   * function.
   */
  if ((strcmp(dev->name, "pc0") == 0) && (event == NETDEV_GOING_DOWN)) {
    priv->shutdown = 1;
  }
  
  return NOTIFY_DONE;
}

static struct notifier_block sccpc_dev_notifier = {
  .notifier_call = sccpc_dev_event_handler
};



void sccpc_cleanup(void)
{
  if (sccpc_dev) {
    unregister_netdev(sccpc_dev);
    free_netdev(sccpc_dev);
  }
}



int sccpc_init_module(void)
{
  int result;

  /* This driver does only work in a bare-metal environment */
  if (!scc_bare_metal()) {
    printk(KERN_INFO "sccpc: startup in non-SCC or paravirtualized environment.\n");
    return -EINVAL;
  }

  /* Allocate the devices */
  sccpc_dev = alloc_netdev(sizeof(struct sccpc_priv), "pc0", sccpc_init);
  if (!sccpc_dev) return -ENOMEM;

  result = register_netdev(sccpc_dev);
  if (result) {
    printk(KERN_WARNING "sccpc: error %i registering device \"%s\"\n",
                        result, sccpc_dev->name);
    free_netdev(sccpc_dev);
    return -ENODEV;
  }

  /* Make sure we are informed about events affecting this device */
  register_netdevice_notifier(&sccpc_dev_notifier);

  return 0;
}



module_init(sccpc_init_module);
module_exit(sccpc_cleanup);
