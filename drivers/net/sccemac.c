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

#include "sccemac.h"

/* Change Log
 * 0.3.0	01/14/2011
 *   o ported driver to NAPI structure
 * 0.2.1	12/08/2010
 *   o fix possible rx packet lost
 * 0.2.0	11/19/2010
 *   o added irq handling
 * 0.1.4	09/06/2010
 *   o added fpga emac port detection
 *   o fix rx illegal packet length bug
 * 0.1.3	08/27/2010
 *   o create general configuration functions
 *   o added support for emac2 and emac3
 * 0.1.2	08/26/2010
 *   o fix emac1 transfer setup
 * 0.1.1	08/19/2010
 *   o handle more than one packet in receive
 *   o overflow check
 * 0.1.0	08/18/2010
 *   o first release
 */

#define MODVERSTRING	"0.3.0"
#define OVERFLOW_CHECK	1

#define IRQ_STATUS	0xD000
#define IRQ_MASK	0xD200
#define IRQ_RESET	0xD400
#define IRQ_CONFIG	0xD800

/** emac0 network driver structure */
static struct net_device *emac0_dev = NULL;
/** emac1 network driver structure */
static struct net_device *emac1_dev = NULL;
/** emac2 network driver structure */
static struct net_device *emac2_dev = NULL;
/** emac3 network driver structure */
static struct net_device *emac3_dev = NULL;

/** optional select ethernet port */
static int ethernet_port = EMAC0 | EMAC1 | EMAC2 | EMAC3;
module_param(ethernet_port, int, 0644);
MODULE_PARM_DESC(ethernet_port,
	"Ethernet ports the driver should use (0x01=emac0, 0x02=emac1,"
	" 0x04=emac2, 0x08=emac3)");

/** debug level */
static int debug_level = 0;
module_param(debug_level, int, 0644);
MODULE_PARM_DESC(debug_level, "Debug level");

/** override sccKit configuration */
static int override = 0;
module_param(override, int, 0644);
MODULE_PARM_DESC(override, "Override sccKit configuration");

static int core0 = 0;

/**
 * \brief Read long from emac
 * \param pAddr address we want to read
 * \return value stored in address
 */
static int emac_readl(void *addr) {
	int ret;

	ret = readl(addr);
	/* no error: read twice, as xilinx ip need some time... */
	ret = readl(addr);

	return ret;
}

/**
 * \brief Write long to emac
 * \param value entry for address
 * \param pAddr address we want to write
 */
static void emac_writel(int value, void *addr) {
	writel(value, addr);
}

static void init_xilinx_port(int emac, int base) {
	int flow_control = 0;
	int transmitter_addr = 0;
	int receiver1_addr = 0;
	int config_add = 0;
	int add_filter_mod = 0;

	transmitter_addr = emac_readl(RA(base + TRANSMITTER_ADDRESS, 0));
	receiver1_addr = emac_readl(RA(base + RECEIVER1_ADDRESS, 0));

	/* Check if IP is already configured */
	//if (!(transmitter_addr & 0x10000000) ||
       //    !(receiver1_addr & 0x10000000)) {
    if (core0 == 0)
       {
		EPRINTK(DEBUG_INFO, "Config eMAC RX %d\n", emac);

		/* Disable tx and rx flow control of eMAC */
		EPRINTK(DEBUG_INFO, "Disabling tx/rx flow control of eMAC%d\n", emac);
		flow_control = emac_readl(RA(base + CONFIG_FLOW_CONTROL_ADD, 0));

		/* Set top 3 bits of the flow control configuration to zero,
		 * therefore disabling tx and rx flow control
		 */
		flow_control &= 0x7FFFFFF;
		emac_writel(flow_control, RA(base + CONFIG_FLOW_CONTROL_ADD, 0));

		/* Sanity check */
		flow_control = emac_readl(RA(base + CONFIG_FLOW_CONTROL_ADD, 0));
		EPRINTK(DEBUG_INFO, "  CONFIG_FLOW_CONTROL_ADD set: 0x%x\n",
                flow_control);

		/* Setting the tx configuration bit to enable the transmitter and
		 * set to full duplex mode.
		 */
		EPRINTK(DEBUG_INFO, "Setting rx configuration of eMAC%d\n", emac);
		transmitter_addr = emac_readl(RA(base + TRANSMITTER_ADDRESS, 0));

		/* Now set the relevant bits and write back into the register:
		 * 26 (half duplex) = 0, 28 (transmit enable) = 1, 31 (reset) = 0
		 */
		transmitter_addr &= ~(1 << 31);
		transmitter_addr &= ~(1 << 26);
		transmitter_addr |= (1 << 28);
		emac_writel(transmitter_addr, RA(base + TRANSMITTER_ADDRESS, 0));

		transmitter_addr = emac_readl(RA(base + TRANSMITTER_ADDRESS, 0));
		EPRINTK(DEBUG_INFO, "  TRANSMITTER_ADDRESS set: %x\n",
                transmitter_addr);

		/* Setting the rx configuration bit to enable the transmitter and
		 * set to full duplex mode.
		 */
		EPRINTK(DEBUG_INFO, "Setting IP configuration of EMAC%d\n", emac);

		/* Read the current config value from the register */
		receiver1_addr = emac_readl(RA(base + RECEIVER1_ADDRESS, 0));

		/* Now set the relevant bits and write back into the register:
		 *  25 = 1, 26 = 0, 28 = 1, 31 = 0
		 */
		/* Length/Type Error Check Disable */
		receiver1_addr |= (1 << 25);
		/* Disable Half Duplex => Full Duplex */
		receiver1_addr &= ~(1 << 26);
		/* Receiver enable */
		receiver1_addr |= (1 << 28);
		/* Reset */
		receiver1_addr &= ~(1 << 31);
		emac_writel(receiver1_addr, RA(base + RECEIVER1_ADDRESS, 0));

		receiver1_addr = emac_readl(RA(base + RECEIVER1_ADDRESS, 0));
		EPRINTK(DEBUG_INFO, "  RECEIVER1_ADDRESS set: %x\n",
                receiver1_addr);

		/* Setting the speed to eMAC to 1Gb/s */
		EPRINTK(DEBUG_INFO, "Setting speed of EMAC%d to 1Gb/s\n", emac);

		/* Read the current config value from register */
		config_add = emac_readl(RA(base + CONFIG_ADD, 0));

		/* Now set the relevant bits and write back into the register:
		 * 31 = 1, 30 = 0
		 */
		/* MAC Speed Configuration: 00 - 10Mbps, 01 - 100Mbps, 10 - 1Gbps */
		config_add |= (1 << 31);
		config_add &= ~(1 << 30);
		emac_writel(config_add, RA(base + CONFIG_ADD, 0));

		config_add = emac_readl(RA(base + CONFIG_ADD, 0));
		EPRINTK(DEBUG_INFO, "  CONFIG_ADD set: %x\n", config_add);

		/* Read the current config addr filter mode */
		add_filter_mod = emac_readl(RA(base + ADD_FILTER_MOD, 0));

		/* Not set the relevant bits and write back into the register:
		 * 31 (promiscuous mode) = 1 not working, but thats ok!
		 */
		add_filter_mod |= (1 << 31);
		emac_writel(add_filter_mod, RA(base + ADD_FILTER_MOD, 0));

		add_filter_mod = emac_readl(RA(base + ADD_FILTER_MOD, 0));
		EPRINTK(DEBUG_INFO, "  ADD_FILTER_MOD set: %x\n", add_filter_mod);
	//} else {
	//	EPRINTK(DEBUG_INFO, "EMAC%d already configured!\n", emac);
	}
}

/**
 * \brief Allocate and return clean mpb
 * \param order order of memory pages
 * \return new buffer
 */
static unsigned char *alloc_buffer(int order) {
	struct page *page = NULL;
	unsigned char *buffer = NULL;
	int num = 1 << order;

	/* We are using alloc_pages and change_page_attr here, as the other
	 * functions do not allow setting our attributes (NC/PSE)
	 */
	page = alloc_pages(GFP_KERNEL, order);
	buffer = page_address(page);

	if (set_memory_wt_mpbt((unsigned long)buffer, 1 << order) < 0) {
		EPRINTK(DEBUG_INFO, "Failed");
		return NULL;
	}

	memset(buffer, 0x00, 0x20);
	memset(buffer + 0x20, 0xDA, num * PAGE_SIZE - 0x20);

	return buffer;
}

/**
 * \brief Get mac address
 * \param dev network device pointer
 */
static inline unsigned long long get_mac_address(struct net_device *dev) {
	struct emac_priv *priv = netdev_priv(dev);
	unsigned long long mac;
	int mac1 = readl(RA(0x7E00, 0));
	int mac2 = readl(RA(0x7E04, 0));

	mac = (((unsigned long long)mac1) << 32) + ( unsigned long long ) mac2;

	if (mac == 0x00) {
		mac = MAC_ADDRESS;
	}

	/* Calculate mac address of core depending on selected emac device */
	return mac + priv->device * 0x100 + priv->pid;
}

/**
 * \brief Setup application register of selected emac port
 * \param dev network device pointer
 * \param pid core pid
 * \param tile_offset tile memory offset
 * \param pos core route
 * \param mode contains route and destination to memory controller
 */
static void setup_emac(struct net_device *dev, int pid,
                 unsigned long long tile_offset, int pos, int mode) {
	struct emac_priv *priv = netdev_priv(dev);
	unsigned long long addr_offset = tile_offset;
	unsigned long long mac = 0;
	u32 tmp = 0;
	u16 write_offset = 0;
	u16 read_offset = 0;
	unsigned char core = 0;

	/* store own id */
	priv->pid = pid;

	EPRINTK(DEBUG_INFO, "Initialize eMAC 0x%x (pid %d)\n", priv->device, priv->pid);

	/**** Receive configuration ****/

	/* Set up ring buffer space */
	priv->rx_buffer_max = CLINE_PACKETS(BUFFER_SIZE) - 1;
	priv->rx_buffer = alloc_buffer(BUFFER_ORDER);

	/* Start address */
	EPRINTK(DEBUG_INFO, "  RX Buffer %p (%p phys)\n", priv->rx_buffer,
            (void*)virt_to_phys(priv->rx_buffer));

	tmp = virt_to_phys(priv->rx_buffer);
	addr_offset = tile_offset + tmp;
	addr_offset >>= 5;
	emac_writel(addr_offset, RA(priv->base + EMAC_RX_BUFFER_START_ADDRESS,
                priv->pid));
	tmp = emac_readl(RA(priv->base + EMAC_RX_BUFFER_START_ADDRESS, priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Buffer set to @%x\n", tmp);

	/* Set buffer write offset */
	write_offset = emac_readl(RA(priv->base + EMAC_RX_BUFFER_WRITE_OFFSET,
                              priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Buffer write offset at: %d\n", write_offset);

	/* Set buffer read offset to write offset */
	emac_writel(write_offset, RA(priv->base + EMAC_RX_BUFFER_READ_OFFSET,
                priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Buffer read offset set to: %d\n",
            emac_readl(RA(priv->base + EMAC_RX_BUFFER_READ_OFFSET, priv->pid)));
	priv->rx_read_offset = write_offset;

	/* Size */
	emac_writel(priv->rx_buffer_max, RA(priv->base + EMAC_RX_BUFFER_SIZE,
                priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Size set to %d\n",
            emac_readl(RA(priv->base + EMAC_RX_BUFFER_SIZE, priv->pid)));

	/* Threshold */
	emac_writel(0x01, RA(priv->base + EMAC_RX_BUFFER_THRESHOLD, priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Threshold set to %x\n",
            emac_readl(RA(priv->base + EMAC_RX_BUFFER_THRESHOLD, priv->pid)));

	/* Route */
	core = pid & 1;
	emac_writel((core << 24) | (pos << 16) | mode, RA(priv->base + EMAC_RX_MODE, priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Mode set to %x\n",
            emac_readl(RA(priv->base + EMAC_RX_MODE, priv->pid)));

	/* MAC */
	mac = get_mac_address(dev); 
	emac_writel(MAC_HI(mac), RA(priv->base +
                EMAC_RX_NETWORK_PORT_MAC_ADDRESS_HI, priv->pid));
	EPRINTK(DEBUG_INFO, "  MAC1 set to %x\n",
            emac_readl(RA(priv->base + EMAC_RX_NETWORK_PORT_MAC_ADDRESS_HI,
                       priv->pid)));
	emac_writel(MAC_LO(mac), RA(priv->base +
                EMAC_RX_NETWORK_PORT_MAC_ADDRESS_LO, priv->pid));
	EPRINTK(DEBUG_INFO, "  MAC2 set to %x\n",
            emac_readl(RA(priv->base + EMAC_RX_NETWORK_PORT_MAC_ADDRESS_LO,
                       priv->pid)));

	/* Activate network port by setting enable bit */
	emac_writel(0x01, RA(priv->base + EMAC_RX_NETWORK_PORT_ENABLE, priv->pid));
	EPRINTK(DEBUG_INFO, "  RX Port enable set to %x\n",
            emac_readl(RA(priv->base + EMAC_RX_NETWORK_PORT_ENABLE,
                       priv->pid)));

	/**** Transfer configuration ****/

	/* Set up ring buffer space */
	priv->tx_buffer_max = CLINE_PACKETS(BUFFER_SIZE) - 1;
	priv->tx_buffer = alloc_buffer(BUFFER_ORDER);

	/* Start address */
	EPRINTK(DEBUG_INFO, "  TX Buffer %p (%p phys)\n", priv->tx_buffer,
            (void*)virt_to_phys(priv->tx_buffer));
	tmp = virt_to_phys(priv->tx_buffer);
	emac_writel((tmp + tile_offset) >> 5, RA(priv->base +
                EMAC_TX_BUFFER_START_ADDRESS, priv->pid));
	tmp = emac_readl(RA(priv->base + EMAC_TX_BUFFER_START_ADDRESS, priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Buffer set to @%x\n", tmp);

	/* Get buffer read offset */
	read_offset = emac_readl(RA(priv->base + EMAC_TX_BUFFER_READ_OFFSET,
                             priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Buffer read offset at: %d\n", read_offset);

	/* Set buffer write offset to read offset */
	emac_writel(read_offset, RA(priv->base + EMAC_TX_BUFFER_WRITE_OFFSET,
                priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Buffer write offset set to: %d\n",
            emac_readl(RA(priv->base + EMAC_TX_BUFFER_WRITE_OFFSET,
                       priv->pid)));
	priv->tx_write_offset = read_offset;

	/* Size */
	emac_writel(priv->tx_buffer_max, RA(priv->base + EMAC_TX_BUFFER_SIZE,
                priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Size set to %d\n",
            emac_readl(RA(priv->base + EMAC_TX_BUFFER_SIZE, priv->pid)));

	/* Route */
	emac_writel(mode, RA(priv->base + EMAC_TX_MODE, priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Mode set to %x\n",
            emac_readl(RA(priv->base + EMAC_TX_MODE, priv->pid)));

	/* Activate network port by setting enable bit */
	emac_writel(0x01, RA(priv->base + EMAC_TX_NETWORK_PORT_ENABLE, priv->pid));
	EPRINTK(DEBUG_INFO, "  TX Port enable set to %x\n",
            emac_readl(RA(priv->base + EMAC_TX_NETWORK_PORT_ENABLE,
                       priv->pid)));
}

/**
 * \brief Resets local interrupt and then global
 * \prev priv emac private data
 */
static void emac_clear_interrupt(struct emac_priv *priv) {
	unsigned int tmp;

	unset_lapic_mask(EMAC_LVT, priv->net_device->irq);

	/* Clear interrupt bit */
	sccsys_clear_irq_direct(sccsys_get_pid(), EMAC_IRQ_MASK);

	/* Reset */
	tmp = priv->device;
	writel(tmp, RA(IRQ_RESET, priv->pid * 2));
}

/**
 * \brief Interrupt handler
 * \param irq current irq number
 * \param dev_id private net_device pointer
 * \return IRQ_HANDLED if we handled this interrupt, otherwise IRQ_NONE
 */
static irqreturn_t emac_interrupt(int irq, void *dev_id) {
	struct net_device *dev = (struct net_device *)dev_id;
	struct emac_priv *priv = netdev_priv(dev);
	unsigned int status = 0;

	if (!dev) {
		printk(KERN_DEBUG "emac interrupt %d for unknown device\n", irq);
		return IRQ_NONE;
	}

	status = readl(RA(IRQ_STATUS, priv->pid * 2));
	//if (printk_ratelimit()) {
	//	EPRINTK(DEBUG_READ, "Interrupt status Core: %x\n", status);
	//}

	if (!(status & priv->device)) {
		//printk(KERN_WARNING "Not an emac interrupt (0x%x)\n", status);
		return IRQ_NONE;
	}

	if (likely(napi_schedule_prep(&priv->napi))) {
		/* disable interrupt */
		set_lapic_mask(EMAC_LVT, priv->net_device->irq);
		__napi_schedule(&priv->napi);
	}

	//if (status & ~priv->device) {
		/* In case status indicates other int. devices... */
	//	printk(KERN_INFO "Status was 0x%x, now is 0x%x\n", status, readl(RA(IRQ_STATUS, priv->pid * 2)));
	//	return IRQ_NONE;
	//}

	return IRQ_HANDLED;
}

/**
 * \brief Open network device
 * \param dev network device
 * \return error code
 */
int emac_open(struct net_device *dev) {
	struct emac_priv *priv = netdev_priv(dev);
	unsigned long long offset = 0;
	unsigned long long mac = 0;
	int tmp = 0;
	scc_coord_t coord;
	scc_lut_t phys_base;
	int position = 0;
	int mode = 0;
	int subdest = 0;
	int route = 0;
	int i = 0;
	int status;

	netif_carrier_off(dev);

	/* Get SCC processor coordinates and number */
	coord = sccsys_get_coord();
	position = sccsys_get_pid();

	EPRINTK(DEBUG_INFO, "Location:\n");
	EPRINTK(DEBUG_INFO, "  X: %d Y: %d, Z: %d => Position: %d\n",
	    coord.x, coord.y, coord.z, position);

	/* Depending on core location read own private data
	 * (offset, subdest, route)
	 */
	phys_base = sccsys_read_lut_entry(sccsys_get_pid(), 0);

	offset = (unsigned long long)((unsigned long long)phys_base.address) << 24;
	subdest = phys_base.subdest;
	route = sccsys_get_route(phys_base);
	mode = (subdest << 8) + route;

	EPRINTK(DEBUG_INFO, "Using offset: %llx\n", offset);

	/* setup ethernet port */
	setup_emac(dev, position, offset, sccsys_get_route(coord), mode);

	/* set network addr */
	mac = get_mac_address(dev);
	for (i = 5; i != 0; i--) {
		dev->dev_addr[i] = mac & 0xFF;
		mac >>= 8;
	}

	status = request_irq(dev->irq, &emac_interrupt, IRQF_SHARED, "emac", dev);
	if (status) {
		printk(KERN_WARNING "Can't get interrupt #%d\n", dev->irq);
		return status;
	}

	napi_enable(&priv->napi);

	emac_clear_interrupt(priv);
	/* Enable interrupt */
	tmp = readl(RA(IRQ_MASK, priv->pid * 2));
	writel(tmp & ~(priv->device), RA(IRQ_MASK, priv->pid * 2));
	writel(EMAC_IRQ_CONFIG, RA(IRQ_CONFIG, priv->pid));
	netif_carrier_on(dev);

	netif_start_queue(priv->net_device);

	return 0;
}

/**
 * \brief Stop network device: stop queue
 * \param dev network device
 * \return error code (0)
 */
int emac_stop(struct net_device *dev) {
	struct emac_priv *priv = netdev_priv(dev);

	/* Shutdown poll */
	EPRINTK(DEBUG_INFO, "shutdown\n");
	napi_disable(&priv->napi);

	/* stop queue */
	EPRINTK(DEBUG_INFO, "stop queue\n");
	netif_stop_queue(dev);

	/* free irq */
	free_irq(dev->irq, dev);

	EPRINTK(DEBUG_INFO, "disable rx/tx ports\n");

	/* Disable network tx/rx port */
	emac_writel(0x00, RA(priv->base + EMAC_TX_NETWORK_PORT_ENABLE, priv->pid));
	emac_writel(0x00, RA(priv->base + EMAC_RX_NETWORK_PORT_ENABLE, priv->pid));

	netif_carrier_off(dev);

	return 0;
}

/**
 * \brief Get first bytes of addr0 (length)
 * \param dev net device
 * \return length of data or 0
 */
int get_addr0(struct net_device *dev) {
	struct emac_priv *priv = NULL;

	if (!dev) {
		return 0;
	}
	priv = netdev_priv(dev);

	if (priv && priv -> rx_buffer) {
		return readl(priv->rx_buffer);
	}

	return 0;
}

/**
 * \brief Receive and process packets from network device
 * \param priv emac private data
 * \param write_offset write offset
 * \param max_num maximum number of packets we can read at once
 * \return new read offset
 */
int emac_rx(struct emac_priv *priv, unsigned short write_offset, int max_num) {
	struct sk_buff *skb = NULL;
	unsigned short read_offset = priv->rx_read_offset;
	void *addr = NULL;
	unsigned short len = 0;
	int packets = 0;

again:
	read_offset++;
	if (read_offset < 1 || read_offset > priv->rx_buffer_max) {
		EPRINTK(DEBUG_READ, "read_offset is %d\n", read_offset);
		read_offset = 1;
	}
	addr = priv->rx_buffer + read_offset * 32;

	len = U16(addr);

	EPRINTK(DEBUG_READ, "device: %x current: start read at %d; write_offset "
            "at %d; addr: %p, packet len: %d (num packets:%d)\n", priv->device,
             read_offset, write_offset, addr, len, packets);

	/* check for over/underflow */
	if (len < sizeof(struct iphdr) || len > 1536) {
		int i = 0;

		printk(KERN_NOTICE "emac_rx(): illegal packet length %d => drop "
               "(num packets: %d)\n", len, packets);
		priv->stats.rx_errors++;
		priv->stats.rx_dropped++;

		read_offset = write_offset;
		printk("Buffer:\n");
		for (i = 0; i < 32; i++) {
			printk("%2.2x ", ((char*)addr)[i] & 0xFF);
		}
		printk("\n");

		printk("Buffer0:\n");
		for (i = 0; i < 32; i++) {
			printk("%2.2x ", ((char*)priv->rx_buffer)[i] & 0xFF);
		}
		printk("\n");

		//napi_disable(&priv->napi);
		//netif_stop_queue(priv->net_device);
		//return 0;

		goto rxDone;
	}

	/* allocate buffer */
	skb = dev_alloc_skb(len);
	if (!skb) {
		if (printk_ratelimit()) {
			printk(KERN_NOTICE "emac_rx(): low on mem - packet dropped\n");
		}

		priv->stats.rx_dropped++;
		return 0;
	}

	skb_put(skb, len);

	if (read_offset < write_offset) {
		memcpy(skb->data, addr + 2, len);
		read_offset += CLINE_PACKETS(skb->len + 2) - 1;
	} else {
		int rest;
		int bytesLeft = len;
		int bytesToCopy = len;

		EPRINTK(DEBUG_READ, "case: read_offset > write_offset (%d > %d)\n",
                read_offset, write_offset);
		/* rest to the end of buffer - 2 bytes length information */
		rest = (priv->rx_buffer_max - read_offset + 1) * 32 - 2;
		if (len > rest) {
			bytesToCopy = rest;
		}
		EPRINTK(DEBUG_READ, "bytes to copy: %d, bytesLeft: %d\n", bytesToCopy,
                bytesLeft);
		memcpy(skb->data, addr + 2, bytesToCopy);
		bytesLeft -= bytesToCopy;

		if (bytesLeft != 0) {
			addr = priv->rx_buffer + 0x20;
			EPRINTK(DEBUG_READ, "copying from %p, left: %d (%x)\n", addr,
                    bytesLeft, ((u8*)addr)[0]);
			memcpy(skb->data+bytesToCopy, addr, bytesLeft);
			read_offset = CLINE_PACKETS(bytesLeft);
		} else {
			read_offset += CLINE_PACKETS(skb->len+2) - 1;
		}
	}

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;
	priv->net_device->last_rx = jiffies;

	skb->dev = priv->net_device;
	skb->protocol = eth_type_trans(skb, priv->net_device);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	/* now process the buffer */
	netif_receive_skb(skb);

	packets++;

rxDone:
	/* set new read pointer */
	EPRINTK(DEBUG_READ, "Update rx read offset: %d\n", read_offset);
	writel(read_offset, RA(priv->base + EMAC_RX_BUFFER_READ_OFFSET, priv->pid));
	priv->rx_read_offset = read_offset;

	if (read_offset != write_offset) {
		if (packets < max_num) {
			goto again;
		}
	}

	return packets;
}

/**
 * \brief Polling interface for emac
 * \param napi napi structure
 * \param budget budget
 * \return error code (0=everything done, 1=still work todo)
 */
int emac_poll(struct napi_struct *napi, int budget) {
	struct emac_priv *priv = container_of(napi, struct emac_priv, napi);
	unsigned int write_offset = 0;
	int read = 0;
	int work_done = 0;

	/* try to read packets */
	while (work_done < budget) {
		/* check for updated write offset */
		CL1FLUSH;
		write_offset = readl(priv->rx_buffer) & 0xFFFF;

		if ((write_offset != 0) && (priv->rx_read_offset != write_offset)) {
			/* Retrieve packets */
			read = emac_rx(priv, write_offset, budget - work_done);
			if (read > 0) {
				work_done += read;
			}
		} else {
			/* Tell the system we are done polling */
			napi_complete(napi);

			/* Clear the interrupt */
			emac_clear_interrupt(priv);
			break;
		}
	}

	return work_done;
}

/**
 * \brief Read and display ethernet statistics
 */
void show_statistic(void) {
	int i;

	printk("Ethernet statistic: \n");
	printk("----------------------------\n");
	for (i = 0; i < 46; i++) {
		printk("0x%4x\t-\t%4d\n", STAT0_TRBYTE + i * 8,
               readl(RA(STAT0_TRBYTE + i * 8, 0)));
	}
	printk("----------------------------\n");
}

/**
 * \brief Transfer packet to network deivce
 * \param skb buffer we want to transfer
 * \param dev network device
 * \return error code
 */
int emac_tx(struct sk_buff *skb, struct net_device *dev) {
	struct emac_priv *priv = netdev_priv(dev);
	void *addr = NULL;
	u16 read_offset = 0;
	int rest = 0;
	int packets = 0;
	int sum = 0;

	EPRINTK(DEBUG_WRITE, "packet len: %d\n", skb->len);

	/* check for over/underflow */
	if (skb->len < sizeof(struct iphdr) || skb->len > 1536) {
		printk(KERN_NOTICE "emac_tx(): illegal packet length %d => drop\n",
               skb->len);
		priv->stats.tx_errors++;
		priv->stats.tx_dropped++;

		return 0;
	}

	priv->tx_write_offset++;
	/* check if we need to wrap */
	if (priv->tx_write_offset > priv->tx_buffer_max) {
		priv->tx_write_offset = 1;
	}

	packets = CLINE_PACKETS(skb->len + 2);

	read_offset = readl(RA(priv->base + EMAC_TX_BUFFER_READ_OFFSET, priv->pid));
#ifdef OVERFLOW_CHECK
again:

	if (read_offset < priv->tx_write_offset) {
		sum = priv->tx_buffer_max - priv->tx_write_offset + read_offset - 1;
	} else if (read_offset > priv->tx_write_offset) {
		sum = read_offset - priv->tx_write_offset - 1;
	}

	if (sum < packets) {
		EPRINTK(DEBUG_WRITE, "2. Warning: not enough space available, "
                "retrying...\n");
		goto again;
	}
#endif

	addr = priv->tx_buffer + priv->tx_write_offset * 32;

	/* Set frame length */
	((u8*)addr)[0] = skb->len % 256;
	((u8*)addr)[1] = skb->len / 256;

	if (priv->tx_write_offset + packets - 1 <= priv->tx_buffer_max) {
		/* enough space, just copy */
		memcpy(addr + 2, skb->data, skb->len);

		/* increment write ptr */
		priv->tx_write_offset += packets - 1;
	} else {
		/* wrap in offsets. first copy to the end, second at the starting
         * point
         */
		int bytes_left = skb->len;
		int bytes_to_copy = (priv->tx_buffer_max - priv->tx_write_offset + 1) *
                             32 - 2;

		if (bytes_left < bytes_to_copy) {
			bytes_to_copy = bytes_left;
		}

		EPRINTK(DEBUG_WRITE, "special case: copy last %d bytes\n",
                bytes_to_copy);

		memcpy(addr + 2, skb->data, bytes_to_copy);
		bytes_left -= bytes_to_copy;

		if (bytes_left != 0) {
			priv->tx_write_offset = 1;
			addr = priv->tx_buffer + 32;
			EPRINTK(DEBUG_WRITE, "special case: copy remaining %d bytes\n",
                    bytes_left);
			memcpy(addr, skb->data + bytes_to_copy, bytes_left);

			rest = bytes_left % 32;
			if (rest != 0) {
				rest = 32 - rest;
			}
			EPRINTK(DEBUG_WRITE, "Rest is %d\n", rest);
			priv->tx_write_offset += CLINE_PACKETS(bytes_left + rest) - 1;
		}
	}

	writel(2, priv->tx_buffer);

	/* set new write offset */
	EPRINTK(DEBUG_WRITE, "Update tx write offset: %d (read offset %d)\n",
            priv->tx_write_offset, read_offset);

	writel(priv->tx_write_offset, RA(priv->base + EMAC_TX_BUFFER_WRITE_OFFSET,
                   priv->pid));

	dev->trans_start = jiffies;
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return 0;
}

/**
 * \brief Handler transmit timeout
 * \param dev network device
 */
void emac_tx_timeout(struct net_device *dev) {
	netif_wake_queue(dev);
}

/**
 * \brief Return network statistics
 * \param dev network device
 * \return network statistic
 */
struct net_device_stats *emac_stats(struct net_device *dev) {
	struct emac_priv *priv = netdev_priv(dev);

	return &(priv->stats);
}

/**
 * \brief Change mtu of network device
 * \param dev network device
 * \param new_mtu new mtu size
 * \return error code
 */
int emac_change_mtu(struct net_device *dev, int new_mtu) {
	if ((new_mtu < sizeof(struct iphdr)) || (new_mtu > BUFFER_SIZE - 1)) {
		return -EINVAL;
	}

	dev->mtu = new_mtu;

	return 0;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_open,
	.ndo_stop = emac_stop,
	.ndo_start_xmit = emac_tx,
	.ndo_tx_timeout = emac_tx_timeout,
	.ndo_change_mtu = emac_change_mtu,
	.ndo_get_stats = emac_stats,
};

/**
 * \brief Initialize emac network device
 * \param dev network device
 */
void emac_init(struct net_device *dev) {
	/* set standard infos */
	ether_setup(dev);

	/* Network driver specific functions */
	dev->netdev_ops = &emac_netdev_ops;
	dev->watchdog_timeo = 5;

	/* Note LVT1's vector is set to 3, LVT0 = 4 */
	dev->irq = EMAC_IRQ_NR;

	dev->hard_header_len = 2 + ETH_HLEN;
}

/**
 * \brief Main network driver entry
 * \return error code
 */
static int __init emac_module_init(void) {
	struct emac_priv *priv = NULL;
	int errorCode;
	int macPorts = 0;
	int tmp;

	printk(KERN_DEBUG "eMAC driver %s\n", MODVERSTRING);

	/* Get SCC processor id */
	core0 = sccsys_get_pid();

	tmp = readl(RA(0x822C, 0));
	if (tmp & EMAC0) {
		emac_writel(0x00, RA(EMAC0 + EMAC_RX_NETWORK_PORT_ENABLE, core0));
		emac_writel(0x00, RA(EMAC0 + EMAC_TX_NETWORK_PORT_ENABLE, core0));
	}
	if (tmp & EMAC1) {
		emac_writel(0x00, RA(EMAC1 + EMAC_RX_NETWORK_PORT_ENABLE, core0));
		emac_writel(0x00, RA(EMAC1 + EMAC_TX_NETWORK_PORT_ENABLE, core0));
	}
	if (tmp & EMAC2) {
		emac_writel(0x00, RA(EMAC2 + EMAC_RX_NETWORK_PORT_ENABLE, core0));
		emac_writel(0x00, RA(EMAC2 + EMAC_TX_NETWORK_PORT_ENABLE, core0));
	}
	if (tmp & EMAC3) {
		emac_writel(0x00, RA(EMAC3 + EMAC_RX_NETWORK_PORT_ENABLE, core0));
		emac_writel(0x00, RA(EMAC3 + EMAC_TX_NETWORK_PORT_ENABLE, core0));
	}

	if (!override) {
		tmp >>= 16;
	}

	macPorts = ((tmp >> 9) & 0xFF);

	printk(KERN_DEBUG "eMAC0: %s eMAC1: %s eMAC2: %s eMAC3: %s\n",
		(macPorts & EMAC0) != 0 ? "present" : "-",
		(macPorts & EMAC1) != 0 ? "present" : "-",
		(macPorts & EMAC2) != 0 ? "present" : "-",
		(macPorts & EMAC3) != 0 ? "present" : "-");


	/* Create emac0 if requested */
	if ((macPorts & EMAC0) && (ethernet_port & EMAC0)) {
		/* initialize xilinx ip */
		init_xilinx_port(0, XILINX_EMAC0_BASE);

		emac0_dev = alloc_netdev(sizeof(struct emac_priv), "emac0", emac_init);
		if (emac0_dev==NULL) {
			printk(KERN_ERR "alloc_netdev() failed\n");
			return -ENOMEM;
		}
		priv = netdev_priv(emac0_dev);
		memset(priv, 0, sizeof(struct emac_priv));
		priv->device = EMAC0;
		priv->base = EMAC0_BASE;
		priv->net_device = emac0_dev;
		netif_napi_add(emac0_dev, &priv->napi, emac_poll, EMAC_NAPI_WEIGHT);

		errorCode = register_netdev(emac0_dev);
		if (errorCode) {
			printk(KERN_ERR "error %i registering device \"%s\"\n", errorCode,
                   emac0_dev->name);
			return -ENODEV;
		}
	}

	/* Create emac1 if requested */
	if ((macPorts & EMAC1) && (ethernet_port & EMAC1)) {
		/* initialize xilinx ip */
		init_xilinx_port(1, XILINX_EMAC1_BASE);

		emac1_dev = alloc_netdev(sizeof(struct emac_priv), "emac1", emac_init);
		if (emac1_dev==NULL) {
			printk(KERN_ERR "alloc_netdev() failed\n");
			return -ENOMEM;
		}
		priv = netdev_priv(emac1_dev);
		memset(priv, 0, sizeof(struct emac_priv));
		priv->device = EMAC1;
		priv->base = EMAC1_BASE;
		priv->net_device = emac1_dev;
		netif_napi_add(emac1_dev, &priv->napi, emac_poll, EMAC_NAPI_WEIGHT);

		errorCode = register_netdev(emac1_dev);
		if (errorCode) {
			printk(KERN_ERR "error %i registering device \"%s\"\n", errorCode,
                    emac1_dev->name);
			return -ENODEV;
		}
	}

	/* Create emac2 if requested */
	if ((macPorts & EMAC2) && (ethernet_port & EMAC2)) {
		/* initialize xilinx ip */
		init_xilinx_port(2, XILINX_EMAC2_BASE);

		emac2_dev = alloc_netdev(sizeof(struct emac_priv), "emac2", emac_init);
		if (emac2_dev==NULL) {
			printk(KERN_ERR "alloc_netdev() failed\n");
			return -ENOMEM;
		}
		priv = netdev_priv(emac2_dev);
		memset(priv, 0, sizeof(struct emac_priv));
		priv->device = EMAC2;
		priv->base = EMAC2_BASE;
		priv->net_device = emac2_dev;
		netif_napi_add(emac2_dev, &priv->napi, emac_poll, EMAC_NAPI_WEIGHT);

		errorCode = register_netdev(emac2_dev);
		if (errorCode) {
			printk(KERN_ERR "error %i registering device \"%s\"\n", errorCode,
                    emac2_dev->name);
			return -ENODEV;
		}
	}

	/* Create emac3 if requested */
	if ((macPorts & EMAC3) && (ethernet_port & EMAC3)) {
		/* initialize xilinx ip */
		init_xilinx_port(3, XILINX_EMAC3_BASE);

		emac3_dev = alloc_netdev(sizeof(struct emac_priv), "emac3", emac_init);
		if (emac3_dev==NULL) {
			printk(KERN_ERR "alloc_netdev() failed\n");
			return -ENOMEM;
		}
		priv = netdev_priv(emac3_dev);
		memset(priv, 0, sizeof(struct emac_priv));
		priv->device = EMAC3;
		priv->base = EMAC3_BASE;
		priv->net_device = emac3_dev;
		netif_napi_add(emac3_dev, &priv->napi, emac_poll, EMAC_NAPI_WEIGHT);

		errorCode = register_netdev(emac3_dev);
		if (errorCode) {
			printk(KERN_ERR "error %i registering device \"%s\"\n", errorCode,
                    emac3_dev->name);
			return -ENODEV;
		}
	}


#ifdef NOTIFIER
	register_netdevice_notifier(&emac_dev_notifier);
#endif

	return 0;
}

/**
 * \brief Main network driver removal function
 */
static void __exit emac_module_exit(void) {
#ifdef NOTIFIER
	/* unregister netdevice notifier: event handler */
	EPRINTK(DEBUG_INFO, "unregister netdevice notifier\n");
	unregister_netdevice_notifier(&emac_dev_notifier);
#endif

	/* if network device has been created, remove it and free structure */
	if (emac0_dev != NULL) {
		unregister_netdev(emac0_dev);
		free_netdev(emac0_dev);
	}

	if (emac1_dev != NULL) {
		unregister_netdev(emac1_dev);
		free_netdev(emac1_dev);
	}

	if (emac2_dev != NULL) {
		unregister_netdev(emac2_dev);
		free_netdev(emac2_dev);
	}

	if (emac3_dev != NULL) {
		unregister_netdev(emac3_dev);
		free_netdev(emac3_dev);
	}
}

module_init(emac_module_init);
module_exit(emac_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jan-Michael Brummer");
MODULE_VERSION(MODVERSTRING);
MODULE_DESCRIPTION("Intel(R) eMAC Network Driver");
