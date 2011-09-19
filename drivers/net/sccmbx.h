
struct sccmbx_hdr {
	unsigned char len[2];
	unsigned char dest;
	unsigned char src;
	unsigned char subsys;
} __attribute__((packed));

struct sccmbx_rx;

#define SCCMBX_SUBSYS_NET	0
#define SCCMBX_SUBSYS_SHM	1

#define SCCMBX_HLEN		16

struct sccmbx_hdrbuf {
	unsigned char __data[SCCMBX_HLEN];
};

struct sccmbx_fragment {
	void* p;
	int len;
};

#define SCCMBX_BUFX_FRAGMENTS	2
struct sccmbx_bufx {
	int len;
	struct sk_buff* skb;
	struct sccmbx_fragment f[SCCMBX_BUFX_FRAGMENTS];
};

/* Type of receive callback routines */
typedef void (*sccmbx_rx_proc)(struct sccmbx_rx* rx, struct sccmbx_hdr* header, int len);


/* Set receive callback for messages to a specific subsystem ID */
extern int sccmbx_set_rx_proc(u8 subsys, sccmbx_rx_proc rx_proc);

/* Clear receive descriptor */
extern void sccmbx_clear_rx(struct sccmbx_rx* rx);

/* Get pid of source node of message */
extern int sccmbx_get_rx_source(struct sccmbx_rx* rx);

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
extern int sccmbx_retrieve_packet_body(struct sccmbx_rx* rx, struct sccmbx_bufx* bufx, int need_skb);

/* Mark the skb referred to by the sccmbx_bufx structure as consumed. After this
 * call, sccmbx_retrieve_packet_body_cleanup will not free the skb.
 */
extern void sccmbx_retrieve_skb_consumed(struct sccmbx_bufx* bufx);

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
extern void sccmbx_retrieve_packet_body_cleanup(struct sccmbx_rx* rx, struct sccmbx_bufx* bufx);

/* Copy data from struct sccmbx_bufx to regular buffer */
static inline int sccmbx_copy_from_bufx(void* target, struct sccmbx_bufx* bufx, int offset, int len)
{
  int clen = 0, i;

  if (offset < 0 || len < 0 || offset > bufx->len) {
    VM_BUG_ON(1);
    return -EINVAL;
  }

  for (i = 0; (i < SCCMBX_BUFX_FRAGMENTS) && (len > 0); i++) {
    /* Length of current fragment. If the requested subrange is not contained
     * in it, simply adjust the offset value for the next fragment and continue.
     */
    int flen = bufx->f[i].len;
    if (offset >= flen) {
      offset -= flen;
      continue;
    }

    /* Adjust flen to reflect the maximum number of bytes that can be copied
     * from the current fragment. */
    flen -= offset;

    if (flen > len) {
      flen = len;
    }

    /* Copy data, then adjust variables for the next fragment */
    memcpy(target, bufx->f[i].p + offset, flen);
    offset = 0;
    target += flen;
    clen += flen;
    len -= flen;
  }

  VM_BUG_ON(clen < len);
  return clen;
}

/* Allocate a packet */
extern struct sk_buff* sccmbx_allocate_packet(u8 subsys, u8 dest, int len, void* *body);

/* Initialize struct bufx for transmitting a packet */
extern void sccmbx_init_transmit_bufx(struct sccmbx_bufx* bufx, struct sccmbx_hdrbuf* hdr, u8 subsys, u8 dest, void* data, int len);

/* Transmit a raw packet described by a struct bufx */
extern int sccmbx_transmit_bufx(struct sccmbx_bufx* bufx);

