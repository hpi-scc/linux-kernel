#ifndef __ASM_LAPIC_H
#define __ASM_LAPIC_H

#include <asm/apic.h>

static __inline void set_lapic_mask(unsigned long reg, unsigned int irq)
{
  unsigned long v;
  v = apic_read(reg);
  apic_write(reg, v | APIC_LVT_MASKED);
}

// unset_lapic_mask enables interrupt
static __inline void unset_lapic_mask(unsigned long reg, unsigned int irq)
{
  unsigned long v;
  v = apic_read(reg);
  apic_write(reg, v & ~APIC_LVT_MASKED);
}

#endif /* __ASM_LAPIC_H */
