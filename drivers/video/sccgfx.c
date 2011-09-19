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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/fb.h>

/* Change Log
 * 0.2.0	04/19/2011
 *   o add the possibility to set the resolution via cmdline
 * 0.1.0	08/19/2010
 *   o first release
 */

#define MODVERSTRING	"0.1.0"

/* Default screen resolution */
#define WIDTH	800
#define HEIGHT	600
#define BPP		16

/* Base address (310MB) */
#define MEGABYTE			0x100000
#define GFX_BASE_ADDRESS	(630 * MEGABYTE)
/* Length (10MB) */
#define GFX_LENGTH			(10 * MEGABYTE)
#define GFX_HEADER			4096

/** framebuffer var screen information */
static struct fb_var_screeninfo sccgfxfb_var = {
	.xres			= WIDTH,
	.yres			= HEIGHT,
	.xres_virtual	= WIDTH,
	.yres_virtual	= HEIGHT,
	.bits_per_pixel	= BPP,
	.activate		= FB_ACTIVATE_NOW,
	.height			= HEIGHT,
	.width			= WIDTH,
	.sync			= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode			= FB_VMODE_NONINTERLACED,
};

/** framebuffer fix screen information */
static struct fb_fix_screeninfo sccgfxfb_fix = {
	.id				= "SCC GFX",
	.smem_len		= GFX_LENGTH,
	.type			= FB_TYPE_PACKED_PIXELS,
	.visual			= FB_VISUAL_PSEUDOCOLOR,
	.line_length	= WIDTH*BPP/8,
	.accel			= FB_ACCEL_NONE,
};

/* Our own framebuffer information structure */
static struct fb_info fb_info;

static long grb_offset = 0xF9000000;
static unsigned long base_address = GFX_BASE_ADDRESS;
static unsigned long memory_size = GFX_LENGTH;
static unsigned long res_x = 0;
static unsigned long res_y = 0;
static unsigned long res_bpp = 0;

/**
 * \brief Set <pseudo> color register
 * \param regno register number
 * \param red red value
 * \param green green value
 * \param blue blue value
 * \param transp alpha value
 * \param fbinfo framebuffer information pointer
 * \return 0 = handled, 1 = not handled
 */
static int sccgfx_setcolreg(unsigned regno, unsigned red, unsigned green,
                            unsigned blue, unsigned transp,
                            struct fb_info *fbinfo) {
	/* if the number is above our color map length, drop it */
	if (regno >= fbinfo->cmap.len) {
		return 1;
	}

	if (fbinfo->var.bits_per_pixel != 8 && regno < 16) {
		switch (fbinfo->var.bits_per_pixel) {
			case 16:
				if (fbinfo->var.red.offset != 10) {
					/* 0:5:6:5 */
					((u32*)(fbinfo->pseudo_palette))[regno] = ((red & 0xf800)) |
                           ((green & 0xfc00) >> 5) | ((blue & 0xf800) >> 11);
				}
				break;
			case 24:
			case 32:
				red >>= 8;
				green >>= 8;
				blue >>= 8;
				((u32*)(fbinfo->pseudo_palette))[regno] = 
                        (red << fbinfo->var.red.offset) |
                        (green << fbinfo->var.green.offset) |
                        (blue << fbinfo->var.blue.offset);
				break;
			default:
				break;
		}
	}

	return 0;
}

/** framebuffer operations */
static struct fb_ops sccgfxfb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg	= sccgfx_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int __init sccgfx_setup(char *options) {
	char *this_opt;

	if (!options || !*options) {
		return 0;
	}

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt) {
			continue;
		}

		if (!strncmp(this_opt, "base:", 5)) {
			base_address = simple_strtoul(this_opt + 5, NULL, 0) * MEGABYTE;
		} else if (!strncmp(this_opt, "size:", 5)) {
			memory_size = simple_strtoul(this_opt + 5, NULL, 0) * MEGABYTE;
		} else if (!strncmp(this_opt, "x:", 2)) {
			res_x = simple_strtoul(this_opt + 2, NULL, 0);
		} else if (!strncmp(this_opt, "y:", 2)) {
			res_y = simple_strtoul(this_opt + 2, NULL, 0);
		} else if (!strncmp(this_opt, "bpp:", 4)) {
			res_bpp = simple_strtoul(this_opt + 4, NULL, 0);
		} else {
			printk(KERN_INFO "Unknown value!\n");
		}
	}

	return 0;
}

/**
 * \brief Initialize scc graphic framebuffer
 * \return error code, 0 = success, otherwise error
 */
int __init sccgfxfb_init(void) {
	unsigned char *shm;
	unsigned char *fb_off;
	int x = WIDTH;
	int y = HEIGHT;
	int bpp = BPP;
	char *option = NULL;

	printk(KERN_INFO "sccgfxfb: Initializing framebuffer\n");

	fb_get_options("rckgfx", &option);
	sccgfx_setup(option);

	if (res_x != 0 && res_y != 0 && res_bpp != 0) {
		printk(KERN_INFO "sccgfxfb: Valid information from cmdline\n");
		x = res_x;
		y = res_y;
		bpp = res_bpp;
	} else {
		printk(KERN_INFO "sccgfxfb: Trying FPGA values\n");
		/* Read requested display values */
		shm = ioremap_nocache(grb_offset, 0x10000);
		if (shm != NULL) {
			res_x = readl(shm + 0x8238) & 0xFFFFFFFF;
			res_y = readl(shm + 0x823C) & 0xFFFFFFFF;
			res_bpp = readl(shm + 0x8240) & 0xFFFFFFFF;
			if (res_x != 0 && res_y != 0 && res_bpp != 0) {
				printk(KERN_INFO "sccgfxfb: Valid information from FPGA\n");
				x = res_x;
				y = res_y;
				bpp = res_bpp;
			}
			iounmap(shm);
		} else {
			printk(KERN_DEBUG "sccgfxfb: could not map grb and therefore not read display "
                   "values, setting default values (%dx%dx%d)!\n", x, y, bpp);
		}
	}

	/* Update var information */
	sccgfxfb_var.xres = x;
	sccgfxfb_var.yres = y;
	sccgfxfb_var.xres_virtual = x;
	sccgfxfb_var.yres_virtual = y;
	sccgfxfb_var.bits_per_pixel = bpp;
	sccgfxfb_var.width = x;
	sccgfxfb_var.height = y;

	/* Update fix information */
	sccgfxfb_fix.line_length = x * bpp / 8;

	printk(KERN_INFO "sccgfxfb: Requested display values (%dx%dx%d)\n",
	       x, y, bpp);

	/* Framebuffer display memory base address */
	sccgfxfb_fix.smem_start = base_address;
	/* Length of buffer */
	sccgfxfb_fix.smem_len = memory_size;
	/* Map in private video memory */
	fb_off = ioremap_nocache(sccgfxfb_fix.smem_start, sccgfxfb_fix.smem_len);
	if (fb_off == NULL) {
		printk(KERN_INFO "sccgfxfb: cannot remap!!\n");
		return -1;
	}

	/* Clear memory */
	memset(fb_off, 0x00, sccgfxfb_fix.smem_len);

	/* Write display informations */
	writel(sccgfxfb_var.xres, (u8*)fb_off + 0x00);
	writel(sccgfxfb_var.yres, (u8*)fb_off + 0x04);
	writel(sccgfxfb_var.bits_per_pixel, (u8*)fb_off + 0x08);

	/* Set start address into shared memory (sccDisplay will use this one as
     * base address)
     */
	shm = ioremap_nocache(0x80000940, 4);
	if (shm != NULL) {
		writel(sccgfxfb_fix.smem_start, shm);
		iounmap(shm);
	} else {
		printk(KERN_DEBUG "sccgfxfb: could not map crb and therefore not write base "
               "address. sccDisplay will not work!\n");
	}

	/* Skip framebuffer header */
	fb_off += GFX_HEADER;
	sccgfxfb_fix.smem_start += GFX_HEADER;
	sccgfxfb_fix.smem_len -= GFX_HEADER;

	/* set color offsets/lengths */
	if (sccgfxfb_var.bits_per_pixel != 8) {
		sccgfxfb_fix.visual = FB_VISUAL_TRUECOLOR;
		switch (sccgfxfb_var.bits_per_pixel) {
			case 16:
				sccgfxfb_var.red.offset = 11;
				sccgfxfb_var.red.length = 5;
				sccgfxfb_var.green.offset = 5;
				sccgfxfb_var.green.length = 6;
				sccgfxfb_var.blue.offset = 0;
				sccgfxfb_var.blue.length = 5;
				break;
			case 32:
				sccgfxfb_var.red.offset = 24;
				sccgfxfb_var.red.length = 8;
				sccgfxfb_var.green.offset = 16;
				sccgfxfb_var.green.length = 8;
				sccgfxfb_var.blue.offset = 8;
				sccgfxfb_var.blue.length = 8;
				sccgfxfb_var.transp.offset = 0;
				sccgfxfb_var.transp.length = 8;
				break;
		}
	}
	sccgfxfb_var.yres_virtual = y;

	fb_info.fbops = &sccgfxfb_ops;
	fb_info.screen_base = fb_off;
	fb_info.var = sccgfxfb_var;
	fb_info.fix = sccgfxfb_fix;
	fb_info.flags = FBINFO_FLAG_DEFAULT;
	fb_info.pseudo_palette = kzalloc(sizeof(u32) * 256, GFP_KERNEL);

	/* Allocate color map */
	fb_alloc_cmap(&fb_info.cmap, 256, 0);

	if (register_framebuffer(&fb_info) < 0) {
		printk(KERN_WARNING "sccgfxfb: failed while register_framebuffer\n");
		return 1;
	}

	printk(KERN_INFO "sccgfxfb: register framebuffer (%dx%dx%d)\n",
           fb_info.var.xres, fb_info.var.yres, fb_info.var.bits_per_pixel);

	return 0;
}

/**
 * \brief Remove framebuffer
 */
static void __exit sccgfxfb_exit(void) {
	unregister_framebuffer(&fb_info);
}

#ifdef MODULE
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jan-Michael Brummer");
MODULE_VERSION(MODVERSTRING);
MODULE_DESCRIPTION("Intel(R) SCC graphic driver");
#endif
module_init(sccgfxfb_init);
module_exit(sccgfxfb_exit);
