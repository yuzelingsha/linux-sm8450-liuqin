// SPDX-License-Identifier: GPL-2.0
/*
 * Early console on a bootloader-provided framebuffer described in the DT.
 *
 * Retail phones and tablets usually bring out no debug UART, so on those the
 * panel is the only way to see anything a kernel prints. fbcon can do that, but
 * it only takes over at device_initcall, long after cmd-db, rpmh-rsc, clk-rpmh,
 * rpmhpd (core_initcall), smem (arch_initcall) and gcc (subsys_initcall) have
 * already run -- which on a fresh port is precisely where a boot tends to die,
 * and it dies with a blank screen and nothing to go on.
 *
 * This earlycon runs from parse_early_param(), before any of that, and prints
 * onto the framebuffer the bootloader left scanning out. Select it with
 * "earlycon=simplefb"; the geometry comes from the /chosen simple-framebuffer
 * node, so nothing has to be hardcoded.
 *
 * Two command line caveats, both of which silently defeat it:
 *   - a bare "earlycon" never reaches here when CONFIG_ACPI_SPCR_TABLE is
 *     enabled, because that takes the SPCR branch and returns before the DT is
 *     ever consulted. The name has to be spelled out.
 *   - "keep_bootcon" is required to keep this console past console_init(),
 *     which runs before every initcall. Without it the VT console registers
 *     there as CON_CONSDEV and printk unregisters every CON_BOOT console
 *     (kernel/printk/printk.c), so the whole window this driver exists to cover
 *     is dark again.
 *
 * Adapted from drivers/firmware/efi/earlycon.c, which does the same job from
 * screen_info. It scrolls the whole framebuffer up one text line at a time;
 * this one instead wipes and restarts from the top once it reaches the bottom.
 * Scrolling reads back write-combining memory, which is slow enough on a
 * 1800x2880x32 panel (20 MiB a line) to look like the hang we came to debug.
 * Wrapping costs one clear per screenful, and the screen still ends up holding
 * the most recent lines, with the last one written being where the kernel got
 * stuck.
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/overflow.h>
#include <linux/pgtable.h>
#include <linux/serial_core.h>
#include <linux/string.h>
#include <asm/early_ioremap.h>

/* Opaque in both a8r8g8b8 and x8r8g8b8: the alpha byte is set, not left zero. */
#define SIMPLEFB_WHITE	0xffffffffU
#define SIMPLEFB_BLACK	0xff000000U

static const struct font_desc *font;
static u64 fb_base;
static u64 fb_size;
static u32 fb_width, fb_height, fb_stride;
static u32 cur_x, cur_y;
static void *fb_mapped;
static bool fb_dead;
static struct console *simplefb_con;

static __ref void *simplefb_map(unsigned long start, unsigned long len)
{
	if (fb_mapped)
		return fb_mapped + start;

	/*
	 * early_memremap() lives in .init. Once the permanent mapping has been
	 * attempted and failed there is no way back, and calling into freed
	 * .init text from a console write would take the machine down far more
	 * confusingly than the fault we are here to diagnose.
	 */
	if (fb_dead)
		return NULL;

	return early_memremap_prot(fb_base + start, len,
				   pgprot_val(pgprot_writecombine(PAGE_KERNEL)));
}

static __ref void simplefb_unmap(void *addr, unsigned long len)
{
	if (fb_mapped)
		return;

	early_memunmap(addr, len);
}

/*
 * early_memremap() is only good for a handful of small mappings and cannot be
 * used at all once the console outlives the early boot ("keep_bootcon"), so
 * switch to a permanent mapping as soon as memremap() is usable.
 */
static int __init simplefb_earlycon_remap(void)
{
	if (!simplefb_con || !console_is_registered(simplefb_con))
		return 0;

	fb_mapped = memremap(fb_base, fb_size, MEMREMAP_WC);
	if (!fb_mapped) {
		fb_dead = true;
		return -ENOMEM;
	}

	return 0;
}
early_initcall(simplefb_earlycon_remap);

static int __init simplefb_earlycon_unmap(void)
{
	if (fb_mapped && !console_is_registered(simplefb_con)) {
		memunmap(fb_mapped);
		fb_mapped = NULL;
		fb_dead = true;
	}
	return 0;
}
late_initcall(simplefb_earlycon_unmap);

static bool simplefb_clear_lines(u32 y, u32 lines)
{
	u32 i;

	for (i = 0; i < lines && y + i < fb_height; i++) {
		void *dst = simplefb_map((y + i) * (unsigned long)fb_stride,
					 fb_stride);

		if (!dst)
			return false;

		memset32(dst, SIMPLEFB_BLACK, fb_stride / 4);
		simplefb_unmap(dst, fb_stride);
	}

	return true;
}

static void simplefb_write_char(u32 *dst, unsigned char c, unsigned int h)
{
	const u8 *src;
	int m, n, bytes;
	u8 x;

	bytes = BITS_TO_BYTES(font->width);
	src = font->data + c * font->height * bytes + h * bytes;

	for (m = 0; m < font->width; m++) {
		n = m % 8;
		x = *(src + m / 8);
		if ((x >> (7 - n)) & 1)
			*dst = SIMPLEFB_WHITE;
		else
			*dst = SIMPLEFB_BLACK;
		dst++;
	}
}

/* Start over at the top rather than scrolling; see the file comment. */
static bool simplefb_wrap(void)
{
	cur_x = 0;
	cur_y = 0;
	return simplefb_clear_lines(0, fb_height);
}

static void simplefb_earlycon_write(struct console *con, const char *str,
				    unsigned int num)
{
	const char *s, *nl;
	void *dst;

	while (num) {
		unsigned int linemax = (fb_width - cur_x) / font->width;
		unsigned int h, count;

		/*
		 * memchr, not strnchrnul: the latter also stops on a NUL, and
		 * an embedded NUL would then yield count == 0 with nothing else
		 * changing, spinning this loop forever inside a console write.
		 */
		nl = memchr(str, '\n', num);
		count = nl ? nl - str : num;
		if (count > linemax)
			count = linemax;

		for (h = 0; h < font->height; h++) {
			unsigned int n, x;

			dst = simplefb_map((cur_y + h) * (unsigned long)fb_stride,
					   fb_stride);
			if (!dst)
				return;

			s = str;
			n = count;
			x = cur_x;

			while (n-- > 0) {
				simplefb_write_char(dst + x * 4, *s, h);
				x += font->width;
				s++;
			}

			simplefb_unmap(dst, fb_stride);
		}

		num -= count;
		cur_x += count * font->width;
		str += count;

		if (num > 0 && *str == '\n') {
			cur_x = 0;
			cur_y += font->height;
			str++;
			num--;
		} else if (cur_x + font->width > fb_width) {
			cur_x = 0;
			cur_y += font->height;
		}

		if (cur_y + font->height > fb_height) {
			if (!simplefb_wrap())
				return;
		}
	}
}

/*
 * The framebuffer is described under /chosen, which is where a bootloader that
 * hands over a live display puts it. Read it straight out of the flat tree:
 * this runs before unflatten_device_tree(), and it keeps the node's address in
 * one place rather than duplicating it on the command line.
 */
static int __init simplefb_earlycon_scan_dt(void)
{
	int chosen, offset, node = -1;
	const __be32 *prop, *reg;
	u32 addr_cells, size_cells;
	const char *format;
	u64 addr, span = 0;
	int len, i;

	chosen = of_get_flat_dt_subnode_by_name(of_get_flat_dt_root(), "chosen");
	if (chosen < 0)
		return -ENODEV;

	for (offset = fdt_first_subnode(initial_boot_params, chosen);
	     offset >= 0;
	     offset = fdt_next_subnode(initial_boot_params, offset)) {
		if (of_flat_dt_is_compatible(offset, "simple-framebuffer")) {
			node = offset;
			break;
		}
	}
	if (node < 0)
		return -ENODEV;

	addr = of_flat_dt_translate_address(node);
	if (addr == OF_BAD_ADDR)
		return -ENXIO;
	fb_base = addr;

	prop = of_get_flat_dt_prop(node, "width", &len);
	if (!prop || len != sizeof(*prop))
		return -EINVAL;
	fb_width = be32_to_cpup(prop);

	prop = of_get_flat_dt_prop(node, "height", &len);
	if (!prop || len != sizeof(*prop))
		return -EINVAL;
	fb_height = be32_to_cpup(prop);

	prop = of_get_flat_dt_prop(node, "stride", &len);
	if (!prop || len != sizeof(*prop))
		return -EINVAL;
	fb_stride = be32_to_cpup(prop);

	/* write_char() stores one 32-bit pixel at a time and nothing else. */
	format = of_get_flat_dt_prop(node, "format", &len);
	if (!format || (strcmp(format, "a8r8g8b8") && strcmp(format, "x8r8g8b8")))
		return -ENODEV;

	if (!fb_width || !fb_height || fb_stride < fb_width * 4 || fb_stride % 4)
		return -EINVAL;

	if (check_mul_overflow((u64)fb_stride, (u64)fb_height, &fb_size))
		return -EINVAL;

	/*
	 * Blanking the screen walks the whole computed extent, so refuse to run
	 * if the geometry claims more than the reg actually reserves: on this
	 * hardware everything past the framebuffer is other firmware's memory.
	 */
	prop = of_get_flat_dt_prop(chosen, "#address-cells", &len);
	if (!prop || len != sizeof(*prop))
		return -EINVAL;
	addr_cells = be32_to_cpup(prop);
	prop = of_get_flat_dt_prop(chosen, "#size-cells", &len);
	if (!prop || len != sizeof(*prop))
		return -EINVAL;
	size_cells = be32_to_cpup(prop);
	if (!addr_cells || addr_cells > 2 || !size_cells || size_cells > 2)
		return -EINVAL;

	reg = of_get_flat_dt_prop(node, "reg", &len);
	if (!reg || len < (int)((addr_cells + size_cells) * sizeof(*reg)))
		return -EINVAL;
	for (i = 0; i < size_cells; i++)
		span = (span << 32) | be32_to_cpu(reg[addr_cells + i]);
	if (span < fb_size)
		return -EINVAL;

	return 0;
}

static int __init simplefb_earlycon_setup(struct earlycon_device *device,
					  const char *opt)
{
	int ret;

	ret = simplefb_earlycon_scan_dt();
	if (ret)
		return ret;

	/*
	 * The only way to read this screen is a camera pointed at a 300 ppi
	 * panel, where an 8x16 glyph is well under a millimetre tall. Ask for
	 * the largest font by name and fall back rather than trusting
	 * get_default_font()'s scoring to pick it.
	 */
	font = find_font("TER16x32");
	if (!font)
		font = get_default_font(fb_width, fb_height, NULL, NULL);
	if (!font)
		return -ENODEV;

	/* get_default_font() scores, it does not reject a font that cannot fit. */
	if (fb_width < font->width || fb_height < font->height)
		return -ENODEV;

	/*
	 * Wipe the bootloader's splash so that "something changed on the panel"
	 * is by itself proof the kernel started executing, even if the very
	 * next thing it does is die before a single character is legible.
	 */
	if (!simplefb_wrap())
		return -ENXIO;

	device->con->write = simplefb_earlycon_write;
	simplefb_con = device->con;

	return 0;
}
EARLYCON_DECLARE(simplefb, simplefb_earlycon_setup);
