/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ATEN PE4102 (Nuvoton NUC977): feed the external watchdog while the zImage
 * decompresses.
 *
 * An off-SoC watchdog resets the board unless its WDI input, PB12, toggles
 * within about 3 s. U-Boot feeds it until it jumps to the zImage and the
 * kernel resumes from an early_initcall (arch/arm/mach-nuc970), but an
 * LZMA/XZ decompression in between takes longer than that.
 *
 * decompress_progress() is called once per input byte by the range decoders;
 * toggle PB12 every 1 KiB. U-Boot leaves PB12 as a GPIO output with the GPIO
 * clock on, so only DATAOUT is touched. The decompressor maps I/O 1:1
 * (uncached) and may not use writable static data, hence the physical
 * address and the position-based divider.
 */
#ifndef NUC970_DECOMP_WDI_H
#define NUC970_DECOMP_WDI_H

#define NUC970_GPIOB_DATAOUT	0xb8003044u	/* GPIO base 0xb8003000 + 0x44 */
#define NUC970_WDI_PB12		(1u << 12)

#define decompress_progress(pos) \
	do { \
		if (((unsigned long)(pos) & 0x3ff) == 0) \
			*(volatile unsigned int *)NUC970_GPIOB_DATAOUT ^= NUC970_WDI_PB12; \
	} while (0)

#endif /* NUC970_DECOMP_WDI_H */
