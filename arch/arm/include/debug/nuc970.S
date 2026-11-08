/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Debugging macro include header for the Nuvoton NUC970 UART
 *
 * Based on the UART register layout used by Nuvoton's NUC970 BSP:
 * THR at 0x00, FSR at 0x18 (TX_FULL bit 23, TE_FLAG bit 28).
 */

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =CONFIG_DEBUG_UART_PHYS
		ldr	\rv, =CONFIG_DEBUG_UART_VIRT
		.endm

		.macro	senduart, rd, rx
		str	\rd, [\rx, #0x00]	@ THR
		.endm

		.macro	waituartcts, rd, rx
		.endm

		.macro	waituarttxrdy, rd, rx
1001:		ldr	\rd, [\rx, #0x18]	@ FSR
		tst	\rd, #1 << 23		@ TX_FULL
		bne	1001b
		.endm

		.macro	busyuart, rd, rx
1002:		ldr	\rd, [\rx, #0x18]	@ FSR
		tst	\rd, #1 << 28		@ TE_FLAG (transmitter empty)
		beq	1002b
		.endm
