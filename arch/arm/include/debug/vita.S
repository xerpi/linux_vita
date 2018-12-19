/*
 *  Copyright (C) 2018 Sergi Granell
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define VITA_UART_STATUS		(0x28)
#define VITA_UART_WRITE_FIFO		(0x70)

#define VITA_UART_STATUS_TXREADY	(1 << 8)
#define VITA_UART_STATUS_DEVREADY	(1 << 9)

	.macro	addruart,rp,rv,tmp
	ldr	\rp, =CONFIG_DEBUG_UART_PHYS
	ldr	\rv, =CONFIG_DEBUG_UART_VIRT
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #(VITA_UART_WRITE_FIFO)]
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(VITA_UART_STATUS)]
	tst	\rd, #VITA_UART_STATUS_TXREADY
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(VITA_UART_STATUS)]
	tst	\rd, #VITA_UART_STATUS_DEVREADY
	beq	1001b
	.endm
