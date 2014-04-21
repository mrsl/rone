/* Reset code for Stellaris LM3S8962
 *
 * Version: Sourcery CodeBench 2011.09-82
 * Support: https://support.codesourcery.com/GNUToolchain/
 *
 * Copyright (c) 2007, 2008, 2009, 2010 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */
#if defined(__thumb2__) || defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_6SM__)
#define THUMB 1
	.thumb
	.thumb_func
#else
	.arm
#endif
	.section .cs3.reset,"ax",%progbits
	.globl __cs3_reset_lm3s8962
	.type	__cs3_reset_lm3s8962, %function
__cs3_reset_lm3s8962:
	/* jump to common start code */
#if THUMB
	ldr	r0,=_start
	bx	r0
#else
	b	_start
#endif
	.pool
	.size	__cs3_reset_lm3s8962,.-__cs3_reset_lm3s8962
/* checksum: 0a5fe3b55361dc42137160d717 */
