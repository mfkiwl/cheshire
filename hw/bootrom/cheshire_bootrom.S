// Copyright 2022 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>

.section .text._start

// Minimal bootrom crt0
.global _start
.global _exit
_start:
    // Init stack and global pointer
    la sp, __stack_pointer$
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    csrrw t0, misa, x0
    andi t1, t0, 40         // D and F extension?
    andi t0, t0, 8          // Only F extension?
    addi t1, t1, -40
    bnez t1, _clear_s_fprs  // No D+F extension combo
    li t1, 1
    slli t1, t1, 13
    csrrs x0, mstatus, t1   // Set FS state to "Initial"

    // Clear all 32 double FP registers
    fcvt.d.l f0, x0
    fcvt.d.l f1, x0
    fcvt.d.l f2, x0
    fcvt.d.l f3, x0
    fcvt.d.l f4, x0
    fcvt.d.l f5, x0
    fcvt.d.l f6, x0
    fcvt.d.l f7, x0
    fcvt.d.l f8, x0
    fcvt.d.l f9, x0
    fcvt.d.l f10, x0
    fcvt.d.l f11, x0
    fcvt.d.l f12, x0
    fcvt.d.l f13, x0
    fcvt.d.l f14, x0
    fcvt.d.l f15, x0
    fcvt.d.l f16, x0
    fcvt.d.l f17, x0
    fcvt.d.l f18, x0
    fcvt.d.l f19, x0
    fcvt.d.l f20, x0
    fcvt.d.l f21, x0
    fcvt.d.l f22, x0
    fcvt.d.l f23, x0
    fcvt.d.l f24, x0
    fcvt.d.l f25, x0
    fcvt.d.l f26, x0
    fcvt.d.l f27, x0
    fcvt.d.l f28, x0
    fcvt.d.l f29, x0
    fcvt.d.l f30, x0
    fcvt.d.l f31, x0

    csrrc x0, mstatus, t1   // Set FS state to "Clean"

_clear_int_gprs:

    // Reset all GPRs
    li x1, 0
    li x4, 0
    li x5, 0
    li x6, 0
    li x7, 0
    li x8, 0
    li x9, 0
    li x10, 0
    li x11, 0
    li x12, 0
    li x13, 0
    li x14, 0
    li x15, 0
    li x16, 0
    li x17, 0
    li x18, 0
    li x19, 0
    li x20, 0
    li x21, 0
    li x22, 0
    li x23, 0
    li x24, 0
    li x25, 0
    li x26, 0
    li x27, 0
    li x28, 0
    li x29, 0
    li x30, 0
    li x31, 0

    // Configure the LLC to all SPM
    la t0, __base_axi_llc
    li t1, -1
    sw t1, 0(t0)    // CFG_SPM_LOW  = 0xFFFFFFFF
    sw t1, 4(t0)    // CFG_SPM_HIGH = 0xFFFFFFFF
    li t1, 1
    sw t1, 16(t0)   // COMMIT_CFG = 0x1
    li t0, 0
    li t1, 0

    // Boldly flush everything
    fence.i

    jal main

    // Main returned - save return value and mark as done
    j _exit

// The unlikely path is moved out of the way
_clear_s_fprs:
    beq t0, x0, _clear_int_gprs // t0 = misa & 0x8 ==> No F extension if t0 == 0

    li t1, 1
    slli t1, t1, 13
    csrrs x0, mstatus, t1   // Set FS state to "Initial"

    // Clear all 32 single FP registers
    fcvt.s.l f0, x0
    fcvt.s.l f1, x0
    fcvt.s.l f2, x0
    fcvt.s.l f3, x0
    fcvt.s.l f4, x0
    fcvt.s.l f5, x0
    fcvt.s.l f6, x0
    fcvt.s.l f7, x0
    fcvt.s.l f8, x0
    fcvt.s.l f9, x0
    fcvt.s.l f10, x0
    fcvt.s.l f11, x0
    fcvt.s.l f12, x0
    fcvt.s.l f13, x0
    fcvt.s.l f14, x0
    fcvt.s.l f15, x0
    fcvt.s.l f16, x0
    fcvt.s.l f17, x0
    fcvt.s.l f18, x0
    fcvt.s.l f19, x0
    fcvt.s.l f20, x0
    fcvt.s.l f21, x0
    fcvt.s.l f22, x0
    fcvt.s.l f23, x0
    fcvt.s.l f24, x0
    fcvt.s.l f25, x0
    fcvt.s.l f26, x0
    fcvt.s.l f27, x0
    fcvt.s.l f28, x0
    fcvt.s.l f29, x0
    fcvt.s.l f30, x0
    fcvt.s.l f31, x0

    csrrc x0, mstatus, t1   // Set FS state to "Clean"

    j _clear_int_gprs

// If main returns we end up here
// Save the return value to the scratch register 0
// and wait forever
_exit:
    slli a0, a0, 1
    ori  a0, a0, 1
    la t0, __base_cheshire_regs
    sw a0, 4(t0)

_exit_wait_loop:
    wfi
    j _exit_wait_loop
