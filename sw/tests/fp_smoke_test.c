// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>
//
// Simple payload to test the FP unit

#include "cheshire_regs.h"
#include "printf.h"
#include "trap.h"
#include "uart.h"

extern void *__base_cheshire_regs;

char uart_initialized = 0;

void __attribute__((aligned(4))) trap_vector(void) { test_trap_vector(&uart_initialized); }

int main(void) {
    volatile uint32_t *reset_freq = (uint32_t *)(((uint64_t)&__base_cheshire_regs) + CHESHIRE_RESET_FREQ_REG_OFFSET);
    volatile float f1 = 0, f2 = 0.75f;
    volatile double d1 = 0, d2 = 0.75f;

    init_uart(*reset_freq, 115200);

    uart_initialized = 1;

    f1 =  0.125f;
    f1 += 0.25f;
    f1 += 0.375f;

    d1 =  0.125f;
    d1 += 0.25f;
    d1 += 0.375f;

    return !((f1 == f2) && (d1 == d2));
}
