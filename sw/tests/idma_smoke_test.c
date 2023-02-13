// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>

#include "printf.h"
#include "trap.h"
#include "uart.h"

char uart_initialized = 0;

extern void *__base_dma_conf;

void __attribute__((aligned(4))) trap_vector(void) { test_trap_vector(&uart_initialized); }

int compare_memory(volatile uint64_t *a, volatile uint64_t *b, uint64_t len)
{
    for(uint64_t i = 0; i < len/8; i++){
        if(a[i] != b[i]){
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    volatile uint64_t *idma = (volatile uint64_t *) & __base_dma_conf;
    uint64_t trans_id = 0;

    init_uart(200000000, 115200);
    uart_initialized = 1;

    // Disable D-Cache
    asm volatile(
            "addi t0, x0, 1\n   \
             csrrc x0, 0x701, t0\n"
             ::: "t0"                                         
            );

    // Src address
    idma[0] = 0x70000000L;

    // Dst address
    idma[1] = 0x80000000L;

    // Num bytes
    idma[2] = 0x1000;

    // Configuration register
    // Bit 0 => Decouple bit
    // Bit 1 => Deburst bit
    // Bit 2 => Serialize bit
    idma[3] = 0x0;

    // Status register
    // Bit 0 => Busy bit
    (void)idma[4];

    asm volatile("fence\n" ::: "memory");

    // Transaction ID (to kickoff operation)
    trans_id = idma[5];
    if(!trans_id){
        printf("Transfer not set up properly!\r\n");
        return -1;
    }

    while(idma[6] != trans_id){}

    return compare_memory((volatile uint64_t *) 0x70000000L, (volatile uint64_t *) 0x80000000L, 0x1000); 
}

