# Copyright 2022 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Nicole Narr <narrn@student.ethz.ch>
# Christopher Reinwardt <creinwar@student.ethz.ch>

if {![info exists BINARY]} {
    puts "Set the \"BINARY\" variable before sourcing the start script"
    set BINARY ""
}

#vsim tb_cheshire_chip -t 1ps -voptargs=+acc +BINARY=$BINARY -permissive -suppress 3009
vsim tb_cheshire_chip -t 100ps -vopt -vopt_verbose -voptargs=+noacc -voptargs=+acc+s25fs512s -voptargs=+acc+stream_xbar -voptargs=+acc+uart_tb_rx -voptargs=-O5 -voptargs=-nofsmdebug -voptargs=+nocover +BINARY=$BINARY -permissive -suppress 3009


set StdArithNoWarnings 1
set NumericStdNoWarnings 1

log -r *

# check exit status in tb and quit the simulation accordingly
proc run_and_exit {} {
    onfinish stop
    run -all

    if {[coverage attribute -concise -name TESTSTATUS] >= 3} {
        # exit with error if we had a $fatal somewhere
        quit -code 1
    } else {
        # assume there is an `exit_status` signal that contains the status of
        # the simulation
        quit -code [examine -radix decimal sim:/tb_cheshire_soc/exit_status]
    }
}
