// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>

module tb_cheshire_soc;

  cheshire_soc_fixture fix();

  string binary;
  longint entry_int;
  logic [63:0] entry;
  int exit_status = 0;

  initial begin

    fix.set_bootmode(2);
    fix.set_testmode(0);
 
    fix.wait_for_reset();

    // Load binaries into memory (if any)
    if ($value$plusargs("BINARY=%s", binary)) begin
      $display("[tb_cheshire_soc] BINARY = %s", binary);
      fix.load_binary(binary);

      // Obtain the entry point from the ELF file
      void'(fix.get_entry(entry_int));
      entry = entry_int[63:0];
      
    end else begin
      // If no ELF file is provided jump to the beginning of the SPM
      entry = cheshire_pkg::SpmBase;
    end

    // Wait for LLC BIST to finish
    #8us;

    fix.jtag_init();

    fix.jtag_cfg_llc_spm();

    // Randomize the SPM
    fix.sl_rand(cheshire_pkg::SpmBase + 48'h0000_5000, 64'h0000_1000);

    fix.sl_preload();

    // Preload memory of I2C EEPROM
    $readmemh("../models/24FC1025.mem", fix.i_i2c_model.MemoryBlock);

    // Preload the sections from an ELF file
    //fix.jtag_preload();
    
    // Check the preloaded sections
    //fix.jtag_preload_check();

    // Run from entrypoint
    fix.jtag_run(entry);

    // Use idle boot mode to start system
    //fix.jtag_idle_boot(entry);

    // Wait for the application to write the return value to the first scratch register
    fix.jtag_wait_for_eoc(cheshire_pkg::ScratchRegsBase + 64'h4, exit_status);

    $finish;
  end

endmodule
