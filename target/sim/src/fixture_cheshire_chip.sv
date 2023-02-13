// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>

module fixture_cheshire_chip;

  `include "axi/assign.svh"
  `include "axi/typedef.svh"
  `include "register_interface/assign.svh"
  `include "register_interface/typedef.svh"

  import cheshire_pkg::*;
  import pkg_cheshire_padframe_sim::*;


  /////////////////////////////
  // ELFLoader C++ functions //
  /////////////////////////////

  import "DPI-C" function byte read_elf(input string filename);
  import "DPI-C" function byte get_entry(output longint entry);
  import "DPI-C" function byte get_section(output longint address, output longint len);
  import "DPI-C" context function byte read_section(input longint address, inout byte buffer[], input longint len);


  ////////////////////
  // Global Signals //
  ////////////////////

  logic clk_sys, rst_n;
  logic clk_jtag;
  logic clk_rtc;

  logic testmode;
  logic [1:0] bootmode;


  //////////////////////
  // Global Variables //
  //////////////////////

  localparam time ClkPeriodSys    = 5ns;
  localparam time ClkPeriodJTAG   = 20ns;
  localparam time ClkPeriodRTC    = 100ns;

  localparam int UartBaudRate     = 115200;
  localparam int UartParityEna    = 0;

  localparam real TA      = 0.1;
  localparam real TT      = 0.9;

  // exit
  localparam int ExitSuccess = 0;
  localparam int ExitFail    = 1;
  int exit_status            = ExitFail;  // per default we fail


  /////////////////////////
  // Clocking and Resets //
  /////////////////////////

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriodSys  ),
    .RstClkCycles ( 5             )
  ) i_clk_rst_sys (
    .clk_o        ( clk_sys       ),
    .rst_no       (               )
  );

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriodJTAG ),
    .RstClkCycles ( 5             )
  ) i_clk_rst_jtag (
    .clk_o        ( clk_jtag      ),
    .rst_no       (               )
  );

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriodRTC  ),
    .RstClkCycles ( 5             )
  ) i_clk_rst_rtc (
    .clk_o        ( clk_rtc       ),
    .rst_no       (               )
  );

  initial begin
    rst_n = 1'b0;

    #(5*ClkPeriodSys);

    rst_n = 1'b1;
  end

  task wait_for_reset;
    @(posedge rst_n);
    @(posedge clk_sys);
  endtask

  task set_bootmode (
    input logic [1:0] mode
  );
    bootmode = mode;
  endtask

  task set_testmode (
    input logic mode
  );
    testmode = mode;
  endtask


  ///////////////////
  // ELF Importing //
  ///////////////////

  // Ensure memory buffer is initialized to 0 to avoid poisoning DUT
  logic [63:0] memory[bit [63:0]] = '{default: '0};
  int sections [bit [63:0]];

  // Load binary into a temporary buffer
  task load_binary(
    input string binary
  );

    longint section_addr, section_len;
    byte buffer[];
    void'(read_elf(binary));

    $display("[BINLOAD] Loading binary %s", binary);
      
    while (get_section(section_addr, section_len)) begin
      automatic int num_words = (section_len + 7)/8;
        
      $display("[BINLOAD] Loading section 0x%x of length 0x%x B", section_addr, section_len);
        
      sections[section_addr/8] = num_words;
      buffer = new [num_words * 8];
        
      void'(read_section(section_addr, buffer, num_words * 8));
        
      for (int i = 0; i < num_words; i++) begin
        automatic logic [7:0][7:0] word = '0;

        for (int k = 0; k < 8; k++) begin
          word[k] = buffer[i * 8 + k];
        end

        memory[section_addr/8 + i] = word;
      end
    end
  endtask


  ////////////////////
  // JTAG Debugging //
  ////////////////////

  logic jtag_tck, jtag_trst_n;
  logic jtag_tms;
  logic jtag_tdi, jtag_tdo;

  // JTAG interface
  JTAG_DV jtag_mst (clk_jtag);

  // Connect DUT signals to JTAG interface
  assign jtag_tck     = clk_jtag;
  assign jtag_trst_n  = jtag_mst.trst_n;
  assign jtag_tms     = jtag_mst.tms;
  assign jtag_tdi     = jtag_mst.tdi;
  assign jtag_mst.tdo = jtag_tdo;

  typedef jtag_test::riscv_dbg #(
    .IrLength ( 5                   ),
    .TA       ( ClkPeriodJTAG * TA  ),
    .TT       ( ClkPeriodJTAG * TT  )
  ) riscv_dbg_t;

  riscv_dbg_t::jtag_driver_t jtag_in = new (jtag_mst);
  riscv_dbg_t riscv_dbg = new (jtag_in);

  dm::sbcs_t jtag_init_sbcs = dm::sbcs_t'{sbautoincrement: 1'b1, sbreadondata: 1'b1, sbaccess: 3, default: '0};

  // Initialize the debug module
  task jtag_init;
    logic [31:0] idcode;
    logic [31:0] dmctrl;
    automatic dm::sbcs_t sbcs = jtag_init_sbcs;

    $display("[JTAG] Initializing debug module");

    // Reset debug module
    riscv_dbg.reset_master();
    riscv_dbg.wait_idle(12);

    // Check ID code to match
    riscv_dbg.get_idcode(idcode);
    assert (idcode == IDCode) else
      $error("[JTAG] IDCode Mismatch (0x%h (act) != 0x%h (exp))", idcode, IDCode);
    $display("[JTAG] Read IDCode 0x%h", idcode);

    // Activate debug module
    riscv_dbg.write_dmi(dm::DMControl, 32'h0000_0001);
    
    // Check for the activation to complete
    do riscv_dbg.read_dmi_exp_backoff(dm::DMControl, dmctrl);
    while (!(dmctrl & 32'h0000_0001));
    
    // Ensure the system bus is ready too
    riscv_dbg.write_dmi(dm::SBCS, sbcs);
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);

  endtask

  task jtag_preload;
    automatic dm::sbcs_t sbcs = jtag_init_sbcs;
    riscv_dbg.write_dmi(dm::SBCS, sbcs);

    $display("[JTAG] Preloading ELF sections");

    // Write sections
    foreach (sections[addr]) begin
      $display("[JTAG] Writing section 0x%h (%0d words)", addr*8, sections[addr]);

      riscv_dbg.write_dmi(dm::SBAddress0, addr*8);

      for (int i = 0; i < sections[addr]; i++) begin

        if (i % 20 == 0)
            $display(" - Word %0d/%0d (%0d%%)", i, sections[addr], i*100/(sections[addr] > 1 ? sections[addr]-1 : 1));

        riscv_dbg.write_dmi(dm::SBData1, memory[addr + i][32+:32]);
        riscv_dbg.write_dmi(dm::SBData0, memory[addr + i][0+:32]);

        // Wait for the write to complete
        do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
        while (sbcs.sbbusy);
      end
    end
  endtask

  task jtag_preload_check;
    logic [63:0] rdata;

    // Update SBCS
    automatic dm::sbcs_t sbcs = jtag_init_sbcs;
    sbcs.sbreadonaddr = 1;

    riscv_dbg.write_dmi(dm::SBCS, sbcs);

    $display("[JTAG] Checking ELF sections");

    // Read sections
    foreach (sections[addr]) begin
      $display("[JTAG] Checking 0x%h (%0d words)", addr*8, sections[addr]);

      riscv_dbg.write_dmi(dm::SBAddress0, addr*8);

      for (int i = 0; i < sections[addr]; i++) begin
        if (i % 20 == 0)
          $display(" - Word %0d/%0d (%0d%%)", i, sections[addr], i*100/(sections[addr] > 1 ? sections[addr]-1 : 1));

        riscv_dbg.read_dmi_exp_backoff(dm::SBData1, rdata[32 +: 32]);
        riscv_dbg.read_dmi_exp_backoff(dm::SBData0, rdata[ 0 +: 32]);

        if (rdata != memory[addr + i])
          $error("[JTAG] ERROR: Readback mismatch at 0x%x: act 0x%x != exp 0x%x", (addr + i) * 8, rdata, memory[addr + i]);
      end
    end
  endtask

  task jtag_cfg_llc_spm;
    automatic dm::sbcs_t sbcs = jtag_init_sbcs;
    automatic logic [31:0] data;

    // Update SBCS
    sbcs.sbreadonaddr = 0;
    sbcs.sbreadondata = 0;
    sbcs.sbautoincrement = 0;
    sbcs.sbaccess = 3;

    riscv_dbg.write_dmi(dm::SBCS, sbcs);
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);

    $display("[JTAG] Configuring all of LLC to SPM");

    data = 32'hff;

    // cfg_spm_low = 0xff;
    riscv_dbg.write_dmi(dm::SBAddress0, RegbusAddrmap[RegbusOutLlc].start_addr[31:0]);
    riscv_dbg.write_dmi(dm::SBData0, data);
    // Wait for the write to complete
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);

    if(sbcs.sberror) begin
      sbcs.sberror = 1;
      riscv_dbg.write_dmi(dm::SBCS, sbcs);
      do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
      while (sbcs.sbbusy);
    end

    // commit_cfg = 0x1;
    data = 32'h1;
    riscv_dbg.write_dmi(dm::SBAddress0, RegbusAddrmap[RegbusOutLlc].start_addr[31:0]+32'h10);
    riscv_dbg.write_dmi(dm::SBData0, data);
    // Wait for the write to complete
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);

    if(sbcs.sberror) begin
      sbcs.sberror = 1;
      riscv_dbg.write_dmi(dm::SBCS, sbcs);
      do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
      while (sbcs.sbbusy);
    end

    sbcs = jtag_init_sbcs;
    // Ensure the system bus is ready again
    riscv_dbg.write_dmi(dm::SBCS, sbcs);
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);
  endtask


  // Run HART 0 from specified address
  task jtag_run(
    input logic [63:0] start_addr
  );
    logic [31:0] dm_data;

    riscv_dbg.reset_dmi();

    
    $display("[JTAG] halting hart 0");
    // Halt hart 0
    dm_data = 32'h8000_0001;
    riscv_dbg.write_dmi(dm::DMControl, dm_data);
    
    // Check that all selected harts have halted
    do riscv_dbg.read_dmi_exp_backoff(dm::DMStatus, dm_data);
    while (!(dm_data & 32'h0000_0200));


    $display("[JTAG] writing start address to register");
    // Write start address to dpc
    // High 4 bytes
    dm_data = start_addr[63:32];
    riscv_dbg.write_dmi(dm::Data1, dm_data);

    // ... and the low 4 bytes
    dm_data = start_addr[31:0];
    riscv_dbg.write_dmi(dm::Data0, dm_data);


    $display("[JTAG] writing abstract command");
    dm_data = 32'h0033_07b1;
    riscv_dbg.write_dmi(dm::Command, dm_data);
    
    // Wait until the abstract command has completed
    do riscv_dbg.read_dmi_exp_backoff(dm::AbstractCS, dm_data);
    while (dm_data & 32'h0000_1000);

    // Set resume request for hart 0
    dm_data = 32'h4000_0001;
    riscv_dbg.write_dmi(dm::DMControl, dm_data);
    $display("[JTAG] Resuming hart 0 from 0x%h", start_addr);
  endtask

  task jtag_wait_for_eoc(
    input logic [63:0] poll_addr,
    output int         exit_status
  );

    logic [31:0] scratch;

    // Update SBCS
    automatic dm::sbcs_t sbcs = jtag_init_sbcs;
    sbcs.sbreadonaddr = 0;
    sbcs.sbreadondata = 1;
    sbcs.sbautoincrement = 0;
    sbcs.sbaccess = 2;

    riscv_dbg.write_dmi(dm::SBCS, sbcs);

    // Wait for scratch register to be set to non-zero value
    $display("[JTAG] Waiting for completion");
    $display("[JTAG] Polling address: 0x%x", poll_addr);
    riscv_dbg.write_dmi(dm::SBAddress1, poll_addr[63:32]);
    riscv_dbg.write_dmi(dm::SBAddress0, poll_addr[31:0]);

    do begin
      riscv_dbg.wait_idle(10);
      riscv_dbg.read_dmi_exp_backoff(dm::SBData0, scratch);
    end while (scratch[0] == 1'b0);

    // Report end of execution
    $display("[JTAG] Execution finished");
    if ((scratch >> 1) != '0) begin
      $error("[JTAG] FAILED: return code %d", scratch >> 1);
      exit_status = ExitFail;
    end
    else begin
      $display("[JTAG] SUCCESSFUL");
      exit_status = ExitSuccess;
    end
  endtask


  ///////////////
  // I2C Model //
  ///////////////

  // I2C wires
  wire i2c_scl, i2c_sda;

  M24FC1025 i_i2c_model (
    .A0     ( 1'b0    ),
    .A1     ( 1'b0    ),
    .A2     ( 1'b1    ),
    .WP     ( 1'b0    ),
    .SDA    ( i2c_sda ),
    .SCL    ( i2c_scl ),
    .RESET  ( rst_n   )
  );


  /////////////////
  // Serial Link //
  /////////////////

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH ( AxiDataWidth  ),
    .AXI_ID_WIDTH   ( AxiXbarMasterIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth  )
  ) axi_bus_sl2tb (
    .clk_i  ( clk_sys )
  );

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH ( AxiDataWidth  ),
    .AXI_ID_WIDTH   ( AxiXbarMasterIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth  )
  ) axi_bus_tb2sl (
    .clk_i  ( clk_sys )
  );

  axi_a48_d64_mst_u0_req_t sl_out_req, sl_in_req;
  axi_a48_d64_mst_u0_resp_t sl_out_resp, sl_in_resp;

  // From the Serial Link to the Testbench
  `AXI_ASSIGN_FROM_REQ(axi_bus_sl2tb, sl_out_req)
  `AXI_ASSIGN_TO_RESP(sl_out_resp, axi_bus_sl2tb)

  // From the Testbench to the Serial Link
  `AXI_ASSIGN_TO_REQ(sl_in_req, axi_bus_tb2sl)
  `AXI_ASSIGN_FROM_RESP(axi_bus_tb2sl, sl_in_resp)

  logic [3:0]   sl_ddr_data_o, sl_ddr_data_i;
  logic         sl_ddr_clk_o, sl_ddr_clk_i;

  serial_link #(
    .axi_req_t      ( axi_a48_d64_mst_u0_req_t     ),
    .axi_rsp_t      ( axi_a48_d64_mst_u0_resp_t    ),
    .cfg_req_t      ( reg_a48_d32_req_t            ),
    .cfg_rsp_t      ( reg_a48_d32_rsp_t            ),
    .aw_chan_t      ( axi_a48_d64_mst_u0_aw_chan_t ),
    .ar_chan_t      ( axi_a48_d64_mst_u0_ar_chan_t ),
    .r_chan_t       ( axi_a48_d64_mst_u0_r_chan_t  ),
    .w_chan_t       ( axi_a48_d64_mst_u0_w_chan_t  ),
    .b_chan_t       ( axi_a48_d64_mst_u0_b_chan_t  ),
    .hw2reg_t       (serial_link_single_channel_reg_pkg::serial_link_single_channel_hw2reg_t),
    .reg2hw_t       (serial_link_single_channel_reg_pkg::serial_link_single_channel_reg2hw_t),
    .NumChannels    ( 1                            ),
    .NumLanes       ( 4                            ),
    .MaxClkDiv      ( 1024                         )
  ) i_fix_serial_link (
    // There are 3 different clock/resets:
    // 1) clk_i & rst_ni: "always-on" clock & reset coming from the SoC domain. Only config registers are conected to this clock
    // 2) clk_sl_i & rst_sl_ni: Same as 1) but clock is gated and reset is SW synchronized.
    // 3) clk_reg_i & rst_reg_ni: peripheral clock and reset. Only connected to RegBus CDC
    // W/o clock gating, reset synchronization -> tie clk_sl_i to clk_i resp. rst_sl_ni to rst_ni
    .clk_i          ( clk_sys       ),
    .rst_ni         ( rst_n         ),
    .clk_sl_i       ( clk_sys       ),
    .rst_sl_ni      ( rst_n         ),
    .clk_reg_i      ( clk_sys       ),
    .rst_reg_ni     ( rst_n         ),
    .testmode_i     ( testmode      ),
    .axi_in_req_i   ( sl_in_req     ),
    .axi_in_rsp_o   ( sl_in_resp    ),
    .axi_out_req_o  ( sl_out_req    ),
    .axi_out_rsp_i  ( sl_out_resp   ),
    .cfg_req_i      ( '0            ),
    .cfg_rsp_o      (               ),
    .ddr_rcv_clk_i  ( sl_ddr_clk_o  ),
    .ddr_rcv_clk_o  ( sl_ddr_clk_i  ),
    .ddr_i          ( sl_ddr_data_o ),
    .ddr_o          ( sl_ddr_data_i ),
    // AXI isolation signals (in/out), if not used tie to 0
    .isolated_i     ( '0            ),
    .isolate_o      (               ),
    // Clock gate register
    .clk_ena_o      (               ),
    // synch-reset register
    .reset_no       (               )
  );

  // Random slave that keeps written data for the slave side
  axi_test::axi_rand_slave #(
    .AW                   ( AxiAddrWidth                  ),
    .DW                   ( AxiDataWidth                  ),
    .IW                   ( AxiXbarMasterIdWidth          ),
    .UW                   ( AxiUserWidth                  ),
    .MAPPED               ( 1'b1                          ),
    .TA                   ( TA                            ),
    .TT                   ( TT                            ),
    .RAND_RESP            ( 0                             ),
    .AX_MIN_WAIT_CYCLES   ( 0                             ),
    .AX_MAX_WAIT_CYCLES   ( 100                           ),
    .R_MIN_WAIT_CYCLES    ( 0                             ),
    .R_MAX_WAIT_CYCLES    ( 5                             ),
    .RESP_MIN_WAIT_CYCLES ( 0                             ),
    .RESP_MAX_WAIT_CYCLES ( 20                            )
  ) sl_axi_rand_slave = new (axi_bus_sl2tb);


  // Start the rand slave directly from the beginning
  initial begin
    sl_axi_rand_slave.run();
  end

  // An AXI driver for the master side
  typedef axi_test::axi_driver #(
    .AW     ( AxiAddrWidth    ),
    .DW     ( AxiDataWidth    ),
    .IW     ( AxiXbarMasterIdWidth ),
    .UW     ( AxiUserWidth    ),
    .TA     ( TA              ),
    .TT     ( TT              )
  ) sl_axi_driver_t;

  sl_axi_driver_t sl_axi_driver = new (axi_bus_tb2sl);

  initial begin
    @(negedge rst_n);
    sl_axi_driver.reset_master();
  end

  // Write data from queue with given granularity (1, 2, 4 or 8 bytes)
  task automatic sl_write_size(
    input logic [AxiAddrWidth-1:0]  addr,
    input logic [3:0]               size,
    ref logic   [AxiDataWidth-1:0]  data [$]
  );
    automatic sl_axi_driver_t::ax_beat_t ax = new();
    automatic sl_axi_driver_t::w_beat_t w = new();
    automatic sl_axi_driver_t::b_beat_t b;
    automatic int i = 0;
    automatic int size_bytes = (1 << size);

    @(posedge clk_sys);

    sl_axi_driver.reset_master();  
    
    //$display("[SL] Write address: %h, len: %0d", addr, data.size()-1);
    ax.ax_addr  = addr;
    ax.ax_id    = '0;
    ax.ax_len   = data.size() - 1;
    ax.ax_size  = size;
    ax.ax_burst = axi_pkg::BURST_INCR;

    sl_axi_driver.cycle_start();
    sl_axi_driver.cycle_end();
     
    //$write("[SL] - Sending AW... ");
    sl_axi_driver.send_aw(ax);

    sl_axi_driver.cycle_start();
    sl_axi_driver.cycle_end();
    
    //$display("OK");
    //$display("[SL] - Writing burst data");
     
    do begin
      w.w_strb = (~('1 << size_bytes)) << addr[$clog2(AxiDataWidth)-1:0];
      w.w_data = data[i];
      w.w_last = (i == ax.ax_len);

      sl_axi_driver.cycle_start();
      sl_axi_driver.cycle_end();
       
      sl_axi_driver.send_w(w);

      sl_axi_driver.cycle_start();
      sl_axi_driver.cycle_end();
         
      i++;
      addr += size_bytes;
      addr &= size_bytes - 1;
    end while (i <= ax.ax_len);
    
    //$write("[SL] - Waiting for B... ");
    sl_axi_driver.recv_b(b);

    //$display("OK (1)");
  endtask

  task automatic sl_read_size(
    input logic [AxiAddrWidth-1:0]  addr,
    input logic [3:0]               size,
    input logic [7:0]               len,
    ref logic   [AxiDataWidth-1:0]  data [$]
  );
    automatic sl_axi_driver_t::ax_beat_t ax = new();
    automatic sl_axi_driver_t::r_beat_t r;

    @(posedge clk_sys);

    sl_axi_driver.reset_master();
     
    $display("[SL] Read address: %h, len: %0d", addr, len);
    ax.ax_addr  = addr;
    ax.ax_id    = '0;
    ax.ax_len   = len;
    ax.ax_size  = size;
    ax.ax_burst = axi_pkg::BURST_INCR;

    sl_axi_driver.cycle_start();
    sl_axi_driver.cycle_end();
     
    $write("[SL] - Sending AR... ");
    sl_axi_driver.send_ar(ax);

    sl_axi_driver.cycle_start();
    sl_axi_driver.cycle_end();
    
    $display("OK");

    do begin
      $write("[SL] - Receiving R... ");
      sl_axi_driver.recv_r(r);

      $display("OK");
      data.push_back(r.r_data);

    end while (!r.r_last);
  endtask

  // Preload the ELF sections using 64-bit bursts
  task sl_preload;
    logic [AxiDataWidth-1:0] wdata [$];
    int loopcount;

    $display("[SL] Preloading ELF sections");

    foreach (sections[addr]) begin
      $display("[SL] Writing section 0x%x (%0d words)", addr * 8, sections[addr]);

      for (int i = 0; i < sections[addr]/256; i++) begin
        wdata = {};

        // Load the queue for one burst
        for(int k = 0; k < 256; k++) begin
          wdata.push_back(memory[addr + 256*i + k]);
        end

        $display(" - Word %0d/%0d (%0d%%)", i*256, sections[addr], i*256*100/(sections[addr] > 1 ? sections[addr]-1 : 1));
        sl_write_size(((addr + i*256) * 8), 3, wdata);

        loopcount = i+1;
      end

      // Complete the remainder in a shorter burst
      if(loopcount*256 < sections[addr]) begin
        wdata = {};

        for(int k = loopcount*256; k < sections[addr]; k++) begin
          wdata.push_back(memory[addr + k]);
        end

        $display(" - Word %0d/%0d (%0d%%)", loopcount*256, sections[addr], loopcount*256*100/(sections[addr] > 1 ? sections[addr]-1 : 1));
        sl_write_size(((addr + loopcount*256) * 8), 3, wdata);
      end
    end
  endtask

  // Randomize memory contents
  task sl_rand(
    input logic [AxiAddrWidth-1:0]  addr,
    input logic [AxiDataWidth-1:0]  len
  );
    logic [AxiDataWidth-1:0] wdata [$];
    logic [AxiDataWidth-1:0] random_data;
    int loopcount;

    $display("[SL] Randomizing 0x%h -> 0x%h", addr, addr+len);

    for (int i = 0; i < len/(256*8); i++) begin
      wdata = {};

      // Load the queue for one burst
      for(int k = 0; k < 256; k++) begin
        void'(std::randomize(random_data));
        wdata.push_back(random_data);
      end

      $display(" - Word %0d/%0d (%0d%%)", i*256, len/8, i*256*100/((len/8) > 1 ? (len/8)-1 : 1));
      sl_write_size(addr + (i*256 * 8), 3, wdata);

      loopcount = i+1;
    end

    // Complete the remainder in a shorter burst
    if(loopcount*256*8 < len) begin
      wdata = {};

      for(int k = loopcount*256*8; k < len; k++) begin
        void'(std::randomize(random_data));
        wdata.push_back(random_data);
      end

      $display(" - Word %0d/%0d (%0d%%)", loopcount*256, len/8, loopcount*256*100/((len/8) > 1 ? (len/8)-1 : 1));
      sl_write_size(addr + (loopcount*256 * 8), 3, wdata);
    end
  endtask


  //////////
  // DRAM //
  //////////

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH ( AxiDataWidth  ),
    .AXI_ID_WIDTH   ( AxiXbarSlaveIdWidth + 1 ),
    .AXI_USER_WIDTH ( AxiUserWidth  )
  ) axi_bus_dram2tb (
    .clk_i  ( clk_sys )
  );

  axi_a48_d64_mst_u0_llc_req_t  dram_req;
  axi_a48_d64_mst_u0_llc_resp_t dram_resp;
   
  `AXI_ASSIGN_FROM_REQ(axi_bus_dram2tb, dram_req)
  `AXI_ASSIGN_TO_RESP(dram_resp, axi_bus_dram2tb)

  // Random slave that keeps written data for the slave side
  axi_test::axi_rand_slave #(
    .AW                   ( AxiAddrWidth                  ),
    .DW                   ( AxiDataWidth                  ),
    .IW                   ( AxiXbarSlaveIdWidth + 1       ),
    .UW                   ( AxiUserWidth                  ),
    .MAPPED               ( 1'b1                          ),
    .TA                   ( TA                            ),
    .TT                   ( TT                            ),
    .RAND_RESP            ( 0                             ),
    .AX_MIN_WAIT_CYCLES   ( 0                             ),
    .AX_MAX_WAIT_CYCLES   ( 100                           ),
    .R_MIN_WAIT_CYCLES    ( 0                             ),
    .R_MAX_WAIT_CYCLES    ( 5                             ),
    .RESP_MIN_WAIT_CYCLES ( 0                             ),
    .RESP_MAX_WAIT_CYCLES ( 20                            )
  ) dram_axi_rand_slave = new (axi_bus_dram2tb);

  // Start the rand slave directly from the beginning
  initial begin
    dram_axi_rand_slave.run();
  end


  ///////////////
  // SPI Model //
  ///////////////

  wire spi_sck, spi_cs;
  wire [3:0] spi_io;
  wire spi_reset_n;
  wire spi_wp_n;

  assign spi_wp_n = 1'b1;

  assign spi_reset_n = rst_n;

  s25fs512s i_spi_model (
    // Data IO
    .SI       ( spi_io[0]   ),
    .SO       ( spi_io[1]   ),
    // Controls
    .SCK      ( spi_sck     ),
    .CSNeg    ( spi_cs      ),
    .WPNeg    ( spi_wp_n    ),
    .RESETNeg ( spi_reset_n )
  );


  ///////////////////
  // UART Receiver //
  ///////////////////

  logic uart_tx;

  uart_tb_rx #(
    .BAUD_RATE ( UartBaudRate     ),
    .PARITY_EN ( UartParityEna    )
  ) i_uart_rx_model (
    .rx        ( uart_tx          ),
    .rx_en     ( 1'b1             ),
    .word_done (                  )
  );


  //////////////////
  // Cheshire SoC //
  //////////////////

  logic core_clk_int;

  logic reset_n_int;
  logic testmode_int;

  logic [1:0] bootmode_int;

  logic [1:0] spim_csb_int;
  logic [1:0] spim_csb_en_int;
  logic spim_sck_int;
  logic spim_sck_en_int;
  logic [3:0] spim_sd_out_int;
  logic [3:0] spim_sd_in_int;
  logic [3:0] spim_sd_en_int;

  logic uart_rx_int;
  logic uart_tx_int;

  logic jtag_tdo_int;
  logic jtag_tck_int;
  logic jtag_tdi_int;
  logic jtag_tms_int;
  logic jtag_trst_n_int;

  logic i2c_scl_out_int;
  logic i2c_scl_in_int;
  logic i2c_scl_en_int;
  logic i2c_sda_out_int;
  logic i2c_sda_in_int;
  logic i2c_sda_en_int;

  logic ddr_link_clk_in_int;
  logic ddr_link_clk_out_int;
  logic [3:0] ddr_link_in_int;
  logic [3:0] ddr_link_out_int;

  logic vga_hsync_int;
  logic vga_vsync_int;

  logic [2:0] vga_red_int;
  logic [2:0] vga_green_int;
  logic [1:0] vga_blue_int;

  reg_a48_d32_req_t external_reg_req;
  reg_a48_d32_rsp_t external_reg_rsp; 
 

  cheshire_soc i_dut_cheshire_soc (
    .clk_i            ( core_clk_int        ),
    .rst_ni           ( reset_n_int         ),

    .testmode_i       ( testmode_int        ),

    // Boot mode selection
    .boot_mode_i      ( bootmode_int        ),

    // Boot address for CVA6
    .boot_addr_i      ( 64'h0100_0000       ),

    // DRAM
    .dram_req_o       ( dram_req            ),
    .dram_resp_i      ( dram_resp           ),
                                    
    // DDR-Link
    .ddr_link_i       ( ddr_link_in_int      ),
    .ddr_link_o       ( ddr_link_out_int     ),
    .ddr_link_clk_i   ( ddr_link_clk_in_int  ),
    .ddr_link_clk_o   ( ddr_link_clk_out_int ),

    // VGA Controller
    .vga_hsync_o      ( vga_hsync_int       ),
    .vga_vsync_o      ( vga_vsync_int       ),
    .vga_red_o        ( vga_red_int         ),
    .vga_green_o      ( vga_green_int       ),
    .vga_blue_o       ( vga_blue_int        ),

    // JTAG Interface
    .jtag_tck_i       ( jtag_tck_int        ),
    .jtag_trst_ni     ( jtag_trst_n_int     ),
    .jtag_tms_i       ( jtag_tms_int        ),
    .jtag_tdi_i       ( jtag_tdi_int        ),
    .jtag_tdo_o       ( jtag_tdo_int        ),

    // UART Interface
    .uart_tx_o        ( uart_tx_int         ),
    .uart_rx_i        ( 1'b0                ),

    // I2C Interface
    .i2c_sda_o        ( i2c_sda_out_int     ),
    .i2c_sda_i        ( i2c_sda_in_int      ),
    .i2c_sda_en_o     ( i2c_sda_en_int      ),
    .i2c_scl_o        ( i2c_scl_out_int     ),
    .i2c_scl_i        ( i2c_scl_in_int      ),
    .i2c_scl_en_o     ( i2c_scl_en_int      ),

    // SPI Host Interface
    .spim_sck_o       ( spim_sck_int         ),
    .spim_sck_en_o    ( spim_sck_en_int      ),
    .spim_csb_o       ( spim_csb_int         ),
    .spim_csb_en_o    ( spim_csb_en_int      ),
    .spim_sd_o        ( spim_sd_out_int      ),
    .spim_sd_en_o     ( spim_sd_en_int       ),
    .spim_sd_i        ( spim_sd_in_int       ),

    // CLINT
    .rtc_i            ( ref_clk_int          ),

    // CLK locked signal
    .clk_locked_i     ( 1'b0                 ),

    // External Regbus
    .external_reg_req_o ( external_reg_req   ),
    .external_reg_rsp_i ( external_reg_rsp   )
  );


  //////////////
  // Padframe //
  //////////////

  // Static connections SoC -> Padframe
  static_connection_signals_soc2pad_t static_soc2pad;

  assign static_soc2pad.main.ddr_link_clk_out_c2p = ddr_link_clk_out_int;
  assign static_soc2pad.main.ddr_link_out0_c2p    = ddr_link_out_int[0];
  assign static_soc2pad.main.ddr_link_out1_c2p    = ddr_link_out_int[1];
  assign static_soc2pad.main.ddr_link_out2_c2p    = ddr_link_out_int[2];
  assign static_soc2pad.main.ddr_link_out3_c2p    = ddr_link_out_int[3];

  assign static_soc2pad.main.i2c_scl_c2p    = i2c_scl_out_int;
  assign static_soc2pad.main.i2c_scl_en_c2p = i2c_scl_en_int;
  assign static_soc2pad.main.i2c_sda_c2p    = i2c_sda_out_int;
  assign static_soc2pad.main.i2c_sda_en_c2p = i2c_sda_en_int;

  assign static_soc2pad.main.jtag_tdo_c2p   = jtag_tdo_int;

  // RPC stubbed currently
  assign static_soc2pad.main.rpc_clk_c2p   = '0;
  assign static_soc2pad.main.rpc_clk_n_c2p = '1;
  assign static_soc2pad.main.rpc_cs_n_c2p  = '1;

  assign static_soc2pad.main.rpc_db0_c2p   = '0;
  assign static_soc2pad.main.rpc_db1_c2p   = '0;
  assign static_soc2pad.main.rpc_db2_c2p   = '0;
  assign static_soc2pad.main.rpc_db3_c2p   = '0;
  assign static_soc2pad.main.rpc_db4_c2p   = '0;
  assign static_soc2pad.main.rpc_db5_c2p   = '0;
  assign static_soc2pad.main.rpc_db6_c2p   = '0;
  assign static_soc2pad.main.rpc_db7_c2p   = '0;
  assign static_soc2pad.main.rpc_db8_c2p   = '0;
  assign static_soc2pad.main.rpc_db9_c2p   = '0;
  assign static_soc2pad.main.rpc_db10_c2p  = '0;
  assign static_soc2pad.main.rpc_db11_c2p  = '0;
  assign static_soc2pad.main.rpc_db12_c2p  = '0;
  assign static_soc2pad.main.rpc_db13_c2p  = '0;
  assign static_soc2pad.main.rpc_db14_c2p  = '0;
  assign static_soc2pad.main.rpc_db15_c2p  = '0;

  assign static_soc2pad.main.rpc_db_ie_c2p = '0;
  assign static_soc2pad.main.rpc_db_oe_c2p = '0;
  assign static_soc2pad.main.rpc_db_pe_c2p = '0;

  assign static_soc2pad.main.rpc_dqs_c2p    = '0;
  assign static_soc2pad.main.rpc_dqs_n_c2p  = '0;
  assign static_soc2pad.main.rpc_dqs_ie_c2p = '0;
  assign static_soc2pad.main.rpc_dqs_oe_c2p = '0;
  assign static_soc2pad.main.rpc_dqs_pe_c2p = '0;

  assign static_soc2pad.main.rpc_stb_c2p = '0;

  assign static_soc2pad.main.spim_csb0_c2p    = spim_csb_int[0];
  assign static_soc2pad.main.spim_csb1_c2p    = spim_csb_int[1];
  assign static_soc2pad.main.spim_csb_en0_c2p = spim_csb_en_int[0];
  assign static_soc2pad.main.spim_csb_en1_c2p = spim_csb_en_int[1];
  assign static_soc2pad.main.spim_sck_c2p     = spim_sck_int;
  assign static_soc2pad.main.spim_sck_en_c2p  = spim_sck_en_int;
  assign static_soc2pad.main.spim_sd0_c2p     = spim_sd_out_int[0];
  assign static_soc2pad.main.spim_sd1_c2p     = spim_sd_out_int[1];
  assign static_soc2pad.main.spim_sd2_c2p     = spim_sd_out_int[2];
  assign static_soc2pad.main.spim_sd3_c2p     = spim_sd_out_int[3];
  assign static_soc2pad.main.spim_sd_en0_c2p  = spim_sd_en_int[0];
  assign static_soc2pad.main.spim_sd_en1_c2p  = spim_sd_en_int[1];
  assign static_soc2pad.main.spim_sd_en2_c2p  = spim_sd_en_int[2];
  assign static_soc2pad.main.spim_sd_en3_c2p  = spim_sd_en_int[3];

  assign static_soc2pad.main.uart_tx_c2p = uart_tx_int;

  assign static_soc2pad.main.vga_hsync_c2p = vga_hsync_int;
  assign static_soc2pad.main.vga_vsync_c2p = vga_vsync_int;
  assign static_soc2pad.main.vga_blue0_c2p = vga_blue_int[0];
  assign static_soc2pad.main.vga_blue1_c2p = vga_blue_int[0];
  assign static_soc2pad.main.vga_green0_c2p = vga_green_int[0];
  assign static_soc2pad.main.vga_green1_c2p = vga_green_int[1];
  assign static_soc2pad.main.vga_green2_c2p = vga_green_int[2];
  assign static_soc2pad.main.vga_red0_c2p = vga_red_int[0];
  assign static_soc2pad.main.vga_red1_c2p = vga_red_int[1];
  assign static_soc2pad.main.vga_red2_c2p = vga_red_int[2];

  // Static connections Padframe -> SoC
  static_connection_signals_pad2soc_t static_pad2soc;

  assign bootmode_int[0] = static_pad2soc.main.bootmode0_p2c;
  assign bootmode_int[1] = static_pad2soc.main.bootmode1_p2c;

  assign ddr_link_clk_in_int = static_pad2soc.main.ddr_link_clk_in_p2c;
  assign ddr_link_in_int[0] = static_pad2soc.main.ddr_link_in0_p2c;
  assign ddr_link_in_int[1] = static_pad2soc.main.ddr_link_in1_p2c;
  assign ddr_link_in_int[2] = static_pad2soc.main.ddr_link_in2_p2c;
  assign ddr_link_in_int[3] = static_pad2soc.main.ddr_link_in3_p2c;

  assign i2c_scl_in_int = static_pad2soc.main.i2c_scl_p2c;
  assign i2c_sda_in_int = static_pad2soc.main.i2c_sda_p2c;

  assign jtag_tck_int = static_pad2soc.main.jtag_tck_p2c;
  assign jtag_tdi_int = static_pad2soc.main.jtag_tdi_p2c;
  assign jtag_tms_int = static_pad2soc.main.jtag_tms_p2c;
  assign jtag_trst_n_int = static_pad2soc.main.jtag_trst_n_p2c;

  assign core_clk_int = static_pad2soc.main.core_clk_p2c;
  assign ref_clk_int = static_pad2soc.main.ref_clk_p2c;
  assign reset_n_int = static_pad2soc.main.reset_n_p2c;

  // RPC stubbed currently
  /* assign rpc_db_in_int[0] = static_pad2soc.main.rpc_db0_p2c;
  assign rpc_db_in_int[1] = static_pad2soc.main.rpc_db1_p2c;
  assign rpc_db_in_int[2] = static_pad2soc.main.rpc_db2_p2c;
  assign rpc_db_in_int[3] = static_pad2soc.main.rpc_db3_p2c;
  assign rpc_db_in_int[4] = static_pad2soc.main.rpc_db4_p2c;
  assign rpc_db_in_int[5] = static_pad2soc.main.rpc_db5_p2c;
  assign rpc_db_in_int[6] = static_pad2soc.main.rpc_db6_p2c;
  assign rpc_db_in_int[7] = static_pad2soc.main.rpc_db7_p2c;
  assign rpc_db_in_int[8] = static_pad2soc.main.rpc_db8_p2c;
  assign rpc_db_in_int[9] = static_pad2soc.main.rpc_db9_p2c;
  assign rpc_db_in_int[10] = static_pad2soc.main.rpc_db10_p2c;
  assign rpc_db_in_int[11] = static_pad2soc.main.rpc_db11_p2c;
  assign rpc_db_in_int[12] = static_pad2soc.main.rpc_db12_p2c;
  assign rpc_db_in_int[13] = static_pad2soc.main.rpc_db13_p2c;
  assign rpc_db_in_int[14] = static_pad2soc.main.rpc_db14_p2c;
  assign rpc_db_in_int[15] = static_pad2soc.main.rpc_db15_p2c;

  assign rpc_dqs_in_int = static_pad2soc.main.rpc_dqs_p2c;
  assign rpc_dqs_n_in_int = static_pad2soc.main.rpc_dqs_n_p2c; */

  assign spim_sd_in_int[0] = static_pad2soc.main.spim_sd0_p2c;
  assign spim_sd_in_int[1] = static_pad2soc.main.spim_sd1_p2c;
  assign spim_sd_in_int[2] = static_pad2soc.main.spim_sd2_p2c;
  assign spim_sd_in_int[3] = static_pad2soc.main.spim_sd3_p2c;

  assign testmode_int = static_pad2soc.main.testmode_p2c;

  assign uart_rx_int = static_pad2soc.main.uart_rx_p2c;


  // Logic to wire conversion for padframe

  wire clk_sys_wire;
  wire clk_rtc_wire;
  wire rst_n_wire;
  wire testmode_wire;
  wire bootmode0_wire;
  wire bootmode1_wire;

  assign clk_sys_wire = clk_sys;
  assign clk_rtc_wire = clk_rtc;
  assign rst_n_wire = rst_n;
  assign testmode_wire = testmode;
  assign bootmode0_wire = bootmode[0];
  assign bootmode0_wire = bootmode[1];

  wire jtag_tck_wire;
  wire jtag_trst_n_wire;
  wire jtag_tms_wire;
  wire jtag_tdi_wire;
  wire jtag_tdo_wire;

  assign jtag_tck_wire = jtag_tck;
  assign jtag_trst_n_wire = jtag_trst_n;
  assign jtag_tms_wire = jtag_tms;
  assign jtag_tdi_wire = jtag_tdi;
  assign jtag_tdo = jtag_tdo_wire;

  wire uart_rx_wire;
  wire uart_tx_wire;

  assign uart_rx_wire = 1'b0;
  assign uart_tx = uart_tx_wire;

  wire spim_sd2_wire;
  wire spim_sd3_wire;

  assign spim_sd2_wire = 1'b0;
  assign spim_sd3_wire = 1'b0;

  wire sl_ddr_clk_i_wire;
  wire sl_ddr_clk_o_wire;
  wire sl_ddr_data0_i_wire;
  wire sl_ddr_data1_i_wire;
  wire sl_ddr_data2_i_wire;
  wire sl_ddr_data3_i_wire;
  wire sl_ddr_data0_o_wire;
  wire sl_ddr_data1_o_wire;
  wire sl_ddr_data2_o_wire;
  wire sl_ddr_data3_o_wire;

  assign sl_ddr_clk_i_wire = sl_ddr_clk_i;
  assign sl_ddr_clk_o = sl_ddr_clk_o_wire;
  assign sl_ddr_data0_i_wire = sl_ddr_data_i[0];
  assign sl_ddr_data1_i_wire = sl_ddr_data_i[1];
  assign sl_ddr_data2_i_wire = sl_ddr_data_i[2];
  assign sl_ddr_data3_i_wire = sl_ddr_data_i[3];
  assign sl_ddr_data_o[0] = sl_ddr_data0_o_wire;
  assign sl_ddr_data_o[1] = sl_ddr_data1_o_wire;
  assign sl_ddr_data_o[2] = sl_ddr_data2_o_wire;
  assign sl_ddr_data_o[3] = sl_ddr_data3_o_wire;

  wire rpc_dqs_wire;
  wire rpc_dqs_n_wire;
  wire rpc_db0_wire;
  wire rpc_db1_wire;
  wire rpc_db2_wire;
  wire rpc_db3_wire;
  wire rpc_db4_wire;
  wire rpc_db5_wire;
  wire rpc_db6_wire;
  wire rpc_db7_wire;
  wire rpc_db8_wire;
  wire rpc_db9_wire;
  wire rpc_db10_wire;
  wire rpc_db11_wire;
  wire rpc_db12_wire;
  wire rpc_db13_wire;
  wire rpc_db14_wire;
  wire rpc_db15_wire;

  assign rpc_dqs_wire = 1'b0;
  assign rpc_dqs_n_wire = 1'b0;
  assign rpc_db0_wire = 1'b0;
  assign rpc_db1_wire = 1'b0;
  assign rpc_db2_wire = 1'b0;
  assign rpc_db3_wire = 1'b0;
  assign rpc_db4_wire = 1'b0;
  assign rpc_db5_wire = 1'b0;
  assign rpc_db6_wire = 1'b0;
  assign rpc_db7_wire = 1'b0;
  assign rpc_db8_wire = 1'b0;
  assign rpc_db9_wire = 1'b0;
  assign rpc_db10_wire = 1'b0;
  assign rpc_db11_wire = 1'b0;
  assign rpc_db12_wire = 1'b0;
  assign rpc_db13_wire = 1'b0;
  assign rpc_db14_wire = 1'b0;
  assign rpc_db15_wire = 1'b0;


  cheshire_padframe_sim #(
    .AW     ( 48                ),
    .DW     ( 32                ),
    .req_t  ( reg_a48_d32_req_t ),
    .resp_t ( reg_a48_d32_rsp_t )
  ) i_cheshire_padframe_sim (
    .clk_i  ( core_clk_int      ),
    .rst_ni ( reset_n_int       ),
    .static_connection_signals_pad2soc ( static_pad2soc ),
    .static_connection_signals_soc2pad ( static_soc2pad ),
    // Landing Pads
    .pad_main_core_clk_pad    ( clk_sys_wire     ),
    .pad_main_ref_clk_pad     ( clk_rtc_wire     ),
    .pad_main_reset_n_pad     ( rst_n_wire       ),
    .pad_main_testmode_pad    ( testmode_wire    ),
    .pad_main_bootmode0_pad   ( bootmode0_wire   ),
    .pad_main_bootmode1_pad   ( bootmode1_wire   ),
    .pad_main_jtag_tck_pad    ( jtag_tck_wire    ),
    .pad_main_jtag_trst_n_pad ( jtag_trst_n_wire ),
    .pad_main_jtag_tms_pad    ( jtag_tms_wire    ),
    .pad_main_jtag_tdi_pad    ( jtag_tdi_wire    ),
    .pad_main_jtag_tdo_pad    ( jtag_tdo_wire    ),
    .pad_main_uart_rx_pad     ( uart_rx_wire     ),
    .pad_main_uart_tx_pad     ( uart_tx_wire     ),
    .pad_main_spim_sck_pad    ( spi_sck     ),
    .pad_main_spim_csb0_pad   (             ), // SD Card
    .pad_main_spim_csb1_pad   ( spi_cs      ),
    .pad_main_spim_sd0_pad    ( spi_io[0]   ),
    .pad_main_spim_sd1_pad    ( spi_io[1]   ),
    .pad_main_spim_sd2_pad    ( spim_sd2_wire ),
    .pad_main_spim_sd3_pad    ( spim_sd3_wire ),
    .pad_main_i2c_scl_pad     ( i2c_scl     ),
    .pad_main_i2c_sda_pad     ( i2c_sda     ),
    .pad_main_ddr_link_clk_in_pad  ( sl_ddr_clk_i_wire     ),
    .pad_main_ddr_link_clk_out_pad ( sl_ddr_clk_o_wire     ),
    .pad_main_ddr_link_in0_pad     ( sl_ddr_data0_i_wire ),
    .pad_main_ddr_link_in1_pad     ( sl_ddr_data1_i_wire ),
    .pad_main_ddr_link_in2_pad     ( sl_ddr_data2_i_wire ),
    .pad_main_ddr_link_in3_pad     ( sl_ddr_data3_i_wire ),
    .pad_main_ddr_link_out0_pad    ( sl_ddr_data0_o_wire ),
    .pad_main_ddr_link_out1_pad    ( sl_ddr_data1_o_wire ),
    .pad_main_ddr_link_out2_pad    ( sl_ddr_data2_o_wire ),
    .pad_main_ddr_link_out3_pad    ( sl_ddr_data3_o_wire ),
    .pad_main_vga_hsync_pad   (    ),
    .pad_main_vga_vsync_pad   (    ),
    .pad_main_vga_red_0_pad   (    ),
    .pad_main_vga_red_1_pad   (    ),
    .pad_main_vga_red_2_pad   (    ),
    .pad_main_vga_green_0_pad (    ),
    .pad_main_vga_green_1_pad (    ),
    .pad_main_vga_green_2_pad (    ),
    .pad_main_vga_blue_0_pad  (    ),
    .pad_main_vga_blue_1_pad  (    ),
    .pad_main_rpc_clk_pad   (      ),
    .pad_main_rpc_clk_n_pad (      ),
    .pad_main_rpc_cs_n_pad  (      ),
    .pad_main_rpc_stb_pad   (      ),
    .pad_main_rpc_dqs_pad   ( rpc_dqs_wire ),
    .pad_main_rpc_dqs_n_pad ( rpc_dqs_n_wire ),
    .pad_main_rpc_db0_pad   ( rpc_db0_wire ),
    .pad_main_rpc_db1_pad   ( rpc_db1_wire ),
    .pad_main_rpc_db2_pad   ( rpc_db2_wire ),
    .pad_main_rpc_db3_pad   ( rpc_db3_wire ),
    .pad_main_rpc_db4_pad   ( rpc_db4_wire ),
    .pad_main_rpc_db5_pad   ( rpc_db5_wire ),
    .pad_main_rpc_db6_pad   ( rpc_db6_wire ),
    .pad_main_rpc_db7_pad   ( rpc_db7_wire ),
    .pad_main_rpc_db8_pad   ( rpc_db8_wire ),
    .pad_main_rpc_db9_pad   ( rpc_db9_wire ),
    .pad_main_rpc_db10_pad  ( rpc_db10_wire ),
    .pad_main_rpc_db11_pad  ( rpc_db11_wire ),
    .pad_main_rpc_db12_pad  ( rpc_db12_wire ),
    .pad_main_rpc_db13_pad  ( rpc_db13_wire ),
    .pad_main_rpc_db14_pad  ( rpc_db14_wire ),
    .pad_main_rpc_db15_pad  ( rpc_db15_wire ),
    // Config Interface
    .config_req_i ( external_reg_req ),
    .config_rsp_o ( external_reg_rsp )
  );

endmodule