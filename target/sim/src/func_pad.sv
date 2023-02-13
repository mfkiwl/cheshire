// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Paul Scheffler <paulsc@iis.ee.ethz.ch>
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>
//
// Functional simulation model for TSMC65 pad
// Models drive strengths for bus conflicts, but not timing

module func_pad #(
  parameter logic PullUpOrDown = 1'b0   // Pull-down pad by default
) (
  inout  wire pad_io,
  input  wire pe_i,
  input  wire ie_i,
  output wire c_o,
  input  wire ds_i,
  input  wire i_i,
  input  wire oen_i
);

  // IF DS=1 and OEN=0: elevated (maximum) drive strength
  assign (supply1, supply0) pad_io = (ds_i == 1 && oen_i == 0) ? i_i : 1'bz;
  // ELSE IF OEN=0: regular drive strength
  assign (strong1, strong0) pad_io = oen_i ? 1'bz : i_i;
  // ELSE IF PE=1: Pull with lowest drive strength
  assign (pull1, pull0)     pad_io = pe_i ? PullUpOrDown : 1'bz;
  // Assign input to bus _anded_ with enable (see pad datasheet); emits X on Z pad
  assign c_o = ie_i & pad_io;

endmodule
