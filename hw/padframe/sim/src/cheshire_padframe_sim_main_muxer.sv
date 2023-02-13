
// File auto-generated by Padrick 0.3.6
module cheshire_padframe_sim_main_muxer
  import pkg_internal_cheshire_padframe_sim_main::*;
  import pkg_cheshire_padframe_sim::*;
  import cheshire_padframe_sim_main_config_reg_pkg::*;
#(
  parameter type              req_t  = logic, // reg_interface request type
  parameter type             resp_t  = logic // reg_interface response type
) (
  input logic clk_i,
  input logic rst_ni,
  // Configuration interface using register_interface protocol
  input req_t config_req_i,
  output resp_t config_rsp_o
);
   // Connections between register file and pads

  // Register File Instantiation
  cheshire_padframe_sim_main_config_reg_top #(
    .reg_req_t(req_t),
    .reg_rsp_t(resp_t)
    ) i_regfile (
    .clk_i,
    .rst_ni,
    .reg_req_i(config_req_i),
    .reg_rsp_o(config_rsp_o),
    .devmode_i(1'b1)
  );


   // SoC -> Pad Multiplex Logic

  // Pad -> SoC Multiplex Logic
endmodule : cheshire_padframe_sim_main_muxer
