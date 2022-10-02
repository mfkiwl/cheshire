// Generated register defines for i2c

// Copyright information found in source file:
// Copyright lowRISC contributors.

// Licensing information found in source file:
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef _I2C_REG_DEFS_
#define _I2C_REG_DEFS_

#ifdef __cplusplus
extern "C" {
#endif
// Depth of FMT, RX, TX, and ACQ FIFOs
#define I2C_PARAM_FIFO_DEPTH 64

// Number of alerts
#define I2C_PARAM_NUM_ALERTS 1

// Register width
#define I2C_PARAM_REG_WIDTH 32

// Common Interrupt Offsets
#define I2C_INTR_COMMON_FMT_WATERMARK_BIT 0
#define I2C_INTR_COMMON_RX_WATERMARK_BIT 1
#define I2C_INTR_COMMON_FMT_OVERFLOW_BIT 2
#define I2C_INTR_COMMON_RX_OVERFLOW_BIT 3
#define I2C_INTR_COMMON_NAK_BIT 4
#define I2C_INTR_COMMON_SCL_INTERFERENCE_BIT 5
#define I2C_INTR_COMMON_SDA_INTERFERENCE_BIT 6
#define I2C_INTR_COMMON_STRETCH_TIMEOUT_BIT 7
#define I2C_INTR_COMMON_SDA_UNSTABLE_BIT 8
#define I2C_INTR_COMMON_TRANS_COMPLETE_BIT 9
#define I2C_INTR_COMMON_TX_EMPTY_BIT 10
#define I2C_INTR_COMMON_TX_NONEMPTY_BIT 11
#define I2C_INTR_COMMON_TX_OVERFLOW_BIT 12
#define I2C_INTR_COMMON_ACQ_OVERFLOW_BIT 13
#define I2C_INTR_COMMON_ACK_STOP_BIT 14
#define I2C_INTR_COMMON_HOST_TIMEOUT_BIT 15

// Interrupt State Register
#define I2C_INTR_STATE_REG_OFFSET 0x0
#define I2C_INTR_STATE_FMT_WATERMARK_BIT 0
#define I2C_INTR_STATE_RX_WATERMARK_BIT 1
#define I2C_INTR_STATE_FMT_OVERFLOW_BIT 2
#define I2C_INTR_STATE_RX_OVERFLOW_BIT 3
#define I2C_INTR_STATE_NAK_BIT 4
#define I2C_INTR_STATE_SCL_INTERFERENCE_BIT 5
#define I2C_INTR_STATE_SDA_INTERFERENCE_BIT 6
#define I2C_INTR_STATE_STRETCH_TIMEOUT_BIT 7
#define I2C_INTR_STATE_SDA_UNSTABLE_BIT 8
#define I2C_INTR_STATE_TRANS_COMPLETE_BIT 9
#define I2C_INTR_STATE_TX_EMPTY_BIT 10
#define I2C_INTR_STATE_TX_NONEMPTY_BIT 11
#define I2C_INTR_STATE_TX_OVERFLOW_BIT 12
#define I2C_INTR_STATE_ACQ_OVERFLOW_BIT 13
#define I2C_INTR_STATE_ACK_STOP_BIT 14
#define I2C_INTR_STATE_HOST_TIMEOUT_BIT 15

// Interrupt Enable Register
#define I2C_INTR_ENABLE_REG_OFFSET 0x4
#define I2C_INTR_ENABLE_FMT_WATERMARK_BIT 0
#define I2C_INTR_ENABLE_RX_WATERMARK_BIT 1
#define I2C_INTR_ENABLE_FMT_OVERFLOW_BIT 2
#define I2C_INTR_ENABLE_RX_OVERFLOW_BIT 3
#define I2C_INTR_ENABLE_NAK_BIT 4
#define I2C_INTR_ENABLE_SCL_INTERFERENCE_BIT 5
#define I2C_INTR_ENABLE_SDA_INTERFERENCE_BIT 6
#define I2C_INTR_ENABLE_STRETCH_TIMEOUT_BIT 7
#define I2C_INTR_ENABLE_SDA_UNSTABLE_BIT 8
#define I2C_INTR_ENABLE_TRANS_COMPLETE_BIT 9
#define I2C_INTR_ENABLE_TX_EMPTY_BIT 10
#define I2C_INTR_ENABLE_TX_NONEMPTY_BIT 11
#define I2C_INTR_ENABLE_TX_OVERFLOW_BIT 12
#define I2C_INTR_ENABLE_ACQ_OVERFLOW_BIT 13
#define I2C_INTR_ENABLE_ACK_STOP_BIT 14
#define I2C_INTR_ENABLE_HOST_TIMEOUT_BIT 15

// Interrupt Test Register
#define I2C_INTR_TEST_REG_OFFSET 0x8
#define I2C_INTR_TEST_FMT_WATERMARK_BIT 0
#define I2C_INTR_TEST_RX_WATERMARK_BIT 1
#define I2C_INTR_TEST_FMT_OVERFLOW_BIT 2
#define I2C_INTR_TEST_RX_OVERFLOW_BIT 3
#define I2C_INTR_TEST_NAK_BIT 4
#define I2C_INTR_TEST_SCL_INTERFERENCE_BIT 5
#define I2C_INTR_TEST_SDA_INTERFERENCE_BIT 6
#define I2C_INTR_TEST_STRETCH_TIMEOUT_BIT 7
#define I2C_INTR_TEST_SDA_UNSTABLE_BIT 8
#define I2C_INTR_TEST_TRANS_COMPLETE_BIT 9
#define I2C_INTR_TEST_TX_EMPTY_BIT 10
#define I2C_INTR_TEST_TX_NONEMPTY_BIT 11
#define I2C_INTR_TEST_TX_OVERFLOW_BIT 12
#define I2C_INTR_TEST_ACQ_OVERFLOW_BIT 13
#define I2C_INTR_TEST_ACK_STOP_BIT 14
#define I2C_INTR_TEST_HOST_TIMEOUT_BIT 15

// Alert Test Register
#define I2C_ALERT_TEST_REG_OFFSET 0xc
#define I2C_ALERT_TEST_FATAL_FAULT_BIT 0

// I2C control register (Functions TBD)
#define I2C_CTRL_REG_OFFSET 0x10
#define I2C_CTRL_ENABLEHOST_BIT 0
#define I2C_CTRL_ENABLETARGET_BIT 1
#define I2C_CTRL_LLPBK_BIT 2

// I2C live status register
#define I2C_STATUS_REG_OFFSET 0x14
#define I2C_STATUS_FMTFULL_BIT 0
#define I2C_STATUS_RXFULL_BIT 1
#define I2C_STATUS_FMTEMPTY_BIT 2
#define I2C_STATUS_HOSTIDLE_BIT 3
#define I2C_STATUS_TARGETIDLE_BIT 4
#define I2C_STATUS_RXEMPTY_BIT 5
#define I2C_STATUS_TXFULL_BIT 6
#define I2C_STATUS_ACQFULL_BIT 7
#define I2C_STATUS_TXEMPTY_BIT 8
#define I2C_STATUS_ACQEMPTY_BIT 9

// I2C read data
#define I2C_RDATA_REG_OFFSET 0x18
#define I2C_RDATA_RDATA_MASK 0xff
#define I2C_RDATA_RDATA_OFFSET 0
#define I2C_RDATA_RDATA_FIELD \
  ((bitfield_field32_t) { .mask = I2C_RDATA_RDATA_MASK, .index = I2C_RDATA_RDATA_OFFSET })

// I2C Format Data
#define I2C_FDATA_REG_OFFSET 0x1c
#define I2C_FDATA_FBYTE_MASK 0xff
#define I2C_FDATA_FBYTE_OFFSET 0
#define I2C_FDATA_FBYTE_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FDATA_FBYTE_MASK, .index = I2C_FDATA_FBYTE_OFFSET })
#define I2C_FDATA_START_BIT 8
#define I2C_FDATA_STOP_BIT 9
#define I2C_FDATA_READ_BIT 10
#define I2C_FDATA_RCONT_BIT 11
#define I2C_FDATA_NAKOK_BIT 12

// I2C FIFO control register
#define I2C_FIFO_CTRL_REG_OFFSET 0x20
#define I2C_FIFO_CTRL_RXRST_BIT 0
#define I2C_FIFO_CTRL_FMTRST_BIT 1
#define I2C_FIFO_CTRL_RXILVL_MASK 0x7
#define I2C_FIFO_CTRL_RXILVL_OFFSET 2
#define I2C_FIFO_CTRL_RXILVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_CTRL_RXILVL_MASK, .index = I2C_FIFO_CTRL_RXILVL_OFFSET })
#define I2C_FIFO_CTRL_RXILVL_VALUE_RXLVL1 0x0
#define I2C_FIFO_CTRL_RXILVL_VALUE_RXLVL4 0x1
#define I2C_FIFO_CTRL_RXILVL_VALUE_RXLVL8 0x2
#define I2C_FIFO_CTRL_RXILVL_VALUE_RXLVL16 0x3
#define I2C_FIFO_CTRL_RXILVL_VALUE_RXLVL30 0x4
#define I2C_FIFO_CTRL_FMTILVL_MASK 0x3
#define I2C_FIFO_CTRL_FMTILVL_OFFSET 5
#define I2C_FIFO_CTRL_FMTILVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_CTRL_FMTILVL_MASK, .index = I2C_FIFO_CTRL_FMTILVL_OFFSET })
#define I2C_FIFO_CTRL_FMTILVL_VALUE_FMTLVL1 0x0
#define I2C_FIFO_CTRL_FMTILVL_VALUE_FMTLVL4 0x1
#define I2C_FIFO_CTRL_FMTILVL_VALUE_FMTLVL8 0x2
#define I2C_FIFO_CTRL_FMTILVL_VALUE_FMTLVL16 0x3
#define I2C_FIFO_CTRL_ACQRST_BIT 7
#define I2C_FIFO_CTRL_TXRST_BIT 8

// I2C FIFO status register
#define I2C_FIFO_STATUS_REG_OFFSET 0x24
#define I2C_FIFO_STATUS_FMTLVL_MASK 0x7f
#define I2C_FIFO_STATUS_FMTLVL_OFFSET 0
#define I2C_FIFO_STATUS_FMTLVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_STATUS_FMTLVL_MASK, .index = I2C_FIFO_STATUS_FMTLVL_OFFSET })
#define I2C_FIFO_STATUS_TXLVL_MASK 0x7f
#define I2C_FIFO_STATUS_TXLVL_OFFSET 8
#define I2C_FIFO_STATUS_TXLVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_STATUS_TXLVL_MASK, .index = I2C_FIFO_STATUS_TXLVL_OFFSET })
#define I2C_FIFO_STATUS_RXLVL_MASK 0x7f
#define I2C_FIFO_STATUS_RXLVL_OFFSET 16
#define I2C_FIFO_STATUS_RXLVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_STATUS_RXLVL_MASK, .index = I2C_FIFO_STATUS_RXLVL_OFFSET })
#define I2C_FIFO_STATUS_ACQLVL_MASK 0x7f
#define I2C_FIFO_STATUS_ACQLVL_OFFSET 24
#define I2C_FIFO_STATUS_ACQLVL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_FIFO_STATUS_ACQLVL_MASK, .index = I2C_FIFO_STATUS_ACQLVL_OFFSET })

// I2C override control register
#define I2C_OVRD_REG_OFFSET 0x28
#define I2C_OVRD_TXOVRDEN_BIT 0
#define I2C_OVRD_SCLVAL_BIT 1
#define I2C_OVRD_SDAVAL_BIT 2

// Oversampled RX values
#define I2C_VAL_REG_OFFSET 0x2c
#define I2C_VAL_SCL_RX_MASK 0xffff
#define I2C_VAL_SCL_RX_OFFSET 0
#define I2C_VAL_SCL_RX_FIELD \
  ((bitfield_field32_t) { .mask = I2C_VAL_SCL_RX_MASK, .index = I2C_VAL_SCL_RX_OFFSET })
#define I2C_VAL_SDA_RX_MASK 0xffff
#define I2C_VAL_SDA_RX_OFFSET 16
#define I2C_VAL_SDA_RX_FIELD \
  ((bitfield_field32_t) { .mask = I2C_VAL_SDA_RX_MASK, .index = I2C_VAL_SDA_RX_OFFSET })

// Detailed I2C Timings (directly corresponding to table 10 in the I2C
// Specification).
#define I2C_TIMING0_REG_OFFSET 0x30
#define I2C_TIMING0_THIGH_MASK 0xffff
#define I2C_TIMING0_THIGH_OFFSET 0
#define I2C_TIMING0_THIGH_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING0_THIGH_MASK, .index = I2C_TIMING0_THIGH_OFFSET })
#define I2C_TIMING0_TLOW_MASK 0xffff
#define I2C_TIMING0_TLOW_OFFSET 16
#define I2C_TIMING0_TLOW_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING0_TLOW_MASK, .index = I2C_TIMING0_TLOW_OFFSET })

// Detailed I2C Timings (directly corresponding to table 10 in the I2C
// Specification).
#define I2C_TIMING1_REG_OFFSET 0x34
#define I2C_TIMING1_T_R_MASK 0xffff
#define I2C_TIMING1_T_R_OFFSET 0
#define I2C_TIMING1_T_R_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING1_T_R_MASK, .index = I2C_TIMING1_T_R_OFFSET })
#define I2C_TIMING1_T_F_MASK 0xffff
#define I2C_TIMING1_T_F_OFFSET 16
#define I2C_TIMING1_T_F_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING1_T_F_MASK, .index = I2C_TIMING1_T_F_OFFSET })

// Detailed I2C Timings (directly corresponding to table 10 in the I2C
// Specification).
#define I2C_TIMING2_REG_OFFSET 0x38
#define I2C_TIMING2_TSU_STA_MASK 0xffff
#define I2C_TIMING2_TSU_STA_OFFSET 0
#define I2C_TIMING2_TSU_STA_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING2_TSU_STA_MASK, .index = I2C_TIMING2_TSU_STA_OFFSET })
#define I2C_TIMING2_THD_STA_MASK 0xffff
#define I2C_TIMING2_THD_STA_OFFSET 16
#define I2C_TIMING2_THD_STA_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING2_THD_STA_MASK, .index = I2C_TIMING2_THD_STA_OFFSET })

// Detailed I2C Timings (directly corresponding to table 10, in the I2C
// Specification).
#define I2C_TIMING3_REG_OFFSET 0x3c
#define I2C_TIMING3_TSU_DAT_MASK 0xffff
#define I2C_TIMING3_TSU_DAT_OFFSET 0
#define I2C_TIMING3_TSU_DAT_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING3_TSU_DAT_MASK, .index = I2C_TIMING3_TSU_DAT_OFFSET })
#define I2C_TIMING3_THD_DAT_MASK 0xffff
#define I2C_TIMING3_THD_DAT_OFFSET 16
#define I2C_TIMING3_THD_DAT_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING3_THD_DAT_MASK, .index = I2C_TIMING3_THD_DAT_OFFSET })

// Detailed I2C Timings (directly corresponding to table 10, in the I2C
// Specification).
#define I2C_TIMING4_REG_OFFSET 0x40
#define I2C_TIMING4_TSU_STO_MASK 0xffff
#define I2C_TIMING4_TSU_STO_OFFSET 0
#define I2C_TIMING4_TSU_STO_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING4_TSU_STO_MASK, .index = I2C_TIMING4_TSU_STO_OFFSET })
#define I2C_TIMING4_T_BUF_MASK 0xffff
#define I2C_TIMING4_T_BUF_OFFSET 16
#define I2C_TIMING4_T_BUF_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMING4_T_BUF_MASK, .index = I2C_TIMING4_T_BUF_OFFSET })

// I2C clock stretching timeout control
#define I2C_TIMEOUT_CTRL_REG_OFFSET 0x44
#define I2C_TIMEOUT_CTRL_VAL_MASK 0x7fffffff
#define I2C_TIMEOUT_CTRL_VAL_OFFSET 0
#define I2C_TIMEOUT_CTRL_VAL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TIMEOUT_CTRL_VAL_MASK, .index = I2C_TIMEOUT_CTRL_VAL_OFFSET })
#define I2C_TIMEOUT_CTRL_EN_BIT 31

// I2C target address and mask pairs
#define I2C_TARGET_ID_REG_OFFSET 0x48
#define I2C_TARGET_ID_ADDRESS0_MASK 0x7f
#define I2C_TARGET_ID_ADDRESS0_OFFSET 0
#define I2C_TARGET_ID_ADDRESS0_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TARGET_ID_ADDRESS0_MASK, .index = I2C_TARGET_ID_ADDRESS0_OFFSET })
#define I2C_TARGET_ID_MASK0_MASK 0x7f
#define I2C_TARGET_ID_MASK0_OFFSET 7
#define I2C_TARGET_ID_MASK0_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TARGET_ID_MASK0_MASK, .index = I2C_TARGET_ID_MASK0_OFFSET })
#define I2C_TARGET_ID_ADDRESS1_MASK 0x7f
#define I2C_TARGET_ID_ADDRESS1_OFFSET 14
#define I2C_TARGET_ID_ADDRESS1_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TARGET_ID_ADDRESS1_MASK, .index = I2C_TARGET_ID_ADDRESS1_OFFSET })
#define I2C_TARGET_ID_MASK1_MASK 0x7f
#define I2C_TARGET_ID_MASK1_OFFSET 21
#define I2C_TARGET_ID_MASK1_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TARGET_ID_MASK1_MASK, .index = I2C_TARGET_ID_MASK1_OFFSET })

// I2C target acquired data
#define I2C_ACQDATA_REG_OFFSET 0x4c
#define I2C_ACQDATA_ABYTE_MASK 0xff
#define I2C_ACQDATA_ABYTE_OFFSET 0
#define I2C_ACQDATA_ABYTE_FIELD \
  ((bitfield_field32_t) { .mask = I2C_ACQDATA_ABYTE_MASK, .index = I2C_ACQDATA_ABYTE_OFFSET })
#define I2C_ACQDATA_SIGNAL_MASK 0x3
#define I2C_ACQDATA_SIGNAL_OFFSET 8
#define I2C_ACQDATA_SIGNAL_FIELD \
  ((bitfield_field32_t) { .mask = I2C_ACQDATA_SIGNAL_MASK, .index = I2C_ACQDATA_SIGNAL_OFFSET })

// I2C target transmit data
#define I2C_TXDATA_REG_OFFSET 0x50
#define I2C_TXDATA_TXDATA_MASK 0xff
#define I2C_TXDATA_TXDATA_OFFSET 0
#define I2C_TXDATA_TXDATA_FIELD \
  ((bitfield_field32_t) { .mask = I2C_TXDATA_TXDATA_MASK, .index = I2C_TXDATA_TXDATA_OFFSET })

// I2C target clock stretching control
#define I2C_STRETCH_CTRL_REG_OFFSET 0x54
#define I2C_STRETCH_CTRL_EN_ADDR_TX_BIT 0
#define I2C_STRETCH_CTRL_EN_ADDR_ACQ_BIT 1
#define I2C_STRETCH_CTRL_STOP_TX_BIT 2
#define I2C_STRETCH_CTRL_STOP_ACQ_BIT 3

// I2C host clock generation timeout value (in units of input clock
// frequency)
#define I2C_HOST_TIMEOUT_CTRL_REG_OFFSET 0x58

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // _I2C_REG_DEFS_
// End generated register defines for i2c