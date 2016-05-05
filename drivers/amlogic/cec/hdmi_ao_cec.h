#ifndef __AO_CEC_H__
#define __AO_CEC_H__

#define AO_BASE				0xc8100000

#define AO_CEC_GEN_CNTL			((0x40 << 2))
#define AO_CEC_RW_REG			((0x41 << 2))
#define AO_CEC_INTR_MASKN		((0x42 << 2))
#define AO_CEC_INTR_CLR			((0x43 << 2))
#define AO_CEC_INTR_STAT		((0x44 << 2))

#define AO_CRT_CLK_CNTL1		((0x1a << 2))

#define AO_DEBUG_REG0			((0x28 << 2))
#define AO_DEBUG_REG1			((0x29 << 2))
#define AO_DEBUG_REG2			((0x2a << 2))
#define AO_DEBUG_REG3			((0x2b << 2))

/* read/write */
#define CEC_TX_MSG_0_HEADER        0x00
#define CEC_TX_MSG_1_OPCODE        0x01
#define CEC_TX_MSG_2_OP1           0x02
#define CEC_TX_MSG_3_OP2           0x03
#define CEC_TX_MSG_4_OP3           0x04
#define CEC_TX_MSG_5_OP4           0x05
#define CEC_TX_MSG_6_OP5           0x06
#define CEC_TX_MSG_7_OP6           0x07
#define CEC_TX_MSG_8_OP7           0x08
#define CEC_TX_MSG_9_OP8           0x09
#define CEC_TX_MSG_A_OP9           0x0A
#define CEC_TX_MSG_B_OP10          0x0B
#define CEC_TX_MSG_C_OP11          0x0C
#define CEC_TX_MSG_D_OP12          0x0D
#define CEC_TX_MSG_E_OP13          0x0E
#define CEC_TX_MSG_F_OP14          0x0F

/* read/write */
#define CEC_TX_MSG_LENGTH          0x10
#define CEC_TX_MSG_CMD             0x11
#define CEC_TX_WRITE_BUF           0x12
#define CEC_TX_CLEAR_BUF           0x13
#define CEC_RX_MSG_CMD             0x14
#define CEC_RX_CLEAR_BUF           0x15
#define CEC_LOGICAL_ADDR0          0x16
#define CEC_LOGICAL_ADDR1          0x17
#define CEC_LOGICAL_ADDR2          0x18
#define CEC_LOGICAL_ADDR3          0x19
#define CEC_LOGICAL_ADDR4          0x1A
#define CEC_CLOCK_DIV_H            0x1B
#define CEC_CLOCK_DIV_L            0x1C

/* The following registers are for fine tuning CEC bit timing parameters.
 * They are only valid in AO CEC, NOT valid in HDMITX CEC.
 * The AO CEC's timing parameters are already set default to work with
 * 32768Hz clock, so hopefully SW never need to program these registers.
 * The timing registers are made programmable just in case. */
#define AO_CEC_QUIESCENT_25MS_BIT7_0            0x20
#define AO_CEC_QUIESCENT_25MS_BIT11_8           0x21
#define AO_CEC_STARTBITMINL2H_3MS5_BIT7_0       0x22
#define AO_CEC_STARTBITMINL2H_3MS5_BIT8         0x23
#define AO_CEC_STARTBITMAXL2H_3MS9_BIT7_0       0x24
#define AO_CEC_STARTBITMAXL2H_3MS9_BIT8         0x25
#define AO_CEC_STARTBITMINH_0MS6_BIT7_0         0x26
#define AO_CEC_STARTBITMINH_0MS6_BIT8           0x27
#define AO_CEC_STARTBITMAXH_1MS0_BIT7_0         0x28
#define AO_CEC_STARTBITMAXH_1MS0_BIT8           0x29
#define AO_CEC_STARTBITMINTOTAL_4MS3_BIT7_0     0x2A
#define AO_CEC_STARTBITMINTOTAL_4MS3_BIT9_8     0x2B
#define AO_CEC_STARTBITMAXTOTAL_4MS7_BIT7_0     0x2C
#define AO_CEC_STARTBITMAXTOTAL_4MS7_BIT9_8     0x2D
#define AO_CEC_LOGIC1MINL2H_0MS4_BIT7_0         0x2E
#define AO_CEC_LOGIC1MINL2H_0MS4_BIT8           0x2F
#define AO_CEC_LOGIC1MAXL2H_0MS8_BIT7_0         0x30
#define AO_CEC_LOGIC1MAXL2H_0MS8_BIT8           0x31
#define AO_CEC_LOGIC0MINL2H_1MS3_BIT7_0         0x32
#define AO_CEC_LOGIC0MINL2H_1MS3_BIT8           0x33
#define AO_CEC_LOGIC0MAXL2H_1MS7_BIT7_0         0x34
#define AO_CEC_LOGIC0MAXL2H_1MS7_BIT8           0x35
#define AO_CEC_LOGICMINTOTAL_2MS05_BIT7_0       0x36
#define AO_CEC_LOGICMINTOTAL_2MS05_BIT9_8       0x37
#define AO_CEC_LOGICMAXHIGH_2MS8_BIT7_0         0x38
#define AO_CEC_LOGICMAXHIGH_2MS8_BIT8           0x39
#define AO_CEC_LOGICERRLOW_3MS4_BIT7_0          0x3A
#define AO_CEC_LOGICERRLOW_3MS4_BIT8            0x3B
#define AO_CEC_NOMSMPPOINT_1MS05                0x3C
#define AO_CEC_DELCNTR_LOGICERR                 0x3E
#define AO_CEC_TXTIME_17MS_BIT7_0               0x40
#define AO_CEC_TXTIME_17MS_BIT10_8              0x41
#define AO_CEC_TXTIME_2BIT_BIT7_0               0x42
#define AO_CEC_TXTIME_2BIT_BIT10_8              0x43
#define AO_CEC_TXTIME_4BIT_BIT7_0               0x44
#define AO_CEC_TXTIME_4BIT_BIT10_8              0x45
#define AO_CEC_STARTBITNOML2H_3MS7_BIT7_0       0x46
#define AO_CEC_STARTBITNOML2H_3MS7_BIT8         0x47
#define AO_CEC_STARTBITNOMH_0MS8_BIT7_0         0x48
#define AO_CEC_STARTBITNOMH_0MS8_BIT8           0x49
#define AO_CEC_LOGIC1NOML2H_0MS6_BIT7_0         0x4A
#define AO_CEC_LOGIC1NOML2H_0MS6_BIT8           0x4B
#define AO_CEC_LOGIC0NOML2H_1MS5_BIT7_0         0x4C
#define AO_CEC_LOGIC0NOML2H_1MS5_BIT8           0x4D
#define AO_CEC_LOGIC1NOMH_1MS8_BIT7_0           0x4E
#define AO_CEC_LOGIC1NOMH_1MS8_BIT8             0x4F
#define AO_CEC_LOGIC0NOMH_0MS9_BIT7_0           0x50
#define AO_CEC_LOGIC0NOMH_0MS9_BIT8             0x51
#define AO_CEC_LOGICERRLOW_3MS6_BIT7_0          0x52
#define AO_CEC_LOGICERRLOW_3MS6_BIT8            0x53
#define AO_CEC_CHKCONTENTION_0MS1               0x54
#define AO_CEC_PREPARENXTBIT_0MS05_BIT7_0       0x56
#define AO_CEC_PREPARENXTBIT_0MS05_BIT8         0x57
#define AO_CEC_NOMSMPACKPOINT_0MS45             0x58
#define AO_CEC_ACK0NOML2H_1MS5_BIT7_0           0x5A
#define AO_CEC_ACK0NOML2H_1MS5_BIT8             0x5B

#define AO_CEC_BUGFIX_DISABLE_0                 0x60
#define AO_CEC_BUGFIX_DISABLE_1                 0x61

/* read only */
#define CEC_RX_MSG_0_HEADER        0x80
#define CEC_RX_MSG_1_OPCODE        0x81
#define CEC_RX_MSG_2_OP1           0x82
#define CEC_RX_MSG_3_OP2           0x83
#define CEC_RX_MSG_4_OP3           0x84
#define CEC_RX_MSG_5_OP4           0x85
#define CEC_RX_MSG_6_OP5           0x86
#define CEC_RX_MSG_7_OP6           0x87
#define CEC_RX_MSG_8_OP7           0x88
#define CEC_RX_MSG_9_OP8           0x89
#define CEC_RX_MSG_A_OP9           0x8A
#define CEC_RX_MSG_B_OP10          0x8B
#define CEC_RX_MSG_C_OP11          0x8C
#define CEC_RX_MSG_D_OP12          0x8D
#define CEC_RX_MSG_E_OP13          0x8E
#define CEC_RX_MSG_F_OP14          0x8F

/* read only */
#define CEC_RX_MSG_LENGTH          0x90
#define CEC_RX_MSG_STATUS          0x91
#define CEC_RX_NUM_MSG             0x92
#define CEC_TX_MSG_STATUS          0x93
#define CEC_TX_NUM_MSG             0x94

/* tx_msg_cmd definition */
#define TX_NO_OP                0  /* No transaction */
#define TX_REQ_CURRENT          1  /* Transmit earliest message in buffer */
#define TX_ABORT                2  /* Abort transmitting earliest message */
/* Overwrite earliest message in buffer and transmit next message */
#define TX_REQ_NEXT             3

/* tx_msg_status definition */
#define TX_IDLE                 0  /* No transaction */
#define TX_BUSY                 1  /* Transmitter is busy */
/* Message has been successfully transmitted */
#define TX_DONE                 2
#define TX_ERROR                3  /* Message has been transmitted with error */

/* rx_msg_cmd */
#define RX_NO_OP                0  /* No transaction */
#define RX_ACK_CURRENT          1  /* Read earliest message in buffer */
#define RX_DISABLE              2  /* Disable receiving latest message */
/* Clear earliest message from buffer and read next message */
#define RX_ACK_NEXT             3

/* rx_msg_status */
#define RX_IDLE                 0  /* No transaction */
#define RX_BUSY                 1  /* Receiver is busy */
#define RX_DONE                 2  /* Message has been received successfully */
#define RX_ERROR                3  /* Message has been received with error */

#ifdef CONFIG_TVIN_HDMI
extern unsigned long hdmirx_rd_top(unsigned long addr);
extern void hdmirx_wr_top(unsigned long addr, unsigned long data);
#define TOP_HPD_PWR5V           0x002
#define TOP_ARCTX_CNTL          0x010
#endif

unsigned int aocec_rd_reg(unsigned long addr);
void aocec_wr_reg(unsigned long addr, unsigned long data);

#endif	/* __AO_CEC_H__ */
