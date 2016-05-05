/**
 * lan8720a.h - Header file with description of vendor-specific registers in
 *              LAN8720A/LAN8720Ai chips (basic and extended registers are
 *              the same as in DP83848x chips)
 * 2016/05, Roman Bartosinski <bartosr@centrum.cz>
 *
 */

#ifndef LAN8720A_PHY_HEADER_FILE
#define LAN8720A_PHY_HEADER_FILE

/*** Registers ***/
/* LAN8720A PHY Vendor-Specific Registers */
#define LAN8720A_REG_MCSR       0x11      /* Mode Control/Status Register */
#define LAN8720A_REG_SPMR       0x12      /* Special Modes Register */
#define LAN8720A_REG_SECR       0x1A      /* Symbol Error Counter Register */
#define LAN8720A_REG_CSIR       0x1B      /* Control/Status Indication Register */
#define LAN8720A_REG_IRSR       0x1D      /* Interrupt Source Register */
#define LAN8720A_REG_IRMR       0x1E      /* Interrupt Mask Register */
#define LAN8720A_REG_PCSR       0x1F      /* PHY Special Control/Status Register */

/* Mode Control/Status Register */
#define LAN8720A_MCSR_EDPWRDOWN       (1<<13)     /* R/W - Enable the Energy Detect Power-Down mode: 0=disabled, 1=enabled */
#define LAN8720A_MCSR_FARLOOPBACK     (1<<9)      /* R/W - Enables far loopback mode: 0=disabled, 1=enabled */
#define LAN8720A_MCSR_ALTINT          (1<<6)      /* R/W - Alternate Interrupt Mode: 0=primary int.system, 1=Alternate int.system */
#define LAN8720A_MCSR_ENERGYON        (1<<1)      /* RO  - Indicates whether energy is detected */

/* Special Modes Register */
#define LAN8720A_SPMR_MODE            (0x7<<5)    /* R/W - Transceiver mode of operation */
#define LAN8720A_SPMR_PHYAD           (0x1F<<0)   /* R/W - PHY Address. The PHY Address is used for the SMI address and for initialization of the Cipher (Scrambler) key. */
  /* modes are default values for */
#define LAN8720A_SPMR_MODE_H10M       (0<<5)    /* 10Base-T Half Duplex. Auto-negotiation disabled. */
#define LAN8720A_SPMR_MODE_F10M       (1<<5)    /* 10Base-T Full Duplex. Auto-negotiation disabled. */
#define LAN8720A_SPMR_MODE_H100M      (2<<5)    /* 100Base-TX Half Duplex. Auto-negotiation disabled. */
#define LAN8720A_SPMR_MODE_F100M      (3<<5)    /* 100Base-TX Full Duplex is advertised. Auto-negotiation disabled. */
#define LAN8720A_SPMR_MODE_HA100M     (4<<5)    /* 100Base-TX Half Duplex is advertised. Auto-negotiation enabled. */
#define LAN8720A_SPMR_MODE_REPEAT     (5<<5)    /* Repeater mode. Auto-negotiation enabled. 100Base-TX Half Duplex is advertised. */
#define LAN8720A_SPMR_MODE_PWRDWN     (6<<5)    /* Power Down mode. In this mode the transceiver will wake-up in Power-Down mode. */
#define LAN8720A_SPMR_MODE_AUTO       (7<<5)    /* All capable. Auto-negotiation enabled. */

/* Symbol Error Counter Register */
#define LAN8720A_SECR_SYMERRCNT_MASK  (0xFFFF)  /* RO  - The symbol error counter increments whenever an invalid code symbol is received (including IDLE symbols) in 100BASE-TX mode. */

/* Control/Status Indication Register */
#define LAN8720A_CSIR_AMDIXCTRL       (1<<15)   /* R/W - HP Auto-MDIX control: 0=Enable Auto-MDIX, 1=Disable Auto-MDIX */
#define LAN8720A_CSIR_CH_SELECT       (1<<13)   /* R/W - Manual channel select: 0=MDI (TX transmits,RX receives), 1=MDIX (TX receives,RX transmits) */
#define LAN8720A_CSIR_SQEOFF          (1<<11)   /* R/W - Disable the SQE test (Heartbeat): 0=SQE test is enabled, 1=SQE test is disabled */
#define LAN8720A_CSIR_XPOL            (1<<4)    /* R/W - Polarity state of the 10BASE-T: 0=Normal polarity, 1=Reversed polarity */

/* Interrupt Source Register */
#define LAN8720A_IRSR_INT7            (1<<7)    /* RO/LH - 1=ENERGYON generated */
#define LAN8720A_IRSR_INT6            (1<<6)    /* RO/LH - 1=Auto-Negotiation complete */
#define LAN8720A_IRSR_INT5            (1<<5)    /* RO/LH - 1=Remote Fault Detected */
#define LAN8720A_IRSR_INT4            (1<<4)    /* RO/LH - 1=Link Down (link status negated) */
#define LAN8720A_IRSR_INT3            (1<<3)    /* RO/LH - 1=Auto-Negotiation LP Acknowledge */
#define LAN8720A_IRSR_INT2            (1<<2)    /* RO/LH - 1=Parallel Detection Fault */
#define LAN8720A_IRSR_INT1            (1<<1)    /* RO/LH - 1=Auto-Negotiation Page Received */

/* Interrupt Mask Register */
#define LAN8720A_IRMR_INT7            (1<<7)    /* R/W - INT7 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT6            (1<<6)    /* R/W - INT6 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT5            (1<<5)    /* R/W - INT5 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT4            (1<<4)    /* R/W - INT4 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT3            (1<<3)    /* R/W - INT3 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT2            (1<<2)    /* R/W - INT2 mask bit, 1=enabled */
#define LAN8720A_IRMR_INT1            (1<<1)    /* R/W - INT1 mask bit, 1=enabled */

/* PHY Special Control/Status Register */
#define LAN8720A_PCSR_AUTODONE        (1<<12)   /* RO - Auto-negotiation done indication: 0=not done or disabled (or not active), 1=done */
//#define LAN8720A_PCSR_SPEED           (7<<2)    /* RO - Speed Indication */
#define LAN8720A_PCSR_DUPLEX          (1<<4)    /* RO - Speed indication: 0=half duplex, 1=full duplex */
#define LAN8720A_PCSR_SPEED_100M      (1<<3)    /* RO - Speed indication: 1=100M */
#define LAN8720A_PCSR_SPEED_10M       (1<<2)    /* RO - Speed indication: 1=10M */



#define LAN8720A_DEF_ID     0x0007c0f1  /* PHY Identifier (PHYIDR1,PHYIDR2 regs.) */


#endif /* LAN8720A_PHY_HEADER_FILE */
