/**
 * dp83848x_phy.h - Header file with description of DP83848C/DP83848I/DP83848VYB/DP83848YB
 * (C)2015, Roman Bartosinski <bartosr@centrum.cz>
 *
 */

#ifndef DP83848X_PHY_HEADER_FILE
#define DP83848X_PHY_HEADER_FILE


/*** Registers ***/
/* DP83848C PHY Registers */
#define DP83848X_REG_BMCR         0x00      /* Basic Mode Control Register */
#define DP83848X_REG_BMSR         0x01      /* Basic Mode Status Register  */
#define DP83848X_REG_PHYIDR1      0x02      /* PHY Identifier 1 */
#define DP83848X_REG_PHYIDR2      0x03      /* PHY Identifier 2 */
#define DP83848X_REG_ANAR         0x04      /* Auto-Negotiation Advertisement */
#define DP83848X_REG_ANLPAR       0x05      /* Auto-Neg. Link Partner Abitily */
#define DP83848X_REG_ANER         0x06      /* Auto-Neg. Expansion Register */
#define DP83848X_REG_ANNPTR       0x07      /* Auto-Neg. Next Page TX */

/* PHY Extended Registers */
#define DP83848X_REG_PHYSTS       0x10      /* Status Register */
#define DP83848X_REG_MICR         0x11      /* MII Interrupt Control Register */
#define DP83848X_REG_MISR         0x12      /* MII Interrupt Status Register */
#define DP83848X_REG_FCSCR        0x14      /* False Carrier Sense Counter */
#define DP83848X_REG_RECR         0x15      /* Receive Error Counter */
#define DP83848X_REG_PCSR         0x16      /* PCS Sublayer Config. and Status */
#define DP83848X_REG_RBR          0x17      /* RMII and Bypass Register */
#define DP83848X_REG_LEDCR        0x18      /* LED Direct Control Register */
#define DP83848X_REG_PHYCR        0x19      /* PHY Control Register */
#define DP83848X_REG_10BTSCR      0x1A      /* 10Base-T Status/Control Register */
#define DP83848X_REG_CDCTRL1      0x1B      /* CD Test Control and BIST Extensions */
#define DP83848X_REG_EDCR         0x1D      /* Energy Detect Control Register */

/* -------------------------------------------------------------------------- */
/*** PHY Basic Registers ***/
/* Basic Mode Control Register */
#define DP83848X_BMCR_RESET       (1<<15)   /* Initiate Software Reset */
#define DP83848X_BMCR_LOOPBACK    (1<<14)   /* Loopback */
#define DP83848X_BMCR_SPEED_100M  (1<<13)   /* Speed selection: 1=100Mbps, 0=10Mbps */
#define DP83848X_BMCR_ENABLE_AN   (1<<12)   /* Auto-Negotiation Enable */
#define DP83848X_BMCR_POWER_DOWN  (1<<11)   /* Power Down */
#define DP83848X_BMCR_ISOLATE     (1<<10)   /* Isolate Port from MII */
#define DP83848X_BMCR_RESTART_AN  (1<<9)    /* Restart Auto-Negotiation */
#define DP83848X_BMCR_FULLDUPLEX  (1<<8)    /* Duplex Mode: 1=Full Duplex, 0=Half Duplex */
#define DP83848X_BMCR_COLL_TEST   (1<<7)    /* Collision test */

/* Basic Mode Status Register */
#define DP83848X_BMSR_100BASE_T4      (1<<15)   /* Device able to perform 100BASE-T4 mode. */
#define DP83848X_BMSR_100BASE_TX_FD   (1<<14)   /* Device able to perform 100BASE-TX in full duplex mode. */
#define DP83848X_BMSR_100BASE_TX_HD   (1<<13)   /* Device able to perform 100BASE-TX in half duplex mode. */
#define DP83848X_BMSR_100BASE_T_FD    (1<<12)   /* Device able to perform 10BASE-T in full duplex mode. */
#define DP83848X_BMSR_100BASE_T_HD    (1<<11)   /* Device able to perform 10BASE-T in half duplex mode. */
#define DP83848X_BMSR_MFPREA_SUPPRESS (1<<6)    /* Device able to perform management transaction with preamble suppressed, 32-bits of preamble needed only once after reset, invalid opcode or invalid turnaround. */
#define DP83848X_BMSR_AN_COMPLETE     (1<<5)    /* Auto-Negotiation process complete. */
#define DP83848X_BMSR_REMOTE_FAULT    (1<<4)    /* Remote Fault condition detected (cleared on read or by reset). Fault criteria: NOTIFICATION from Link Partner of Remote Fault. */
#define DP83848X_BMSR_AN_ABILITY      (1<<3)    /* Device is able to perform Auto-Negotiation. */
#define DP83848X_BMSR_LINK_STATUS     (1<<2)    /* Valid link established (for either 10 or 100 Mb/s operation). */
#define DP83848X_BMSR_JABBER_DETECT   (1<<1)    /* Jabber condition detected. */
#define DP83848X_BMSR_EXTENDED_CAP    (1<<0)    /* Extended register capabilities. */

/* PHY Identifier Register #1 */
#define DP83848X_PHYIDR1_OUI_MSB_MASK (0xFFFF)  /* OUI Most Significant Bits: Bits 3 to 18 of the OUI (080017h) are stored in bits 15 to 0 of this register. The most significant two bits of the OUI are ignored (the IEEE standard refers to these as bits 1 and 2). */

/* PHY Identifier Register #2 */
#define DP83848X_PHYIDR2_OUI_LSB_MASK  (0xFC00)  /* OUI Least Significant Bits: Bits 19 to 24 of the OUI (080017h) are mapped from bits 15 to 10 of this register respectively. */
#define DP83848X_PHYIDR2_VNDR_MDL_MASK (0x03F0)  /* Vendor Model Number: The six bits of vendor model number are mapped from bits 9 to 4 (most significant bit to bit 9). */
#define DP83848X_PHYIDR2_MDL_REV_MASK  (0x000F)  /* Model Revision Number: Four bits of the vendor model revision number are mapped from bits 3 to 0 (most significant bit to bit 3). This field will be incremented for all major device changes. */

/* Auto-Negotiation Advertisement Register */
#define DP83848X_ANAR_NP              (1<<15)   /* Next Page Transfer desired. */
#define DP83848X_ANAR_RF              (1<<13)   /* Advertises that this device has detected a Remote Fault. */
#define DP83848X_ANAR_ASM_DIR         (1<<11)   /* Asymmetric PAUSE Support for Full Duplex Links: The ASM_DIR bit indicates that asymmetric PAUSE is supported. */
#define DP83848X_ANAR_PAUSE           (1<<10)   /* The PAUSE bit indicates that the device is capable of providing the symmetric PAUSE functions as defined in Annex 31B. */
#define DP83848X_ANAR_T4              (1<<9)    /* 100BASE-T4 is supported by the local device. */
#define DP83848X_ANAR_TX_FD           (1<<8)    /* 100BASE-TX Full Duplex is supported by the local device. */
#define DP83848X_ANAR_TX              (1<<7)    /* 100BASE-TX is supported by the local device. */
#define DP83848X_ANAR_10_FD           (1<<6)    /* 10BASE-T Full Duplex is supported by the local device. */
#define DP83848X_ANAR_10              (1<<5)    /* 10BASE-T is supported by the local device. */
#define DP83848X_ANAR_SELECTOR_MASK   (0x001F)  /* Protocol Selection Bits: */

/* Auto-Negotiation Link Partner Ability Register (BASE page) */
#define DP83848X_ANLPAR_NP            (1<<15)   /* Next Page Indication: Link Partner desires Next Page Transfer. */
#define DP83848X_ANLPAR_ACK           (1<<14)   /* Acknowledge: Link Partner acknowledges reception of the ability data word. */
#define DP83848X_ANLPAR_RF            (1<<13)   /* Remote Fault: Remote Fault indicated by Link Partner. */
#define DP83848X_ANLPAR_ASM_DIR       (1<<11)   /* ASYMMETRIC PAUSE: Asymmetric pause is supported by the Link Partner. */
#define DP83848X_ANLPAR_PAUSE         (1<<10)   /* Pause function is supported by the Link Partner. */
#define DP83848X_ANLPAR_T4            (1<<9)    /* 100BASE-T4 is supported by the Link Partner. */
#define DP83848X_ANLPAR_TX_FD         (1<<8)    /* 100BASE-TX Full Duplex is supported by the Link Partner. */
#define DP83848X_ANLPAR_TX            (1<<7)    /* 100BASE-TX is supported by the Link Partner. */
#define DP83848X_ANLPAR_10_FD         (1<<6)    /* 10BASE-T Full Duplex is supported by the Link Partner. */
#define DP83848X_ANLPAR_10            (1<<5)    /* 10BASE-T is supported by the Link Partner. */
#define DP83848X_ANLPAR_SELECTOR_MASK (0x001F)  /* Link Partners binary encoded protocol selector. */

/* Auto-Negotiation Link Partner Ability Register (Next page) */
#define DP83848X_ANLPAR2_NP           (1<<15)   /* Next Page Indication: Link Partner desires Next Page Transfer. */
#define DP83848X_ANLPAR2_ACK          (1<<14)   /* Acknowledge: Link Partner acknowledges reception of the ability data word. */
#define DP83848X_ANLPAR2_MP           (1<<13)   /* Message Page. */
#define DP83848X_ANLPAR2_ACK2         (1<<12)   /* Acknowledge 2: Link Partner does have the ability to comply to next page message. */
#define DP83848X_ANLPAR2_TOGGLE       (1<<11)   /* Toggle: Previous value of the transmitted Link Code word equaled 0. */
#define DP83848X_ANLPAR2_CODE_MASK    (0x07FF)  /* Code: This field represents the code field of the next page transmission. */

/* Auto-Negotiate Expansion Register */
#define DP83848X_ANER_PDF             (1<<4)    /* Parallel Detection Fault:A fault has been detected through the Parallel Detection function. */
#define DP83848X_ANER_LP_NP_ABLE      (1<<3)    /* Link Partner Next Page Able: Link Partner does support Next Page. */
#define DP83848X_ANER_NP_ABLE         (1<<2)    /* Next Page Able: Indicates local device is able to send additional Next Pages. */
#define DP83848X_ANER_PAGE_RX         (1<<1)    /* Link Code Word Page Received: Link Code Word has been received, cleared on a read. */
#define DP83848X_ANER_LP_AN_ABLE      (1<<0)    /* Link Partner Auto-Negotiation Able: indicates that the Link Partner supports Auto-Negotiation. */

/* Auto-Negotiation Next Page Transmit Register */
#define DP83848X_ANNPTR_NP            (1<<15)   /* Another Next Page desired. */
#define DP83848X_ANNPTR_MP            (1<<13)   /* Message Page */
#define DP83848X_ANNPTR_ACK2          (1<<12)   /* Acknowledge2: Will comply with message. */
#define DP83848X_ANNPTR_TOG_TX        (1<<11)   /* Toggle: Value of toggle bit in previously transmitted Link Code Word was 0. */
#define DP83848X_ANNPTR_CODE_MASK     (0x07FF)  /* Code: This field represents the code field of the next page transmission. */

/* PHY Extended Registers */
/* PHY Status Register */
#define DP83848X_PHYSTS_MDIX_MODE     (1<<14)   /* MDI pairs swapped */
#define DP83848X_PHYSTS_RCV_ERROR     (1<<13)   /* Receive error event has occurred since last read of RXERCNT */
#define DP83848X_PHYSTS_POLARITY_STAT (1<<12)   /* Polarity Status: Inverted Polarity detected. */
#define DP83848X_PHYSTS_FALSE_CARRIER (1<<11)   /* False Carrier Sense Latch: False Carrier event has occurred since last read of FCSCR (address 14h). */
#define DP83848X_PHYSTS_SIGNAL_DETECT (1<<10)   /* 100Base-TX qualified Signal Detect from PMA */
#define DP83848X_PHYSTS_DESCRAMB_LOCK (1<<9)    /* 100Base-TX Descrambler Lock from PMD. */
#define DP83848X_PHYSTS_PAGE_RECEIVED (1<<8)    /* Link Code Word Page Received: A new Link Code Word Page has been received. */
#define DP83848X_PHYSTS_MII_INTR      (1<<7)    /* MII Interrupt Pending */
#define DP83848X_PHYSTS_REMOTE_FAULT  (1<<6)    /* Remote Fault */
#define DP83848X_PHYSTS_JABBER_DETECT (1<<5)    /* Jabber Detect: This bit only has meaning in 10 Mb/s mode */
#define DP83848X_PHYSTS_AN_COMPLETE   (1<<4)    /* Auto-Negotiation Complete */
#define DP83848X_PHYSTS_LOOPBACK_STAT (1<<3)    /* Loopback enabled */
#define DP83848X_PHYSTS_DUPLEX_STATUS (1<<2)    /* Duplex: 1 = Full duplex mode. / 0 = Half duplex mode. */
#define DP83848X_PHYSTS_SPEED_STATUS  (1<<1)    /* Speed10: 1 = 10 Mb/s mode. / 0 = 100 Mb/s mode. */
#define DP83848X_PHYSTS_LINK_STATUS   (1<<0)    /* Link Status: 1 = Valid link established (for either 10 or 100 Mb/s operation). / 0 = Link not established. */

/* MII Interrupt Control Register */
#define DP83848X_MICR_TINT            (1<<2)    /* Test Interrupt: Generate an interrupt. */
#define DP83848X_MICR_INTEN           (1<<1)    /* Interrupt Enable: Enable event based interrupts. */
#define DP83848X_MICR_INT_OE          (1<<0)    /* Interrupt Output Enable: Enable interrupt events to signal through the PWRDOWN_INT pin by configuring the PWRDOWN_INT pin as an output. */

/* MII Interrupt Status and Misc. Control Register */
#define DP83848X_MISR_LQ_INT          (1<<15)   /* Link Quality interrupt */
#define DP83848X_MISR_ED_INT          (1<<14)   /* Energy Detect interrupt */
#define DP83848X_MISR_LINK_INT        (1<<13)   /* Change of Link Status interrupt */
#define DP83848X_MISR_SPD_INT         (1<<12)   /* Change of speed status interrupt */
#define DP83848X_MISR_DUP_INT         (1<<11)   /* Change of duplex status interrupt */
#define DP83848X_MISR_ANC_INT         (1<<10)   /* Auto-Negotiation Complete interrupt */
#define DP83848X_MISR_FHF_INT         (1<<9)   /* False Carrier Counter half-full interrupt */
#define DP83848X_MISR_RHF_INT         (1<<8)   /* Receive Error Counter half-full interrupt */
#define DP83848X_MISR_LQ_EN           (1<<7)   /* Enable Interrupt on Link Quality Monitor event */
#define DP83848X_MISR_ED_EN           (1<<6)   /* Enable Interrupt on energy detect event */
#define DP83848X_MISR_LINK_EN         (1<<5)   /* Enable Interrupt on change of link status */
#define DP83848X_MISR_SPD_EN          (1<<4)   /* Enable Interrupt on change of speed status */
#define DP83848X_MISR_DUP_EN          (1<<3)   /* Enable Interrupt on change of duplex status */
#define DP83848X_MISR_ANC_EN          (1<<2)   /* Enable Interrupt on Auto-negotiation complete event */
#define DP83848X_MISR_FHF_EN          (1<<1)   /* Enable Interrupt on False Carrier Counter Register half-full event */
#define DP83848X_MISR_RHF_EN          (1<<0)   /* Enable Interrupt on Receive Error Counter Register half-full event */

/* False Carrier Sense Counter Register */
#define DP83848X_FCSCR_FCSCNT_MASK    (0x00FF)  /* False Carrier Event Counter */

/* Receiver Error Counter Register */
#define DP83848X_RECR_RXERCNT_MASK    (0x00FF)  /* RX_ER Counter */

/* 100 Mb/s PCS Configuration and Status Register */
#define DP83848X_PCSR_FREE_CLK        (1<<11)   /* Receive Clock */
#define DP83848X_PCSR_TQ_EN           (1<<10)   /* 100Mbs True Quiet Mode Enable */
#define DP83848X_PCSR_SD_FORCE_PMA    (1<<9)    /* Signal Detect Force PMA */
#define DP83848X_PCSR_SD_OPTION       (1<<8)    /* Signal Detect Option */
#define DP83848X_PCSR_DESC_TIME       (1<<7)    /* Descrambler Timeout */
#define DP83848X_PCSR_FORCE_100_OK    (1<<5)    /* Force 100 Mb/s Good Link */
#define DP83848X_PCSR_NRZI_BYPASS     (1<<2)    /* NRZI Bypass Enable */

/* RMII and Bypass Register */
#define DP83848X_RBR_RMII_MODE        (1<<5)    /* Reduced MII Mode */
#define DP83848X_RBR_RMII_REV1_0      (1<<4)    /* Reduced MII Revision 1.0 */
#define DP83848X_RBR_RX_OVF_STS       (1<<3)    /* RX FIFO Over Flow Status */
#define DP83848X_RBR_RX_UNF_STS       (1<<2)    /* RX FIFO Under Flow Status */
#define DP83848X_RBR_ELAST_BUF_MASK   (3<<0)    /* Receive Elasticity Buffer */

/* LED Direct Control Register */
#define DP83848X_LEDCR_DRV_SPDLED     (1<<5)    /* Drive value of SPDLED bit onto LED_SPEED output */
#define DP83848X_LEDCR_DRV_LNKLED     (1<<4)    /* Drive value of LNKLED bit onto LED_LINK output */
#define DP83848X_LEDCR_DRV_ACTLED     (1<<3)    /* Drive value of ACTLED bit onto LED_ACT/LED_COL output */
#define DP83848X_LEDCR_SPDLED         (1<<2)    /* Value to force on LED_SPEED output */
#define DP83848X_LEDCR_LNKLED         (1<<1)    /* Value to force on LED_LINK output */
#define DP83848X_LEDCR_ACTLED         (1<<0)    /* Value to force on LED_ACT/LED_COL output */

/* PHY Control Register */
#define DP83848X_PHYCR_MDIX_EN        (1<<15)   /* Auto-MDIX Enable */
#define DP83848X_PHYCR_FORCE_MDIX     (1<<14)   /* Force MDIX */
#define DP83848X_PHYCR_PAUSE_RX       (1<<13)   /* Pause Receive Negotiated */
#define DP83848X_PHYCR_PAUSE_TX       (1<<12)   /* Pause Transmit Negotiated */
#define DP83848X_PHYCR_BIST_FE        (1<<11)   /* BIST Force Error */
#define DP83848X_PHYCR_PSR_15         (1<<10)   /* BIST Sequence select */
#define DP83848X_PHYCR_BIST_STATUS    (1<<9)    /* BIST Test Status */
#define DP83848X_PHYCR_BIST_START     (1<<8)    /* BIST Start */
#define DP83848X_PHYCR_BP_STRETCH     (1<<7)    /* Bypass LED Stretching */
#define DP83848X_PHYCR_LED_CFG_MASK   (3<<5)    /* LED Configuration */
#define DP83848X_PHYCR_PHYADDR_MASK   (0x000F)  /* PHY Address: PHY address for port */

/* 10 Base-T Status/Control Register */
#define DP83848X_10BTSCR_10BT_SERIAL  (1<<10)   /* 10Base-T Serial Mode (SNI) */
#define DP83848X_10BTSCR_SQUELCH_MASK (7<<9)    /* Squelch Configuration */
#define DP83848X_10BTSCR_LOOPBACK_DIS (1<<8)    /* 10Base-T Loopback Disable */
#define DP83848X_10BTSCR_LP_DIS       (1<<7)    /* Normal Link Pulse Disable */
#define DP83848X_10BTSCR_FORCE_LNK_10 (1<<6)    /* Force 10Mb Good Link */
#define DP83848X_10BTSCR_POLARITY     (1<<4)    /* 10Mb Polarity Status: 1 = Inverted Polarity detected. */
#define DP83848X_10BTSCR_HB_DIS       (1<<1)    /* Heartbeat Disable: This bit only has influence in half-duplex 10Mb mode. */
#define DP83848X_10BTSCR_JABBER_DIS   (1<<0)    /* Jabber Disable */

/* CD Test and BIST Extensions Register */
#define DP83848X_CDCTRL1_BIST_ERR_CNT   (0xff00)  /* BIST ERROR Counter */
#define DP83848X_CDCTRL1_BIST_CONT_MODE (1<<5)    /* Packet BIST Continuous Mode */
#define DP83848X_CDCTRL1_BIST_CDPATT_10 (1<<4)    /* CD Pattern Enable for 10Mb */
#define DP83848X_CDCTRL1_10M_PATT_GAP   (1<<2)    /* Defines gap between data or NLP test sequences */
#define DP83848X_CDCTRL1_CDPATTSEL      (3<<0)    /* CD Pattern Select[1:0] */

/* Energy Detect Control */
#define DP83848X_EDCR_ED_EN             (1<<15)   /* Energy Detect Enable */
#define DP83848X_EDCR_ED_AUTO_UP        (1<<14)   /* Energy Detect Automatic Power Up */
#define DP83848X_EDCR_ED_AUTO_DOWN      (1<<13)   /* Energy Detect Automatic Power Down */
#define DP83848X_EDCR_ED_MAN            (1<<12)   /* Energy Detect Manual Power Up/Down */
#define DP83848X_EDCR_ED_BURST_DIS      (1<<11)   /* Energy Detect Burst Disable */
#define DP83848X_EDCR_ED_PWR_STATE      (1<<10)   /* Energy Detect Power State */
#define DP83848X_EDCR_ED_ERR_MET        (1<<9)    /* Energy Detect Error Threshold Met */
#define DP83848X_EDCR_ED_DATA_MET       (1<<8)    /* Energy Detect Data Threshold Met */
#define DP83848X_EDCR_ED_ERR_COUNT      (0x00F0)  /* Energy Detect Error Threshold */
#define DP83848X_EDCR_ED_DATA_COUNT     (0x000F)  /* Energy Detect Data Threshold */


/* Predefined modes for BMCR */
#define DP83848X_MODE_FULL_100M   0x2100      /* Full Duplex 100Mbit */
#define DP83848X_MODE_HALF_100M   0x2000      /* Half Duplex 100Mbit */
#define DP83848X_MODE_FULL_10M    0x0100      /* Full Duplex 10Mbit */
#define DP83848X_MODE_HALF_10M    0x0000      /* Half Duplex 10MBit */
#define DP83848X_MODE_AUTO_NEG    0x3000      /* Select Auto Negotiation */


#define DP83848C_DEF_ADR    0x0100      /* Default PHY device address */
#define DP83848C_DEF_ID     0x20005C92  /* PHY Identifier (PHYIDR1,PHYIDR2 regs.) */

#endif /* DP83848X_PHY_HEADER_FILE */
