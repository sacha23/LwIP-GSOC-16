/******************************************************************************
 *
 * lpc_emac.h - LPCxxxx Ethernet Peripheral Descriptions
 * (Based on LPC17xx and LPC23xx user manuals)
 * 2015, Roman Bartosinski <bartosr@centrum.cz>
 *
 *****************************************************************************/

#ifndef _lpcEMAC_H
#define _lpcEMAC_H

/******************************************************************************/
/*** Ethernet MAC register definitions ***/
/* MAC Configuration Register 1 */
#define EMAC_MAC1_RCV_ENB       (1<<0)    /* Receive Enable */
#define EMAC_MAC1_PASS_ALL      (1<<1)    /* Pass All Receive Frames */
#define EMAC_MAC1_RX_FLOWCTRL   (1<<2)    /* RX Flow Control */
#define EMAC_MAC1_TX_FLOWCTRL   (1<<3)    /* TX Flow Control */
#define EMAC_MAC1_LOOPBACK      (1<<4)    /* Loop Back Mode */
#define EMAC_MAC1_RST_TX        (1<<8)    /* Reset TX Logic */
#define EMAC_MAC1_RST_MCS_TX    (1<<9)    /* Reset MAC TX Control Sublayer */
#define EMAC_MAC1_RST_RX        (1<<10)   /* Reset RX Logic */
#define EMAC_MAC1_RST_MCS_RX    (1<<11)   /* Reset MAC RX Control Sublayer */
#define EMAC_MAC1_SIM_RST       (1<<14)   /* Simulation Reset */
#define EMAC_MAC1_SOFT_RST      (1<<15)   /* Soft Reset MAC */

/* MAC Configuration Register 2 */
#define EMAC_MAC2_FULL_DUPLEX   (1<<0)    /* Full Duplex Mode */
#define EMAC_MAC2_FRM_LEN_CHK   (1<<1)    /* Frame Length Checking */
#define EMAC_MAC2_HUGE_FRM_ENB  (1<<2)    /* Huge Frame Enable */
#define EMAC_MAC2_DELAYED_CRC   (1<<3)    /* Delayed CRC Mode */
#define EMAC_MAC2_CRC_ENB       (1<<4)    /* Append CRC to every Frame */
#define EMAC_MAC2_PAD_ENB       (1<<5)    /* Pad all Short Frames */
#define EMAC_MAC2_VLAN_PAD_ENB  (1<<6)    /* VLAN Pad Enable */
#define EMAC_MAC2_ADET_PAD_ENB  (1<<7)    /* Auto Detect Pad Enable */
#define EMAC_MAC2_PURE_PREAMBL  (1<<8)    /* Pure Preamble Enforcement */
#define EMAC_MAC2_LONG_PREAMBL  (1<<9)    /* Long Preamble Enforcement */
#define EMAC_MAC2_NO_BACKOFF    (1<<12)   /* No Backoff Algorithm */
#define EMAC_MAC2_BACK_PRESSURE (1<<13)   /* Backoff Presurre / No Backoff */
#define EMAC_MAC2_EXCESS_DEFER  (1<<14)   /* Excess Defer */

/* Back-to-Back Inter-Packet-Gap Register */
#define EMAC_IPGT_MASK          0x0000007F  /* Back-to-Back Inter-Packet-gap mask */
#define EMAC_IPGT_HALF_DUP      0x00000012  /* Recommended value for Half Duplex (960ns 100Mbps/9.6us 10Mbps) */
#define EMAC_IPGT_FULL_DUP      0x00000015  /* Recommended value for Full Duplex (960ns 100Mbps/9.6us 10Mbps) */

/* Non Back-to-Back Inter-Packet-Gap Register */
#define EMAC_IPGR_PART2_MASK    0x0000007F  /* Non Back-to-Back Inter-Packet-Gap Part2 mask */
#define EMAC_IPGR_PART1_MASK    0x00007F00  /* Non Back-to-Back Inter-Packet-Gap Part1 mask */
#define EMAC_IPGR_RECOMMENDED   0x00000C12  /* Recommended value */

/* Collision Window/Retry Register */
#define EMAC_CLRT_RETR_MAX_MASK 0x0000000F  /* Retransmission Maximum */
#define EMAC_CLRT_COL_WND_MASK  0x00003F00  /* Retransmission Maximum */

#define EMAC_CLRT_RESET_VALUE   0x0000370F  /* Reset value of CLRT register */

/* Maximum Frame Register Register */
#define EMAC_MAXF_RETR_MAX_MASK 0x0000FFFF  /* Maximum Frame Length */

/* PHY Support Register */
#define EMAC_SUPP_SPEED         (1<<8)      /* Reduced MII Logic Current Speed   */
#define EMAC_SUPP_SPEED_100MBPS (EMAC_SUPP_SPEED)  /* Red.MII Log. 100Mbps mode */
#define EMAC_SUPP_SPEED_10MBPS  (0)                /* Red.MII Log. 10Mbps mode */

/* Test Register */
#define EMAC_TEST_SHC_PAUSE_QUANTA  (1<<0)  /* Shortcut Pause Quanta */
#define EMAC_TEST_TEST_PAUSE        (1<<1)  /* Test Pause */
#define EMAC_TEST_TEST_BACKPRES     (1<<2)  /* Test Back Pressure */

/* MII Management Configuration Register */
#define EMAC_MCFG_SCAN_INC          (1<<0)    /* Scan Increment PHY Address */
#define EMAC_MCFG_SUPP_PREAMBLE     (1<<1)    /* Suppress Preamble */
#define EMAC_MCFG_CLK_SEL_MASK      (0xF<<2)  /* Clock Select Mask */
#define EMAC_MCFG_RST_MII_HW        (1<<15)   /* Reset MII Management Hardware */

#define EMAC_MCFG_CLK_SEL_DIV4      (1<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV6      (2<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV8      (3<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV10     (4<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV14     (5<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV20     (6<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV28     (7<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV36     (8<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV40     (9<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV44    (10<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV48    (11<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV52    (12<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV56    (13<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV60    (14<<2)  /* Host Clock divided by 4 */
#define EMAC_MCFG_CLK_SEL_DIV64    (15<<2)  /* Host Clock divided by 4 */

/* MII Management Command Register */
#define EMAC_MCMD_READ          (1<<0)    /* MII Read */
#define EMAC_MCMD_SCAN          (1<<1)    /* MII Scan continuously */

/* MII Management Address Register */
#define EMAC_MADR_REG_ADR_MASK  0x0000001F  /* MII Register Address Mask */
#define EMAC_MADR_PHY_ADR_MASK  0x00001F00  /* PHY Address Mask */

/* MII Management Write Data Register */
#define EMAC_MWTD_WR_DATA_MASK  0x0000FFFF

/* MII Management Read Data Register */
#define EMAC_MRDD_RD_DATA_MASK  0x0000FFFF

/* MII Management Indicators Register */
#define EMAC_MIND_BUSY          (1<<0)    /* MII is Busy */
#define EMAC_MIND_SCANNING      (1<<1)    /* MII Scanning in Progress */
#define EMAC_MIND_NOT_VALID     (1<<2)    /* MII Read Data not valid */
#define EMAC_MIND_MII_LINK_FAIL (1<<3)    /* MII Link Failed */

/* Station Address 0 Register */
#define EMAC_SA0_SADDR_2_MASK   0x000000FF  /* Station Address the 2nd Octet */
#define EMAC_SA0_SADDR_1_MASK   0x0000FF00  /* Station Address the 1st Octet */
/* Station Address 1 Register */
#define EMAC_SA1_SADDR_4_MASK   0x000000FF  /* Station Address the 4th Octet */
#define EMAC_SA1_SADDR_3_MASK   0x0000FF00  /* Station Address the 3rd Octet */
/* Station Address 2 Register */
#define EMAC_SA2_SADDR_6_MASK   0x000000FF  /* Station Address the 6th Octet */
#define EMAC_SA2_SADDR_5_MASK   0x0000FF00  /* Station Address the 5th Octet */

/*** Control register definitions ***/
/* Command Register */
#define EMAC_CMD_RX_ENB         (1<<0)      /* Enable Receive */
#define EMAC_CMD_TX_ENB         (1<<1)      /* Enable Transmit */
#define EMAC_CMD_REG_RST        (1<<3)      /* Reset Host Registers */
#define EMAC_CMD_TX_RST         (1<<4)      /* Reset Transmit Datapath */
#define EMAC_CMD_RX_RST         (1<<5)      /* Reset Receive Datapath */
#define EMAC_CMD_PASS_RUNT_FRM  (1<<6)      /* Pass Runt Frames  */
#define EMAC_CMD_PASS_RX_FLT    (1<<7)      /* Pass RX Filter  */
#define EMAC_CMD_TX_FLOW_CTRL   (1<<8)      /* TX Flow Control */
#define EMAC_CMD_RMII           (1<<9)      /* Reduced MII Interface */
#define EMAC_CMD_FULL_DUPLEX    (1<<10)     /* Full Duplex */

/* Status Register */
#define EMAC_STAT_RX_ACTIVE     (1<<0)      /* Enable Receive */
#define EMAC_STAT_TX_ACTIVE     (1<<1)      /* Enable Transmit */

/* Receive Descriptor Base Address Register */
#define EMAC_RDBA_MASK          0xFFFFFFFC

/* Receive Status Base Address Register */
#define EMAC_RSBA_MASK          0xFFFFFFF8

/* Receive Number of Descriptors Register */
#define EMAC_RND_MASK          0x0000FFFF

/* Receive Produce Index Register */
#define EMAC_RPI_MASK          0x0000FFFF

/* Receive Consume Index Register */
#define EMAC_RCI_MASK          0x0000FFFF

/* Transmit Descriptor Base Address Register */
#define EMAC_TDBA_MASK          0xFFFFFFFC

/* Transmit Status Base Address Register */
#define EMAC_TSBA_MASK          0xFFFFFFFC

/* Transmit Number of Descriptors Register */
#define EMAC_TND_MASK          0x0000FFFF

/* Transmit Produce Index Register */
#define EMAC_TPI_MASK          0x0000FFFF

/* Transmit Consume Index Register */
#define EMAC_TCI_MASK          0x0000FFFF

/* Transmit Status Vector 0 Register */
#define EMAC_TSV0_CRC_ERROR     (1<<0)        /* CRC error */
#define EMAC_TSV0_LEN_CHKERR    (1<<1)        /* Length Check Error */
#define EMAC_TSV0_LEN_OUTRNG    (1<<2)        /* Length Out of Range */
#define EMAC_TSV0_DONE          (1<<3)        /* Tramsmission Completed */
#define EMAC_TSV0_MULTICAST     (1<<4)        /* Multicast Destination */
#define EMAC_TSV0_BROADCAST     (1<<5)        /* Broadcast Destination */
#define EMAC_TSV0_PKT_DEFER     (1<<6)        /* Packet Deferred */
#define EMAC_TSV0_EXCESS_DEFER  (1<<7)        /* Excessive Packet Deferral */
#define EMAC_TSV0_EXCESS_COLL   (1<<8)        /* Excessive Collision */
#define EMAC_TSV0_LATE_COLL     (1<<9)        /* Late Collision Occured */
#define EMAC_TSV0_GIANT         (1<<10)       /* Giant Frame */
#define EMAC_TSV0_UNDERRUN      (1<<11)       /* Buffer Underrun */
#define EMAC_TSV0_BYTES_MASK    (0xFFFF<<12)  /* Total Bytes Transferred */
#define EMAC_TSV0_CTRL_FRAME    (1<<28)       /* Control Frame */
#define EMAC_TSV0_PAUSE         (1<<29)       /* Pause Frame */
#define EMAC_TSV0_BACKPRESSURE  (1<<30)       /* Backpressure Method Applied */
#define EMAC_TSV0_VLAN          (1<<31)       /* VLAN Frame */

/* Transmit Status Vector 1 Register */
#define EMAC_TSV1_BYTE_MASK     (0xFFFF<<0)  /* Transmit Byte Count */
#define EMAC_TSV1_COLL_MASK     (0xF<<16)    /* Transmit Collision Count */

/* Receive Status Vector Register */
#define EMAC_RSV_BYTES_MASK     (0xFFFF<<0)  /* Receive Byte Count */
#define EMAC_RSV_PKT_IGNORED    (1<<16)      /* Packet Previously Ignored */
#define EMAC_RSV_RXDV_SEEN      (1<<17)      /* RXDV Event Previously Seen */
#define EMAC_RSV_CARRIER_SEEN   (1<<18)      /* Carrier Event Previously Seen */
#define EMAC_RSV_REC_CODEV      (1<<19)      /* Receive Code Violation */
#define EMAC_RSV_CRC_ERROR      (1<<20)      /* CRC Error */
#define EMAC_RSV_LEN_CHKERR     (1<<21)      /* Length Check Error */
#define EMAC_RSV_LEN_OUTRNG     (1<<22)      /* Length Out of Range */
#define EMAC_RSV_REC_OK         (1<<23)      /* Frame Received OK */
#define EMAC_RSV_MULTICAST      (1<<24)      /* Multicast Frame */
#define EMAC_RSV_BROADCAST      (1<<25)      /* Broadcast Frame */
#define EMAC_RSV_DRIBBLE_NIBBLE (1<<26)      /* Dribble Nibble */
#define EMAC_RSV_CTRL_FRAME     (1<<27)      /* Control Frame */
#define EMAC_RSV_PAUSE          (1<<28)      /* Pause Frame */
#define EMAC_RSV_UNSUPP_OPCODE  (1<<29)      /* Unsupported Opcode */
#define EMAC_RSV_VLAN           (1<<30)      /* VLAN Frame */

/* Flow Control Counter Register */
#define EMAC_FCC_MIRROR_CNT_MASK  0x0000FFFF  /* Mirror Counter */
#define EMAC_FCC_PAUSE_TMR_MASK   0xFFFF0000  /* Pause Timer */

/* Flow Control Status Register */
#define EMAC_FCS_MIRROR_CNT_MASK  0x0000FFFF  /* Mirror Counter Current */

/*** Receive filter register definitions ***/
/* Receive Filter Control Register */
#define EMAC_RFC_UNICAST_ENB        (1<<0)  /* Accept Unicast Frames Enable  */
#define EMAC_RFC_BROADCAST_ENB      (1<<1)  /* Accept Broadcast Frames Enable */
#define EMAC_RFC_MULTICAST_ENB      (1<<2)  /* Accept Multicast Frames Enable */
#define EMAC_RFC_UNICAST_HASH_ENB   (1<<3)  /* Accept Unicast Hash Filter Frames */
#define EMAC_RFC_MULTICAST_HASH_ENB (1<<4)  /* Accept Multicast Hash Filter Frames */
#define EMAC_RFC_PERFECT_ENB        (1<<5)  /* Accept Perfect Match Enable */
#define EMAC_RFC_MAGIC_PKT_WOL_ENB  (1<<12) /* Magic Packet Filter WoL Enable */
#define EMAC_RFC_RX_FILTER_WOL_ENB  (1<<13) /* Perfect Filter WoL Enable */

/* Receive Filter WoL Status Registers */
#define EMAC_RFWS_UNICAST           (1<<0)  /* Unicast Frame caused WoL */
#define EMAC_RFWS_BROADCAST         (1<<1)  /* Broadcast Frame caused WoL */
#define EMAC_RFWS_MULTICAST         (1<<2)  /* Multicast Frame caused WoL */
#define EMAC_RFWS_UNICAST_HASH      (1<<3)  /* Unicast Hash Filter Frame WoL */
#define EMAC_RFWS_MULTICAST_HASH    (1<<4)  /* Multicast Hash Filter Frame WoL */
#define EMAC_RFWS_PERFECT           (1<<5)  /* Perfect Filter WoL */
#define EMAC_RFWS_RX_FILTER         (1<<7)  /* RX Filter caused WoL */
#define EMAC_RFWS_MAGIC_PKT         (1<<8)  /* Magic Packet Filter caused WoL */

/* Receive Filter WoL Clear Registers */
#define EMAC_RFWC_UNICAST           (1<<0)  /* Clear Unicast Frame caused WoL */
#define EMAC_RFWC_BROADCAST         (1<<1)  /* Clear Broadcast Frame caused WoL */
#define EMAC_RFWC_MULTICAST         (1<<2)  /* Clear Multicast Frame caused WoL */
#define EMAC_RFWC_UNICAST_HASH      (1<<3)  /* Clear Unicast Hash Filter Frame WoL */
#define EMAC_RFWC_MULTICAST_HASH    (1<<4)  /* Clear Multicast Hash Filter Frame WoL */
#define EMAC_RFWC_PERFECT           (1<<5)  /* Clear Perfect Filter WoL */
#define EMAC_RFWC_RX_FILTER         (1<<7)  /* Clear RX Filter caused WoL */
#define EMAC_RFWC_MAGIC_PKT         (1<<8)  /* Clear Magic Packet Filter caused WoL */

/* Hash Filter Table LSBs Register */
#define EMAC_HFL_MASK               (0xFFFFFFFF)  /* 32 LSB bits of the imperfect filter hash table for receiving filtering */

/* Hash Filter Table MSBs Register */
#define EMAC_HFH_MASK               (0xFFFFFFFF)  /* 32 MSB bits of the imperfect filter hash table for receiving filtering */

/*** Module control register definitions ***/
/* Interrupt Status Registers */
/* Interrupt Enable Registers */
/* Interrupt Clear Registers */
/* Interrupt Set Registers */
#define EMAC_INT_RX_OVERRUN         (1<<0)  /* Overrun Error in RX Queue */
#define EMAC_INT_RX_ERROR           (1<<1)  /* Receive Error */
#define EMAC_INT_RX_FINISHED        (1<<2)  /* RX Finished Process Descriptors */
#define EMAC_INT_RX_DONE            (1<<3)  /* Receive Done */
#define EMAC_INT_TX_UNDERRUN        (1<<4)  /* Transmit Underrun */
#define EMAC_INT_TX_ERROR           (1<<5)  /* Transmit Error */
#define EMAC_INT_TX_FINISHED        (1<<6)  /* TX Finished Process Descriptors */
#define EMAC_INT_TX_DONE            (1<<7)  /* Transmit Done */
#define EMAC_INT_SOFT_INT           (1<<12) /* Software Triggered Interrupt */
#define EMAC_INT_WAKEUP             (1<<13) /* Wakeup Event Interrupt */

/* Power Down Register */
#define EMAC_PD_POWER_DOWN_MACAHB   (1<<31) /* Power Down MAC and AHB interface */


/******************************************************************************/
/*** Descriptors and status formats ***/

typedef struct emac_rx_descriptor {
  uint32_t  ptrpktbuff;
  uint32_t  control;
} emac_rx_descriptor_t;

typedef struct emac_rx_status {
  uint32_t  info;
  uint32_t  hash_crc;
} emac_rx_status_t;

/* RX Descriptor Control Word */
#define EMAC_RXCTRL_SIZE            0x000007FF  /* Size in bytes of the data buffer */
#define EMAC_RXCTRL_INT             (1<<31)     /* If true generate an RxDone interrupt */

/* RX Status Information Word */
#define EMAC_RXSTAT_INFO_RXSIZE      0x000007FF  /* The size in bytes of the actual data transferred into one fragment buffer. */
#define EMAC_RXSTAT_INFO_CTRL_FRAME  (1<<18)     /* Control Frame */
#define EMAC_RXSTAT_INFO_VLAN        (1<<19)     /* VLAN Frame */
#define EMAC_RXSTAT_INFO_FAIL_FILTER (1<<20)     /* RX Filter Failed */
#define EMAC_RXSTAT_INFO_MCAST       (1<<21)     /* Multicast Frame */
#define EMAC_RXSTAT_INFO_BCAST       (1<<22)     /* Broadcast Frame */
#define EMAC_RXSTAT_INFO_CRC_ERROR   (1<<23)     /* CRC Error in Frame */
#define EMAC_RXSTAT_INFO_SYM_ERROR   (1<<24)     /* Symbol Error from PHY */
#define EMAC_RXSTAT_INFO_LEN_ERROR   (1<<25)     /* Length Error */
#define EMAC_RXSTAT_INFO_RANGE_ERROR (1<<26)     /* Range Error (exceeded max. size) */
#define EMAC_RXSTAT_INFO_ALIGN_ERROR (1<<27)     /* Alignment Error  */
#define EMAC_RXSTAT_INFO_OVERRUN     (1<<28)     /* Receive overrun */
#define EMAC_RXSTAT_INFO_NO_DESCR    (1<<29)     /* No new Descriptor available */
#define EMAC_RXSTAT_INFO_LAST_FLAG   (1<<30)     /* Last Fragment in Frame */
#define EMAC_RXSTAT_INFO_ERROR       (1<<31)     /* Error Occured (OR of all errors) */

#define EMAC_RXSTAT_ERROR_MASK      (EMAC_RSTAT_INFO_FAIL_FILTER | \
                                    EMAC_RSTAT_INFO_CRC_ERROR | \
                                    EMAC_RSTAT_INFO_SYM_ERROR | \
                                    EMAC_RSTAT_INFO_LEN_ERROR | \
                                    EMAC_RSTAT_INFO_ALIGN_ERROR | \
                                    EMAC_RSTAT_INFO_OVERRUN)

/* RX Status Hash CRC Word */
#define EMAC_RXSTAT_HASHCRC_SA      0x000001FF  /* Hash CRC for Source Address */
#define EMAC_RXSTAT_HASHCRC_DA      0x001FF000  /* Hash CRC for Destination Address */


/**************************************/
typedef struct emac_tx_descriptor {
  uint32_t  ptrpktbuff;
  uint32_t  control;
} emac_tx_descriptor_t;

typedef struct emac_tx_status {
  uint32_t  info;
} emac_tx_status_t;

/* TX Descriptor Control Word */
#define EMAC_TXCTRL_SIZE          0x000007FF  /* Size in bytes of the data buffer */
#define EMAC_TXCTRL_OVERRIDE      (1<<26)     /* Override Default MAC Registers */
#define EMAC_TXCTRL_HUGE          (1<<27)     /* Enable Huge Frame */
#define EMAC_TXCTRL_PAD           (1<<28)     /* Pad short Frames to 64 bytes */
#define EMAC_TXCTRL_CRC           (1<<29)     /* Append a hardware CRC to Frame */
#define EMAC_TXCTRL_LAST          (1<<30)     /* Last Descriptor for TX Frame */
#define EMAC_TXCTRL_INT           (1<<31)     /* Generate TxDone Interrupt */

/* TX Status Information Word */
#define EMAC_TXSTAT_INFO_COL_CNT    (0x01E00000)  /* Collision Count */
#define EMAC_TXSTAT_INFO_DEFER      (1<<25)       /* Packet Deferred (not an error) */
#define EMAC_TXSTAT_INFO_EXCESS_DEF (1<<26)       /* Excessive Deferral  */
#define EMAC_TXSTAT_INFO_EXCESS_COL (1<<27)       /* Excessive Collision  */
#define EMAC_TXSTAT_INFO_LATE_COL   (1<<28)       /* Late Collision Occured */
#define EMAC_TXSTAT_INFO_UNDERRUN   (1<<29)       /* Transmit Underrun  */
#define EMAC_TXSTAT_INFO_NO_DESCR   (1<<30)       /* No new Descriptor available */
#define EMAC_TXSTAT_INFO_ERR        (1<<31)       /* Error Occured (OR of all errors) */

#endif /* _lpcEMAC_H */
