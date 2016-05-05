/**
 * @file
 * LPC17xx and LPC40xx network interface driver.
 *
 */

/*
 * Copyright (c) 2015 - 2016 PiKRON s.r.o. and others
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Roman Bartosinsky <bartosr@centrum.cz>
 *         Pavel Pisa <pisa@pikron.com>
 * code is based on NXP provided examples and application notes
 * and LwIP examples
 *
 */


#include "lwip/opt.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
//#include "lwip/ethip6.h"
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"

#include "LPC17xx.h"
#include "dp83848x_phy.h"
#include "lan8720a.h"
#include "netif/lpc_emac.h"
#include "netif/emac_config.h"

#if LWIP_IGMP
#include "lwip/igmp.h"
#endif

#include <stdlib.h>

typedef struct {
  uint32_t phyid;
  uint32_t maskoutid;
  uint32_t flags;
} lpc_emac_know_phy_t;

const static lpc_emac_know_phy_t lpc_emac_know_phy[] = {
  {DP83848C_DEF_ID, DP83848X_PHYIDR2_MDL_REV_MASK, 0},
  {LAN8720A_DEF_ID,  DP83848X_PHYIDR2_MDL_REV_MASK, 0},
  {0,  0}
};

/* workaround for different definitions of peripherals in LPCxxxx.h */
#ifndef LPC_EMAC
  #define LPC_EMAC    EMAC
#endif
#ifndef LPC_SC
  #define LPC_SC      SC
#endif
#if !defined(LPC_PINCON) && defined(PINCON)
  #define LPC_PINCON  PINCON
#endif


/* EMAC Packet Buffer Sizes and Placement */
#ifdef CONFIG_EMAC_PKT_FRAG_SIZE
  #define EMAC_FRAG_SIZE    CONFIG_EMAC_PKT_FRAG_SIZE
#else
  #define EMAC_FRAG_SIZE      1536
#endif

#ifdef CONFIG_EMAC_ETH_FRAME_SIZE
  #define EMAC_MAX_FLEN    CONFIG_EMAC_ETH_FRAME_SIZE
#else
  #define EMAC_MAX_FLEN       1536
#endif

#ifdef CONFIG_EMAC_NUM_RX_FRAGS
  #define EMAC_NUM_RX_FRAG    CONFIG_EMAC_NUM_RX_FRAGS
#else
  #define EMAC_NUM_RX_FRAG    4
#endif

#ifdef CONFIG_EMAC_NUM_TX_FRAGS
  #define EMAC_NUM_TX_FRAG    CONFIG_EMAC_NUM_TX_FRAGS
#else
  #define EMAC_NUM_TX_FRAG    2
#endif

#ifndef CONFIG_EMAC_DESC_SECTION
/* placement of Rx descriptors. Required space = NUM_RX_FRAGS*(sizeof(emac_rx_descriptor_t)+sizeof(emac_rx_status_t)) */
#ifdef CONFIG_EMAC_RX_DESC_BASE
  #define EMAC_RX_DESC_BASE   CONFIG_EMAC_RX_DESC_BASE
#else
  /* FIXME: MACH/BOARD specific, obtain automatically from linker */
  #define EMAC_RX_DESC_BASE   0x20080000
#endif

#ifdef CONFIG_EMAC_TX_DESC_BASE
  #define EMAC_TX_DESC_BASE   CONFIG_EMAC_TX_DESC_BASE
#else
  #define EMAC_TX_DESC_BASE   (EMAC_RX_DESC_BASE + EMAC_NUM_RX_FRAG*(sizeof(emac_rx_descriptor_t)+sizeof(emac_rx_status_t)))
#endif

#ifdef CONFIG_EMAC_RX_DATABUF_BASE
  #define EMAC_RX_DBUF_BASE   CONFIG_EMAC_RX_DATABUF_BASE
#else
  #define EMAC_RX_DBUF_BASE   (EMAC_TX_DESC_BASE + EMAC_NUM_TX_FRAG*(sizeof(emac_tx_descriptor_t)+sizeof(emac_tx_status_t)))
#endif

#ifdef CONFIG_EMAC_TX_DATABUF_BASE
  #define EMAC_TX_DBUF_BASE   CONFIG_EMAC_TX_DATABUF_BASE
#else
  #define EMAC_TX_DBUF_BASE   (EMAC_RX_DBUF_BASE + EMAC_NUM_RX_FRAG*EMAC_FRAG_SIZE)
#endif
#else /*CONFIG_EMAC_DESC_SECTION*/
  /* Allocate memory from specified section */
  #define EMAC_DESC_ATTR  \
            __attribute__((section(CONFIG_EMAC_DESC_SECTION))) \
            __attribute__((aligned(8)))

  uint8_t lpc_emac_rx_desc[EMAC_NUM_RX_FRAG*(sizeof(emac_rx_descriptor_t)+sizeof(emac_rx_status_t))] EMAC_DESC_ATTR;
  #define EMAC_RX_DESC_BASE   ((uintptr_t)lpc_emac_rx_desc)
  uint8_t lpc_emac_tx_desc[EMAC_NUM_TX_FRAG*(sizeof(emac_tx_descriptor_t)+sizeof(emac_tx_status_t))] EMAC_DESC_ATTR;
  #define EMAC_TX_DESC_BASE   ((uintptr_t)lpc_emac_tx_desc)
  uint8_t lpc_emac_rx_dbuf[EMAC_NUM_RX_FRAG*EMAC_FRAG_SIZE] EMAC_DESC_ATTR;
  #define EMAC_RX_DBUF_BASE   ((uintptr_t)lpc_emac_rx_dbuf)
  uint8_t lpc_emac_tx_dbuf[EMAC_NUM_TX_FRAG*EMAC_FRAG_SIZE] EMAC_DESC_ATTR;
  #define EMAC_TX_DBUF_BASE   ((uintptr_t)lpc_emac_tx_dbuf)
#endif /*CONFIG_EMAC_DESC_SECTION*/

/* Macros for obtaining addresses of buffers and descriptors. */
#define EMAC_RX_DESCRIPTOR(i)   ((emac_rx_descriptor_t *)(EMAC_RX_DESC_BASE + i*sizeof(emac_rx_descriptor_t)))
#define EMAC_RX_STATUS(i)       ((emac_rx_status_t *)(EMAC_RX_DESC_BASE + EMAC_NUM_RX_FRAG*sizeof(emac_rx_descriptor_t) + i*sizeof(emac_rx_status_t)))
#define EMAC_TX_DESCRIPTOR(i)   ((emac_tx_descriptor_t *)(EMAC_TX_DESC_BASE + i*sizeof(emac_tx_descriptor_t)))
#define EMAC_TX_STATUS(i)       ((emac_tx_status_t *)(EMAC_TX_DESC_BASE + EMAC_NUM_TX_FRAG*sizeof(emac_tx_descriptor_t) + i*sizeof(emac_tx_status_t)))
#define EMAC_RX_DBUF(i)         (EMAC_RX_DBUF_BASE + i*EMAC_FRAG_SIZE)
#define EMAC_TX_DBUF(i)         (EMAC_TX_DBUF_BASE + i*EMAC_FRAG_SIZE)


/* user defined timeouts */
#define EMAC_MII_TIMEOUT_RD      0x00050000  /* MII Read timeout count */
#define EMAC_MII_TIMEOUT_WR      0x00050000  /* MII Write timeout count */

/* Define those to better describe your network interface. */
#define IFNAME0 'l'
#define IFNAME1 'e'

/* -------------------------------------------------------------------------- */
/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct lpc_netif_data {
  struct eth_addr ethaddr;
  ///* Add whatever per-interface state that is needed here. */
  unsigned char hwaddr[6];
  int lastlinkstate;
  uint32_t phyid; /* ID of the connected/detected PHY */
};

/*---------------------------------------------------------------------------*/
static void write_PHY(int PhyReg, int Value)
{
  unsigned int tout;

  LPC_EMAC->MCMD = 0;
  LPC_EMAC->MADR = DP83848C_DEF_ADR | PhyReg;
  LPC_EMAC->MWTD = Value;

  /* Wait utill operation completed */
  tout = 0;
  for (tout = 0; tout < EMAC_MII_TIMEOUT_WR; tout++) {
    if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
        break;
    }
  }
}

static unsigned short read_PHY(unsigned char PhyReg)
{
  unsigned int tout;

  LPC_EMAC->MCMD = EMAC_MCMD_READ;
  LPC_EMAC->MADR = DP83848C_DEF_ADR | PhyReg;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < EMAC_MII_TIMEOUT_RD; tout++) {
    if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
      break;
    }
  }
  LPC_EMAC->MCMD = 0;
  return (LPC_EMAC->MRDD);
}

/* LPC 1768 (and also probably others) updates EMAC->MIND.LinkFail only if
 * PHY status register is read. Therefore the PHY status register will be
 * continuously scanned. Otherwise it should be read before checking MIND reg.
 */
static void scan_PHY_status(void)
{
  LPC_EMAC->MCFG &= ~EMAC_MCFG_SCAN_INC;
  LPC_EMAC->MCMD = EMAC_MCMD_SCAN;
  LPC_EMAC->MADR = DP83848C_DEF_ADR | DP83848X_REG_BMSR;
}

/*---------------------------------------------------------------------------*/
static void rx_descr_init(void)
{
  unsigned int i;

  for (i = 0; i < EMAC_NUM_RX_FRAG; i++) {
    EMAC_RX_DESCRIPTOR(i)->ptrpktbuff  = EMAC_RX_DBUF(i);
    EMAC_RX_DESCRIPTOR(i)->control     = EMAC_RXCTRL_INT | (EMAC_FRAG_SIZE-1);
    EMAC_RX_STATUS(i)->info     = 0;
    EMAC_RX_STATUS(i)->hash_crc = 0;
  }

  /* Set EMAC Receive Descriptor Registers. */
  LPC_EMAC->RxDescriptor       = EMAC_RX_DESC_BASE;
  LPC_EMAC->RxStatus           = (uint32_t)EMAC_RX_STATUS(0);
  LPC_EMAC->RxDescriptorNumber = EMAC_NUM_RX_FRAG - 1;

  /* Rx Descriptors Point to 0 */
  LPC_EMAC->RxConsumeIndex  = 0;
}

static void tx_descr_init(void)
{
  unsigned int i;

  for (i = 0; i < EMAC_NUM_TX_FRAG; i++) {
    EMAC_TX_DESCRIPTOR(i)->ptrpktbuff = EMAC_TX_DBUF(i);
    EMAC_TX_DESCRIPTOR(i)->control    = 0;
    EMAC_TX_STATUS(i)->info = 0;
  }

  /* Set EMAC Transmit Descriptor Registers. */
  LPC_EMAC->TxDescriptor       = EMAC_TX_DESC_BASE;
  LPC_EMAC->TxStatus           = (uint32_t)EMAC_TX_STATUS(0);
  LPC_EMAC->TxDescriptorNumber = EMAC_NUM_TX_FRAG - 1;

  /* Tx Descriptors Point to 0 */
  LPC_EMAC->TxProduceIndex  = 0;
}

/*---------------------------------------------------------------------------*/

#if LWIP_IGMP

#define MAC_HASH_BITS 64
static uint8_t mac_hash_use_count[MAC_HASH_BITS];

/* code based on example from ST forum sourcer32@gmail.com */
static uint32_t mac_hash_fast(const uint8_t *mac)
{
  static const uint8_t rev_crc6_tbl[] = {
    0x00,0x20,0x10,0x30,0x08,0x28,0x18,0x38,0x04,0x24,0x14,0x34,0x0C,0x2C,0x1C,0x3C,
    0x02,0x22,0x12,0x32,0x0A,0x2A,0x1A,0x3A,0x06,0x26,0x16,0x36,0x0E,0x2E,0x1E,0x3E,
    0x01,0x21,0x11,0x31,0x09,0x29,0x19,0x39,0x05,0x25,0x15,0x35,0x0D,0x2D,0x1D,0x3D,
    0x03,0x23,0x13,0x33,0x0B,0x2B,0x1B,0x3B,0x07,0x27,0x17,0x37,0x0F,0x2F,0x1F,0x3F };

  static const uint32_t crc32_tbl[] = {
    0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
    0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
    0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
    0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000 };

  int i;
  uint32_t crc;

  crc = 0xffffffff;

  for(i=0; i<6; i++)
  {
    crc = crc ^ (uint32_t)mac[i];

    crc = (crc >> 4) ^ crc32_tbl[crc & 0x0F];  /* lower nibble */
    crc = (crc >> 4) ^ crc32_tbl[crc & 0x0F];  /* upper nibble */
  }

  //return(rev_crc6_tbl[(crc >> 23) & 0x3F]);
  return(rev_crc6_tbl[(crc) & 0x3F]);
  //return (crc >> 3) & 0x3F;
}

static err_t multicast_ip_to_mac(const ip4_addr_t *ip_addr,
                                 struct eth_addr *mac_addr)
{
  if ((NULL == ip_addr) || (NULL == mac_addr)) {
    return ERR_ARG;
  }
  mac_addr->addr[0] = 0x01;
  mac_addr->addr[1] = 0x00;
  mac_addr->addr[2] = 0x5E;
  mac_addr->addr[3] = ip4_addr2_16(ip_addr) & ~0x80;
  mac_addr->addr[4] = ip4_addr3_16(ip_addr);
  mac_addr->addr[5] = ip4_addr4_16(ip_addr);

  return ERR_OK;
}

/*
 * Include or remove group address from accepted Rx frames filters
 *
 * @param netif network interface
 * @param group goup address
 * @param action request one of IGMP_ADD_MAC_FILTER or IGMP_DEL_MAC_FILTER
 */

static err_t mac_filter(struct netif *netif,
                        const ip4_addr_t *group, u8_t action)
{
  LWIP_DEBUGF(NETIF_DEBUG | LWIP_DBG_STATE,
                ("lpcnetif: adding group mac filter"));

  /* Receive all frames without filtering */
  /* LPC_EMAC->Command |= EMAC_CMD_PASS_RX_FLT; */

  /* Receive all multicast frames without HASH filtering */
  /* LPC_EMAC->RxFilterCtrl |= EMAC_RFC_MULTICAST_ENB; */

  if (1) {
    struct eth_addr mac_addr;
    uint32_t hash;
    uint32_t hasmap_lo = 0;
    uint32_t hasmap_hi = 0;
    int i;

    multicast_ip_to_mac(group, &mac_addr);
    hash = mac_hash_fast(mac_addr.addr);

    if (action == IGMP_ADD_MAC_FILTER) {
      if (mac_hash_use_count[hash] < 255)
        mac_hash_use_count[hash]++;
    } else if (action == IGMP_DEL_MAC_FILTER) {
      if (mac_hash_use_count[hash])
        mac_hash_use_count[hash]--;
    }

    for (i = 0; i < 32; i++)
      if (mac_hash_use_count[i])
        hasmap_lo |= 1 << i;
    for (i = 0; i < 32; i++)
      if (mac_hash_use_count[i + 32])
        hasmap_hi |= 1 << i;

    LPC_EMAC->HashFilterL = hasmap_lo;
    LPC_EMAC->HashFilterH = hasmap_hi;

    /* Receive multicast frames which pass HASH filtering */
    LPC_EMAC->RxFilterCtrl |= EMAC_RFC_MULTICAST_HASH_ENB;
  }

  return ERR_OK;
}

#endif

/*---------------------------------------------------------------------------*/

static int low_level_reinit(uint32_t phyidx)
{
  uint32_t regv = 0;
  volatile uint32_t tcnt;

  /* read PHY-PHYSTS and check if AutoNegotiation is complete */
  switch(phyidx) {
    case DP83848C_DEF_ID:
      for (tcnt = 0; tcnt < 1000; ++tcnt) {
        regv = read_PHY(DP83848X_REG_PHYSTS);
        if (regv & DP83848X_PHYSTS_AN_COMPLETE) break;
      }
      if (!(regv & DP83848X_PHYSTS_AN_COMPLETE)) {
        scan_PHY_status();
        LWIP_DEBUGF(NETIF_DEBUG, ("PHY autoneg.not completed 0x%lX\n", regv));
        return -1;
      }
      /* Configure Full/Half Duplex mode. */
      if (regv & DP83848X_PHYSTS_DUPLEX_STATUS) { /* Full duplex is enabled. */
        LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUPLEX;
        LPC_EMAC->Command |= EMAC_CMD_FULL_DUPLEX;
        LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
      } else {                                    /* Half duplex mode. */
        LPC_EMAC->MAC2    &= ~EMAC_MAC2_FULL_DUPLEX;
        LPC_EMAC->Command &= ~EMAC_CMD_FULL_DUPLEX;
        LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
      }
      /* Configure 100MBit/10MBit mode. */
      if (regv & DP83848X_PHYSTS_SPEED_STATUS)  /* 10MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED_10MBPS;
      else                                      /* 100MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED_100MBPS;
      break;

    case LAN8720A_DEF_ID:
      for (tcnt = 0; tcnt < 1000; ++tcnt) {
        regv = read_PHY(LAN8720A_REG_PCSR);
        if (regv & LAN8720A_PCSR_AUTODONE) break;
      }
      if (!(regv & LAN8720A_PCSR_AUTODONE)) {
        scan_PHY_status();
        LWIP_DEBUGF(NETIF_DEBUG, ("PHY autoneg.not completed 0x%lX\n", regv));
        return -1;
      }
      /* Configure Full/Half Duplex mode. */
      if (regv & LAN8720A_PCSR_DUPLEX) { /* Full duplex is enabled. */
        LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUPLEX;
        LPC_EMAC->Command |= EMAC_CMD_FULL_DUPLEX;
        LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
      } else {                                    /* Half duplex mode. */
        LPC_EMAC->MAC2    &= ~EMAC_MAC2_FULL_DUPLEX;
        LPC_EMAC->Command &= ~EMAC_CMD_FULL_DUPLEX;
        LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
      }
      /* Configure 100MBit/10MBit mode. */
      if (regv & LAN8720A_PCSR_SPEED_10M) {           /* 10MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED_10MBPS;
      } else if (regv & LAN8720A_PCSR_SPEED_100M) {   /* 100MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED_100MBPS;
      }
      break;

    default:
      return -2;
  }

  scan_PHY_status();
  return 0;
}


/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this lpcnetif
 */
static int low_level_init(struct netif *netif)
{
  int rc = -1;
  int i;
  struct lpc_netif_data *ldata = (struct lpc_netif_data *) netif->state;
  const lpc_emac_know_phy_t *phydes;
  uint32_t regv, id1, id2, phyid;
  volatile uint32_t tout;

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  for (i = 0; i < netif->hwaddr_len; i++)
    netif->hwaddr[i] = ldata->ethaddr.addr[i];

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  #if LWIP_IGMP
  netif->flags |= NETIF_FLAG_IGMP;
  netif_set_igmp_mac_filter(netif, mac_filter);
  #endif

  /* Do whatever else is needed to initialize interface. */

  /* Power Up the EMAC controller. */
  LPC_SC->PCONP |= 0x40000000;
  /* Enable P1 Ethernet Pins. */
  /* on rev. 'A' and later, P1.6 should NOT be set. */
 #ifdef LPC_PINCON
  LPC_PINCON->PINSEL2 = 0x50150105;

  LPC_PINCON->PINSEL3 = (LPC_PINCON->PINSEL3 & ~0x0000000F) | 0x00000005;
 #endif /*LPC_PINCON*/

  /* Reset all EMAC internal modules. */
  LPC_EMAC->MAC1 = EMAC_MAC1_RST_TX | EMAC_MAC1_RST_MCS_TX | EMAC_MAC1_RST_RX | EMAC_MAC1_RST_MCS_RX | EMAC_MAC1_SIM_RST | EMAC_MAC1_SOFT_RST;
  LPC_EMAC->Command = EMAC_CMD_REG_RST | EMAC_CMD_TX_RST | EMAC_CMD_RX_RST | EMAC_CMD_PASS_RUNT_FRM;

  /* set MDIO clock to be about 2.5 MHz */
  LPC_EMAC->MCFG = (LPC_EMAC->MCFG & ~EMAC_MCFG_CLK_SEL_MASK) |
                   EMAC_MCFG_CLK_SEL_DIV36;
  /* A short delay after reset. */
  for (tout = 100; tout; tout--);

  /* Initialize MAC control registers. */
  LPC_EMAC->MAC1 = EMAC_MAC1_PASS_ALL;
  LPC_EMAC->MAC2 = EMAC_MAC2_CRC_ENB | EMAC_MAC2_PAD_ENB;

  LPC_EMAC->MAXF = netif->mtu; /* ETH_MAX_FLEN; */
  LPC_EMAC->CLRT = EMAC_CLRT_RESET_VALUE;
  LPC_EMAC->IPGR = EMAC_IPGR_RECOMMENDED;

  /* Enable Reduced MII interface. */
  LPC_EMAC->Command = EMAC_CMD_RMII | EMAC_CMD_PASS_RUNT_FRM;

/* part for specific PHY */
  /* Check if this is known PHY. */
  phyid = 0;
  for (i = 4; i > 0; i--) {
    id1 = read_PHY(DP83848X_REG_PHYIDR1);
    id2 = read_PHY(DP83848X_REG_PHYIDR2);
    if (!id1 || !id2 || (id1 == 0xffff) || (id2 == 0xffff))
      continue;
    if (((id1 << 16) | id2) == phyid)
      break;
    phyid = (id1 << 16) | id2;
  }

  for (phydes = lpc_emac_know_phy; phydes->phyid; phydes++) {
    if (!((phydes->phyid ^ phyid) & ~phydes->maskoutid)) {
      LWIP_DEBUGF(NETIF_DEBUG, ("Supported PHY (0x%04lX,0x%04lX)\n", id1, id2));
      ldata->phyid = phyid;
      break;
    }
  }

  if (phydes->phyid) {
    do {

      /* Put the DP83848C in reset mode */
      write_PHY(DP83848X_REG_BMCR, DP83848X_BMCR_RESET);

      /* Wait for hardware reset to end. */
      for (tout = 0; tout < 0x10000; tout++) {
        regv = read_PHY(DP83848X_REG_BMCR);
        if (!(regv & DP83848X_BMCR_RESET)) /* Check if reset is complete */
          break;
      }
      if (regv & DP83848X_BMCR_RESET) break; /* still in RESET - skip EMAC init */

      /* Use autonegotiation about the link speed. */
      write_PHY(DP83848X_REG_BMCR, DP83848X_BMCR_SPEED_100M | DP83848X_BMCR_ENABLE_AN);

      /* Try to wait to complete Auto_Negotiation and initialize EMAC (speed, duplex). */
      for (tout = 0; tout < 10; ++tout) {
        rc = low_level_reinit(ldata->phyid);
        if (!rc) break; /* AN completed and PHY/EMAC set */
      }

      rc = 0; /* it argues that HW is OK to initiate all other structures */
    } while(0);
  }

  /* Set the Ethernet MAC Address registers */
  LPC_EMAC->SA0 = ((u16_t)netif->hwaddr[5] << 8) | (u16_t)netif->hwaddr[4];
  LPC_EMAC->SA1 = ((u16_t)netif->hwaddr[3] << 8) | (u16_t)netif->hwaddr[2];
  LPC_EMAC->SA2 = ((u16_t)netif->hwaddr[1] << 8) | (u16_t)netif->hwaddr[0];

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Receive Broadcast and Perfect Match Packets */
  LPC_EMAC->RxFilterCtrl = EMAC_RFC_BROADCAST_ENB | EMAC_RFC_PERFECT_ENB;

  /* Enable EMAC interrupts. */
  LPC_EMAC->IntEnable = EMAC_INT_RX_DONE | EMAC_INT_TX_DONE;

  /* Reset all interrupts */
  LPC_EMAC->IntClear  = ~0;

  /* Enable receive and transmit mode of MAC Ethernet core */
  LPC_EMAC->Command  |= (EMAC_CMD_RX_ENB | EMAC_CMD_TX_ENB);
  LPC_EMAC->MAC1     |= EMAC_MAC1_RCV_ENB;

  return rc;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this lpcnetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
//  struct lpcnetif *lpcnetif = netif->state;
  struct pbuf *q;

  /* initiate transfer(); */
  unsigned char *piSource;
  unsigned short TxLen = 0;

  volatile unsigned int idx = LPC_EMAC->TxProduceIndex;
  unsigned char *tptr = (unsigned char *)EMAC_TX_DESCRIPTOR(idx)->ptrpktbuff;
  EMAC_TX_DESCRIPTOR(idx)->control = (p->tot_len & EMAC_TXCTRL_SIZE) | EMAC_TXCTRL_LAST;

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  for(q = p; q != NULL; q = q->next) {
    /* Send the data from the pbuf to the interface, one pbuf at a
       time. The size of the data in each pbuf is kept in the ->len
       variable. */
   /* send data from(q->payload, q->len); */
    piSource = q->payload;
    TxLen = q->len;    /* round Size up to next even number */
    while (TxLen > 0) {
      *tptr++ = *piSource++;
      TxLen--;
    }
  }

  /* signal that packet should be sent(); */
  idx = LPC_EMAC->TxProduceIndex;
  if (++idx == EMAC_NUM_TX_FRAG) idx = 0;
  LPC_EMAC->TxProduceIndex = idx;

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  LINK_STATS_INC(link.xmit);

  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this lpcnetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif)
{
//  struct lpcnetif *lpcnetif = netif->state;
  struct pbuf *p, *q;
  u16_t len;

  /* Obtain the size of the packet and put it into the "len"
     variable. */
  volatile unsigned int idx = LPC_EMAC->RxConsumeIndex;

  if (LPC_EMAC->RxProduceIndex == idx) { /* no new packet received ? */
    len = 0;
    return NULL;
  } else {
    /* start receiving from EMAC */
    len = (EMAC_RX_STATUS(idx)->info & EMAC_RXSTAT_INFO_RXSIZE) - 3;
  }
  unsigned char *rptr = (unsigned char *)EMAC_RX_DESCRIPTOR(idx)->ptrpktbuff;
  unsigned char *pout;
  unsigned short cnt;

#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL && len>0) {

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

    /* We iterate over the pbuf chain until we have read the entire
     * packet into the pbuf. */
    for(q = p; q != NULL; q = q->next) {
      /* Read enough bytes to fill this pbuf in the chain. The
       * available data in the pbuf is given by the q->len
       * variable.
       * This does not necessarily have to be a memcpy, you can also preallocate
       * pbufs for a DMA-enabled MAC and after receiving truncate it to the
       * actually received size. In this case, ensure the tot_len member of the
       * pbuf is the sum of the chained pbuf len members.
       */
      /* read data into(q->payload, q->len); */
      cnt = q->len;
      pout = q->payload;

      while(cnt>0) {
        *pout++ = *rptr++;
        cnt--;
      }
    }
    /* acknowledge that packet has been read(); */
    idx = LPC_EMAC->RxConsumeIndex;
    if (++idx == EMAC_NUM_RX_FRAG) idx = 0;
    LPC_EMAC->RxConsumeIndex = idx;

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.recv);
  } else {
    /* drop packet(); */
    LINK_STATS_INC(link.memerr);
    LINK_STATS_INC(link.drop);
  }

  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this lpcnetif
 */
void lpcnetif_input(struct netif *netif)
{
  struct eth_hdr *ethhdr;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);
  /* no packet could be read, silently ignore this */
  if (p == NULL) return;
  /* points to packet payload, which starts with an Ethernet header */
  ethhdr = p->payload;

  switch (htons(ethhdr->type)) {
  /* IP or ARP packet? */
  case ETHTYPE_IP:
  case ETHTYPE_IPV6:
  case ETHTYPE_ARP:
#if PPPOE_SUPPORT
  /* PPPoE packet? */
  case ETHTYPE_PPPOEDISC:
  case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
    /* full packet send to tcpip_thread to process */
    if (netif->input(p, netif)!=ERR_OK)
     { LWIP_DEBUGF(NETIF_DEBUG, ("lpcnetif_input: IP input error\n"));
       pbuf_free(p);
       p = NULL;
     }
    break;

  default:
    pbuf_free(p);
    p = NULL;
    break;
  }
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this lpcnetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t lpcnetif_init(struct netif *netif)
{
  struct lpc_netif_data *lpcnetif_data;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

  lpcnetif_data = mem_malloc(sizeof(struct lpc_netif_data));
  if (lpcnetif_data == NULL) {
    LWIP_DEBUGF(NETIF_DEBUG, ("lpcnetif_init: out of memory\n"));
    return ERR_MEM;
  }
  if (netif->state) { /* state can be set to user HWaddress (MAC) in application level */
    int i;
    for (i=0;i<ETHARP_HWADDR_LEN;++i) lpcnetif_data->ethaddr.addr[i] = *((uint8_t *)netif->state +i);
  } else {            /* or it can be generated */
    int i;
    for (i=0;i<ETHARP_HWADDR_LEN;++i) lpcnetif_data->ethaddr.addr[i] = *((uint8_t *)i); /* TODO: get from ResetHandler :) */
  }
  lpcnetif_data->lastlinkstate = 0;
  netif->state = lpcnetif_data;


#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
  netif->linkoutput = low_level_output;

//  lpcnetif_data->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  /* initialize the hardware */
  if (low_level_init(netif)) {
    lpcnetif_data->lastlinkstate &= ~0x04; /* AN not completed or EMAC not initiated */
    return ERR_IF;
  }
  lpcnetif_data->lastlinkstate |= 0x04; /* AN completed, EMAC initiated */

  return ERR_OK;
}

/**
 * It should be called if LINK status is changed to GOOD.
 */
err_t lpcnetif_re_init(struct netif *netif)
{
  struct lpc_netif_data *ldata = (struct lpc_netif_data *) netif->state;
  if (low_level_reinit(ldata->phyid)) {
    ldata->lastlinkstate &= ~0x04;
    return ERR_IF;
  }
  ldata->lastlinkstate |= 0x04;
  return ERR_OK;
}

/**
 * check LinkStatus - it returns
 *     0: link up and OK
 *    -1: link down
 *    -2: link switched to down
 *     1: link up,but not reinitialized
 *     2: link switched to up
 */
int lpcnetif_checklink(struct netif *netif)
{
  int rc = -1;
  struct lpc_netif_data *ldata = (struct lpc_netif_data *) netif->state;
  if (LPC_EMAC->MIND & EMAC_MIND_MII_LINK_FAIL) {   /* the actual state of the link - DOWN */
    if (!(ldata->lastlinkstate & 0x01)) {
      rc = -2;
    } else {
      rc = -1;
    }
    ldata->lastlinkstate &= ~0x02; /* link is not up */
    ldata->lastlinkstate |= 0x01;  /* link is down */
  } else {                                      /* the actual state of the link - UP */
    if (!(ldata->lastlinkstate & 0x02)) {
      rc = 2;
      lpcnetif_re_init(netif); /* try to re-initialize PHY+EMAC */
    } else {
      if (ldata->lastlinkstate & 0x04) { /* test if PHY+EMAC is initialized */
        rc = 0;
      } else {
        lpcnetif_re_init(netif); /* PHY+EMAC not initialized - try again */
        rc = 1;
      }
    }
    ldata->lastlinkstate &= ~0x01; /* link is not down */
    ldata->lastlinkstate |= 0x02;  /* link is up */
  }
  return rc;
}

/* -------------------------------------------------------------------------- */
#if 0
/* Example how to initialize LPC EMAC interface */
#include <arch/lwip-lpc.h>

static struct netif lpc_emac_netif;

struct netif *init_netif(void)
{
  static ip_addr_t ipaddr, netmask, gw;

  IP4_ADDR(&gw, 192,168,0,1);
  IP4_ADDR(&ipaddr, 192,168,0,2);
  IP4_ADDR(&netmask, 255,255,255,0);

  if (netif_add(&lpc_emac_netif, &ipaddr, &netmask, &gw, NULL, lpcnetif_init, ethernet_input)
      != &lpc_emac_netif) {
    return NULL;
  }
  return &lpc_emac_netif;
}
#endif
