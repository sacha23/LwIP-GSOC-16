#ifndef _LWIP_LPC_HEADER_FILE_
#define _LWIP_LPC_HEADER_FILE_

#include "lwip/init.h"
#include "lwip/err.h"
#include "lwip/tcp.h"
#include <netif/etharp.h>
#include <lwip/tcp_impl.h>


void lpcnetif_input(struct netif *netif);
err_t lpcnetif_init(struct netif *netif);
//struct netif *init_netif(void);

#endif /* _LWIP_LPC_HEADER_FILE_ */
