#ifndef _LWIP_LPC_HEADER_FILE_
#define _LWIP_LPC_HEADER_FILE_

#include "lwip/init.h"
#include "lwip/err.h"
#include "lwip/tcp.h"
#include <netif/etharp.h>
#include <lwip/tcp_impl.h>


void lpcnetif_input(struct netif *netif);
/* init netif - call once */
err_t lpcnetif_init(struct netif *netif);
/* reinit after AutoNeg. when LinkStatus has been changed from FAIL to OK */
err_t lpcnetif_re_init(struct netif *netif);
/* check LinkStatus - it returns 0:OK; -1:link down; -2: link switched to down, 1:link up,but not reinitialized, 2:link switched to up */
int lpcnetif_checklink(struct netif *netif);

#define LPCNETIF_LINK_OK        (0)
#define LPCNETIF_LINK_DOWN      (-1)
#define LPCNETIF_LINK_GO_DOWN   (-2)
#define LPCNETIF_LINK_GO_UP     (2)
#define LPCNETIF_LINK_W4INIT    (1)

/* release memory with netif privite data (not necessary) ... and it should disable the netif */
//err_t lpcnetif_done(struct netif *netif);

#endif /* _LWIP_LPC_HEADER_FILE_ */
