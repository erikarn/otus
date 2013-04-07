#ifndef	__IF_OTUS_DEBUG__
#define	__IF_OTUS_DEBUG__

enum {
	OTUS_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	OTUS_DEBUG_XMIT_DUMP	= 0x00000002,	/* xmit dump */
	OTUS_DEBUG_RECV		= 0x00000004,	/* basic recv operation */
	OTUS_DEBUG_TX_PROC	= 0x00000008,	/* tx ISR proc */
	OTUS_DEBUG_RX_PROC	= 0x00000010,	/* rx ISR proc */
	OTUS_DEBUG_RECV_ALL	= 0x00000020,	/* trace all frames (beacons) */
	OTUS_DEBUG_INIT		= 0x00000040,	/* initialization of dev */
	OTUS_DEBUG_DEVCAP	= 0x00000080,	/* dev caps */
	OTUS_DEBUG_CMDS		= 0x00000100,	/* commands */
	OTUS_DEBUG_CMDS_DUMP	= 0x00000200,	/* command buffer dump */
	OTUS_DEBUG_RESET	= 0x00000400,	/* reset processing */
	OTUS_DEBUG_STATE	= 0x00000800,	/* 802.11 state transitions */
	OTUS_DEBUG_MULTICAST	= 0x00001000,	/* multicast */
	OTUS_DEBUG_WME		= 0x00002000,	/* WME */
	OTUS_DEBUG_CHANNEL	= 0x00004000,	/* channel */
	OTUS_DEBUG_RATES	= 0x00008000,	/* rates */
	OTUS_DEBUG_CRYPTO	= 0x00010000,	/* crypto */
	OTUS_DEBUG_LED		= 0x00020000,	/* LED */
	OTUS_DEBUG_ANY		= 0xffffffff
};
#define DPRINTF(sc, m, fmt, ...) do {			\
	if (sc->sc_debug & (m))				\
		printf(fmt, __VA_ARGS__);		\
} while (0)

#endif	/* __IF_OTUS_DEBUG__ */
