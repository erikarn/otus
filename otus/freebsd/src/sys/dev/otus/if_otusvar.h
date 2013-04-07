#ifndef	__IF_OTUSVAR_H__
#define	__IF_OTUSVAR_H__

#define	OTUS_CONFIG_INDEX		0
#define	OTUS_IFACE_INDEX		0

/*
 * The carl9170 firmware has the following specification:
 *
 * 0 - USB control
 * 1 - TX
 * 2 - RX
 * 3 - IRQ
 * 4 - CMD
 * ..
 * 10 - end
 */
enum {
	OTUS_BULK_TX,
	OTUS_BULK_RX,
	OTUS_BULK_IRQ,
	OTUS_BULK_CMD,
	OTUS_N_XFER
};

/* XXX the TX/RX endpoint dump says it's 0x200, (512)? */
#define OTUS_MAX_TXSZ           512
#define OTUS_MAX_RXSZ           512
/* intr/cmd endpoint dump says 0x40 */
#define OTUS_MAX_CTRLSZ         64

struct otus_softc {
	device_t		sc_dev;
	struct usb_device	*sc_udev;
	struct ifnet		*sc_ifp;
	struct mtx		sc_mtx;
	struct usb_xfer		*sc_xfer[OTUS_N_XFER];
};

#endif	/* __IF_OTUSVAR__ */
