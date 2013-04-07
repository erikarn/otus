/*-
 * Copyright (c) 2013 Adrian Chadd <adrian@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*-
 * Driver for Atheros AR9170 based parts.
 *
 * For now this driver doesn't support 802.11n.
 * Supporting 802.11n transmit requires TX aggregation handling
 * to be done.
 */
#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/linker.h>
#include <sys/firmware.h>
#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "usbdevs.h"

#include "if_otusreg.h"
#include "if_otusvar.h"

/* unaligned little endian access */
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

/* recognized device vendors/products */
static const STRUCT_USB_HOST_ID otus_devs[] = {
	/* TP-Link TL-WN821N v2 - XXX has WPS button and one LED */
	{ USB_VP(0x0cf3, 0x1002) },
};

static usb_callback_t	otus_bulk_tx_callback;
static usb_callback_t	otus_bulk_rx_callback;
static usb_callback_t	otus_bulk_irq_callback;
static usb_callback_t	otus_bulk_cmd_callback;
static int		otus_attach(device_t self);
static int		otus_detach(device_t self);

static const struct usb_config otus_config[OTUS_N_XFER] = {
	[OTUS_BULK_TX] = {
	.type = UE_BULK,
	.endpoint = 1,
	.direction = UE_DIR_OUT,
	.bufsize = OTUS_MAX_TXSZ,
	.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
	.callback = otus_bulk_tx_callback,
	.timeout = 5000,        /* ms */
	},
	[OTUS_BULK_RX] = {
	.type = UE_BULK,
	.endpoint = 2,
	.direction = UE_DIR_IN,
	.bufsize = OTUS_MAX_RXSZ,
	.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
	.callback = otus_bulk_rx_callback,
	},
	[OTUS_BULK_IRQ] = {
	.type = UE_INTERRUPT,
	.endpoint = 3,
	.direction = UE_DIR_IN,
	.bufsize = OTUS_MAX_CTRLSZ,
	.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
	.callback = otus_bulk_irq_callback,
	},
	[OTUS_BULK_CMD] = {
	.type = UE_INTERRUPT,
	.endpoint = 4,
	.direction = UE_DIR_OUT,
	.bufsize = OTUS_MAX_CTRLSZ,
	.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
	.callback = otus_bulk_cmd_callback,
	.timeout = 5000,        /* ms */
	},
};

static int
otus_match(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	if (uaa->info.bConfigIndex != OTUS_CONFIG_INDEX)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != OTUS_IFACE_INDEX)
		return (ENXIO);

	return (usbd_lookup_id_by_uaa(otus_devs, sizeof(otus_devs), uaa));
}

static int
otus_attach(device_t dev)
{
	struct otus_softc *sc = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	uint8_t iface_index;
	int error;

	device_printf(dev, "%s: called\n", __func__);

	device_set_usb_desc(dev);
	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;

	mtx_init(&sc->sc_mtx, device_get_nameunit(sc->sc_dev),
	    MTX_NETWORK_LOCK, MTX_DEF);

	iface_index = OTUS_IFACE_INDEX;

	error = usbd_transfer_setup(uaa->device, &iface_index,
		sc->sc_xfer, otus_config, OTUS_N_XFER, sc, &sc->sc_mtx);
	if (error) {
		device_printf(dev, "could not allocate USB transfers, "
		    "err=%s\n", usbd_errstr(error));
		goto detach;
	}

	return (0);

detach:
	otus_detach(dev);
	return (ENXIO);
}

static int
otus_detach(device_t dev)
{
	struct otus_softc *sc = device_get_softc(dev);

	if (sc->sc_xfer != NULL)
		usbd_transfer_unsetup(sc->sc_xfer, OTUS_N_XFER);

	device_printf(dev, "%s: called\n", __func__);
	mtx_destroy(&sc->sc_mtx);
	return (0);
}

static void
otus_bulk_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct otus_softc *sc = usbd_xfer_softc(xfer);
	int actlen;
	int sumlen;

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	device_printf(sc->sc_dev,
	    "%s: called; state=%d\n",
	    __func__,
	    USB_GET_STATE(xfer));

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		/*
		 * Setup xfer frame lengths/count and data
		 */
		usbd_transfer_submit(xfer);
	break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		break;
	default: /* Error */
		/*
		 * Print error message and clear stall
		 * for example.
		 */
		break;
	}
}

static void
otus_bulk_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct otus_softc *sc = usbd_xfer_softc(xfer);
	int actlen;
	int sumlen;

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	device_printf(sc->sc_dev,
	    "%s: called; state=%d\n",
	    __func__,
	    USB_GET_STATE(xfer));

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		/*
		 * Setup xfer frame lengths/count and data
		 */
		usbd_transfer_submit(xfer);
	break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		break;
	default: /* Error */
		/*
		 * Print error message and clear stall
		 * for example.
		 */
		break;
	}
}

static void
otus_bulk_cmd_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct otus_softc *sc = usbd_xfer_softc(xfer);
	int actlen;
	int sumlen;

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	device_printf(sc->sc_dev,
	    "%s: called; state=%d\n",
	    __func__,
	    USB_GET_STATE(xfer));

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		/*
		 * Setup xfer frame lengths/count and data
		 */
		usbd_transfer_submit(xfer);
	break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		break;
	default: /* Error */
		/*
		 * Print error message and clear stall
		 * for example.
		 */
		break;
	}
}

static void
otus_bulk_irq_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct otus_softc *sc = usbd_xfer_softc(xfer);
	int actlen;
	int sumlen;

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	device_printf(sc->sc_dev,
	    "%s: called; state=%d\n",
	    __func__,
	    USB_GET_STATE(xfer));

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		/*
		 * Setup xfer frame lengths/count and data
		 */
		usbd_transfer_submit(xfer);
	break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		break;
	default: /* Error */
		/*
		 * Print error message and clear stall
		 * for example.
		 */
		break;
	}
}

static device_method_t otus_methods[] = {
	DEVMETHOD(device_probe, otus_match),
	DEVMETHOD(device_attach, otus_attach),
	DEVMETHOD(device_detach, otus_detach),
	{ 0, 0 }
};
static driver_t otus_driver = {
	.name = "otus",
	.methods = otus_methods,
	.size = sizeof(struct otus_softc)
};
static devclass_t otus_devclass;

DRIVER_MODULE(otus, uhub, otus_driver, otus_devclass, NULL, 0);
MODULE_DEPEND(otus, wlan, 1, 1, 1);
MODULE_DEPEND(otus, usb, 1, 1, 1);
MODULE_VERSION(otus, 1);
