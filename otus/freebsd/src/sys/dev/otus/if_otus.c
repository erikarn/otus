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
#include "if_otus_firmware.h"
#include "if_otus_cmd.h"
#include "if_otusvar.h"

#include "if_otus_sysctl.h"

#include "fwcmd.h"

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

/*
 * Command send/receive/queue functions.
 *
 * These (partially) belong in if_otus_cmd.[ch], however until the
 * "right" abstraction is written up, this'll have to do.
 */
/*
 * Low-level function to send read or write commands to the firmware.
 */
static int
otus_cmdsend(struct otus_softc *sc, uint32_t code, const void *idata,
    int ilen, void *odata, int olen, int flags)
{
	struct carl9170_cmd_head *hdr;
	struct otus_cmd *cmd;
//	int error;

	OTUS_LOCK_ASSERT(sc);

	/* grab a xfer */
	cmd = otus_get_cmdbuf(sc);
	if (cmd == NULL) {
		device_printf(sc->sc_dev, "%s: empty inactive queue\n",
		    __func__);
		return (ENOBUFS);
	}

	/*
	 * carl9170 header fields:
	 * + len
	 * + cmd
	 * + seq
	 * + ext
	 *
	 * XXX the Linux headers use u8 fields which the compiler may decide
	 * to align.  We need to ensure this doesn't occur!
	 */
	hdr = (struct carl9170_cmd_head *)cmd->buf;
	bzero(hdr, sizeof (struct carl9170_cmd_head));	/* XXX not needed */
	hdr->h.c.cmd = code;
	hdr->h.c.len = ilen;
	hdr->h.c.seq = 0; /* XXX */
	hdr->h.c.ext = 0; /* XXX */

	/* Copy the payload, if needed */
	/* XXX TODO: check payload len? */
	bcopy(idata, (uint8_t *)(hdr + 1), ilen);

	/*
	 * Finalise buffer configuration.
	 */
	cmd->flags = flags;

	/* XXX should I ensure I always TX a multiple of 4 bytes? */
	/* XXX 4 == (sizeof(struct carl9170_cmd_head)) */
	cmd->buflen = ilen + 4;

#ifdef OTUS_DEBUG
	if (sc->sc_debug & OTUS_DEBUG_CMDS) {
	printf("%s: send  %d [flags 0x%x] olen %d\n",
	    __func__, code, cmd->flags, olen);
#if 0
	if (sc->sc_debug & OTUS_DEBUG_CMDS_DUMP)
		otus_dump_cmd(cmd->buf, cmd->buflen, '+');
	}
#endif
#endif

	cmd->odata = odata;
	KASSERT(odata == NULL ||
	    olen < OTUS_MAX_CMDSZ - sizeof(*hdr) + sizeof(uint32_t),
	    ("odata %p olen %u", odata, olen));
	 cmd->olen = olen;

	STAILQ_INSERT_TAIL(&sc->sc_cmd_pending, cmd, next);
	OTUS_STAT_INC(sc, st_cmd_pending);
	
	/* Kick off the actual USB command queue transaction */
	usbd_transfer_start(sc->sc_xfer[OTUS_BULK_CMD]);

	/*
	 * XXX check to see whether command responses come in the
	 * IRQ or RX queue.
	 *
	 * XXX TODO: It looks like it's actually pushing the command
	 * response into the RX queue, not the IRQ queue.
	 *
	 * So I'm going to have to dig deeper into this and figure
	 * out the best way to separate command responses and
	 * receive frames.
	 */
	if (cmd->flags & OTUS_CMD_FLAG_READ) {
		/* XXX enable both for now, just to see what happens */
		usbd_transfer_start(sc->sc_xfer[OTUS_BULK_IRQ]);
		usbd_transfer_start(sc->sc_xfer[OTUS_BULK_RX]);

		/*
		 * For now, let's not support synchronous command
		 * sending.
		 *
		 * I'll look into supporting this in a more generic
		 * fashion later.
		 */
#if 0
                /* wait at most two seconds for command reply */
                error = mtx_sleep(cmd, &sc->sc_mtx, 0, "otuscmd", 2 * hz);
                cmd->odata = NULL;      /* in case reply comes too late */
                if (error != 0) {
                        device_printf(sc->sc_dev, "timeout waiting for reply "
                            "to cmd 0x%x (%u)\n", code, code);
                } else if (cmd->olen != olen) {
                        device_printf(sc->sc_dev, "unexpected reply data count "
                            "to cmd 0x%x (%u), got %u, expected %u\n",
                            code, code, cmd->olen, olen);
                        error = EINVAL;
                }
                return (error);
#endif
	}
	return (0);
}

static int
otus_cmd_read(struct otus_softc *sc, uint32_t code, const void *idata,
    int ilen, void *odata, int olen, int flags)
{

	flags |= OTUS_CMD_FLAG_READ;
	return otus_cmdsend(sc, code, idata, ilen, odata, olen, flags);
}

#if 0
static int
otus_cmd_write(struct otus_softc *sc, uint32_t code, const void *data,
    int len, int flags)
{

	flags &= ~OTUS_CMD_FLAG_READ;
	return otus_cmdsend(sc, code, data, len, NULL, 0, flags);
}
#endif

static int
otus_cmd_send_ping(struct otus_softc *sc, uint32_t value)
{
	uint32_t retval = 0xf0f0f0f0;
	int r;

	/*
	 * ECHO request / response is a READ request with the response
	 * coming back in the read response.  Nice and straightforward.
	 */
	r = otus_cmd_read(sc, CARL9170_CMD_ECHO, (void *) &value, 4,
	    (void *) &retval, 4, 0);

	if (r != 0) {
		device_printf(sc->sc_dev, "%s: cmd_send failed\n", __func__);
//		return (EIO);
		return (0);
	}

	if (retval != value) {
		device_printf(sc->sc_dev, "%s: PING: sent 0x%08x, got 0x%08x\n",
		    __func__,
		    value,
		    retval);
		
//		return (EIO);
		return (0);
	}

	return (0);
}


static void
otus_cmd_dump(struct otus_softc *sc, struct otus_cmd *cmd)
{
	int i;

	device_printf(sc->sc_dev, "CMD:");
	for (i = 0; i < cmd->buflen; i++) {
		printf(" %02x", cmd->buf[i] & 0xff);
	}
	printf("\n");
}

#define	AR_FW_DOWNLOAD		0x30
#define	AR_FW_DOWNLOAD_COMPLETE	0x31

static int
otus_load_microcode(struct otus_softc *sc)
{
	usb_device_request_t req;
//	int error;
	const uint8_t *ptr;
	int addr;
	int size, mlen;
	char *buf;

	ptr = sc->fwinfo.fw->data;
	addr = sc->fwinfo.address;
	size = sc->fwinfo.fw->datasize;

	/*
	 * Skip the offset
	 */
	ptr += sc->fwinfo.offset;
	size -= sc->fwinfo.offset;

	buf = malloc(4096, M_TEMP, M_NOWAIT | M_ZERO);
	if (buf == NULL) {
		device_printf(sc->sc_dev, "%s: malloc failed\n",
		    __func__);
		return (ENOMEM);
	}

	OTUS_LOCK(sc);

	/*
	 * Initial firmware load.
	 */
	bzero(&req, sizeof(req));
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD;
	USETW(req.wIndex, 0);

	addr >>= 8;
	while (size > 0) {
		mlen = MIN(size, 4096);
		memcpy(buf, ptr, mlen);

		USETW(req.wValue, addr);
		USETW(req.wLength, mlen);
		device_printf(sc->sc_dev, "%s: loading %d bytes at 0x%08x\n",
		    __func__,
		    mlen,
		    addr);
		if (usbd_do_request(sc->sc_udev, &sc->sc_mtx, &req, buf)) {
			device_printf(sc->sc_dev,
			    "%s: failed to write firmware\n", __func__);
			OTUS_UNLOCK(sc);
			free(buf, M_TEMP);
			return (EIO);
		}
		addr += mlen >> 8;
		ptr  += mlen;
		size -= mlen;
	}

	/*
	 * Firmware complete.
	 */
	bzero(&req, sizeof(req));
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD_COMPLETE;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	if (usbd_do_request(sc->sc_udev, &sc->sc_mtx, &req, NULL)) {
		device_printf(sc->sc_dev,
		    "%s: firmware initialization failed\n",
		    __func__);
		OTUS_UNLOCK(sc);
		free(buf, M_TEMP);
		return (EIO);
	}

	OTUS_UNLOCK(sc);
	free(buf, M_TEMP);

	/*
	 * It's up to the caller to check whether we were successful.
	 */
	return (0);
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

	/*
	 * Allocate xfers for firmware commands.
	 */
	error = otus_alloc_cmd_list(sc, sc->sc_cmd, OTUS_CMD_LIST_COUNT,
	    OTUS_MAX_CMDSZ);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate Tx command list\n");
		goto detach;
	}

	/*
	 * Setup USB transfer pipes.
	 */
	iface_index = OTUS_IFACE_INDEX;
	error = usbd_transfer_setup(uaa->device, &iface_index,
		sc->sc_xfer, otus_config, OTUS_N_XFER, sc, &sc->sc_mtx);
	if (error) {
		device_printf(dev, "could not allocate USB transfers, "
		    "err=%s\n", usbd_errstr(error));
		goto detach;
	}

	/*
	 * Attach sysctl nodes.
	 */
	if_otus_sysctl_attach(sc);

	/*
	 * XXX Can we do a detach at any point? Ie, can we defer the
	 * XXX rest of this setup path and if attach fails, just
	 * XXX call detach from some taskqueue/callout?
	 */

	/*
	 * Load in the firmware, so we know the firmware config and
	 * capabilities.
	 */
	error = otus_firmware_load(&sc->fwinfo);
	if (error != 0)
		goto detach;

	/*
	 * Squeeze in firmware at attach phase for now.
	 */
	error = otus_load_microcode(sc);
	if (error != 0)
		goto detach;

	/* XXX doing a ping here is likely evil (we're in probe/attach!) .. */
	OTUS_LOCK(sc);
	(void) otus_cmd_send_ping(sc, 0x89abcdef);
	OTUS_UNLOCK(sc);

	/* XXX read eeprom, device setup, etc */

	/* XXX setup ifp */

	/* XXX setup net80211 */

	return (0);

detach:
	otus_detach(dev);
	return (ENXIO);
}

static int
otus_detach(device_t dev)
{
	struct otus_softc *sc = device_get_softc(dev);

	OTUS_LOCK(sc);
	/*
	 * Free command list.
	 */
	otus_free_cmd_list(sc, sc->sc_cmd, OTUS_CMD_LIST_COUNT);
	OTUS_UNLOCK(sc);

	otus_firmware_cleanup(&sc->fwinfo);

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
		device_printf(sc->sc_dev, "%s: setup\n", __func__);
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
	break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		device_printf(sc->sc_dev, "%s: comp; %d bytes\n",
		    __func__,
		    actlen);
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
	struct otus_cmd *cmd;
	int actlen;
	int sumlen;

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	device_printf(sc->sc_dev,
	    "%s: called; state=%d\n",
	    __func__,
	    USB_GET_STATE(xfer));

	switch (USB_GET_STATE(xfer)) {

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */

		/*
		 * Complete a frame.
		 */
		device_printf(sc->sc_dev, "%s: comp\n", __func__);
		(void) otus_comp_cmdbuf(sc);

		/* XXX FALLTHROUGH */

	case USB_ST_SETUP:
		/*
		 * Setup xfer frame lengths/count and data
		 */

		/*
		 * If we have anything next to queue, do so.
		 */
		cmd = otus_get_next_cmdbuf(sc);
		if (cmd != NULL) {
			device_printf(sc->sc_dev, "%s: setup - %d bytes\n",
			    __func__,
			    cmd->buflen);
			otus_cmd_dump(sc, cmd);
			usbd_xfer_set_frame_data(xfer, 0, cmd->buf,
			    cmd->buflen);
			usbd_transfer_submit(xfer);
		}
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
		device_printf(sc->sc_dev, "%s: setup\n", __func__);
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;

	case USB_ST_TRANSFERRED:
		/*
		 * Read usb frame data, if any.
		 * "actlen" has the total length for all frames
		 * transferred.
		 */
		device_printf(sc->sc_dev, "%s: comp; %d bytes\n",
		    __func__,
		    actlen);
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
