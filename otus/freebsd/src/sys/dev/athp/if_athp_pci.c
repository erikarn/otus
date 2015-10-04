/*-
 * Copyright (c) 2015 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Playground for QCA988x chipsets.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/endian.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/firmware.h>
#include <sys/module.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_input.h>
#ifdef	IEEE80211_SUPPORT_SUPERG
#include <net80211/ieee80211_superg.h>
#endif

#include "hal/linux_compat.h"
#include "hal/chip_id.h"
#include "hal/hw.h"

#include "if_athp_debug.h"
#include "if_athp_var.h"
#include "if_athp_pci.h"

static device_probe_t athp_pci_probe;
static device_attach_t athp_pci_attach;
static device_detach_t athp_pci_detach;

static device_method_t athp_methods[] = {
	DEVMETHOD(device_probe,		athp_pci_probe),
	DEVMETHOD(device_attach,	athp_pci_attach),
	DEVMETHOD(device_detach,	athp_pci_detach),

	DEVMETHOD_END
};

static driver_t athp_driver = {
	.name = "athp",
	.methods = athp_methods,
	.size = sizeof(struct athp_pci_softc)
};

static devclass_t athp_devclass;

DRIVER_MODULE(athp, pci, athp_driver, athp_devclass, NULL, 0);
MODULE_DEPEND(athp, wlan, 1, 1, 1);
MODULE_DEPEND(athp, firmware, 1, 1, 1);
MODULE_VERSION(athp, 1);

/*
 * For now: let's just attach on this device:
 none4@pci0:5:0:0:	class=0x028000 card=0x00000000 chip=0x003c168c rev=0x00 hdr=0x00
 vendor     = 'Qualcomm Atheros'
 device     = 'QCA986x/988x 802.11ac Wireless Network Adapter'
 class      = network
 */

static const struct athp_pci_supp_chip athp_pci_supp_chips[] = {
	/*
	 * QCA988X pre 2.0 chips are not supported because they need some
	 * nasty hacks. ath10k doesn't have them and these devices crash
	 * horribly because of that.
	 */
	{ QCA988X_2_0_DEVICE_ID, QCA988X_HW_2_0_CHIP_ID_REV },

	{ QCA6164_2_1_DEVICE_ID, QCA6174_HW_2_1_CHIP_ID_REV },
	{ QCA6164_2_1_DEVICE_ID, QCA6174_HW_2_2_CHIP_ID_REV },
	{ QCA6164_2_1_DEVICE_ID, QCA6174_HW_3_0_CHIP_ID_REV },
	{ QCA6164_2_1_DEVICE_ID, QCA6174_HW_3_1_CHIP_ID_REV },
	{ QCA6164_2_1_DEVICE_ID, QCA6174_HW_3_2_CHIP_ID_REV },

	{ QCA6174_2_1_DEVICE_ID, QCA6174_HW_2_1_CHIP_ID_REV },
	{ QCA6174_2_1_DEVICE_ID, QCA6174_HW_2_2_CHIP_ID_REV },
	{ QCA6174_2_1_DEVICE_ID, QCA6174_HW_3_0_CHIP_ID_REV },
	{ QCA6174_2_1_DEVICE_ID, QCA6174_HW_3_1_CHIP_ID_REV },
	{ QCA6174_2_1_DEVICE_ID, QCA6174_HW_3_2_CHIP_ID_REV },

	{ QCA99X0_2_0_DEVICE_ID, QCA99X0_HW_2_0_CHIP_ID_REV },
};


/*
 * Bus-space access.
 */
static void
athp_pci_write32(struct athp_pci_softc *psc, uint32_t offset, uint32_t value)
{
#if 0
        int ret;
#endif

#if 0
        if (unlikely(offset + sizeof(value) > ar_pci->mem_len)) {
                ath10k_warn(ar, "refusing to write mmio out of bounds at 0x%08x - 0x%08zx (max 0x%08zx)\n",
                            offset, offset + sizeof(value), ar_pci->mem_len);
                return;
        }

        ret = ath10k_pci_wake(ar);
        if (ret) {
                ath10k_warn(ar, "failed to wake target for write32 of 0x%08x at 0x%08x: %d\n",
                            value, offset, ret);
                return;
        }
#endif

	OS_REG_WRITE(psc, offset, value);
#if 0
        ath10k_pci_sleep(ar);
#endif
}

static uint32_t
athp_pci_read32(struct athp_pci_softc *psc, uint32_t offset)
{
        uint32_t val;
#if 0
        int ret;
#endif

#if 0
        if (unlikely(offset + sizeof(val) > ar_pci->mem_len)) {
                ath10k_warn(ar, "refusing to read mmio out of bounds at 0x%08x - 0x%08zx (max 0x%08zx)\n",
                            offset, offset + sizeof(val), ar_pci->mem_len);
                return 0;
        }

        ret = ath10k_pci_wake(ar);
        if (ret) {
                ath10k_warn(ar, "failed to wake target for read32 at 0x%08x: %d\n",
                            offset, ret);
                return 0xffffffff;
        }
#endif
        val = OS_REG_READ(psc, offset);
#if 0
        ath10k_pci_sleep(ar);
#endif
        return val;
}

static uint32_t
athp_pci_soc_read32(struct athp_pci_softc *psc, uint32_t addr)
{
	//return athp_pci_read32(psc, RTC_SOC_BASE_ADDRESS + addr);

	/* Get base address based on current hardware id */
	return athp_pci_read32(psc, psc->sc_hwregs->rtc_soc_base_address + addr);
}

#if 0
void
athp_pci_soc_write32(struct ath10k *ar, uint32_t addr, uint32_t val)
{
        athp_pci_write32(ar, RTC_SOC_BASE_ADDRESS + addr, val);
}

uint32_t
athp_pci_reg_read32(struct ath10k *ar, uint32_t addr)
{
        return athp_pci_read32(ar, PCIE_LOCAL_BASE_ADDRESS + addr);
}

void
athp_pci_reg_write32(struct ath10k *ar, uint32_t addr, uint32_t val)
{
        athp_pci_write32(ar, PCIE_LOCAL_BASE_ADDRESS + addr, val);
}
#endif

static int
athp_pci_chip_is_supported(uint32_t dev_id, uint32_t chip_id)
{
	const struct athp_pci_supp_chip *supp_chip;
	int i;
	u32 rev_id = MS(chip_id, SOC_CHIP_ID_REV);

	for (i = 0; i < nitems(athp_pci_supp_chips); i++) {
		supp_chip = &athp_pci_supp_chips[i];

		if (supp_chip->dev_id == dev_id &&
		    supp_chip->rev_id == rev_id)
		return (1);
	}

	return (0);
}

/*
 * Driver setup stuff.
 */

static int
athp_pci_probe(device_t dev)
{
	int vendor_id, device_id;

	vendor_id = pci_get_vendor(dev);
	device_id = pci_get_device(dev);
	if (vendor_id == 0x168c && device_id == 0x003c) {
		device_set_desc(dev, "QCA988x");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static void
athp_pci_setup(device_t dev)
{

	/* XXX TODO: whatever initial CE, etc initialisation is required */
}

static void
athp_pci_intr(void *arg)
{
	struct athp_pci_softc *psc = arg;

	device_printf(psc->sc_sc.sc_dev, "%s: called\n", __func__);
}

#define	BS_BAR	0x10

/* XXX */
#define	ATHP_MAX_SCATTER	8
#define MSI_NUM_REQUEST_LOG2	3
#define MSI_NUM_REQUEST		(1<<MSI_NUM_REQUEST_LOG2)

static int
athp_set_regs_mapping(struct athp_pci_softc *psc)
{
	/*
	 * Figure out the hardware revision id and various
	 * hardware maps to be using.
	 */
	switch (psc->sc_deviceid) {
	case QCA988X_2_0_DEVICE_ID:
		psc->sc_hwrev = ATH10K_HW_QCA988X;
		psc->sc_hwregs = &qca988x_regs;
		break;
	case QCA6164_2_1_DEVICE_ID:
	case QCA6174_2_1_DEVICE_ID:
		psc->sc_hwrev = ATH10K_HW_QCA6174;
		psc->sc_hwregs = &qca6174_regs;
		break;
	case QCA99X0_2_0_DEVICE_ID:
		psc->sc_hwrev = ATH10K_HW_QCA99X0;
		psc->sc_hwregs = &qca99x0_regs;
		break;
	default:
		return (0);
	}

	return (1);
}

#if 0
static void
athp_pci_init_irq_legacy(struct athp_pci_softc *psc)
{
	athp_pci_write32(psc, SOC_CORE_BASE_ADDRESS + PCIE_INTR_ENABLE_ADDRESS,
	    PCIE_INTR_FIRMWARE_MASK | PCIE_INTR_CE_MASK_ALL);
	return;
}

static void
athp_pci_deinit_irq_legacy(struct athp_pci_softc *psc)
{
	athp_pci_write32(psc, SOC_CORE_BASE_ADDRESS + PCIE_INTR_ENABLE_ADDRESS, 0);
}
#endif

#if 0
static int
athp_pci_chip_reset(struct athp_pci_softc *psc)
{
        if (QCA_REV_988X(psc))
                return ath10k_pci_qca988x_chip_reset(psc);
        else if (QCA_REV_6174(psc))
                return ath10k_pci_qca6174_chip_reset(psc);
        else if (QCA_REV_99X0(psc))
                return ath10k_pci_qca99x0_chip_reset(psc);
        else
                return (-1);
}
#endif

static int
athp_pci_attach(device_t dev)
{
	struct athp_pci_softc *psc = device_get_softc(dev);
	struct athp_softc *sc = &psc->sc_sc;
	int rid;
//	int ret;
//	uint32_t chip_id;

	/* Get PCI vendor/device id */
	psc->sc_vendorid = pci_get_vendor(dev);
	psc->sc_deviceid = pci_get_device(dev);

	sc->sc_dev = dev;
	sc->sc_invalid = 1;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);

	/* Figure out hardware revision and register map based on above */
	if (! athp_set_regs_mapping(psc)) {
		device_printf(dev, "cannot find initial hardware map\n");
		goto bad;
	}

	/*
	 * Enable bus mastering.
	 */
	pci_enable_busmaster(dev);

	/*
	 * Setup memory-mapping of PCI registers.
	 */
	rid = BS_BAR;
	psc->sc_sr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (psc->sc_sr == NULL) {
		device_printf(dev, "cannot map register space\n");
		goto bad;
	}

	psc->sc_st = rman_get_bustag(psc->sc_sr);
	psc->sc_sh = rman_get_bushandle(psc->sc_sr);

	/*
	 * Mark device invalid so any interrupts (shared or otherwise)
	 * that arrive before the HAL is setup are discarded.
	 */
	sc->sc_invalid = 1;

	/*
	 * Arrange interrupt line.
	 *
	 * XXX TODO: implement MSIX; we should be getting one MSI for
	 * (almost) each CE ring.
	 */
	rid = 0;
	psc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE|RF_ACTIVE);
	if (psc->sc_irq == NULL) {
		device_printf(dev, "could not map interrupt\n");
		goto bad1;
	}
	if (bus_setup_intr(dev, psc->sc_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, athp_pci_intr, sc, &psc->sc_ih)) {
		device_printf(dev, "could not establish interrupt\n");
		goto bad2;
	}

#if 0
	/*
	 * Reset chip; see what the story is.
	 */
	ret = athp_pci_chip_reset(psc);
	if (ret != 0) {
		device_printf(dev, "couldn't reset chip\n");
		goto bad2;
	}

	/* Check the SoC version! */
	//chip_id = athp_pci_soc_read32(psc, SOC_CHIP_ID_ADDRESS);
	chip_id = athp_pci_soc_read32(psc,
	    psc->sc_hwregs->soc_chip_id_address);

	device_printf(dev, "%s: chip_id=0x%08x\n", __func__, chip_id);
#endif
#if 0
        if (chip_id == 0xffffffff) {
                ath10k_err(ar, "failed to get chip id\n");
                goto err_free_irq;
        }

        if (!ath10k_pci_chip_is_supported(pdev->device, chip_id)) {
                ath10k_err(ar, "device %04x with chip_id %08x isn't supported\n",
                           pdev->device, chip_id);
                goto err_free_irq;
        }
#endif

	/*
	 * Setup DMA descriptor area.
	 */
	if (bus_dma_tag_create(bus_get_dma_tag(dev),    /* parent */
	    1, 0,		    /* alignment, bounds */
	    BUS_SPACE_MAXADDR_32BIT, /* lowaddr */
	    BUS_SPACE_MAXADDR,       /* highaddr */
	    NULL, NULL,	      /* filter, filterarg */
	    0x3ffff,		 /* maxsize XXX */
	    ATHP_MAX_SCATTER,	 /* nsegments */
	    0x3ffff,		 /* maxsegsize XXX */
	    BUS_DMA_ALLOCNOW,	/* flags */
	    NULL,		    /* lockfunc */
	    NULL,		    /* lockarg */
	    &psc->sc_dmat)) {
		device_printf(dev, "cannot allocate DMA tag\n");
		goto bad3;
	}

	/* Call main attach method with given info */
	return (0);

#ifdef	ATHP_EEPROM_FIRMWARE
bad4:
#endif
	bus_dma_tag_destroy(psc->sc_dmat);
bad3:
	bus_teardown_intr(dev, psc->sc_irq, psc->sc_ih);
bad2:
	bus_release_resource(dev, SYS_RES_IRQ, 0, psc->sc_irq);
bad1:
	bus_release_resource(dev, SYS_RES_MEMORY, BS_BAR, psc->sc_sr);

bad:
	mtx_destroy(&sc->sc_mtx);
	/* XXX disable bus mastering? */
	return (ENXIO);
}

static int
athp_pci_detach(device_t dev)
{
	struct athp_pci_softc *psc = device_get_softc(dev);
	struct athp_softc *sc = &psc->sc_sc;

	/* Signal things we're going down.. */
	ATHP_LOCK(sc);
	sc->sc_invalid = 1;
	ATHP_UNLOCK(sc);

	/* XXX TODO: synchronise with running things first */

	/*
	 * Do a config read to clear pre-existing pci error status.
	 */
	(void) pci_read_config(dev, PCIR_COMMAND, 4);

	/* XXX TODO: detach main driver */

	/* Free bus resources */
	bus_generic_detach(dev);
	bus_teardown_intr(dev, psc->sc_irq, psc->sc_ih);
	bus_release_resource(dev, SYS_RES_IRQ, 0, psc->sc_irq);

	bus_dma_tag_destroy(psc->sc_dmat);
	bus_release_resource(dev, SYS_RES_MEMORY, BS_BAR, psc->sc_sr);

	return (0);
}
