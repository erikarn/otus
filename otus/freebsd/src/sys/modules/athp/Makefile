# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/athp

KMOD    = if_athp

# Ok, I'll start using more linux-y things soon.
CFLAGS+=	-I${SYSDIR}/compat/linuxkpi/common/include/

# Top-level FreeBSD glue (device, netif, busdma, etc)
SRCS    = if_athp_pci.c if_athp_main.c			\
	  if_athp_debug.c				\
	  if_athp_var.h					\
	  bus_if.h device_if.h				\
	  opt_wlan.h

# This is the chip specific register offsets used for MMIO access.
#
# XXX TODO: it should just folded into if_athp_pci_chip.c as I bet
# it's PCI specific.
SRCS    += if_athp_regs.c if_athp_regio.c

# This is the PCI chip interface - handling reset, enable/disable
# interrupts, etc.
SRCS    += if_athp_pci_config.c
SRCS    += if_athp_pci_chip.c

# This is the busdma memory layer.
SRCS    += if_athp_desc.c

# This is the athp buffer layer.
SRCS    += if_athp_buf.c

# This is the Copy Engine interface - the DMA ring hardware access.
SRCS    += if_athp_pci_ce.c

# This is the PCI pipe code - linking the PCI bus code to the CE.
SRCS    += if_athp_pci_pipe.c

# This is the PCI HIF (host interface) code - providing the HIF
# abstraction for PCI.
SRCS    += if_athp_pci_hif.c

# This is the BMI (bootloader messaging interface) code
# for speaking to the bootloader for setup and firmware loading
SRCS    += if_athp_bmi.c

# This is to implement the 'swap' feature - running some of the firmware
# code in host memory.
SRCS    += if_athp_swap.c

# This is the core routines for firmware load, HIF wakeup/sleep,
# programming the target features, etc.
SRCS    += if_athp_core.c

# This is the host-target control protocol (HTC).
# It forms the basis of host/target communications - control and
# data commands are layered on top of this.
SRCS    += if_athp_htc.c

# This is the command structure format to speak to the firmware (WMI).
SRCS    += if_athp_wmi_tlv.c
SRCS    += if_athp_wmi.c

# This is the MAC layer - in FreeBSD, this is the MAC/VIF abstraction
# layer and I'll break out the command handling and net80211 bits
# into separate source files
SRCS    += if_athp_mac.c

# This is the transport layer for transmit/receive MSDUs.
SRCS    += if_athp_htt.c
SRCS    += if_athp_txrx.c
SRCS    += if_athp_htt_tx.c
SRCS    += if_athp_htt_rx.c

.include <bsd.kmod.mk>
