# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/otus

KMOD    = if_otus
SRCS    = if_otus.c if_otus_firmware.c	\
	  if_otus_firmware.h		\
	  if_otusvar.h if_otusreg.h	\
	  if_otus_cmd.c if_otus_cmd.h	\
	  if_otus_debug.h		\
	  fwdesc.h			\
	  version.h			\
	  bus_if.h device_if.h		\
	  opt_bus.h opt_usb.h usb_if.h usbdevs.h

.include <bsd.kmod.mk>
