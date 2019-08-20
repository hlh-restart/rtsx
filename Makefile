
# Makefile for mypci driver
CFLAGS="-ferror-limit=0"
KMOD=	rtsx
SRCS=	rtsx_pci.c
SRCS+=	device_if.h bus_if.h pci_if.h
# mmcbr_if.h sdhci_if.h

.include <bsd.kmod.mk>
