# Makefile for rtsx driver
# CFLAGS="-ferror-limit=0"
.if RTSX_INVERTION
CFLAGS+= -DRTSX_INVERTION
.endif
KMOD=	rtsx
SRCS=	rtsx.c
SRCS+=	device_if.h bus_if.h pci_if.h mmcbr_if.h

.include <bsd.kmod.mk>
