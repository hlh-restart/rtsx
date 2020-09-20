# Makefile for rtsx driver
# CFLAGS="-ferror-limit=0"
.if RTSX_INVERSION
CFLAGS+= -DRTSX_INVERSION
.endif
.if RTSX_CAM
CFLAGS+= -DMMCCAM
.endif
KMOD=	rtsx
SRCS=	rtsx.c
SRCS+=	device_if.h bus_if.h pci_if.h mmcbr_if.h

.include <bsd.kmod.mk>
