
An attempt on porting OpenBSD rtsx driver to FreeBSD.

Based on https://www.freebsd.org/doc/en_US.ISO8859-1/books/arch-handbook/pci.html example,

and https://lists.freebsd.org/pipermail/freebsd-hackers/2018-April/052520.html

--------------------------------------------------------------------------

HOWTO COMPILE AND INSTALL:

Place source in /usr/src/sys/dev/rtsx

make

make install

HOWTO RUN:

kldload mmc

kldload mmcsd

kldload rtsx

sysctl debug.bootverbose=1 for debugging

WHAT WORKS:

 - probe of Vendor ID: 0x10ec - Device ID: 0x5287
 - attach (and dev initialization...)
 - detect card inserted / ejected
 - get card information
 - detach (at least kldunload)
 - Patch to add detection of SD card insertion/removal (from Gary Jennejohn <gj@freebsd.org>)
 - Patch to read SCR and STATUS (CMD51 & CMD13) (from Jesper Schmitz Mouridsen <jsm@FreeBSD.org>)
 - Patch to allow successful read operations (from Jesper Schmitz Mouridsen <jsm@FreeBSD.org>)
 - write operation completed with Jesper Schmitz Mouridsen <jsm@FreeBSD.org>.
 - patch for RTS525A from Lutz Bichler <Lutz.Bichler@gmail.com>

TODO:

 - Implement sleep and resume
 - Test... test.... test... and more tests

TESTED ON:

 - RTL8411B under FreeBSD-12.1-STABLE (ACER Aspire E15 - E5-576-77W6)
 - RTS5227  under FreeBSD-11.4-STABLE, FreeBSD 12.1-RELEASE (HP Probook 430 g2, Thinkpad T450s, Lenovo T450)
 - RTS522A  under FreeBSD-13.0-CURRENT
 - RTS525A  under FreeBSD-12.1-RELEASE (Dell Latitude E5570)

NOTES:
 
 - RTS522A on a lenovo P50s show a card detection problem.

 - See https://bugs.freebsd.org/bugzilla/show_bug.cgi?id=204521
