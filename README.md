
An attempt on porting OpenBSD rtsx driver to FreeBSD.

Based on https://www.freebsd.org/doc/en_US.ISO8859-1/books/arch-handbook/pci.html example,

and https://lists.freebsd.org/pipermail/freebsd-hackers/2018-April/052520.html

--------------------------------------------------------------------------

Howto run:

kldload mmc

kldload mmcsd

kldload rtsx

sysctl debug.bootverbose=1 for debugging

What works:

 - probe of Vendor ID: 0x10ec - Device ID: 0x5287
 - attach (and dev initialization...)
 - detect card inserted / ejected
 - get card information
 - detach (at least kldunload)
 - Patch to add detection of SD card insertion/removal (from Gary Jennejohn <gljennjohn@gmail.com>)

TODO:

 - io operations (for now return all 0 or timeout)
