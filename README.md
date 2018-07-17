A sad attempt on porting OpenBSD rtsx driver to FreeBSD.

Based on this: https://www.freebsd.org/doc/en_US.ISO8859-1/books/arch-handbook/pci.html example.

What works:

 - probe (with fixed dev id x)
 - attach (and pcidev initialization... even detects if card is present or not when kldloading the module)
 - detach (atleast kldunload)

TODO:

 - hao doai FreeBSD BUS
 - devmethods to implement the mmc interface

