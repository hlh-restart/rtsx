
A sad attempt on porting OpenBSD rtsx driver to FreeBSD.

Based on this: https://www.freebsd.org/doc/en_US.ISO8859-1/books/arch-handbook/pci.html example.

--- from https://lists.freebsd.org/pipermail/freebsd-hackers/2018-April/052520.html

sdhci is a good, full function driver. However, it handles a lot of
odd-ball exceptions and edge cases common in a popular interface that's
chasing an evolving standard, so it may be a bit overwhelming.

Here's a quick outline. You'll need a rtsz_pci.c that handles claiming the
device (probe) and initializing it (attach). Most of the initialization
will be the same as OpenBSD, though the glue into the system is somewhat
different between OpenBSD and FreeBSD (which is why I'm suggesting
rtsz_pci.c to help keep that walled off). busdma is similar, but the
details are different between the systems. They've evolved from a common
ancestor, though, so that's good. Bus_space is the same, but the resource
allocations will be different (see bus_alloc_resource). Interrupt handling
will be different.

The interface you want to look for is the mmcbr_if.m inteface. In sdhci,
these routines implement the mmc interface:
sdhci_pci.c: DEVMETHOD(mmcbr_update_ios, sdhci_generic_update_ios),
sdhci_pci.c: DEVMETHOD(mmcbr_switch_vccq, sdhci_generic_switch_vccq),
sdhci_pci.c: DEVMETHOD(mmcbr_tune, sdhci_generic_tune),
sdhci_pci.c: DEVMETHOD(mmcbr_retune, sdhci_generic_retune),
sdhci_pci.c: DEVMETHOD(mmcbr_request, sdhci_generic_request),
sdhci_pci.c: DEVMETHOD(mmcbr_get_ro, sdhci_generic_get_ro),
sdhci_pci.c: DEVMETHOD(mmcbr_acquire_host,   sdhci_generic_acquire_host),
sdhci_pci.c: DEVMETHOD(mmcbr_release_host,   sdhci_generic_release_host),

rtsz will almost certainly need it's own versions of these routines (which
is why I suggest having your own driver will be simpler: otherwise each of
these routines would be if (rtsz) do_rtsz_stuff(); else do_sdhci_stuff();
which won't end well and would be uncomittable to FreeBSD. You can see how
other chips implement these methods by grepping for them in the tree. You
may not need a tune/retune if rtsz doesn't support the latest, fastest
cards, for example. Switch vccq may not be needed either. update_ios will
be needed, and request is needed. Acquire and release host may be able to
be done as a dummy routine if there's only one slot.

I know this is a super-quick gloss of what needs to be done.

Warner

--------------------------------------------------------------------------

What works:

 - probe (with fixed dev id x)
 - attach (and pcidev initialization... even detects if card is present or not when kldloading the module)
 - detach (atleast kldunload)

TODO:

 - DMA allocation
 - DEVMETHOD to implement the mmc interface
