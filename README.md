
# A port of OpenBSD rtsx driver to FreeBSD

Based on https://www.freebsd.org/doc/en_US.ISO8859-1/books/arch-handbook/pci.html example,

and https://lists.freebsd.org/pipermail/freebsd-hackers/2018-April/052520.html

--------------------------------------------------------------------------

#### HOWTO COMPILE AND INSTALL:

Place source in `/usr/src/sys/dev/rtsx`

```
make clean
make
make install
```

#### HOWTO RUN:
```
kldload mmc
kldload mmcsd
kldload rtsx
```
For debugging:
 `sysctl debug.bootverbose=1`
 
#### HISTORY:

 - probe of Vendor ID: 0x10ec - Device ID: 0x5287
 - attach (and dev initialization...)
 - detect card inserted / ejected
 - get card information
 - detach (at least kldunload)
 - patch to add detection of SD card insertion/removal (from Gary Jennejohn <gj@freebsd.org>)
 - patch to read SCR and STATUS (CMD51 & CMD13) (from Jesper Schmitz Mouridsen <jsm@FreeBSD.org>)
 - patch to allow successful read operations (from Jesper Schmitz Mouridsen <jsm@FreeBSD.org>)
 - write operation completed with Jesper Schmitz Mouridsen <jsm@FreeBSD.org>.
 - patch for RTS525A from Lutz Bichler <Lutz.Bichler@gmail.com>
 - add read-only detection
 - add suspend and resume (at least on Acer Aspire E 15 E5-576-77W6)

#### TODO:

 - **More testing please!**

#### TESTED ON:

 - RTS5209 under head (Lenovo ThinkPad L520, Packard Bell EN LV44HC i3-2328M)
 - RTS5227 under stable/11, releng/12.1 and head
   (HP ProBook 430 g2, Lenovo ThinkPad S440/T450/T450s/X270, Fujitsu H730)
 - RTS5229 under releng/12.1 and head (Lenovo IdeaPad 120S-14IAP, ASUS GL553VE)
 - RTS522A under releng/12.1 and head
   (Intel NUC8i5BE, ThinkPad P50s, ThinkPad T470p, Thinkpad x260)
 - RTS525A under releng/12.1 (Dell Latitude E5570, Dell XPS 13 - model 9360)
 - RTL8411B under stable/12 and head
   (Acer Aspire E 15 E5-576-77W6, ACER ASPIRE 5 A515-51G-C97B)

#### KNOWN BUGS:
 - The timeouts experienced during card insert and during I/O are solved in version 1.0g.
 - RTS522A on Lenovo P50s and Lenovo T470p, card detection and read-only switch are reversed.
   To adapt the driver: make -D RTSX_INVERSION.
 - Mounting a filesystem with write access on a card write protected may involve a kernel crash.

#### NOTE:
 
 - See https://bugs.freebsd.org/bugzilla/show_bug.cgi?id=204521
