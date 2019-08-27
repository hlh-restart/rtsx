/*
 *
 * Base on OpenBSD /sys/dev/pci/rtsx_pci.c & /dev/ic/rtsx.c 
 *
 * Copyright (c) 2006 Uwe Stuehler <uwe@openbsd.org>
 * Copyright (c) 2012 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Port to freeBSD
 *
 * Copyright (c) 2019 Henri Hennebert <hlh@restart.be>
 */ 

/*
 * Simple KLD to play with the PCI functions.
 *
 * Murray Stokely
 */

#include <sys/param.h>		/* defines used in kernel.h */
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kernel.h>		/* types used in module initialization */
#include <sys/conf.h>		/* cdevsw struct */
#include <sys/uio.h>		/* uio struct */
#include <sys/malloc.h>
#include <sys/bus.h>		/* structs, prototypes for pci bus stuff and DEVMETHOD macros! */

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>	/* For pci_get macros! */
#include <dev/pci/pcireg.h>

#include <dev/rtsx/rtsx_pci.h>
#include <dev/rtsx/rtsxreg.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include "mmcbr_if.h"

#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

/*
 *
 * Character device fuctions
 *
 */

/* Character device function prototypes */
static d_open_t		rtsx_pci_open;
static d_close_t	rtsx_pci_close;
static d_read_t		rtsx_pci_read;
static d_write_t	rtsx_pci_write; 

/* Character device entry points */
static struct cdevsw rtsx_pci_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	rtsx_pci_open,
	.d_close =	rtsx_pci_close,
	.d_read =	rtsx_pci_read,
	.d_write =	rtsx_pci_write,
	.d_name =	"rtsx",
};

/*
 * In the cdevsw routines, we find our softc by using the si_drv1 member
 * of struct cdev.  We set this variable to point to our softc in our
 * attach routine when we create the /dev entry.
 */

int
rtsx_pci_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct rtsx_pci_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Opened.\n");
	return 0;
}

int
rtsx_pci_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct rtsx_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Closed.\n");
	return 0;
}

int
rtsx_pci_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct rtsx_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Read %d bytes.\n", (int) uio->uio_resid);
	return 0;
}

int
rtsx_pci_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct rtsx_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Write %d bytes.\n", (int) uio->uio_resid);
	return 0;
}

/*
 *
 * PCI Support Functions
 *
 */

static int
rtsx_pci_probe(device_t dev)
{
/*
 * Compare the device ID (chip) of this device against the IDs that this driver
 * supports. If there is a match, set the description and return success.
 *
 *   none0@pci0:3:0:0:	class=0xff0000 card=0x121a1025 chip=0x528710ec rev=0x01 hdr=0x00
 *   vendor     = 'Realtek Semiconductor Co., Ltd.'
 *   device     = 'RTL8411B PCI Express Card Reader'
 *   bar   [10] = type Memory, range 32, base 0xb1005000, size 4096, enabled
 *   cap 01[40] = powerspec 3  supports D0 D1 D2 D3  current D0
 *   cap 05[50] = MSI supports 1 message, 64 bit 
 *   cap 10[70] = PCI-Express 2 endpoint max data 128(128) RO
 *                link x1(x1) speed 2.5(2.5) ASPM L1(L0s/L1)
 *   cap 11[b0] = MSI-X supports 1 message
 *                Table in map 0x10[0x0], PBA in map 0x10[0x0]
 *   cap 03[d0] = VPD
 *   ecap 0001[100] = AER 2 0 fatal 0 non-fatal 2 corrected
 *   ecap 0002[140] = VC 1 max VC0
 *   ecap 0003[160] = Serial 1 0000000000000000
 *   ecap 0018[170] = LTR 1
 *   ecap 001e[178] = unknown 1
 * PCI-e errors = Correctable Error Detected
 *                Unsupported Request Detected
 *    Corrected = Receiver Error
 *                Advisory Non-Fatal Error
 *
 *   none2@pci0:2:0:0:       class=0xff0000 card=0x221417aa chip=0x522710ec rev=0x01 hdr=0x00
 *   vendor     = 'Realtek Semiconductor Co., Ltd.'
 *   device     = 'RTS5227 PCI Express Card Reader'
 *   bar   [10] = type Memory, range 32, base 0xf0500000, size 4096, enabled
 */
	uint16_t vendor;
	uint16_t device;
	struct rtsx_pci_softc *sc;
	int i, result;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

	device_printf(dev, "Probe - Vendor ID: 0x%x - Device ID: 0x%x\n", vendor, device);

	result = ENXIO;
	for (i = 0; rtsx_devices[i].vendor != 0; i++) {
		if (rtsx_devices[i].vendor == vendor &&
		    rtsx_devices[i].device == device) {
		        device_set_desc(dev, rtsx_devices[i].desc);
			sc = device_get_softc(dev);
			sc->rtsx_flags = rtsx_devices[i].flags;
			result = BUS_PROBE_DEFAULT;
			break;
		}
	}

	return result;
}

static int
rtsx_pci_attach(device_t dev)
{
/* Attach function is only called if the probe is successful. */

	struct rtsx_pci_softc *sc = device_get_softc(dev);
	int		msi_count = 1;
	u_int32_t	sdio_cfg;
	int		error;
	
	device_printf(dev, "Attach - Vendor ID: 0x%x - Device ID: 0x%x\n",
		      pci_get_vendor(dev), pci_get_device(dev));

	sc->rtsx_dev = dev;

	/* Allocate IRQ. */
	sc->rtsx_irq_res_id = 0;
	if (pci_alloc_msi(dev, &msi_count) == 0)
		sc->rtsx_irq_res_id = 1;
	sc->rtsx_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->rtsx_irq_res_id,
						  RF_ACTIVE | (sc->rtsx_irq_res_id != 0 ? 0 : RF_SHAREABLE));
	if (sc->rtsx_irq_res == NULL) {
		device_printf(dev, "couldn't allocate IRQ resources for %d\n", sc->rtsx_irq_res_id);
		pci_release_msi(dev);
		return 1;
	}

	/* Allocate memory resource */
	sc->rtsx_res_id = PCIR_BAR(0);
	sc->rtsx_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rtsx_res_id, RF_ACTIVE);
	if (sc->rtsx_res == NULL) {
		device_printf(dev, "couldn't allocate memory resource for %d\n", sc->rtsx_res_id);
		goto destroy_rtsx_irq_res;
	}
	sc->rtsx_btag = rman_get_bustag(sc->rtsx_res);
	sc->rtsx_bhandle = rman_get_bushandle(sc->rtsx_res);
	
	/* Activate the interrupt */
	error = bus_setup_intr(dev, sc->rtsx_irq_res, INTR_TYPE_MISC,
			       NULL, rtsx_intr, sc, &sc->rtsx_irq_cookie);
	if (error) {
		device_printf(dev, "couldn't set up irq [0x%x]!\n", error);
		goto destroy_rtsx_res;
	}
	pci_enable_busmaster(dev);
	
	/* Initialize device */
	if (rtsx_init(sc, 1)) {
		device_printf(dev, "Error during rtsx_init()\n");
		goto destroy_rtsx_irq;
	}

	if (rtsx_read_cfg(sc, 0, RTSX_SDIOCFG_REG, &sdio_cfg) == 0) {
		if ((sdio_cfg & RTSX_SDIOCFG_SDIO_ONLY) ||
		    (sdio_cfg & RTSX_SDIOCFG_HAVE_SDIO))
			sc->rtsx_flags |= RTSX_F_SDIO_SUPPORT;
	}

	/*
	 *
	 * Allocate three DMA buffers: a command buffer, a data buffer, and a buffer for
	 * ADMA transfer descriptors which describe scatter-gather (SG) I/O operations.
	 *
	 */
	error = rtsx_dma_alloc(sc);
	if (error) {
		goto destroy_rtsx_irq;
	}

	/*
	 * Create a /dev entry for this device.  The kernel will assign us
	 * a major number automatically.  We use the unit number of this
	 * device as the minor number and name the character device
	 * "rtsx<unit>".
	 */
	sc->rtsx_cdev = make_dev(&rtsx_pci_cdevsw, device_get_unit(dev),
	    UID_ROOT, GID_WHEEL, 0600, "rtsx%u", device_get_unit(dev));
	sc->rtsx_cdev->si_drv1 = sc;
	device_printf(dev, "Device attached\n");
	return 0;

 destroy_rtsx_irq:
	bus_teardown_intr(dev, sc->rtsx_irq_res, sc->rtsx_irq_cookie);	
 destroy_rtsx_res:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->rtsx_res_id,
			     sc->rtsx_res);
 destroy_rtsx_irq_res:
	bus_release_resource(dev, SYS_RES_IRQ, sc->rtsx_irq_res_id,
			     sc->rtsx_irq_res);
	pci_release_msi(dev);

	return 1;
}

	
/* Detach device. */

static int
rtsx_pci_detach(device_t dev)
{
	struct rtsx_pci_softc *sc = device_get_softc(dev);

	device_printf(dev, "Detach deviceID: 0x%x\n", pci_get_devid(dev));
	
	/* Teardown the state in our softc created in our attach routine. */
	rtsx_dma_free(sc);
        if (sc->rtsx_res != NULL)
                bus_release_resource(dev, SYS_RES_MEMORY, sc->rtsx_res_id,
				     sc->rtsx_res);	
	if (sc->rtsx_irq_cookie != NULL)
                bus_teardown_intr(dev, sc->rtsx_irq_res, sc->rtsx_irq_cookie);	
        if (sc->rtsx_irq_res != NULL) {
	        bus_release_resource(dev, SYS_RES_IRQ, sc->rtsx_irq_res_id,
				     sc->rtsx_irq_res);
		pci_release_msi(dev);
	}
	destroy_dev(sc->rtsx_cdev);
		
	return (0);
}

/* Called during system shutdown after sync. */

static int
rtsx_pci_shutdown(device_t dev)
{

	device_printf(dev, "Shutdown\n");
	return (0);
}

/*
 * Device suspend routine.
 */
static int
rtsx_pci_suspend(device_t dev)
{

	device_printf(dev, "Suspend\n");
	return (0);
}

/*
 * Device resume routine.
 */
static int
rtsx_pci_resume(device_t dev)
{

	device_printf(dev, "Resume\n");
	return (0);
}

static device_method_t rtsx_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rtsx_pci_probe),
	DEVMETHOD(device_attach,	rtsx_pci_attach),
	DEVMETHOD(device_detach,	rtsx_pci_detach),
	DEVMETHOD(device_shutdown,	rtsx_pci_shutdown),
	DEVMETHOD(device_suspend,	rtsx_pci_suspend),
	DEVMETHOD(device_resume,	rtsx_pci_resume),

	/* Bus interface */
	/*---
	DEVMETHOD(bus_read_ivar,	rtsx_read_ivar),
	DEVMETHOD(bus_write_ivar,	rtsx_write_ivar),
	---*/
	
	/* MMC bridge interface */
	/*---
       	DEVMETHOD(mmcbr_update_ios,	rtsx_update_ios),
	DEVMETHOD(mmcbr_switch_vccq,	rtsx_switch_vccq),
	DEVMETHOD(mmcbr_tune,		rtsx_tune),
	DEVMETHOD(mmcbr_retune,		rtsx_retune),
	DEVMETHOD(mmcbr_request,	rtsx_request),
	DEVMETHOD(mmcbr_get_ro,		rtsx_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	rtsx_acquire_host),
	DEVMETHOD(mmcbr_release_host,	rtsx_release_host),
	---*/

	/*---
	DEVMETHOD(sdhci_read_4,		rtsx_read),
	DEVMETHOD(sdhci_write_4,	rtsx_write),
	---*/
	
	DEVMETHOD_END
};

/*---
static driver_t rtsx_pci_driver = {
        "rtsx",
        rtsx_pci_methods,
        sizeof(struct rtsx_pci_softc),
};
---*/

/*
static devclass_t sdhci_rtsx_pci_devclass;

DRIVER_MODULE(rtsx_pci_rtsx, pci, sdhci_rtsx_pci_driver, sdhci_rtsx_pci_devclass, NULL, NULL);
MODULE_DEPEND(rtsx_pci_rtsx, sdhci, 1, 1, 1);
MMC_DECLARE_BRIDGE(rtsx_pci_rtsx);
*/


static devclass_t rtsx_pci_devclass;

DEFINE_CLASS_0(rtsx_pci, rtsx_pci_driver, rtsx_pci_methods, sizeof(struct rtsx_pci_softc));
DRIVER_MODULE(rtsx_pci, pci, rtsx_pci_driver, rtsx_pci_devclass, NULL, NULL);
MODULE_DEPEND(rtsx_pci, rtsx, 1, 1, 1);
/*
DRIVER_MODULE(mmc, rtsx_pci, mmc_driver, mmc_devclass, NULL, NULL);
MODULE_DEPEND(rtsx_pci, mmc, 1, 1, 1);
*/
