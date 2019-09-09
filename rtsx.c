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
 * Started by Raul Becker < raul.becker@iki.fi>
 *
 * Continued by Henri Hennebert <hlh@restart.be>
 *
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
#include <sys/endian.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <sys/sysctl.h>

#include <dev/pci/pcivar.h>	/* For pci_get macros! */
#include <dev/pci/pcireg.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include "mmcbr_if.h"

#include <dev/rtsx/rtsxreg.h>

#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

/* flag values */
#define	RTSX_F_DEFAULT		0x00
#define	RTSX_F_CARD_PRESENT	0x01
#define	RTSX_F_SDIO_SUPPORT	0x02
#define	RTSX_F_5209		0x04
#define	RTSX_F_5229		0x08
#define	RTSX_F_5229_TYPE_C	0x10
#define	RTSX_F_525A		0x20

/* The softc holds our per-instance data. */
struct rtsx_softc {
	struct mtx	rtsx_mtx;		/* device mutex */
	device_t	rtsx_dev;		/* device */
	int		rtsx_flags;		/* device flags */
	device_t	rtsx_mmc_dev;		/* device of mmc bus */
	struct cdev	*rtsx_cdev;
	int		rtsx_irq_res_id;	/* bus IRQ resource id */
	struct resource *rtsx_irq_res;		/* bus IRQ resource */
	void		*rtsx_irq_cookie;	/* bus IRQ resource cookie */
	int		rtsx_res_id;		/* bus memory resource id */
	struct resource *rtsx_res;		/* bus memory resource */
	int		rtsx_res_type;		/* bus memory resource type */
	bus_space_tag_t	rtsx_btag;		/* host register set tag */
	bus_space_handle_t rtsx_bhandle;	/* host register set handle */
	struct callout	rtsx_timeoutc;		/* callout structure */
	int		rtsx_timeout;		/* timeout value */     

	bus_dma_tag_t	rtsx_cmd_dma_tag;	/* DMA tag for command transfer */
	bus_dmamap_t	rtsx_cmd_dmamap;	/* DMA map for command transfer */
	void		*rtsx_cmd_dmamem;	/* DMA mem for command transfer */
	bus_addr_t	rtsx_cmd_buffer;	/* device visible address of the DMA segment */
	bus_dma_tag_t	rtsx_data_dma_tag;	/* DMA tag for data transfer */
	bus_dmamap_t	rtsx_data_dmamap;	/* DMA map for data transfer */
	void		*rtsx_data_dmamem;	/* DMA mem for data transfer */
	bus_dmamap_t	rtsx_adma_dmamap;	/* DMA map for ADMA SG descriptors */
	void		*rtsx_adma_dmamem;	/* DMA mem for ADMA SG descriptors */
	bus_dma_segment_t rtsx_adma_segs[1];	/* segments for ADMA SG buffer */

	u_char		rtsx_bus_busy;		/* Bus busy status */ 
	struct mmc_host rtsx_host;		/* Host parameters */
	uint32_t 	rtsx_intr_status;	/* soft interrupt status */
	struct mmc_request *rtsx_req;		/* MMC request */
	uint8_t		regs[RTSX_NREG];	/* host controller state */
	uint32_t	regs4[6];		/* host controller state */
};

static const struct rtsx_device {
	uint16_t	vendor;
	uint16_t	device;
	int		flags;	
	const char	*desc;
} rtsx_devices[] = {
	{ 0x10ec,	0x5287,	RTSX_F_DEFAULT, "Realtek RTL8411B PCI MMC/SD Card Reader"},
	{ 0, 		0,	0,		NULL}
};

static int	rtsx_dma_alloc(struct rtsx_softc *sc);
static void	rtsx_dma_free(struct rtsx_softc *sc);
static void	rtsx_intr(void *arg);
static int	rtsx_init(struct rtsx_softc *sc);
static void	rtsx_start(struct rtsx_softc *sc);
static void	rtsx_stop(struct rtsx_softc *sc);
static int	rtsx_read(struct rtsx_softc *, uint16_t, uint8_t *);
static int	rtsx_read_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr, uint32_t *val);
static int	rtsx_write(struct rtsx_softc *, uint16_t, uint8_t, uint8_t);
static int	rtsx_write_phy(struct rtsx_softc *sc, uint8_t addr, uint16_t val);
static int	rtsx_stop_sd_clock(struct rtsx_softc *sc);
static int	rtsx_switch_sd_clock(struct rtsx_softc *sc, uint8_t n, int div, int mcu);
static int	rtsx_bus_power_off(struct rtsx_softc *sc);
static int	rtsx_bus_power_up(struct rtsx_softc *sc);
static int	rtsx_bus_power_on(struct rtsx_softc *);
static int	rtsx_read_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr, uint32_t *val);
static int	rtsx_is_card_present(struct rtsx_softc *);
static void	rtsx_card_insert(struct rtsx_softc *);
static void	rtsx_card_remove(struct rtsx_softc *);
static int	rtsx_led_enable(struct rtsx_softc *);
static int	rtsx_led_disable(struct rtsx_softc *);

static uint8_t	rtsx_response_type(uint16_t mmc_rsp);
static void	rtsx_hostcmd(uint32_t *cmdbuf, int *n, uint8_t cmd, uint16_t reg,
			     uint8_t mask, uint8_t data);
static void	rtsx_hostcmd_send(struct rtsx_softc *, int);
static void	rtsx_req_timeout(void *arg);
static void	rtsx_req_done(struct rtsx_softc *sc);
static void	rtsx_soft_reset(struct rtsx_softc *sc);

static int	rtsx_read_ivar(device_t bus, device_t child, int which, uintptr_t *result);
static int	rtsx_write_ivar(device_t bus, device_t child, int which, uintptr_t value);

static int	rtsx_probe(device_t dev);
static int	rtsx_attach(device_t dev);
static int	rtsx_detach(device_t dev);
static int	rtsx_shutdown(device_t dev);
static int	rtsx_suspend(device_t dev);
static int	rtsx_resume(device_t dev);

static int	rtsx_mmcbr_update_ios(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_request(device_t bus, device_t child __unused, struct mmc_request *req);
static int	rtsx_mmcbr_acquire_host(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_release_host(device_t bus, device_t child __unused);

#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

/*
 *
 * Character device fuctions
 *
 */

/* Character device function prototypes */
static d_open_t		rtsx_cdev_open;
static d_close_t	rtsx_cdev_close;
static d_read_t		rtsx_cdev_read;
static d_write_t	rtsx_cdev_write; 

/* Character device entry points */
static struct cdevsw rtsx_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	rtsx_cdev_open,
	.d_close =	rtsx_cdev_close,
	.d_read =	rtsx_cdev_read,
	.d_write =	rtsx_cdev_write,
	.d_name =	"rtsx",
};

/*
 * In the cdevsw routines, we find our softc by using the si_drv1 member
 * of struct cdev.  We set this variable to point to our softc in our
 * attach routine when we create the /dev entry.
 */

static int
rtsx_cdev_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct rtsx_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Opened.\n");
	return (0);
}

static int
rtsx_cdev_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct rtsx_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Closed.\n");
	return (0);
}

static int
rtsx_cdev_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct rtsx_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Read %d bytes.\n", (int) uio->uio_resid);
	return (0);
}

static int
rtsx_cdev_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct rtsx_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->rtsx_dev, "Write %d bytes.\n", (int) uio->uio_resid);
	return (0);
}

#define RTSX_LOCK_INIT(_sc)	mtx_init(&(_sc)->rtsx_mtx,	\
					 device_get_nameunit(sc->rtsx_dev), "rtsx", MTX_DEF)
#define RTSX_LOCK(_sc)		mtx_lock(&(_sc)->rtsx_mtx)
#define RTSX_UNLOCK(_sc)	mtx_unlock(&(_sc)->rtsx_mtx)
#define RTSX_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->rtsx_mtx)

#define RTSX_DMA_ALIGN		4
#define RTSX_DMA_BLOCK_SIZE	4096

#define	RTSX_DMA_MAX_SEGSIZE	0x80000
#define	RTSX_HOSTCMD_MAX	256
#define	RTSX_HOSTCMD_BUFSIZE	(sizeof(uint32_t) * RTSX_HOSTCMD_MAX)
#define	RTSX_DMA_DATA_BUFSIZE	MAXPHYS
#define	RTSX_ADMA_DESC_SIZE	(sizeof(uint64_t) * SDMMC_MAXNSEGS)

#define ISSET(t, f) ((t) & (f))

#define READ4(sc, reg)						\
	(bus_space_read_4((sc)->rtsx_btag, (sc)->rtsx_bhandle, (reg)))
#define WRITE4(sc, reg, val)					\
	(bus_space_write_4((sc)->rtsx_btag, (sc)->rtsx_bhandle, (reg), (val)))

#define	RTSX_READ(sc, reg, val) 				\
	do { 							\
		int err = rtsx_read((sc), (reg), (val)); 	\
		if (err) 					\
			return (err);				\
	} while (0)

#define	RTSX_WRITE(sc, reg, val) 				\
	do { 							\
		int err = rtsx_write((sc), (reg), 0xff, (val));	\
		if (err) 					\
			return (err);				\
	} while (0)
#define	RTSX_CLR(sc, reg, bits)					\
	do { 							\
		int err = rtsx_write((sc), (reg), (bits), 0); 	\
		if (err) 					\
			return (err);				\
	} while (0)

#define	RTSX_SET(sc, reg, bits)					\
	do { 							\
		int err = rtsx_write((sc), (reg), (bits), 0xff);\
		if (err) 					\
			return (err);				\
	} while (0)


/* 
 * We use three DMA buffers: a command buffer, a data buffer, and a buffer for
 * ADMA transfer descriptors which describe scatter-gather (SG) I/O operations.
 *
 * The command buffer contains a command queue for the host controller,
 * which describes SD/MMC commands to run, and other parameters. The chip
 * runs the command queue when a special bit in the RTSX_HCBAR register is
 * set and signals completion with the TRANS_OK interrupt.
 * Each command is encoded as a 4 byte sequence containing command number
 * (read, write, or check a host controller register), a register address,
 * and a data bit-mask and value.
 * SD/MMC commands which do not transfer any data from/to the card only use
 * the command buffer.
 *
 * The smmmc stack provides DMA-safe buffers with data transfer commands.
 * In this case we write a list of descriptors to the ADMA descriptor buffer,
 * instructing the chip to transfer data directly from/to sdmmc DMA buffers.
 *
 * However, some sdmmc commands used during card initialization also carry
 * data, and these don't come with DMA-safe buffers. In this case, we transfer
 * data from/to the SD card via a DMA data bounce buffer.
 *
 * In both cases, data transfer is controlled via the RTSX_HDBAR register
 * and completion is signalled by the TRANS_OK interrupt.
 *
 * The chip is unable to perform DMA above 4GB.
 */

static void rtsx_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error);

static int
rtsx_dma_alloc(struct rtsx_softc *sc) {
	int	error = 0;

	error = bus_dma_tag_create(bus_get_dma_tag(sc->rtsx_dev), /* inherit from parent */
	    RTSX_DMA_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    RTSX_DMA_BLOCK_SIZE, 1,	/* maxsize, nsegments */
	    RTSX_DMA_BLOCK_SIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rtsx_cmd_dma_tag);
	if (error) {
                device_printf(sc->rtsx_dev,
			      "Can't create cmd parent DMA tag\n");
		return (error);
	}
	error = bus_dmamem_alloc(sc->rtsx_cmd_dma_tag,		/* DMA tag */
	    &sc->rtsx_cmd_dmamem,				/* will hold the KVA pointer */
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,	/* flags */
	    &sc->rtsx_cmd_dmamap); 				/* DMA map */
	if (error) {
                device_printf(sc->rtsx_dev,
			      "Can't create DMA map for command transfer\n");
		goto destroy_cmd_dma_tag;
	}
	error = bus_dmamap_load(sc->rtsx_cmd_dma_tag,	/* DMA tag */
	    sc->rtsx_cmd_dmamap,	/* DMA map */
	    sc->rtsx_cmd_dmamem,	/* KVA pointer to be mapped */
	    RTSX_HOSTCMD_BUFSIZE,	/* size of buffer */
	    rtsx_dmamap_cb,		/* callback */
	    &sc->rtsx_cmd_buffer,	/* first arg of callback */
	    0);			/* flags */
	if (error != 0 || sc->rtsx_cmd_buffer == 0) {
                device_printf(sc->rtsx_dev,
			      "Can't load DMA memory for command transfer\n");
                error = (error) ? error : EFAULT;
		goto destroy_cmd_dmamem_alloc;
        }

	error = bus_dma_tag_create(bus_get_dma_tag(sc->rtsx_dev),	/* inherit from parent */
	    RTSX_DMA_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    RTSX_DMA_BLOCK_SIZE, 1,	/* maxsize, nsegments */
	    RTSX_DMA_BLOCK_SIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rtsx_data_dma_tag);
	if (error) {
                device_printf(sc->rtsx_dev,
			      "Can't create data parent DMA tag\n");
		goto destroy_cmd_dmamap_load;
	}
	error = bus_dmamap_create(sc->rtsx_data_dma_tag,	/* DMA tag */
				  0,				/* flags */
				  &sc->rtsx_data_dmamap);	/* DMA map */
	if (error) {
		device_printf(sc->rtsx_dev,
			      "Can't create DMA map for data transfer\n");
		goto destroy_data_dma_tag;
	}

	return (error);
	
 destroy_data_dma_tag:
	bus_dma_tag_destroy(sc->rtsx_data_dma_tag);
 destroy_cmd_dmamap_load:
	bus_dmamap_unload(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap);
 destroy_cmd_dmamem_alloc:
	bus_dmamem_free(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamem, sc->rtsx_cmd_dmamap);
 destroy_cmd_dma_tag:
	bus_dma_tag_destroy(sc->rtsx_cmd_dma_tag);

	return (error);
}

static void
rtsx_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
        if (error != 0) {
                printf("rtsx_dmamap_cb: error %d\n", error);
                return;
        }
        *(bus_addr_t *)arg = segs[0].ds_addr;
}

static void
rtsx_dma_free(struct rtsx_softc *sc) {

	if (sc->rtsx_cmd_dma_tag != NULL) {
		if (sc->rtsx_cmd_dmamap != NULL)
                        bus_dmamap_unload(sc->rtsx_cmd_dma_tag,
					  sc->rtsx_cmd_dmamap);
                if (sc->rtsx_cmd_dmamem != NULL)
                        bus_dmamem_free(sc->rtsx_cmd_dma_tag,
					sc->rtsx_cmd_dmamem,
					sc->rtsx_cmd_dmamap);
		sc->rtsx_cmd_dmamap = NULL;
		sc->rtsx_cmd_dmamem = NULL;
                sc->rtsx_cmd_buffer = 0;
                bus_dma_tag_destroy(sc->rtsx_cmd_dma_tag);
                sc->rtsx_cmd_dma_tag = NULL;
	}
	if (sc->rtsx_data_dma_tag != NULL) {
		if (sc->rtsx_data_dmamap != NULL)
                        bus_dmamap_destroy(sc->rtsx_data_dma_tag,
					   sc->rtsx_data_dmamap);
		sc->rtsx_data_dmamap = NULL;
                bus_dma_tag_destroy(sc->rtsx_data_dma_tag);
                sc->rtsx_data_dma_tag = NULL;
	}
}
	
static void
rtsx_intr(void *arg)
{
	struct rtsx_softc *sc = arg;
	uint32_t enabled, status;
	struct mmc_command *cmd;
	uint32_t *cmd_buffer;
	
	RTSX_LOCK(sc);
	enabled = READ4(sc, RTSX_BIER);	/* read Bus Interrupt Enable Register */
	status = READ4(sc, RTSX_BIPR);	/* read Bus Interrupt pending Register */

	device_printf(sc->rtsx_dev, "Interrupt handler - enabled: %#x, status: %#x\n", enabled, status);

	/* Ack interrupts. */
	WRITE4(sc, RTSX_BIPR, status);

	if (((enabled & status) == 0) || status == 0xffffffff) {
		RTSX_UNLOCK(sc);
		return;
	}

	/*---
	if (status & RTSX_SD_INT) {
		if (status & RTSX_SD_EXIST) {
			if (!ISSET(sc->rtsx_flags, RTSX_F_CARD_PRESENT))
				rtsx_card_insert(sc);
		} else {
			rtsx_card_remove(sc);
		}
		return;
	}
	---*/

	/* Sync command DMA buffer. */

	sc->rtsx_intr_status |= status;
	if (sc->rtsx_req == NULL) {
		device_printf(sc->rtsx_dev, "Spurious interrupt - no active request\n");
		RTSX_UNLOCK(sc);
		return;
	}
	cmd = sc->rtsx_req->cmd;
	if (status & (RTSX_TRANS_OK_INT | RTSX_TRANS_FAIL_INT)) {
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTREAD);
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTWRITE);

		/* Copy card response into mmc response buffer. */
		if (ISSET(cmd->flags, MMC_RSP_PRESENT)) {
			cmd_buffer = (uint32_t *)(sc->rtsx_cmd_dmamem);
			/* Copy bytes like sdhc(4), which on little-endian uses
			 * different byte order for short and long responses... */
			if (ISSET(cmd->flags, MMC_RSP_136)) {
				memcpy(cmd->resp, cmd_buffer + 1, sizeof(cmd->resp));
			} else {
				/* First byte is CHECK_REG_CMD return value, second
				 * one is the command op code -- we skip those. */
				cmd->resp[0] =
					((be32toh(cmd_buffer[0]) & 0x0000ffff) << 16) |
					((be32toh(cmd_buffer[1]) & 0xffff0000) >> 16);
			}
		}
		rtsx_req_done(sc);
	}
	RTSX_UNLOCK(sc);
}

static int
rtsx_init(struct rtsx_softc *sc)
{
	uint32_t status;
	uint8_t version;
	int error;

	sc->rtsx_host.host_ocr = RTSX_SUPPORTED_VOLTAGE;
			
	/* Read IC version from dummy register. */
	if (sc->rtsx_flags & RTSX_F_5229) {
		RTSX_READ(sc, RTSX_DUMMY_REG, &version);
		switch (version & 0x0F) {
		case RTSX_IC_VERSION_A:
		case RTSX_IC_VERSION_B:
		case RTSX_IC_VERSION_D:
			break;
		case RTSX_IC_VERSION_C:
			sc->rtsx_flags |= RTSX_F_5229_TYPE_C;
			break;
		default:
			device_printf(sc->rtsx_dev, "RTSX_F_5229 unknown ic version 0x%x\n", version);
			return (1);
		}
	}

	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(sc, RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);

	/* Clear any pending interrupts. */
	status = READ4(sc, RTSX_BIPR);
	WRITE4(sc, RTSX_BIPR, status);

	/* Enable interrupts. */
	WRITE4(sc, RTSX_BIER,
	    RTSX_TRANS_OK_INT_EN | RTSX_TRANS_FAIL_INT_EN | RTSX_SD_INT_EN);

	/* Power on SSC clock. */
	RTSX_CLR(sc, RTSX_FPDCTL, RTSX_SSC_POWER_DOWN);
	DELAY(200);

	/* XXX magic numbers from linux driver */
	if (sc->rtsx_flags & RTSX_F_5209)
		error = rtsx_write_phy(sc, 0x00, 0xB966);
	else
		error = rtsx_write_phy(sc, 0x00, 0xBA42);
	if (error) {
		device_printf(sc->rtsx_dev, "Can't write phy register\n");
		return (1);
	}

	RTSX_SET(sc, RTSX_CLK_DIV, 0x07);

	/* Disable sleep mode. */
	RTSX_CLR(sc, RTSX_HOST_SLEEP_STATE,
	    RTSX_HOST_ENTER_S1 | RTSX_HOST_ENTER_S3);

	/* Disable card clock. */
	RTSX_CLR(sc, RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);

	RTSX_CLR(sc, RTSX_CHANGE_LINK_STATE,
	    RTSX_FORCE_RST_CORE_EN | RTSX_NON_STICKY_RST_N_DBG | 0x04);
	RTSX_WRITE(sc, RTSX_SD30_DRIVE_SEL, RTSX_SD30_DRIVE_SEL_3V3);

	/* Enable SSC clock. */
	RTSX_WRITE(sc, RTSX_SSC_CTL1, RTSX_SSC_8X_EN | RTSX_SSC_SEL_4M);
	RTSX_WRITE(sc, RTSX_SSC_CTL2, 0x12);

	RTSX_SET(sc, RTSX_CHANGE_LINK_STATE, RTSX_MAC_PHY_RST_N_DBG);
	RTSX_SET(sc, RTSX_IRQSTAT0, RTSX_LINK_READY_INT);

	RTSX_WRITE(sc, RTSX_PERST_GLITCH_WIDTH, 0x80);

	/* Set RC oscillator to 400K. */
	RTSX_CLR(sc, RTSX_RCCTL, RTSX_RCCTL_F_2M);

	/* Request clock by driving CLKREQ pin to zero. */
	RTSX_SET(sc, RTSX_PETXCFG, RTSX_PETXCFG_CLKREQ_PIN);

	/* Set up LED GPIO. */
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_WRITE(sc, RTSX_CARD_GPIO, 0x03);
		RTSX_WRITE(sc, RTSX_CARD_GPIO_DIR, 0x03);
	} else {
		RTSX_SET(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		/* Switch LDO3318 source from DV33 to 3V3. */
		RTSX_CLR(sc, RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_DV33);
		RTSX_SET(sc, RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_3V3);
		/* Set default OLT blink period. */
		RTSX_SET(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_PERIOD);
	}

	return (0);
}

static void
rtsx_start(struct rtsx_softc *sc)
{
	if (rtsx_is_card_present(sc))
		rtsx_card_insert(sc);
	else
		rtsx_card_remove(sc);
}

static void
rtsx_stop(struct rtsx_softc *sc)
{
	if (sc->rtsx_mmc_dev != NULL) {
		/* detach mmc bus */
		device_delete_child(sc->rtsx_dev, sc->rtsx_mmc_dev);
		sc->rtsx_mmc_dev = NULL;
	}
}

static int
rtsx_read(struct rtsx_softc *sc, uint16_t addr, uint8_t *val)
{
	int tries = 1024;
	uint32_t reg;
	
	WRITE4(sc, RTSX_HAIMR, RTSX_HAIMR_BUSY |
	    (uint32_t)((addr & 0x3FFF) << 16));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY))
			break;
	}

	*val = (reg & 0xff);
	return (tries == 0) ? ETIMEDOUT : 0;
}

static int
rtsx_read_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr, uint32_t *val)
{
	int tries = 1024;
	uint8_t data0, data1, data2, data3, rwctl;

	RTSX_WRITE(sc, RTSX_CFGADDR0, addr);
	RTSX_WRITE(sc, RTSX_CFGADDR1, addr >> 8);
	RTSX_WRITE(sc, RTSX_CFGRWCTL, RTSX_CFG_BUSY | (func & 0x03 << 4));

	while (tries--) {
		RTSX_READ(sc, RTSX_CFGRWCTL, &rwctl);
		if (!(rwctl & RTSX_CFG_BUSY))
			break;
	}

	if (tries == 0)
		return (EIO);
	
	RTSX_READ(sc, RTSX_CFGDATA0, &data0);
	RTSX_READ(sc, RTSX_CFGDATA1, &data1);
	RTSX_READ(sc, RTSX_CFGDATA2, &data2);
	RTSX_READ(sc, RTSX_CFGDATA3, &data3);

	*val = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

	return (0);
}

static int
rtsx_write(struct rtsx_softc *sc, uint16_t addr, uint8_t mask, uint8_t val)
{
	int tries = 1024;
	uint32_t reg;
	WRITE4(sc, RTSX_HAIMR,
	    RTSX_HAIMR_BUSY | RTSX_HAIMR_WRITE |
	    (uint32_t)(((addr & 0x3FFF) << 16) |
	    (mask << 8) | val));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY)) {
			if (val != (reg & 0xff))
				return (EIO);
			return (0);
		}
	}

	return (ETIMEDOUT);
}

static int
rtsx_write_phy(struct rtsx_softc *sc, uint8_t addr, uint16_t val)
{
	int timeout = 100000;
	uint8_t rwctl;

	RTSX_WRITE(sc, RTSX_PHY_DATA0, val);
	RTSX_WRITE(sc, RTSX_PHY_DATA1, val >> 8);
	RTSX_WRITE(sc, RTSX_PHY_ADDR, addr);
	RTSX_WRITE(sc, RTSX_PHY_RWCTL, RTSX_PHY_BUSY|RTSX_PHY_WRITE);

	while (timeout--) {
		RTSX_READ(sc, RTSX_PHY_RWCTL, &rwctl);
		if (!(rwctl & RTSX_PHY_BUSY))
			break;
	}
	
	if (timeout == 0)
		return (ETIMEDOUT);
		
	return (0);
}

static int
rtsx_stop_sd_clock(struct rtsx_softc *sc)
{
	RTSX_CLR(sc, RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);
	RTSX_SET(sc, RTSX_SD_BUS_STAT, RTSX_SD_CLK_FORCE_STOP);
	return (0);
}

static int
rtsx_switch_sd_clock(struct rtsx_softc *sc, uint8_t n, int div, int mcu)
{
	/* Enable SD 2.0 mode. */
	RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_SD_MODE_MASK);

	RTSX_SET(sc, RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);

	RTSX_WRITE(sc, RTSX_CARD_CLK_SOURCE,
		   RTSX_CRC_FIX_CLK | RTSX_SD30_VAR_CLK0 | RTSX_SAMPLE_VAR_CLK1);
	RTSX_CLR(sc, RTSX_SD_SAMPLE_POINT_CTL, RTSX_SD20_RX_SEL_MASK);
	RTSX_WRITE(sc, RTSX_SD_PUSH_POINT_CTL, RTSX_SD20_TX_NEG_EDGE);
	RTSX_WRITE(sc, RTSX_CLK_DIV, (div << 4) | mcu);
	RTSX_CLR(sc, RTSX_SSC_CTL1, RTSX_RSTB);
	RTSX_CLR(sc, RTSX_SSC_CTL2, RTSX_SSC_DEPTH_MASK);
	RTSX_WRITE(sc, RTSX_SSC_DIV_N_0, n);
	RTSX_SET(sc, RTSX_SSC_CTL1, RTSX_RSTB);
	DELAY(100);

	RTSX_CLR(sc, RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);
	return (0);
}

/*
 * Notice that the meaning of RTSX_PWR_GATE_CTRL changes between RTS5209 and
 * RTS5229. In RTS5209 it is a mask of disabled power gates, while in RTS5229
 * it is a mask of *enabled* gates.
 */
static int
rtsx_bus_power_off(struct rtsx_softc *sc)
{
	int error;
	uint8_t disable3;

	if ((error = rtsx_stop_sd_clock(sc)))
		return (error);

	/* Disable SD output. */
	RTSX_CLR(sc, RTSX_CARD_OE, RTSX_CARD_OUTPUT_EN);

	/* Turn off power. */
	disable3 = RTSX_PULL_CTL_DISABLE3;
	if (sc->rtsx_flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else {
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1 | RTSX_LDO3318_VCC2);
		if (sc->rtsx_flags & RTSX_F_5229_TYPE_C)
			disable3 = RTSX_PULL_CTL_DISABLE3_TYPE_C;
	}

	RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_PMOS_STRG_800mA);

	/* Disable pull control. */
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, disable3);

	return (0);
}

static int
rtsx_bus_power_up(struct rtsx_softc *sc)
{
	uint8_t enable3;
	int error;

	if (sc->rtsx_flags & RTSX_F_525A) {
		error = rtsx_write(sc, RTSX_LDO_VCC_CFG1, RTSX_LDO_VCC_TUNE_MASK,
				   RTSX_LDO_VCC_3V3);
		if (error) {
			device_printf(sc->rtsx_dev, "Error bus power on RTSX_F_F525A");
			return (error);
		}
	}

	/* Select SD card. */
	RTSX_WRITE(sc, RTSX_CARD_SELECT, RTSX_SD_MOD_SEL);
	RTSX_WRITE(sc, RTSX_CARD_SHARE_MODE, RTSX_CARD_SHARE_48_SD);
	RTSX_SET(sc, RTSX_CARD_CLK_EN, RTSX_SD_CLK_EN);

	/* Enable pull control. */
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
	if (sc->rtsx_flags & RTSX_F_5229_TYPE_C)
		enable3 = RTSX_PULL_CTL_ENABLE3_TYPE_C;
	else
		enable3 = RTSX_PULL_CTL_ENABLE3;
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, enable3);

	/*
	 * To avoid a current peak, enable card power in two phases with a
	 * delay in between.
	 */

	/* Partial power. */
	RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PARTIAL_PWR_ON);
	if (sc->rtsx_flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);

	return (0);
}

static int
rtsx_bus_power_on(struct rtsx_softc *sc)
{

	/* Full power. */
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	if (sc->rtsx_flags & RTSX_F_5209)
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);

	/* Enable SD card output. */
	RTSX_WRITE(sc, RTSX_CARD_OE, RTSX_SD_OUTPUT_EN);

	return (0);
}

static int
rtsx_is_card_present(struct rtsx_softc *sc)
{
	uint32_t status;

	status = READ4(sc, RTSX_BIPR);
	return (status & RTSX_SD_EXIST);
}

static void
rtsx_card_insert(struct rtsx_softc *sc)
{
	device_t mmc_dev;

	device_printf(sc->rtsx_dev, "Card inserted\n");

	RTSX_LOCK(sc);
	sc->rtsx_flags |= RTSX_F_CARD_PRESENT;
	/* Schedule card discovery task. */
	if (sc->rtsx_mmc_dev == NULL) {
		/* attach mmc bus */
		sc->rtsx_mmc_dev = device_add_child(sc->rtsx_dev, "mmc", -1);
		if ((mmc_dev = sc->rtsx_mmc_dev) == NULL)
			device_printf(sc->rtsx_dev, "Adding MMC bus failed\n");
		RTSX_UNLOCK(sc);
		if (mmc_dev != NULL)
			if (device_probe_and_attach(sc->rtsx_mmc_dev))
				device_printf(sc->rtsx_dev, "Attaching MMC bus failed\n");
	}
	rtsx_led_enable(sc);
}

static void
rtsx_card_remove(struct rtsx_softc *sc)
{
	device_t mmc_dev;
	
	device_printf(sc->rtsx_dev, "Card removed\n");

	RTSX_LOCK(sc);
	sc->rtsx_flags &= ~RTSX_F_CARD_PRESENT;
	/* Schedule card discovery task. */
	if ((mmc_dev = sc->rtsx_mmc_dev) != NULL) {
		/* detach mmc bus */
		sc->rtsx_mmc_dev = NULL;
		RTSX_UNLOCK(sc);
		if (device_delete_child(sc->rtsx_dev, mmc_dev))
			device_printf(sc->rtsx_dev, "Detaching MMC bus failed\n");
	} else {
		RTSX_UNLOCK(sc);
	}
	rtsx_led_disable(sc);
}

static int
rtsx_led_enable(struct rtsx_softc *sc)
{
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
		RTSX_WRITE(sc, RTSX_CARD_AUTO_BLINK,
			   RTSX_LED_BLINK_EN | RTSX_LED_BLINK_SPEED);
	} else {
		RTSX_SET(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		RTSX_SET(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
	}

	return (0);
}

static int
rtsx_led_disable(struct rtsx_softc *sc)
{
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_AUTO_BLINK, RTSX_LED_BLINK_EN);
		RTSX_WRITE(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
	} else {
		RTSX_CLR(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
		RTSX_CLR(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
	}

	return (0);
}

static uint8_t
rtsx_response_type(uint16_t mmc_rsp)
{
	int i;
	struct rsp_type {
		uint16_t mmc_rsp;
		uint8_t  rtsx_rsp;
	} rsp_types[] = {
		{ MMC_RSP_NONE,	RTSX_SD_RSP_TYPE_R0 },
		{ MMC_RSP_R1,	RTSX_SD_RSP_TYPE_R1 },
		{ MMC_RSP_R1B,	RTSX_SD_RSP_TYPE_R1B },
		{ MMC_RSP_R2,	RTSX_SD_RSP_TYPE_R2 },
		{ MMC_RSP_R3,	RTSX_SD_RSP_TYPE_R3 },
		{ MMC_RSP_R4,	RTSX_SD_RSP_TYPE_R4 },
		{ MMC_RSP_R5,	RTSX_SD_RSP_TYPE_R5 },
		{ MMC_RSP_R6,	RTSX_SD_RSP_TYPE_R6 },
		{ MMC_RSP_R7,	RTSX_SD_RSP_TYPE_R7 }
	};

	for (i = 0; i < nitems(rsp_types); i++) {
		if (mmc_rsp == rsp_types[i].mmc_rsp)
			return rsp_types[i].rtsx_rsp;
	}

	return (0);
}

/* Append a properly encoded host command to the host command buffer. */
static void
rtsx_hostcmd(uint32_t *cmdbuf, int *n, uint8_t cmd, uint16_t reg,
	     uint8_t mask, uint8_t data)
{
	KASSERT(*n < RTSX_HOSTCMD_MAX, ("rtsx: too many host commands (%d)\n", *n));

	cmdbuf[(*n)++] = htole32((uint32_t)(cmd & 0x3) << 30) |
	    ((uint32_t)(reg & 0x3fff) << 16) |
	    ((uint32_t)(mask) << 8) |
	    ((uint32_t)data);
}

static void
rtsx_hostcmd_send(struct rtsx_softc *sc, int ncmd)
{
	/* Tell the chip where the command buffer is and run the commands. */
	WRITE4(sc, RTSX_HCBAR, (uint32_t)sc->rtsx_cmd_buffer);
	WRITE4(sc, RTSX_HCBCTLR,
	       ((ncmd * 4) & 0x00ffffff) | RTSX_START_CMD | RTSX_HW_AUTO_RSP);
}

static void
rtsx_req_timeout(void *arg)
{
	struct rtsx_softc *sc;

	sc = (struct rtsx_softc *)arg;
	if (sc->rtsx_req != NULL) {
		device_printf(sc->rtsx_dev, "Controller timeout\n");
		sc->rtsx_req->cmd->error =  MMC_ERR_TIMEOUT;
		rtsx_req_done(sc);
	} else {
		device_printf(sc->rtsx_dev, "Spurious timeout - no active request\n");
	}
}

static void
rtsx_req_done(struct rtsx_softc *sc)
{
	struct mmc_request *req;

	callout_stop(&sc->rtsx_timeoutc);
	req = sc->rtsx_req;
	if (req->cmd->error != MMC_ERR_NONE)
		rtsx_soft_reset(sc);
	sc->rtsx_req = NULL;
	req->done(req);
}

/*---
  static int
rtsx_wait_intr(struct rtsx_softc *sc, int mask, int timeout)
{
	int status;
	int error = 0;

	 RTSX_LOCK() done by caller
	
	mask |= RTSX_TRANS_FAIL_INT;

	status = sc->rtsx_intr_status & mask;
	while (status == 0) {

		device_printf(sc->rtsx_dev, "waiting for %#x - rtsx_intr_status = %#x\n", mask, sc->rtsx_intr_status);
		
		if ((msleep(&sc->rtsx_intr_status, &sc->rtsx_mtx, PRIBIO, "rtsxintr", timeout) == EWOULDBLOCK)) {
			rtsx_soft_reset(sc);
			error = ETIMEDOUT;
			break;
		}
		status = sc->rtsx_intr_status & mask;
	}
	sc->rtsx_intr_status &= ~status;

	if (!ISSET(sc->rtsx_flags, RTSX_F_CARD_PRESENT))
		error = ENODEV;

	if (error == 0 && (status & RTSX_TRANS_FAIL_INT))
		error = EIO;

	device_printf(sc->rtsx_dev, "rtsx_wait_intr return %d\n", error);

	return (error);
}
---*/

/* Prepare for another command. */
static void
rtsx_soft_reset(struct rtsx_softc *sc)
{
	device_printf(sc->rtsx_dev, "Soft reset\n");

	/* Stop command transfer. */
	WRITE4(sc, RTSX_HCBCTLR, RTSX_STOP_CMD);

	(void)rtsx_write(sc, RTSX_CARD_STOP, RTSX_SD_STOP|RTSX_SD_CLR_ERR,
		    RTSX_SD_STOP|RTSX_SD_CLR_ERR);

	/* Stop DMA transfer. */
	WRITE4(sc, RTSX_HDBCTLR, RTSX_STOP_DMA);
	(void)rtsx_write(sc, RTSX_DMACTL, RTSX_DMA_RST, RTSX_DMA_RST);

	(void)rtsx_write(sc, RTSX_RBCTL, RTSX_RB_FLUSH, RTSX_RB_FLUSH);
}

static int
rtsx_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	case MMCBR_IVAR_BUS_MODE:
		*result = sc->rtsx_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*result = sc->rtsx_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*result = sc->rtsx_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*result = sc->rtsx_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*result = sc->rtsx_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*result = sc->rtsx_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR: 	/* host operation conditions register */
		*result = sc->rtsx_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*result = sc->rtsx_host.mode;
		break;
	case MMCBR_IVAR_OCR:		/* operation conditions register */
		*result = sc->rtsx_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*result = sc->rtsx_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*result = sc->rtsx_host.ios.vdd;
		break;
	case MMCBR_IVAR_VCCQ:
		*result = sc->rtsx_host.ios.vccq;
		break;
	case MMCBR_IVAR_CAPS:
		*result = sc->rtsx_host.caps;
		break;
	case MMCBR_IVAR_TIMING:
		*result = sc->rtsx_host.ios.timing;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*result = MAXPHYS / MMC_SECTOR_SIZE;
		break;
	case MMCBR_IVAR_RETUNE_REQ:
		*result = retune_req_none;
		break;
	default:
		return (EINVAL);
	}

	device_printf(bus, "Read ivar #%d, value %#x / #%d\n", which, *(int *)result, *(int *)result);

	return (0);
}

static int
rtsx_write_ivar(device_t bus, device_t child, int which, uintptr_t value)
{
	struct rtsx_softc *sc;

	device_printf(bus, "Write ivar #%d, value %#x / #%d\n", which, (int)value, (int)value);

	sc = device_get_softc(bus);
	switch (which) {
	case MMCBR_IVAR_BUS_MODE:
		sc->rtsx_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->rtsx_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->rtsx_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->rtsx_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->rtsx_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:		/* operation conditions register */
		sc->rtsx_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->rtsx_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->rtsx_host.ios.vdd = value;
		break;
	case MMCBR_IVAR_VCCQ:
		sc->rtsx_host.ios.vccq = value;
		break;
	case MMCBR_IVAR_TIMING:
		sc->rtsx_host.ios.timing = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
	default:
		return (EINVAL);
	}

	return (0);
}

static int
rtsx_mmcbr_update_ios(device_t bus, device_t child)
{
	struct rtsx_softc *sc;
	struct mmc_ios *ios;
	uint32_t bus_width;
	int error;

	sc = device_get_softc(bus);
	ios = &sc->rtsx_host.ios;

	device_printf(bus, "rtsx_mmcbr_update_ios()\n");

	/* Set the bus width. */
	switch (ios->bus_width) {
	case bus_width_1:
		bus_width = RTSX_BUS_WIDTH_1;
		break;
	case bus_width_4:	
		bus_width = RTSX_BUS_WIDTH_4;
		break;
	case bus_width_8:
		bus_width = RTSX_BUS_WIDTH_8;
		break;
	}
	if ((error = rtsx_write(sc, RTSX_SD_CFG1, RTSX_BUS_WIDTH_MASK, bus_width)))
		return (error);
	
	/* Set power mode */
	switch (ios->power_mode) {
	case power_off:
		rtsx_bus_power_off(sc);
		break;
	case power_up:
		rtsx_bus_power_up(sc);
		break;
	case power_on:
		rtsx_bus_power_on(sc);
		break;
	};

	/* play with the clock */
	/* To be done */

	return (0);
}

static int
rtsx_mmcbr_request(device_t bus, device_t child __unused,struct mmc_request *req)
{
	struct rtsx_softc *sc;
	struct mmc_command *cmd;
	uint32_t *cmd_buffer;
	uint8_t rsp_type;
	uint16_t r;
	int ncmd =0;
	int error = 0;
	
	sc = device_get_softc(bus);

	RTSX_LOCK(sc);
	if (sc->rtsx_req != NULL) {
		RTSX_UNLOCK(sc);
                return (EBUSY);
        }
	sc->rtsx_req = req;
	cmd = req->cmd;

	device_printf(sc->rtsx_dev, "rtsx_mmcbr_request(CMD%u arg %#x flags %#x dlen %u dflags %#x)\n",
		      cmd->opcode, cmd->arg, cmd->flags,
		      cmd->data != NULL ? (unsigned int)cmd->data->len : 0,
		      cmd->data != NULL ? cmd->data->flags: 0);

	/* Start the request */
	rsp_type = rtsx_response_type(cmd->flags & MMC_RSP_MASK);
	if (rsp_type == 0) {
		device_printf(sc->rtsx_dev, "Unknown response type 0x%lx\n", (cmd->flags & MMC_RSP_MASK));
		error = EINVAL;
		goto done;
	}

	/* Queue commands to set SD command index and argument. */
	cmd_buffer = (uint32_t *)(sc->rtsx_cmd_dmamem);
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD0, 0xff, 0x40  | cmd->opcode); 
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD1, 0xff, cmd->arg >> 24);
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD2, 0xff, cmd->arg >> 16);
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD3, 0xff, cmd->arg >> 8);
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD4, 0xff, cmd->arg);

	/* Queue command to set response type. */
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CFG2, 0xff, rsp_type);

	/* Use the ping-pong buffer for commands which do not transfer data. */
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
	    0x01, RTSX_PINGPONG_BUFFER);

	/* Queue commands to perform SD transfer. */
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
	    0xff, RTSX_TM_CMD_RSP | RTSX_SD_TRANSFER_START);
	rtsx_hostcmd(cmd_buffer, &ncmd,
	    RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE);

	/* If needed queue commands to read back card status response.*/
	if (rsp_type == RTSX_SD_RSP_TYPE_R2) {
		for (r = RTSX_PPBUF_BASE2 + 15; r > RTSX_PPBUF_BASE2; r--)
			rtsx_hostcmd(cmd_buffer, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
		rtsx_hostcmd(cmd_buffer, &ncmd, RTSX_READ_REG_CMD, RTSX_SD_CMD5,
		    0, 0);
	} else if (rsp_type != RTSX_SD_RSP_TYPE_R0) {
		for (r = RTSX_SD_CMD0; r <= RTSX_SD_CMD4; r++)
			rtsx_hostcmd(cmd_buffer, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
	}

	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREWRITE);

	/* Run the command queue and wait for completion. */
	rtsx_hostcmd_send(sc, ncmd);

	callout_reset(&sc->rtsx_timeoutc, sc->rtsx_timeout * hz,
		      rtsx_req_timeout, sc);

	/*---
	if (cmd->c_data) {
		error = rtsx_xfer(sc, cmd, cmdbuf);
		if (error) {
			u_int8_t stat1;

			if (rtsx_read(sc, RTSX_SD_STAT1, &stat1) == 0 &&
			    (stat1 & RTSX_SD_CRC_ERR))
				printf("%s: CRC error\n", DEVNAME(sc));
		}
	}
	---*/
		
 done:
	RTSX_UNLOCK(sc);
	return (error);
}

static int
rtsx_mmcbr_acquire_host(device_t bus, device_t child __unused)
{
	struct rtsx_softc *sc;
	
	device_printf(bus, "rtsx_mmcbr_acquite_host()\n");

	sc = device_get_softc(bus);
	RTSX_LOCK(sc);
	while (sc->rtsx_bus_busy)
                msleep(sc, &sc->rtsx_mtx, 0, "rtsxah", 0);
	sc->rtsx_bus_busy++;
	RTSX_UNLOCK(sc);
	return (0);
}
	       
static int
rtsx_mmcbr_release_host(device_t bus, device_t child __unused)
{
	struct rtsx_softc *sc;

	device_printf(bus, "rtsx_mmcbr_release_host()\n");

	sc = device_get_softc(bus);
	RTSX_LOCK(sc);
	sc->rtsx_bus_busy--;
	RTSX_UNLOCK(sc);
	wakeup(sc);
	
	return (0);
}

/*
 *
 * PCI Support Functions
 *
 */

static int
rtsx_probe(device_t dev)
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
	struct rtsx_softc *sc;
	int i, result;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

//	device_printf(dev, "Probe - Vendor ID: 0x%x - Device ID: 0x%x\n", vendor, device);

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

	return (result);
}

static int
rtsx_attach(device_t dev)
{
/* Attach function is only called if the probe is successful. */

	struct rtsx_softc 	*sc = device_get_softc(dev);
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid_list	*tree;
	int			msi_count = 1;
	uint32_t		sdio_cfg;
	int			error;
	
	device_printf(dev, "Attach - Vendor ID: 0x%x - Device ID: 0x%x\n",
		      pci_get_vendor(dev), pci_get_device(dev));

	sc->rtsx_dev = dev;
	RTSX_LOCK_INIT(sc);
	callout_init_mtx(&sc->rtsx_timeoutc, &sc->rtsx_mtx, 0);

	/* timeout parameter for callout */
	sc->rtsx_timeout = 10;
	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "req_timeout", CTLFLAG_RW,
		       &sc->rtsx_timeout, 0, "Request timeout in seconds");

	/* Allocate IRQ. */
	sc->rtsx_irq_res_id = 0;
	if (pci_alloc_msi(dev, &msi_count) == 0)
		sc->rtsx_irq_res_id = 1;
	sc->rtsx_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->rtsx_irq_res_id,
						  RF_ACTIVE | (sc->rtsx_irq_res_id != 0 ? 0 : RF_SHAREABLE));
	if (sc->rtsx_irq_res == NULL) {
		device_printf(dev, "Can't allocate IRQ resources for %d\n", sc->rtsx_irq_res_id);
		pci_release_msi(dev);
		return (ENXIO);
	}

	/* Allocate memory resource */
	sc->rtsx_res_id = PCIR_BAR(0);
	sc->rtsx_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rtsx_res_id, RF_ACTIVE);
	if (sc->rtsx_res == NULL) {
		device_printf(dev, "Can't allocate memory resource for %d\n", sc->rtsx_res_id);
		goto destroy_rtsx_irq_res;
	}
	device_printf(dev, "rtsx_irq_res_id: %d - rtsx_res_id: %d\n", sc->rtsx_irq_res_id, sc->rtsx_res_id);
	sc->rtsx_btag = rman_get_bustag(sc->rtsx_res);
	sc->rtsx_bhandle = rman_get_bushandle(sc->rtsx_res);
	
	/* Activate the interrupt */
	error = bus_setup_intr(dev, sc->rtsx_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			       NULL, rtsx_intr, sc, &sc->rtsx_irq_cookie);
	if (error) {
		device_printf(dev, "Can't set up irq [0x%x]!\n", error);
		goto destroy_rtsx_res;
	}
//	callout_init_mtx(&sc->rtsx_timeoutc, &sc->rtsx_mtx, 0);
	pci_enable_busmaster(dev);
	
	/* Initialize device */
	if (rtsx_init(sc)) {
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
	/* Start device */
	rtsx_start(sc);

	/*
	 * Create a /dev entry for this device.  The kernel will assign us
	 * a major number automatically.  We use the unit number of this
	 * device as the minor number and name the character device
	 * "rtsx<unit>".
	 */
	sc->rtsx_cdev = make_dev(&rtsx_cdevsw, device_get_unit(dev),
	    UID_ROOT, GID_WHEEL, 0600, "rtsx%u", device_get_unit(dev));
	sc->rtsx_cdev->si_drv1 = sc;
	device_printf(dev, "Device attached\n");
	return (0);

 destroy_rtsx_irq:
	bus_teardown_intr(dev, sc->rtsx_irq_res, sc->rtsx_irq_cookie);	
 destroy_rtsx_res:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->rtsx_res_id,
			     sc->rtsx_res);
 destroy_rtsx_irq_res:
	bus_release_resource(dev, SYS_RES_IRQ, sc->rtsx_irq_res_id,
			     sc->rtsx_irq_res);
	pci_release_msi(dev);
	callout_drain(&sc->rtsx_timeoutc);
	RTSX_LOCK_DESTROY(sc);
	return (ENXIO);
}

	
static int
rtsx_detach(device_t dev)
{
	struct rtsx_softc *sc = device_get_softc(dev);

	device_printf(dev, "Detach - Vendor ID: 0x%x - Device ID: 0x%x\n",
		      pci_get_vendor(dev), pci_get_device(dev));
	
	/* Stop device */
	rtsx_stop(sc);

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
	RTSX_LOCK_DESTROY(sc);
	return (0);
}

static int
rtsx_shutdown(device_t dev)
{

	device_printf(dev, "Shutdown\n");
	return (0);
}

/*
 * Device suspend routine.
 */
static int
rtsx_suspend(device_t dev)
{

	device_printf(dev, "Suspend\n");
	return (0);
}

/*
 * Device resume routine.
 */
static int
rtsx_resume(device_t dev)
{

	device_printf(dev, "Resume\n");
	return (0);
}

static device_method_t rtsx_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rtsx_probe),
	DEVMETHOD(device_attach,	rtsx_attach),
	DEVMETHOD(device_detach,	rtsx_detach),
	DEVMETHOD(device_shutdown,	rtsx_shutdown),
	DEVMETHOD(device_suspend,	rtsx_suspend),
	DEVMETHOD(device_resume,	rtsx_resume),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	rtsx_read_ivar),
	DEVMETHOD(bus_write_ivar,	rtsx_write_ivar),
	
	/* MMC bridge interface */
     	DEVMETHOD(mmcbr_update_ios,	rtsx_mmcbr_update_ios),
//	DEVMETHOD(mmcbr_switch_vccq,	rtsx_mmcbr_switch_vccq),
//	DEVMETHOD(mmcbr_tune,		rtsx_mmcbr_tune),
//	DEVMETHOD(mmcbr_retune,		rtsx_mmcbr_retune),
	DEVMETHOD(mmcbr_request,	rtsx_mmcbr_request),
//	DEVMETHOD(mmcbr_get_ro,		rtsx_mmcbr_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	rtsx_mmcbr_acquire_host),
	DEVMETHOD(mmcbr_release_host,	rtsx_mmcbr_release_host),

	DEVMETHOD_END
};

static devclass_t rtsx_devclass;

DEFINE_CLASS_0(rtsx, rtsx_driver, rtsx_methods, sizeof(struct rtsx_softc));
DRIVER_MODULE(rtsx, pci, rtsx_driver, rtsx_devclass, NULL, NULL);
MMC_DECLARE_BRIDGE(rtsx);
