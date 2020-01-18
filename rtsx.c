/*
 *
 * Base on OpenBSD /sys/dev/pci/rtsx_pci.c & /dev/ic/rtsx.c
 *      on Linux   /drivers/mmc/host/rtsx_pci_sdmmc.c,
 *                 /include/linux/rtsx_pci.h &
 *                 /drivers/misc/cardreader/rtsx_pcr.c
 *      on NetBSD  /sys/dev/ic/rtsx.c
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
#include <sys/uio.h>		/* uio struct */
#include <sys/malloc.h>
#include <sys/bus.h>		/* structs, prototypes for pci bus stuff and DEVMETHOD macros! */
#include <sys/endian.h>

#include <machine/bus.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <machine/resource.h>
#include <sys/sysctl.h>

#include <dev/pci/pcivar.h>	/* For pci_get macros! */
#include <dev/pci/pcireg.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include "mmcbr_if.h"

#include "rtsxreg.h"

#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

/* flag values */
#define	RTSX_F_DEFAULT		0x0000
#define	RTSX_F_CARD_PRESENT	0x0001
#define	RTSX_F_SDIO_SUPPORT	0x0002
#define	RTSX_F_5209		0x0004
#define	RTSX_F_5227		0x0008
#define	RTSX_F_5229		0x0010
#define	RTSX_F_5229_TYPE_C	0x0020
#define	RTSX_F_525A		0x0040
#define RTSX_F_8411B		0x0080
#define RTSX_F_8411B_QFN48	0x0100

/* The softc holds our per-instance data. */
struct rtsx_softc {
	struct mtx	rtsx_mtx;		/* device mutex */
	device_t	rtsx_dev;		/* device */
	int		rtsx_flags;		/* device flags */
	device_t	rtsx_mmc_dev;		/* device of mmc bus */
	/* for card insert/remove - from dwmmc.c */
	struct task		card_task;	/* Card presence check task */
	struct timeout_task	card_delayed_task;/* Card insert delayed task */

	int		rtsx_irq_res_id;	/* bus IRQ resource id */
	struct resource *rtsx_irq_res;		/* bus IRQ resource */
	void		*rtsx_irq_cookie;	/* bus IRQ resource cookie */
	int		rtsx_res_id;		/* bus memory resource id */
	struct resource *rtsx_res;		/* bus memory resource */
	int		rtsx_res_type;		/* bus memory resource type */
	bus_space_tag_t	rtsx_btag;		/* host register set tag */
	bus_space_handle_t rtsx_bhandle;	/* host register set handle */
	int		rtsx_timeout;		/* timeout value */     

	bus_dma_tag_t	rtsx_cmd_dma_tag;	/* DMA tag for command transfer */
	bus_dmamap_t	rtsx_cmd_dmamap;	/* DMA map for command transfer */
	void		*rtsx_cmd_dmamem;	/* DMA mem for command transfer */
	bus_addr_t	rtsx_cmd_buffer;	/* device visible address of the DMA segment */
	int		rtsx_cmd_index;		/* index in rtsx_cmd_buffer */

	bus_dma_tag_t	rtsx_data_dma_tag;	/* DMA tag for data transfer */
	bus_dmamap_t	rtsx_data_dmamap;	/* DMA map for data transfer */
	void		*rtsx_data_dmamem;	/* DMA mem for data transfer */
	bus_addr_t	rtsx_data_buffer;	/* device visible address of the DMA segment */

	u_char		rtsx_bus_busy;		/* Bus busy status */ 
	struct mmc_host rtsx_host;		/* Host parameters */
	uint32_t	rtsx_sd_clock;		/* Current sd clock */
	enum mmc_power_mode rtsx_power_mode;	/* Current power mode */
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
	{ 0x10ec,	0x5209,	RTSX_F_5209,    "Realtek RTS5209 PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x5227,	RTSX_F_5227,	"Realtek RTS5227 PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x5229,	RTSX_F_5229,    "Realtek RTS5229 PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x522a,	RTSX_F_DEFAULT, "Realtek RTS522A PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x5249,	RTSX_F_5229,    "Realtek RTS5249 PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x525A,	RTSX_F_525A,    "Realtek RTS525A PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x5286,	RTSX_F_DEFAULT, "Realtek RTS5286 PCI MMC/SD Card Reader"},
	{ 0x10ec,	0x5287,	RTSX_F_8411B,	"Realtek RTL8411B PCI MMC/SD Card Reader"},
	{ 0, 		0,	0,		NULL}
};

static int	rtsx_dma_alloc(struct rtsx_softc *sc);
static void	rtsx_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error);
static void	rtsx_dma_free(struct rtsx_softc *sc);
static void	rtsx_intr(void *arg);
static int	rtsx_init(struct rtsx_softc *sc);
static int	rtsx_stop(struct rtsx_softc *sc);
static int	rtsx_read(struct rtsx_softc *, uint16_t, uint8_t *);
static int	rtsx_read_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr, uint32_t *val);
static int	rtsx_write(struct rtsx_softc *sc, uint16_t addr, uint8_t mask, uint8_t val);
static int	rtsx_write_phy(struct rtsx_softc *sc, uint8_t addr, uint16_t val);
static int	rtsx_set_sd_clock(struct rtsx_softc *sc, uint32_t freq);
static int	rtsx_stop_sd_clock(struct rtsx_softc *sc);
static int	rtsx_switch_sd_clock(struct rtsx_softc *sc, uint8_t n, int div, int mcu);
static int	rtsx_bus_power_off(struct rtsx_softc *sc);
static int	rtsx_bus_power_up(struct rtsx_softc *sc);
static int	rtsx_bus_power_on(struct rtsx_softc *sc);
static int	rtsx_is_card_present(struct rtsx_softc *sc);
#if 0 /* done in task */
static void	rtsx_start(struct rtsx_softc *sc);
static void	rtsx_card_insert(struct rtsx_softc *sc);
static void	rtsx_card_remove(struct rtsx_softc *sc);
#endif
static int	rtsx_led_enable(struct rtsx_softc *sc);
static int	rtsx_led_disable(struct rtsx_softc *sc);

static uint8_t	rtsx_response_type(uint16_t mmc_rsp);
static void	rtsx_init_cmd(struct rtsx_softc *sc, struct mmc_command *cmd);
static void	rtsx_push_cmd(struct rtsx_softc *sc, uint8_t cmd, uint16_t reg,
			      uint8_t mask, uint8_t data);
static int	rtsx_send_cmd(struct rtsx_softc *sc, struct mmc_command *cmd);
static void	rtsx_send_cmd_nowait(struct rtsx_softc *sc, struct mmc_command *cmd);
static void	rtsx_req_done(struct rtsx_softc *sc);
static void	rtsx_soft_reset(struct rtsx_softc *sc);
static int	rtsx_send_req_get_resp(struct rtsx_softc *sc, struct mmc_command *cmd);
static int	rtsx_xfer_short(struct rtsx_softc *sc, struct mmc_command *cmd);
static int	rtsx_read_ppbuf(struct rtsx_softc *sc, struct mmc_command *cmd);
static int	rtsx_write_ppbuf(struct rtsx_softc *sc, struct mmc_command *cmd);
static int	rtsx_xfer(struct rtsx_softc *sc, struct mmc_command *cmd);

static int	rtsx_read_ivar(device_t bus, device_t child, int which, uintptr_t *result);
static int	rtsx_write_ivar(device_t bus, device_t child, int which, uintptr_t value);

static int	rtsx_probe(device_t dev);
static int	rtsx_attach(device_t dev);
static int	rtsx_detach(device_t dev);
static int	rtsx_shutdown(device_t dev);
static int	rtsx_suspend(device_t dev);
static int	rtsx_resume(device_t dev);

static int	rtsx_mmcbr_update_ios(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_switch_vccq(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_tune(device_t bus, device_t child __unused, bool hs400 __unused);
static int	rtsx_mmcbr_retune(device_t bus, device_t child __unused, bool reset __unused);
static int	rtsx_mmcbr_request(device_t bus, device_t child __unused, struct mmc_request *req);
static int	rtsx_mmcbr_get_ro(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_acquire_host(device_t bus, device_t child __unused);
static int	rtsx_mmcbr_release_host(device_t bus, device_t child __unused);
static void rtsx_card_task(void *arg, int pending __unused);
static void rtsx_handle_card_present(struct rtsx_softc *sc);

#define RTSX_LOCK_INIT(_sc)	mtx_init(&(_sc)->rtsx_mtx,	\
					 device_get_nameunit(sc->rtsx_dev), "rtsx", MTX_DEF)
#define RTSX_LOCK(_sc)		mtx_lock(&(_sc)->rtsx_mtx)
#define RTSX_UNLOCK(_sc)	mtx_unlock(&(_sc)->rtsx_mtx)
#define RTSX_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->rtsx_mtx)

#define RTSX_SDCLK_OFF		0
#define RTSX_SDCLK_400KHZ	400000
#define RTSX_SDCLK_25MHZ	25000000
#define RTSX_SDCLK_50MHZ	50000000

#define RTSX_MAX_DATA_BLKLEN	512

#define RTSX_DMA_ALIGN		4
#define RTSX_DMA_BLOCK_SIZE	4096

#define	RTSX_DMA_MAX_SEGSIZE	0x80000
#define	RTSX_HOSTCMD_MAX	256
#define	RTSX_HOSTCMD_BUFSIZE	(sizeof(uint32_t) * RTSX_HOSTCMD_MAX)
#define	RTSX_DMA_DATA_BUFSIZE	MAXPHYS / 4
#define	RTSX_ADMA_DESC_SIZE	(sizeof(uint64_t) * SDMMC_MAXNSEGS)

#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

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

#define RTSX_BITOP(sc, reg, mask, bits)				\
	do {							\
        	int err = rtsx_write((sc), (reg), (mask), (bits));\
		if (err)					\
			return err;				\
	} while (0)

/* 
 * We use two DMA buffers: a command buffer and a data buffer.
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
 * The data buffer is used for transfer longer than 512. Data transfer is
 * controlled via the RTSX_HDBAR register and completion is signalled by
 * the TRANS_OK interrupt.
 *
 * The chip is unable to perform DMA above 4GB.
 */

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
	    RTSX_DMA_DATA_BUFSIZE, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    RTSX_DMA_DATA_BUFSIZE, 1,	/* maxsize, nsegments */
	    RTSX_DMA_DATA_BUFSIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rtsx_data_dma_tag);
	if (error) {
                device_printf(sc->rtsx_dev,
			      "Can't create data parent DMA tag\n");
		goto destroy_cmd_dmamap_load;
	}
	error = bus_dmamem_alloc(sc->rtsx_data_dma_tag,		/* DMA tag */
	    &sc->rtsx_data_dmamem,				/* will hold the KVA pointer */
	    BUS_DMA_WAITOK | BUS_DMA_ZERO,			/* flags */
	    &sc->rtsx_data_dmamap); 				/* DMA map */
	if (error) {
                device_printf(sc->rtsx_dev,
			      "Can't create DMA map for data transfer\n");
		goto destroy_data_dma_tag;
	}
	error = bus_dmamap_load(sc->rtsx_data_dma_tag,	/* DMA tag */
	    sc->rtsx_data_dmamap,	/* DMA map */
	    sc->rtsx_data_dmamem,	/* KVA pointer to be mapped */
	    RTSX_DMA_DATA_BUFSIZE,	/* size of buffer */
	    rtsx_dmamap_cb,		/* callback */
	    &sc->rtsx_data_buffer,	/* first arg of callback */
	    0);			/* flags */
	if (error != 0 || sc->rtsx_data_buffer == 0) {
                device_printf(sc->rtsx_dev,
			      "Can't load DMA memory for data transfer\n");
                error = (error) ? error : EFAULT;
		goto destroy_data_dmamem_alloc;
        }
	return (error);
	
destroy_data_dmamem_alloc:
	bus_dmamem_free(sc->rtsx_data_dma_tag, sc->rtsx_data_dmamem, sc->rtsx_data_dmamap);
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
                        bus_dmamap_unload(sc->rtsx_data_dma_tag,
					  sc->rtsx_data_dmamap);
                if (sc->rtsx_data_dmamem != NULL)
                        bus_dmamem_free(sc->rtsx_data_dma_tag,
					sc->rtsx_data_dmamem,
					sc->rtsx_data_dmamap);
		sc->rtsx_data_dmamap = NULL;
		sc->rtsx_data_dmamem = NULL;
                sc->rtsx_data_buffer = 0;
                bus_dma_tag_destroy(sc->rtsx_data_dma_tag);
                sc->rtsx_data_dma_tag = NULL;
	}
}
	
/* from dwmmc.c */
/* called from the IRQ handler */
static void
rtsx_handle_card_present(struct rtsx_softc *sc)
{
	bool was_present;
	bool is_present;

	was_present = sc->rtsx_mmc_dev != NULL;
	is_present = (rtsx_is_card_present(sc) != 0);

	if (!was_present && is_present) {
		/* small delay for the controller */
		taskqueue_enqueue_timeout(taskqueue_swi_giant,
		  &sc->card_delayed_task, -(hz / 2));
	} else if (was_present && !is_present) {
		taskqueue_enqueue(taskqueue_swi_giant, &sc->card_task);
	}
}

/* this is called at startup */
static void
rtsx_card_task(void *arg, int pending __unused)
{
	struct rtsx_softc *sc = arg;

	RTSX_LOCK(sc);

	if (rtsx_is_card_present(sc) != 0) {
		sc->rtsx_flags |= RTSX_F_CARD_PRESENT;
		/* Card is present, attach if necessary */
		if (sc->rtsx_mmc_dev == NULL) {
			if (bootverbose)
				device_printf(sc->rtsx_dev, "Card inserted\n");

			sc->rtsx_mmc_dev = device_add_child(sc->rtsx_dev, "mmc", -1);
			RTSX_UNLOCK(sc);
			if (sc->rtsx_mmc_dev != NULL) {
				device_set_ivars(sc->rtsx_mmc_dev, sc);
				(void)device_probe_and_attach(sc->rtsx_mmc_dev);
			}
		} else
			RTSX_UNLOCK(sc);
		
	} else {
		sc->rtsx_flags &= ~RTSX_F_CARD_PRESENT;
		/* Card isn't present, detach if necessary */
		if (sc->rtsx_mmc_dev != NULL) {
			if (bootverbose)
				device_printf(sc->rtsx_dev, "Card removed\n");

			RTSX_UNLOCK(sc);
			device_delete_child(sc->rtsx_dev, sc->rtsx_mmc_dev);
			sc->rtsx_mmc_dev = NULL;
		} else
			RTSX_UNLOCK(sc);
	}
}

static void
rtsx_intr(void *arg)
{
	struct rtsx_softc *sc = arg;
	uint32_t enabled, status;

	RTSX_LOCK(sc);
	enabled = READ4(sc, RTSX_BIER);	/* read Bus Interrupt Enable ister */
	status = READ4(sc, RTSX_BIPR);	/* read Bus Interrupt pending Register */

	if (bootverbose)
		device_printf(sc->rtsx_dev, "Interrupt handler - enabled: %#x, status: %#x\n", enabled, status);

	/* Ack interrupts. */
	WRITE4(sc, RTSX_BIPR, status);

	if (((enabled & status) == 0) || status == 0xffffffff) {
device_printf(sc->rtsx_dev, "FLAGS\n");
		RTSX_UNLOCK(sc);
		return;
	}

	/* start task to handle SD card status change */
	/* from dwmmc.c */
	if (status & RTSX_SD_INT) {
device_printf(sc->rtsx_dev, "A\n");
		rtsx_handle_card_present(sc);
	}
	if (sc->rtsx_req == NULL) {
device_printf(sc->rtsx_dev, "B\n");
#if 0 /* might have been SD int */
		device_printf(sc->rtsx_dev, "Spurious interrupt - no active request\n");
#endif
		RTSX_UNLOCK(sc);
		return;
	}
	if (status & (RTSX_TRANS_OK_INT | RTSX_TRANS_FAIL_INT)) {
		sc->rtsx_intr_status |= status;
		wakeup(&sc->rtsx_intr_status);
	} else
		device_printf(sc->rtsx_dev, "NODMA\n");

	RTSX_UNLOCK(sc);


	/*--- See taskqueue_enqueue_timeout()
	
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

}

static int
rtsx_wait_intr(struct rtsx_softc *sc, int mask, int timeout)
{
	int status;
	int error = 0;

	mask |= RTSX_TRANS_FAIL_INT;

	status = sc->rtsx_intr_status & mask;
	while (status == 0) {
		if (msleep(&sc->rtsx_intr_status, &sc->rtsx_mtx, 0, "rtsxintr", timeout)
		    == EWOULDBLOCK) {
			device_printf(sc->rtsx_dev, "Controller timeout\n");
			error = MMC_ERR_TIMEOUT;
			break;
		}
		status = sc->rtsx_intr_status & mask;
	}
	sc->rtsx_intr_status &= ~status;

	/* Has the card disappeared? */
	if (!ISSET(sc->rtsx_flags, RTSX_F_CARD_PRESENT))
		error = MMC_ERR_NO_MEMORY;

	if (error == 0 && (status & RTSX_TRANS_FAIL_INT))
		error = MMC_ERR_FAILED;

	return (error);
}

static int
rtsx_init(struct rtsx_softc *sc)
{
	uint32_t status;
	uint8_t version;
	int error;

	sc->rtsx_host.host_ocr = RTSX_SUPPORTED_VOLTAGE;
	sc->rtsx_host.caps = MMC_CAP_4_BIT_DATA;

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
	} else if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_READ(sc, RTSX_RTL8411B_PACKAGE, &version);
		if (version & RTSX_RTL8411B_QFN48)
			sc->rtsx_flags |= RTSX_F_8411B_QFN48;
	}

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_init() rtsx_flags = 0x%04x\n", sc->rtsx_flags);

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
	else if (sc->rtsx_flags & (RTSX_F_5227 | RTSX_F_5229))
		error = rtsx_write_phy(sc, 0x00, 0xBA42);
	else
		error = 0;
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
	} else if (sc->rtsx_flags & RTSX_F_8411B) {
		if (sc->rtsx_flags & RTSX_F_8411B_QFN48)
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xf5);
		/* Enable SD interrupt */
		RTSX_WRITE(sc, RTSX_CARD_PAD_CTL, 0x05);
		RTSX_BITOP(sc, RTSX_EFUSE_CONTENT, 0xe0, 0x80);
		RTSX_WRITE(sc, RTSX_FUNC_FORCE_CTL, 0x00);
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

#if 0 /* done in task */
static void
rtsx_start(struct rtsx_softc *sc)
{
	if (rtsx_is_card_present(sc))
		rtsx_card_insert(sc);
	else
		rtsx_card_remove(sc);
}
#endif

static int
rtsx_stop(struct rtsx_softc *sc)
{
	int ret = 0;

	/* automatically deletes any children */
	ret = device_delete_children(sc->rtsx_dev);
	sc->rtsx_mmc_dev = NULL;

	return (ret);
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

/*
 * Set or change SDCLK frequency or disable the SD clock.
 * Return zero on success.
 */
static int
rtsx_set_sd_clock(struct rtsx_softc *sc, uint32_t freq)
{
	uint8_t n;
	int div;
	int mcu;
	int error = 0;

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_set_sd_clock(%u)\n", freq);

	if (freq == RTSX_SDCLK_OFF) {
		error = rtsx_stop_sd_clock(sc);
		goto done;
	}

	/* Round down to a supported frequency. */
	if (freq >= RTSX_SDCLK_50MHZ)
		freq = RTSX_SDCLK_50MHZ;
	else if (freq >= RTSX_SDCLK_25MHZ)
		freq = RTSX_SDCLK_25MHZ;
	else
		freq = RTSX_SDCLK_400KHZ;

	/*
	 * Configure the clock frequency.
	 */
	switch (freq) {
	case RTSX_SDCLK_400KHZ:
		n = 80; /* minimum */
		div = RTSX_CLK_DIV_8;
		mcu = 7;
		RTSX_SET(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_128);
		break;
	case RTSX_SDCLK_25MHZ:
		n = 100;
		div = RTSX_CLK_DIV_4;
		mcu = 7;
		RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
		break;
	case RTSX_SDCLK_50MHZ:
		n = 100;
		div = RTSX_CLK_DIV_2;
		mcu = 7;
		RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
		break;
	default:
		error = EINVAL;
		goto done;
	}

	/*
	 * Enable SD clock.
	 */
	error = rtsx_switch_sd_clock(sc, n, div, mcu);
done:
	return error;
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

	DELAY(200);

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

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_bus_power_off()\n");
	
	if ((error = rtsx_stop_sd_clock(sc)))
		return (error);

	/* Disable SD output. */
	RTSX_CLR(sc, RTSX_CARD_OE, RTSX_CARD_OUTPUT_EN);

	/* Turn off power. */
	disable3 = RTSX_PULL_CTL_DISABLE3;
	if (sc->rtsx_flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_OFF);
		RTSX_BITOP(sc, RTSX_LDO_CTL, RTSX_BPP_LDO_POWB,
			   RTSX_BPP_LDO_SUSPEND);
  	} else {
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1 | RTSX_LDO3318_VCC2);
		if (sc->rtsx_flags & RTSX_F_5229_TYPE_C)
			disable3 = RTSX_PULL_CTL_DISABLE3_TYPE_C;
	}

	RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_PMOS_STRG_800mA);

	/* Disable pull control. */
	if (sc->rtsx_flags & RTSX_F_8411B) {
		if (sc->rtsx_flags & RTSX_F_8411B_QFN48) {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0x55);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xf5);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x15);
		} else {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, 0x65);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0x55);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xd9);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL4, 0x59);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL5, 0x55);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x15);
		}
	} else {
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_DISABLE12);
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_DISABLE12);
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, disable3);
	}

	return (0);
}

static int
rtsx_bus_power_up(struct rtsx_softc *sc)
{
	uint8_t enable3;
	int error;

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_bus_power_up()\n");
	
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
	if (sc->rtsx_flags & RTSX_F_8411B) {
		if (sc->rtsx_flags & RTSX_F_8411B_QFN48) {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xf9);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x19);
		} else {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xd9);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL4, 0x59);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL5, 0x59);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x15);
		}
	} else {
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
		if (sc->rtsx_flags & RTSX_F_5229_TYPE_C)
			enable3 = RTSX_PULL_CTL_ENABLE3_TYPE_C;
		else
			enable3 = RTSX_PULL_CTL_ENABLE3;
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, enable3);
	}

	/*
	 * To avoid a current peak, enable card power in two phases with a
	 * delay in between.
	 */

	/* Partial power. */
	if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_5_PERCENT_ON);
		RTSX_BITOP(sc, RTSX_LDO_CTL, RTSX_BPP_LDO_POWB,
			   RTSX_BPP_LDO_SUSPEND);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_10_PERCENT_ON);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_15_PERCENT_ON);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_ON);
		RTSX_BITOP(sc, RTSX_LDO_CTL, RTSX_BPP_LDO_POWB,
			   RTSX_BPP_LDO_ON);
	} else {
		/* Partial power. */
		RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PARTIAL_PWR_ON);
		if (sc->rtsx_flags & RTSX_F_5209)
			RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
		else
			RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);
	}
	
	return (0);
}

static int
rtsx_bus_power_on(struct rtsx_softc *sc)
{

	uint8_t enable3;
	int error;

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_bus_power_on()\n");
	
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
	if (sc->rtsx_flags & RTSX_F_8411B) {
		if (sc->rtsx_flags & RTSX_F_8411B_QFN48) {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xf9);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x19);
		} else {
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, 0xaa);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, 0xd9);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL4, 0x59);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL5, 0x59);
			RTSX_WRITE(sc, RTSX_CARD_PULL_CTL6, 0x15);
		}
	} else {
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
		if (sc->rtsx_flags & RTSX_F_5229_TYPE_C)
			enable3 = RTSX_PULL_CTL_ENABLE3_TYPE_C;
		else
			enable3 = RTSX_PULL_CTL_ENABLE3;
		RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, enable3);
	}

	/*
	 * To avoid a current peak, enable card power in two phases with a
	 * delay in between.
	 */
	if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_5_PERCENT_ON);
		RTSX_BITOP(sc, RTSX_LDO_CTL, RTSX_BPP_LDO_POWB,
			   RTSX_BPP_LDO_SUSPEND);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_10_PERCENT_ON);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_15_PERCENT_ON);
		DELAY(150);
		RTSX_BITOP(sc, RTSX_CARD_PWR_CTL, RTSX_BPP_POWER_MASK,
			   RTSX_BPP_POWER_ON);
		RTSX_BITOP(sc, RTSX_LDO_CTL, RTSX_BPP_LDO_POWB,
			   RTSX_BPP_LDO_ON);
	} else {
		/* Partial power. */
		RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PARTIAL_PWR_ON);
		if (sc->rtsx_flags & RTSX_F_5209)
			RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
		else
			RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);
		
		DELAY(200);
	
		/* Full power. */
		RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
		if (sc->rtsx_flags & RTSX_F_5209)
			RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
		else
			RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);
	}

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

#if 0 /* done in the task */
static void
rtsx_card_insert(struct rtsx_softc *sc)
{
	device_t mmc_dev;
	int error;

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
			if ((error = device_probe_and_attach(sc->rtsx_mmc_dev)))
				device_printf(sc->rtsx_dev, "Attaching MMC bus failed [%d]\n", error);
	}
	else {
		RTSX_UNLOCK(sc);
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
#endif

static int
rtsx_led_enable(struct rtsx_softc *sc)
{
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
		RTSX_WRITE(sc, RTSX_CARD_AUTO_BLINK,
			   RTSX_LED_BLINK_EN | RTSX_LED_BLINK_SPEED);
	} else if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_CLR(sc, RTSX_GPIO_CTL, 0x01);
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
	} else if (sc->rtsx_flags & RTSX_F_8411B) {
		RTSX_CLR(sc, RTSX_CARD_AUTO_BLINK, RTSX_LED_BLINK_EN);
		RTSX_SET(sc, RTSX_GPIO_CTL, 0x01);
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

/* init command buffer with SD command index and argument. */
static void
rtsx_init_cmd(struct rtsx_softc *sc, struct mmc_command *cmd)
{
	sc->rtsx_cmd_index = 0;
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CMD0,
		      0xff, RTSX_SD_CMD_START  | cmd->opcode); 
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CMD1,
		     0xff, cmd->arg >> 24);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CMD2,
		      0xff, cmd->arg >> 16);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CMD3,
		     0xff, cmd->arg >> 8);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CMD4,
		     0xff, cmd->arg);
}

/* Append a properly encoded host command to the host command buffer. */
static void
rtsx_push_cmd(struct rtsx_softc *sc, uint8_t cmd, uint16_t reg,
	      uint8_t mask, uint8_t data)
{
	KASSERT(sc->rtsx_cmd_index < RTSX_HOSTCMD_MAX,
		("rtsx: too many host commands (%d)\n", sc->rtsx_cmd_index));

	uint32_t *cmd_buffer = (uint32_t *)(sc->rtsx_cmd_dmamem);
	cmd_buffer[sc->rtsx_cmd_index++] =
		htole32((uint32_t)(cmd & 0x3) << 30) |
		((uint32_t)(reg & 0x3fff) << 16) |
		((uint32_t)(mask) << 8) |
		((uint32_t)data);

	/*---
	if (bootverbose && sc->rtsx_cmd_index < 33)
		device_printf(sc->rtsx_dev, "cmd_buffer[%02d] = 0x%08x\n",
			      sc->rtsx_cmd_index - 1,
			      cmd_buffer[sc->rtsx_cmd_index - 1]);
	---*/

}

/* Run the command queue and wait for completion. */
static int
rtsx_send_cmd(struct rtsx_softc *sc, struct mmc_command *cmd)
{
	int error = 0;

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_send_cmd()\n");
		
	sc->rtsx_intr_status = 0;

	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREWRITE);

	/* Tell the chip where the command buffer is and run the commands. */
	WRITE4(sc, RTSX_HCBAR, (uint32_t)sc->rtsx_cmd_buffer);
	WRITE4(sc, RTSX_HCBCTLR,
	       ((sc->rtsx_cmd_index * 4) & 0x00ffffff) | RTSX_START_CMD | RTSX_HW_AUTO_RSP);

	if ((error = rtsx_wait_intr(sc, RTSX_TRANS_OK_INT, hz * sc->rtsx_timeout)))
		cmd->error = MMC_ERR_TIMEOUT;

	return (error);
}

/* Run the command queue and don't wait for completion. */
static void
rtsx_send_cmd_nowait(struct rtsx_softc *sc, struct mmc_command *cmd)
{

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_send_cmd_nowait()\n");
		
	sc->rtsx_intr_status = 0;
	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_PREWRITE);

	/* Tell the chip where the command buffer is and run the commands. */
	WRITE4(sc, RTSX_HCBAR, (uint32_t)sc->rtsx_cmd_buffer);
	WRITE4(sc, RTSX_HCBCTLR,
	       ((sc->rtsx_cmd_index * 4) & 0x00ffffff) | RTSX_START_CMD | RTSX_HW_AUTO_RSP);
}

static void
rtsx_req_done(struct rtsx_softc *sc)
{
	struct mmc_request *req;

	req = sc->rtsx_req;
	if (req->cmd->error != MMC_ERR_NONE)
		rtsx_soft_reset(sc);
	sc->rtsx_req = NULL;
	req->done(req);
}

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
rtsx_send_req_get_resp(struct rtsx_softc *sc, struct mmc_command *cmd) {
	uint8_t rsp_type;
	uint16_t reg;
	int error = 0;

	/* Convert response type */
	rsp_type = rtsx_response_type(cmd->flags & MMC_RSP_MASK);
	if (rsp_type == 0) {
		device_printf(sc->rtsx_dev, "Unknown response type 0x%lx\n", (cmd->flags & MMC_RSP_MASK));
		cmd->error = MMC_ERR_FAILED;
		return (EINVAL);
	}

	rtsx_init_cmd(sc, cmd);

	/* Queue command to set response type. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
		      0xff, rsp_type);

	/* Use the ping-pong buffer (cmd buffer) for commands which do not transfer data. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
		      0x01, RTSX_PINGPONG_BUFFER);

	/* Queue commands to perform SD transfer. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
		      0xff, RTSX_TM_CMD_RSP | RTSX_SD_TRANSFER_START);
	rtsx_push_cmd(sc, RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
		      RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE,
		      RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE);

	/* If needed queue commands to read back card status response.*/
	if (rsp_type == RTSX_SD_RSP_TYPE_R2) {
		/* Read data from ping-pong buffer */
		for (reg = RTSX_PPBUF_BASE2; reg < RTSX_PPBUF_BASE2 + 16; reg++)
			rtsx_push_cmd(sc, RTSX_READ_REG_CMD, reg, 0, 0);
	} else if (rsp_type != RTSX_SD_RSP_TYPE_R0) {
		/* Read data from SD_CMDx registers */
		for (reg = RTSX_SD_CMD0; reg <= RTSX_SD_CMD4; reg++)
			rtsx_push_cmd(sc, RTSX_READ_REG_CMD, reg, 0, 0);
	}
	rtsx_push_cmd(sc, RTSX_READ_REG_CMD, RTSX_SD_STAT1, 0, 0);

	/* Run the command queue and wait for completion. */
	if ((error = rtsx_send_cmd(sc, cmd)))
		return (error);

	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTREAD);
	bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTWRITE);
	
	/* Copy card response into mmc response buffer. */
	if (ISSET(cmd->flags, MMC_RSP_PRESENT)) {
		uint32_t *cmd_buffer = (uint32_t *)(sc->rtsx_cmd_dmamem);

		if (bootverbose) {
			device_printf(sc->rtsx_dev, "cmd_buffer: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
				      cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3], cmd_buffer[4]);
		}

		if (rsp_type == RTSX_SD_RSP_TYPE_R2) {
			/* First byte is CHECK_REG_CMD return value, skip it. */
			unsigned char *ptr = (unsigned char *)cmd_buffer + 1;
			int i;
			
			/*
			 * The controller offloads the last byte {CRC-7, end bit 1}
			 * of response type R2. Assign dummy CRC, 0, and end bit to this
			 * byte (ptr[16], goes into the LSB of resp[3] later).
			 */
			ptr[16] = 0x01;
			/* The second byte is the status of response, skip it */
			for (i = 0; i < 4; i++)
				cmd->resp[i] = be32dec(ptr + 1 + i * 4);
		} else {
			/*
			 * First byte is CHECK_REG_CMD return value, second
			 * one is the command op code -- we skip those.
			 */
			cmd->resp[0] =
				((be32toh(cmd_buffer[0]) & 0x0000ffff) << 16) |
				((be32toh(cmd_buffer[1]) & 0xffff0000) >> 16);
		}
		
		if (bootverbose)
			device_printf(sc->rtsx_dev, "cmd->resp = 0x%08x 0x%08x 0x%08x 0x%08x\n",
				      cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		
	}
	return (error);
}

static int
rtsx_xfer_short(struct rtsx_softc *sc, struct mmc_command *cmd)
{
	uint8_t rsp_type;
	int read;
	int tmode;
	int error = 0;

	if (cmd->data->xfer_len == 0)
		cmd->data->xfer_len = (cmd->data->len > RTSX_MAX_DATA_BLKLEN) ?
			RTSX_MAX_DATA_BLKLEN : cmd->data->len;
				
	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_xfer_short(): %ld bytes with block size %ld\n",
			      cmd->data->len, cmd->data->xfer_len);

	if (cmd->data->len > 512) {
		device_printf(sc->rtsx_dev, "rtsx_xfer_short() length too large: %ld > 512\n",
			      cmd->data->len);
		cmd->error = MMC_ERR_FAILED;
		return ENOMEM;
	}

	rsp_type = rtsx_response_type(cmd->flags & MMC_RSP_MASK);
	if (rsp_type == 0) {
		device_printf(sc->rtsx_dev, "Unknown response type 0x%lx\n", (cmd->flags & MMC_RSP_MASK));
		cmd->error = MMC_ERR_FAILED;
		return (EINVAL);
	}
	
	read = ISSET(cmd->data->flags, MMC_DATA_READ);
	if (!read && cmd->data != NULL && cmd->data->len > 0) {
		error = rtsx_write_ppbuf(sc, cmd);
		if (error)
			return (error);
	}
	rtsx_init_cmd(sc, cmd);

	/* Queue commands to configure data transfer size. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_L,
		      0xff, (cmd->data->xfer_len & 0xff));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_H,
		      0xff, (cmd->data->xfer_len >> 8));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_L,
		      0xff, ((cmd->data->len / cmd->data->xfer_len) & 0xff));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_H,
		      0xff, ((cmd->data->len / cmd->data->xfer_len) >> 8));

	/* Queue command to set response type. */
	/*--- from netbsd ---
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
		      0xff, rsp_type);
	---*/

	/* from linux: rtsx_pci_sdmmc.c sd_read_data() */
	rtsx_push_cmd(sc,  RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
		      0xff, RTSX_SD_CALCULATE_CRC7 | RTSX_SD_CHECK_CRC16 |
		      RTSX_SD_NO_WAIT_BUSY_END | RTSX_SD_CHECK_CRC7 | RTSX_SD_RSP_LEN_6);
	
	/* Use the ping-pong buffer (cmd buffer). */
	if (read)
		rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
			      0x01, RTSX_PINGPONG_BUFFER);

	/* Queue commands to perform SD transfer. */
	tmode = read ? RTSX_TM_NORMAL_READ : RTSX_TM_AUTO_WRITE2;
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
		      0xff, tmode | RTSX_SD_TRANSFER_START);
	rtsx_push_cmd(sc, RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
		      RTSX_SD_TRANSFER_END, RTSX_SD_TRANSFER_END);

	/* Run the command queue and wait for completion. */
	if ((error = rtsx_send_cmd(sc, cmd)))
		return (error);

	if (read && cmd->data != NULL && cmd->data->len > 0) {
		error = rtsx_read_ppbuf(sc, cmd);
	}

	if (bootverbose) {
		uint32_t *data = (uint32_t *)cmd->data->data;
		int i;
		int n = (cmd->data->len > 64) ? 8 : ((cmd->data->len - 1) / 4) + 1;
		device_printf(sc->rtsx_dev, "rtsx_xfer_short():\n");
		for (i = 0; i < n; i++)
			device_printf(sc->rtsx_dev, "\t\t\t0x%08x\n", data[i]);
	}

	return (error);
}

/* Use ping-pong buffer (cmd buffer) for transfer */
static int
rtsx_read_ppbuf(struct rtsx_softc *sc, struct mmc_command *cmd)
{

	uint32_t *cmd_buffer = (uint32_t *)(sc->rtsx_cmd_dmamem);
	
	uint16_t reg = RTSX_PPBUF_BASE2;
	uint8_t *ptr = cmd->data->data;
	int remain = cmd->data->len;
	int i, j;
	int error;
		
	for (j = 0; j < cmd->data->len / RTSX_HOSTCMD_MAX; j++) {
		sc->rtsx_cmd_index = 0;
		for (i = 0; i < RTSX_HOSTCMD_MAX; i++) {
			rtsx_push_cmd(sc, RTSX_READ_REG_CMD, reg++,
				      0, 0);
		}
		if ((error = rtsx_send_cmd(sc, cmd)))
		    return (error);

		/* Sync command DMA buffer. */
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTREAD);
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTWRITE);

		memcpy(ptr, sc->rtsx_cmd_dmamem, RTSX_HOSTCMD_MAX);
		ptr += RTSX_HOSTCMD_MAX;
		remain -= RTSX_HOSTCMD_MAX;

		if (bootverbose) {
			int i;
			for (i = 48; i < 52; i++) { 
				device_printf(sc->rtsx_dev, "cmd_buffer[%d]: 0x%08x\n",
					      i, cmd_buffer[i]);
			}
		}
			
	}
	if (remain > 0) {
		sc->rtsx_cmd_index = 0;
		for (i = 0; i < remain; i++) {
			rtsx_push_cmd(sc, RTSX_READ_REG_CMD, reg++,
				      0, 0);
		}
		if ((error = rtsx_send_cmd(sc, cmd)))
			return (error);

		/* Sync command DMA buffer. */
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTREAD);
		bus_dmamap_sync(sc->rtsx_cmd_dma_tag, sc->rtsx_cmd_dmamap, BUS_DMASYNC_POSTWRITE);

		memcpy(ptr, sc->rtsx_cmd_dmamem, remain);
    }
    return (0);
}

/* Use ping-pong buffer (cmd buffer) for transfer */
static int
rtsx_write_ppbuf(struct rtsx_softc *sc, struct mmc_command *cmd)
{
	return (0);
}

static int
rtsx_xfer(struct rtsx_softc *sc, struct mmc_command *cmd)
{
	uint8_t cfg2;
	int read = ISSET(cmd->data->flags, MMC_DATA_READ);
	int dma_dir;
	int tmode;
	int error = 0;

	if (cmd->data->xfer_len == 0)
		cmd->data->xfer_len = (cmd->data->len > RTSX_MAX_DATA_BLKLEN) ?
			RTSX_MAX_DATA_BLKLEN : cmd->data->len;
				
	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_xfer() - %s xfer: %ld bytes with block size %ld\n",
			      read ? "Read" : "Write",
			      cmd->data->len, cmd->data->xfer_len);

	if (cmd->data->len > RTSX_DMA_DATA_BUFSIZE) {
		device_printf(sc->rtsx_dev, "rtsx_xfer() length too large: %ld > %d\n",
			      cmd->data->len, RTSX_DMA_DATA_BUFSIZE);
		cmd->error = MMC_ERR_FAILED;
		return ENOMEM;
	}

	/* Configure DMA transfer mode parameters. */
	cfg2 = RTSX_SD_NO_CHECK_WAIT_CRC_TO | RTSX_SD_CHECK_CRC16 |
		RTSX_SD_NO_WAIT_BUSY_END | RTSX_SD_RSP_LEN_0;
	if (read) {
		dma_dir = RTSX_DMA_DIR_FROM_CARD;
		/*
		 * Use transfer mode AUTO_READ3, which assumes we've already
		 * sent the read command and gotten the response, and will
		 * send CMD 12 manually after reading multiple blocks.
		 */
     		tmode = RTSX_TM_AUTO_READ3;
		cfg2 |= RTSX_SD_CALCULATE_CRC7 | RTSX_SD_CHECK_CRC7;
	} else {
		dma_dir = RTSX_DMA_DIR_TO_CARD;
		/*
		 * Use transfer mode AUTO_WRITE3, which assumes we've already
		 * sent the write command and gotten the response, and will
		 * send CMD 12 manually after writing multiple blocks.
		 */
		tmode = RTSX_TM_AUTO_WRITE3;
		cfg2 |= RTSX_SD_NO_CALCULATE_CRC7 | RTSX_SD_NO_CHECK_CRC7;
	}

	sc->rtsx_cmd_index = 0;

	/* Queue command to set response type. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
		     0xff, cfg2); 

	/* Queue commands to configure data transfer size. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_L,
		      0xff, (cmd->data->xfer_len & 0xff));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_H,
		      0xff, (cmd->data->xfer_len >> 8));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_L,
		      0xff, ((cmd->data->len / cmd->data->xfer_len) & 0xff));
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_H,
		      0xff, ((cmd->data->len / cmd->data->xfer_len) >> 8));

	/* Use the DMA ring buffer for commands which transfer data. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
		      0x01, RTSX_RING_BUFFER);

	/* Configure DMA controller. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_IRQSTAT0,
		     RTSX_DMA_DONE_INT, RTSX_DMA_DONE_INT);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_DMATC3,
		     0xff, cmd->data->len >> 24);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_DMATC2,
		     0xff, cmd->data->len >> 16);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_DMATC1,
		     0xff, cmd->data->len >> 8);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_DMATC0,
		     0xff, cmd->data->len);
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_DMACTL,
		     RTSX_DMA_EN | RTSX_DMA_DIR | RTSX_DMA_PACK_SIZE_MASK,
		     RTSX_DMA_EN | dma_dir | RTSX_DMA_512);

	/* Queue commands to perform SD transfer. */
	rtsx_push_cmd(sc, RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
		      0xff, tmode | RTSX_SD_TRANSFER_START);
	rtsx_push_cmd(sc, RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
		      RTSX_SD_TRANSFER_END, RTSX_SD_TRANSFER_END);

	/* Run the command queue and don't wait for completion. */
	rtsx_send_cmd_nowait(sc, cmd);

	sc->rtsx_intr_status = 0;
	
	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_data_dma_tag, sc->rtsx_data_dmamap, BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->rtsx_data_dma_tag, sc->rtsx_data_dmamap, BUS_DMASYNC_PREWRITE);

	/* Tell the chip where the data buffer is and run the transfer. */
	WRITE4(sc, RTSX_HDBAR, sc->rtsx_data_buffer);
	WRITE4(sc, RTSX_HDBCTLR, RTSX_TRIG_DMA | (read ? RTSX_DMA_READ : 0) |
	       (cmd->data->len & 0x00ffffff));

	if ((error = rtsx_wait_intr(sc, RTSX_TRANS_OK_INT, hz * sc->rtsx_timeout))) {
		cmd->error = MMC_ERR_TIMEOUT;
		return (error);
	}

	/* Sync command DMA buffer. */
	bus_dmamap_sync(sc->rtsx_data_dma_tag, sc->rtsx_data_dmamap, BUS_DMASYNC_POSTREAD);
	bus_dmamap_sync(sc->rtsx_data_dma_tag, sc->rtsx_data_dmamap, BUS_DMASYNC_POSTWRITE);

	memcpy(cmd->data->data, sc->rtsx_data_dmamem, cmd->data->len);
	
	return (error);
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
	default:
		return (EINVAL);
	}

	if (bootverbose)
		device_printf(bus, "Read ivar #%d, value %#x / #%d\n",
			      which, *(int *)result, *(int *)result);

	return (0);
}

static int
rtsx_write_ivar(device_t bus, device_t child, int which, uintptr_t value)
{
	struct rtsx_softc *sc;

	if (bootverbose)
		device_printf(bus, "Write ivar #%d, value %#x / #%d\n",
			      which, (int)value, (int)value);

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
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_CAPS:
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

	if (bootverbose)
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
	default:
		return (EINVAL);
	}
	if ((error = rtsx_write(sc, RTSX_SD_CFG1, RTSX_BUS_WIDTH_MASK, bus_width)))
		return (error);
	
	/* Set sd clock */
	if (sc->rtsx_sd_clock != ios->clock) {
		sc->rtsx_sd_clock = ios->clock;
		if ((error = rtsx_set_sd_clock(sc, ios->clock)))
			return (error);
	}
	
	/* Set power mode */
	switch (ios->power_mode) {
	case power_off:
		if (sc->rtsx_power_mode != power_off) {
			rtsx_bus_power_off(sc);
			sc->rtsx_power_mode = power_off;
		}
		break;
	case power_up:
		if (sc->rtsx_power_mode != power_up) {
			rtsx_bus_power_up(sc);
			sc->rtsx_power_mode = power_up;
		}
		break;
	case power_on:
		if (sc->rtsx_power_mode != power_on) {
			rtsx_bus_power_on(sc);
			sc->rtsx_power_mode = power_on;
		}
		break;
	};

	return (0);
}

/* Set output stage logic power voltage */
static int
rtsx_mmcbr_switch_vccq(device_t bus, device_t child __unused)
{
	struct rtsx_softc *sc;
	int vccq = 0;
	
	sc = device_get_softc(bus);

	switch (sc->rtsx_host.ios.vccq) {
	case vccq_120:
		vccq = 120;
		break;
	case vccq_180:
		vccq = 180;
		break;
	case vccq_330:
		vccq = 330;
		break;
	};

	if ((sc->rtsx_flags & RTSX_F_8411B) && vccq == 330) {
		(void)rtsx_write(sc, RTSX_SD30_DRIVE_SEL, RTSX_SD30_DRIVE_SEL_MASK, RTSX_SD30_DRIVE_SEL_3V3);
		(void)rtsx_write(sc, RTSX_LDO_CTL,
				 (RTSX_BPP_ASIC_MASK << RTSX_BPP_SHIFT_8411) | RTSX_BPP_PAD_MASK,
				 (RTSX_BPP_ASIC_3V3 << RTSX_BPP_SHIFT_8411) | RTSX_BPP_PAD_3V3);
		DELAY(200);
	}

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_mmcbr_switch_vccq(%d)\n", vccq);

	rtsx_bus_power_off(sc);

	DELAY(200);
	
	rtsx_bus_power_on(sc);
	sc->rtsx_power_mode = power_on;
		
	return (0);

}

static int
rtsx_mmcbr_tune(device_t bus, device_t child __unused, bool hs400 __unused)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_mmcbr_tune()\n");

	return (0);

}

static int
rtsx_mmcbr_retune(device_t bus, device_t child __unused, bool reset __unused)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_mmcbr_retune()\n");

	return (0);

}

static int
rtsx_mmcbr_request(device_t bus, device_t child __unused, struct mmc_request *req)
{
	struct rtsx_softc *sc;
	struct mmc_command *cmd;
	int error = 0;
	
	sc = device_get_softc(bus);

	RTSX_LOCK(sc);
	if (sc->rtsx_req != NULL) {
		RTSX_UNLOCK(sc);
                return (EBUSY);
        }
	sc->rtsx_req = req;
	sc->rtsx_intr_status = 0;
	cmd = req->cmd;
	cmd->error = MMC_ERR_NONE;

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_mmcbr_request(CMD%u arg %#x flags %#x dlen %u dflags %#x)\n",
			      cmd->opcode, cmd->arg, cmd->flags,
			      cmd->data != NULL ? (unsigned int)cmd->data->len : 0,
			      cmd->data != NULL ? cmd->data->flags : 0);

	/* Refuse SDIO probe if the chip doesn't support SDIO. */
	if (cmd->opcode == IO_SEND_OP_COND &&
	    !ISSET(sc->rtsx_flags, RTSX_F_SDIO_SUPPORT)) {
		cmd->error = MMC_ERR_FAILED;
		error = ENOTSUP;
		goto done;
	}

	if (cmd->data == NULL) {
		error = rtsx_send_req_get_resp(sc, cmd);
	} else if (cmd->data->len <= 512) {
		error = rtsx_xfer_short(sc, cmd);
		if (error) {
			uint8_t stat1;
			if (rtsx_read(sc, RTSX_SD_STAT1, &stat1) == 0 &&
			    (stat1 & RTSX_SD_CRC_ERR)) {
				device_printf(sc->rtsx_dev, "CRC error\n");
				cmd->error = MMC_ERR_BADCRC;
			}
		}
	} else {
		error = rtsx_send_req_get_resp(sc, cmd);
		if (!error) {
			error = rtsx_xfer(sc, cmd);
			if (error) {
				uint8_t stat1;
				if (rtsx_read(sc, RTSX_SD_STAT1, &stat1) == 0 &&
				    (stat1 & RTSX_SD_CRC_ERR)) {
					device_printf(sc->rtsx_dev, "CRC error\n");
					cmd->error = MMC_ERR_BADCRC;
				}
			}
		}
	}

 done:
	rtsx_req_done(sc);
	RTSX_UNLOCK(sc);
	return (error);
}

static int
rtsx_mmcbr_get_ro(device_t bus, device_t child __unused)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);

	if (bootverbose)
		device_printf(sc->rtsx_dev, "rtsx_mmcbr_get_ro()\n");

	return (0);

}

static int
rtsx_mmcbr_acquire_host(device_t bus, device_t child __unused)
{
	struct rtsx_softc *sc;

	if (bootverbose)
		device_printf(bus, "rtsx_mmcbr_acquire_host()\n");

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

	if (bootverbose)
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
	
	if (bootverbose)
		device_printf(dev, "Attach - Vendor ID: 0x%x - Device ID: 0x%x\n",
			      pci_get_vendor(dev), pci_get_device(dev));

	sc->rtsx_dev = dev;
	RTSX_LOCK_INIT(sc);

	/* timeout parameter for rtsx_wait_intr() */
	sc->rtsx_timeout = 2;
	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "req_timeout", CTLFLAG_RW,
		       &sc->rtsx_timeout, 0, "Request timeout in seconds");
	sc->rtsx_host.f_min = RTSX_SDCLK_400KHZ;
	sc->rtsx_host.f_max = RTSX_SDCLK_400KHZ;

	
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
	
	if (bootverbose)
		device_printf(dev, "rtsx_irq_res_id: %d - rtsx_res_id: %d\n",
			      sc->rtsx_irq_res_id, sc->rtsx_res_id);

	sc->rtsx_btag = rman_get_bustag(sc->rtsx_res);
	sc->rtsx_bhandle = rman_get_bushandle(sc->rtsx_res);
	
	/* Activate the interrupt */
	error = bus_setup_intr(dev, sc->rtsx_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			       NULL, rtsx_intr, sc, &sc->rtsx_irq_cookie);
	if (error) {
		device_printf(dev, "Can't set up irq [0x%x]!\n", error);
		goto destroy_rtsx_res;
	}
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
#if 0 /* done in task */
	/* Start device */
	rtsx_start(sc);
#else
	/* from dwmmc.c */
	TASK_INIT(&sc->card_task, 0, rtsx_card_task, sc);
	/* really giant? */
	TIMEOUT_TASK_INIT(taskqueue_swi_giant, &sc->card_delayed_task, 0,
		rtsx_card_task, sc);

	/* 
	 * Schedule a card detection as we won't get an interrupt
	 * if the card is inserted when we attach
	 */
	rtsx_card_task(sc, 0);
#endif

	if (bootverbose)
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
	RTSX_LOCK_DESTROY(sc);
	return (ENXIO);
}

	
static int
rtsx_detach(device_t dev)
{
	struct rtsx_softc *sc = device_get_softc(dev);
	int err;

	if (bootverbose)
		device_printf(dev, "Detach - Vendor ID: 0x%x - Device ID: 0x%x\n",
			      pci_get_vendor(dev), pci_get_device(dev));
	
	/* Stop device */
	err = rtsx_stop(sc);
	if (err != 0)
		return err;

	taskqueue_drain(taskqueue_swi_giant, &sc->card_task);
	taskqueue_drain_timeout(taskqueue_swi_giant, &sc->card_delayed_task);

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
	RTSX_LOCK_DESTROY(sc);

	return (0);
}

static int
rtsx_shutdown(device_t dev)
{

	if (bootverbose)
		device_printf(dev, "Shutdown\n");
	
	return (0);
}

/*
 * Device suspend routine.
 */
static int
rtsx_suspend(device_t dev)
{

	if (bootverbose)
		device_printf(dev, "Suspend\n");
	
	return (0);
}

/*
 * Device resume routine.
 */
static int
rtsx_resume(device_t dev)
{

	if (bootverbose)
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
	DEVMETHOD(mmcbr_switch_vccq,	rtsx_mmcbr_switch_vccq),

	DEVMETHOD(mmcbr_tune,		rtsx_mmcbr_tune),
	DEVMETHOD(mmcbr_retune,		rtsx_mmcbr_retune),
	DEVMETHOD(mmcbr_request,	rtsx_mmcbr_request),
	DEVMETHOD(mmcbr_get_ro,		rtsx_mmcbr_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	rtsx_mmcbr_acquire_host),
	DEVMETHOD(mmcbr_release_host,	rtsx_mmcbr_release_host),

	DEVMETHOD_END
};

static devclass_t rtsx_devclass;

DEFINE_CLASS_0(rtsx, rtsx_driver, rtsx_methods, sizeof(struct rtsx_softc));
DRIVER_MODULE(rtsx, pci, rtsx_driver, rtsx_devclass, NULL, NULL);
MMC_DECLARE_BRIDGE(rtsx);
