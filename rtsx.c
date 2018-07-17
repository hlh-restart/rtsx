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

#include <dev/rtsx/rtsxreg.h>

#include <dev/mmc/bridge.h>

//#include <dev/sdhci/sdhci.h>

//#include "mmcbr_if.h"
//#include "sdhci_if.h"


#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))
#define SDMMC_MAXNSEGS	((MAXPHYS / PAGE_SIZE) + 1)

/* flag values */
#define	RTSX_F_CARD_PRESENT	0x01
#define	RTSX_F_SDIO_SUPPORT	0x02
#define	RTSX_F_5227		0x03
#define	RTSX_F_5209		0x04
#define	RTSX_F_5229		0x08
#define	RTSX_F_5229_TYPE_C	0x10
#define	RTSX_F_525A		0x20

#define	RTSX_DMA_MAX_SEGSIZE 0x80000
#define	RTSX_HOSTCMD_MAX	256
#define	RTSX_HOSTCMD_BUFSIZE	(sizeof(u_int32_t) * RTSX_HOSTCMD_MAX)
#define	RTSX_DMA_DATA_BUFSIZE	MAXPHYS
#define	RTSX_ADMA_DESC_SIZE	(sizeof(uint64_t) * SDMMC_MAXNSEGS)


#define ISSET(t, f) ((t) & (f))

#define READ4(sc, reg)								\
	(bus_space_read_4((sc)->iot, (sc)->ioh, (reg)))
#define WRITE4(sc, reg, val)						\
	bus_space_write_4((sc)->iot, (sc)->ioh, (reg), (val))

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


/* The softc holds our per-instance data. */
struct mypci_softc {
  //	struct 	sc_dev;
  device_t	my_dev;
  struct cdev	*my_cdev;
  //	device_t 	sc_dev;
  //	struct resource	*sdmmc;		/* generic SD/MMC device */
  bus_space_tag_t	iot;		/* host register set tag */
  bus_space_handle_t ioh;		/* host register set handle */
  bus_dma_tag_t	dmat;		/* DMA tag from attachment driver */
  bus_dmamap_t	dmap_cmd;	/* DMA map for command transfer */
  bus_dmamap_t	dmap_data;	/* DMA map for data transfer */
  bus_dmamap_t	dmap_adma;	/* DMA map for ADMA SG descriptors */
  void *		admabuf;	/* buffer for ADMA SG descriptors */
  bus_dma_segment_t adma_segs[1];	/* segments for ADMA SG buffer */
  int		flags;
  u_int32_t 	intr_status;	/* soft interrupt status */
  u_int8_t	regs[RTSX_NREG];/* host controller state */
  u_int32_t	regs4[6];	/* host controller state */
  int bar0id;
  struct resource * bar0res;
};
/*
struct mypci_softc {
  device_t	my_dev;
  struct cdev	*my_cdev;
};
*/
/* Function prototypes */
static d_open_t		mypci_open;
static d_close_t	mypci_close;
static d_read_t		mypci_read;
static d_write_t	mypci_write;

/* Character device entry points */

static struct cdevsw mypci_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	mypci_open,
	.d_close =	mypci_close,
	.d_read =	mypci_read,
	.d_write =	mypci_write,
	.d_name =	"rtsx",
};

/*
 * OpenBSD rtsx.c function declarations
 */
int	rtsx_bus_power_on(struct mypci_softc *);
int	rtsx_read(struct mypci_softc *, u_int16_t, u_int8_t *);
int	rtsx_write(struct mypci_softc *, u_int16_t, u_int8_t, u_int8_t);
int rtsx_write_phy(struct mypci_softc *sc, u_int8_t addr, u_int16_t val);
int rtsx_init(struct mypci_softc *sc, int attaching);
int rtsx_card_detect(struct mypci_softc *sch);
int rtsx_read_cfg(struct mypci_softc *sc, u_int8_t func, u_int16_t addr, u_int32_t *val);

/*
 * FreeBSD PCI probe, attach etc. fu 
 */


int
rtsx_card_detect(struct mypci_softc *sch)
{
	struct mypci_softc *sc = sch;
	return ISSET(sc->flags, RTSX_F_CARD_PRESENT);
}

int
rtsx_read(struct mypci_softc *sc, u_int16_t addr, u_int8_t *val)
{
	int tries = 1024;
	u_int32_t reg;
	
	WRITE4(sc, RTSX_HAIMR, RTSX_HAIMR_BUSY |
	    (u_int32_t)((addr & 0x3FFF) << 16));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY))
			break;
	}

	*val = (reg & 0xff);
	return (tries == 0) ? ETIMEDOUT : 0;
}

int
rtsx_read_cfg(struct mypci_softc *sc, u_int8_t func, u_int16_t addr, u_int32_t *val)
{
	int tries = 1024;
	u_int8_t data0, data1, data2, data3, rwctl;

	RTSX_WRITE(sc, RTSX_CFGADDR0, addr);
	RTSX_WRITE(sc, RTSX_CFGADDR1, addr >> 8);
	RTSX_WRITE(sc, RTSX_CFGRWCTL, RTSX_CFG_BUSY | (func & 0x03 << 4));

	while (tries--) {
		RTSX_READ(sc, RTSX_CFGRWCTL, &rwctl);
		if (!(rwctl & RTSX_CFG_BUSY))
			break;
	}

	if (tries == 0)
		return EIO;
	
	RTSX_READ(sc, RTSX_CFGDATA0, &data0);
	RTSX_READ(sc, RTSX_CFGDATA1, &data1);
	RTSX_READ(sc, RTSX_CFGDATA2, &data2);
	RTSX_READ(sc, RTSX_CFGDATA3, &data3);

	*val = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

	return 0;
}

int
rtsx_write(struct mypci_softc *sc, u_int16_t addr, u_int8_t mask, u_int8_t val)
{
	int tries = 1024;
	u_int32_t reg;
	WRITE4(sc, RTSX_HAIMR,
	    RTSX_HAIMR_BUSY | RTSX_HAIMR_WRITE |
	    (u_int32_t)(((addr & 0x3FFF) << 16) |
	    (mask << 8) | val));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY)) {
			if (val != (reg & 0xff))
				return EIO;
			return 0;
		}
	}

	return ETIMEDOUT;
}

int
rtsx_write_phy(struct mypci_softc *sc, u_int8_t addr, u_int16_t val)
{
	int timeout = 100000;
	u_int8_t rwctl;

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
		return ETIMEDOUT;
		
	return 0;
}

int
rtsx_bus_power_on(struct mypci_softc *sc)
{
	u_int8_t enable3;
	int err;

	if (sc->flags & RTSX_F_525A) {
		err = rtsx_write(sc, RTSX_LDO_VCC_CFG1, RTSX_LDO_VCC_TUNE_MASK,
		    RTSX_LDO_VCC_3V3);
		if (err) {
		  printf("error RTSX_F525A");
		  return (err);
		}
	}

	/* Select SD card. */
	RTSX_WRITE(sc, RTSX_CARD_SELECT, RTSX_SD_MOD_SEL);
	RTSX_WRITE(sc, RTSX_CARD_SHARE_MODE, RTSX_CARD_SHARE_48_SD);
	RTSX_SET(sc, RTSX_CARD_CLK_EN, RTSX_SD_CLK_EN);

	/* Enable pull control. */
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
	if (sc->flags & RTSX_F_5229_TYPE_C)
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
	if (sc->flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);

	DELAY(200);

	/* Full power. */
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	if (sc->flags & RTSX_F_5209)
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);

	/* Enable SD card output. */
	RTSX_WRITE(sc, RTSX_CARD_OE, RTSX_SD_OUTPUT_EN);

	return 0;
}


int
rtsx_init(struct mypci_softc *sc, int attaching)
{
	u_int32_t status;
	u_int8_t version;
	int error;
 	printf("rtsx_init()\n");

	/* Read IC version from dummy register. */
	if (sc->flags & RTSX_F_5229) {
		RTSX_READ(sc, RTSX_DUMMY_REG, &version);
		switch (version & 0x0F) {
		case RTSX_IC_VERSION_A:
		case RTSX_IC_VERSION_B:
		case RTSX_IC_VERSION_D:
			break;
		case RTSX_IC_VERSION_C:
			sc->flags |= RTSX_F_5229_TYPE_C;
			break;
		default:
			printf("rtsx_init: unknown ic %02x\n", version);
			return (1);
		}
	}

	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(sc, RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);

	printf("enable interrupt write-clear: jees\n");
	/* Clear any pending interrupts. */
	status = READ4(sc, RTSX_BIPR);
	WRITE4(sc, RTSX_BIPR, status);

	printf("clear any pending interrupts: jees\n");

	/* Check for cards already inserted at attach time. */
	if (attaching==1 && (status & RTSX_SD_EXIST)) {
	  sc->flags |= RTSX_F_CARD_PRESENT;
	  printf("card inserted\n");

	}
	/* Enable interrupts. */
	WRITE4(sc, RTSX_BIER,
	    RTSX_TRANS_OK_INT_EN | RTSX_TRANS_FAIL_INT_EN | RTSX_SD_INT_EN);

	/* Power on SSC clock. */
	RTSX_CLR(sc, RTSX_FPDCTL, RTSX_SSC_POWER_DOWN);
	DELAY(200);

	/* XXX magic numbers from linux driver */
	if (sc->flags & RTSX_F_5209)
		error = rtsx_write_phy(sc, 0x00, 0xB966);
	else
		error = rtsx_write_phy(sc, 0x00, 0xBA42);
	if (error) {
	  //printf("%s: cannot write phy register\n", DEVNAME(sc));
		printf("cannot write phy register\n");
		return (1);
	} else {
	  printf("can write phy register\n");
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
	if (sc->flags & RTSX_F_5209) {
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


/*
 * In the cdevsw routines, we find our softc by using the si_drv1 member
 * of struct cdev.  We set this variable to point to our softc in our
 * attach routine when we create the /dev entry.
 */

int
mypci_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct mypci_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->my_dev, "Opened successfully.\n");
	return (0);
}

int
mypci_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct mypci_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->my_dev, "Closed.\n");
	return (0);
}

int
mypci_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct mypci_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->my_dev, "Asked to read %d bytes.\n", (int) uio->uio_resid);
	return (0);
}

int
mypci_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct mypci_softc *sc;

	/* Look up our softc. */
	sc = dev->si_drv1;
	device_printf(sc->my_dev, "Asked to write %d bytes.\n", (int) uio->uio_resid);
	return (0);
}

/* PCI Support Functions */

/*
 * Compare the device ID of this device against the IDs that this driver
 * supports.  If there is a match, set the description and return success.
 */
static int
mypci_probe(device_t dev)
{
  /*none2@pci0:2:0:0:       class=0xff0000 card=0x221417aa chip=0x522710ec rev=0x01 hdr=0x00
    vendor     = 'Realtek Semiconductor Co., Ltd.'
    device     = 'RTS5227 PCI Express Card Reader'
    bar   [10] = type Memory, range 32, base 0xf0500000, size 4096, enabled

Vendor ID : 0x10ec
Device ID : 0x5227

  */
	device_printf(dev, "PCI probe\nVendor ID : 0x%x\nDevice ID : 0x%x\n",
	    pci_get_vendor(dev), pci_get_device(dev));

	if (pci_get_vendor(dev) == 0x10ec) {  // no device specific yet
	  //		printf("We've got the sdcard reader, probe successful!\n");
		device_set_desc(dev, "RealtekSD");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

/* Attach function is only called if the probe is successful. */

static int
mypci_attach(device_t dev)
{
	struct mypci_softc *sc;
	// int rsegs;
	 u_int32_t sdio_cfg;
	printf("attaching deviceID: 0x%x\n", pci_get_devid(dev));


	/* Look up our softc and initialize its fields. */
	sc = device_get_softc(dev);
	sc->my_dev = dev;
    sc->bar0id = PCIR_BAR(0);
    sc->bar0res = bus_alloc_resource(dev, SYS_RES_MEMORY, &sc->bar0id,
				  0, ~0, 1, RF_ACTIVE);
    if (sc->bar0res == NULL) {
        printf("Memory allocation of PCI base register 0 failed!\n");
        return 1;
    }


	sc->iot = rman_get_bustag(sc->bar0res);
	sc->ioh = rman_get_bushandle(sc->bar0res);
	
	sc->flags = RTSX_F_5227;
	if (rtsx_init(sc, 1)) {
	  printf("rekd");
	  return 1;
	}


	if (rtsx_read_cfg(sc, 0, RTSX_SDIOCFG_REG, &sdio_cfg) == 0) {
	  if ((sdio_cfg & RTSX_SDIOCFG_SDIO_ONLY) ||
		  (sdio_cfg & RTSX_SDIOCFG_HAVE_SDIO))
		sc->flags |= RTSX_F_SDIO_SUPPORT;
	}
	
	printf("RTSX_HOSTCMD_BUFSIZE %lu \n", RTSX_HOSTCMD_BUFSIZE);
	printf("RTSX_DMA_DATA_BUFSIZE %d \n", RTSX_DMA_DATA_BUFSIZE);
	printf("RTSX_ADMA_DESC_SIZE %lu \n", RTSX_ADMA_DESC_SIZE);

	/*	
	if (bus_dmamap_create(sc->dmat, RTSX_HOSTCMD_BUFSIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_cmd) != 0)
		return 1;
	if (bus_dmamap_create(sc->dmat, RTSX_DMA_DATA_BUFSIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_data) != 0)
	    	goto destroy_cmd;
	if (bus_dmamap_create(sc->dmat, RTSX_ADMA_DESC_SIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_adma) != 0)
	    	goto destroy_data;
	if (bus_dmamem_alloc(sc->dmat, RTSX_ADMA_DESC_SIZE, 0, 0,
	    sc->adma_segs, 1, &rsegs, BUS_DMA_WAITOK|BUS_DMA_ZERO))
	    	goto destroy_adma;
	if (bus_dmamem_map(sc->dmat, sc->adma_segs, rsegs, RTSX_ADMA_DESC_SIZE,
	    &sc->admabuf, BUS_DMA_WAITOK|BUS_DMA_COHERENT))
	    	goto free_adma;
*/
	/*
	 * Create a /dev entry for this device.  The kernel will assign us
	 * a major number automatically.  We use the unit number of this
	 * device as the minor number and name the character device
	 * "mypci<unit>".
	 */
	sc->my_cdev = make_dev(&mypci_cdevsw, device_get_unit(dev),
	    UID_ROOT, GID_WHEEL, 0600, "rtsx%u", device_get_unit(dev));
	sc->my_cdev->si_drv1 = sc;
	printf("rtsx device loaded.\n");

	//	printf("here goes nothing\n");
	//printf("nothing is done and card detected val is: %d\n", rtsx_card_detect(sc));
	
	return (0);
	/*
unmap_adma:
	bus_dmamem_unmap(sc->dmat, sc->admabuf, RTSX_ADMA_DESC_SIZE);
free_adma:
	bus_dmamem_free(sc->dmat, sc->adma_segs, rsegs);
destroy_adma:
	bus_dmamap_destroy(sc->dmat, sc->dmap_adma);
destroy_data:
	bus_dmamap_destroy(sc->dmat, sc->dmap_data);
destroy_cmd:
	bus_dmamap_destroy(sc->dmat, sc->dmap_cmd);
	return 1;
	*/
}

	
/* Detach device. */

static int
mypci_detach(device_t dev)
{
	struct mypci_softc *sc;

	/* Teardown the state in our softc created in our attach routine. */
	sc = device_get_softc(dev);
	destroy_dev(sc->my_cdev);
	printf("rtsx detach!\n");
	return (0);
}

/* Called during system shutdown after sync. */

static int
mypci_shutdown(device_t dev)
{

	printf("rtsx shutdown!\n");
	return (0);
}

/*
 * Device suspend routine.
 */
static int
mypci_suspend(device_t dev)
{

	printf("rtsx suspend!\n");
	return (0);
}

/*
 * Device resume routine.
 */
static int
mypci_resume(device_t dev)
{

	printf("rtsx resume!\n");
	return (0);
}

static device_method_t rtsx_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mypci_probe),
	DEVMETHOD(device_attach,	mypci_attach),
	DEVMETHOD(device_detach,	mypci_detach),
	DEVMETHOD(device_shutdown,	mypci_shutdown),
	DEVMETHOD(device_suspend,	mypci_suspend),
	DEVMETHOD(device_resume,	mypci_resume),

	/* Bus interface */
	//	DEVMETHOD(bus_read_ivar,	sdhci_generic_read_ivar),
	//	DEVMETHOD(bus_write_ivar,	sdhci_generic_write_ivar),

	
	/* MMC bridge interface */
	/*	DEVMETHOD(mmcbr_update_ios,	rtsx_update_ios),
	DEVMETHOD(mmcbr_switch_vccq, rtsx_switch_vccq),
	DEVMETHOD(mmcbr_tune, rtsx_tune),
	DEVMETHOD(mmcbr_retune, rtsx_retune),
	DEVMETHOD(mmcbr_request,	rtsx_request),
	DEVMETHOD(mmcbr_get_ro,		rtsx_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	rtsx_acquire_host),
	DEVMETHOD(mmcbr_release_host,	rtsx_release_host),

	DEVMETHOD(sdhci_read_4,		rtsx_read),
	DEVMETHOD(sdhci_write_4,	rtsx_write),
	*/
	
	DEVMETHOD_END
};


static driver_t rtsx_pci_driver = {
        "rtsx_pci",
        rtsx_pci_methods,
        sizeof(struct mypci_softc),
};
/*
static devclass_t sdhci_mypci_devclass;

DRIVER_MODULE(mypci_rtsx, pci, sdhci_mypci_driver, sdhci_mypci_devclass, NULL, NULL);
MODULE_DEPEND(mypci_rtsx, sdhci, 1, 1, 1);
MMC_DECLARE_BRIDGE(mypci_rtsx);
*/


static devclass_t rtsx_pci_devclass;

//DEFINE_CLASS_0(rtsx, rtsx_driver, rtsx_methods, sizeof(struct mypci_softc));  // old pre 5.1 compilant stuff
DRIVER_MODULE(rtsx_pci, pci, rtsx_pci_driver, rtsx_pci_devclass, 0, 0);

