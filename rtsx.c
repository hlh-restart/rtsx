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

#define DMA_BLOCK_SIZE		4096

#define	RTSX_DMA_MAX_SEGSIZE	0x80000
#define	RTSX_HOSTCMD_MAX	256
#define	RTSX_HOSTCMD_BUFSIZE	(sizeof(u_int32_t) * RTSX_HOSTCMD_MAX)
#define	RTSX_DMA_DATA_BUFSIZE	MAXPHYS
#define	RTSX_ADMA_DESC_SIZE	(sizeof(uint64_t) * SDMMC_MAXNSEGS)

int
rtsx_dma_alloc(struct rtsx_pci_softc *sc) {
	int	error = 0;

	/* Allocate DMA tag. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->rtsx_dev),	/* parent */
	    DMA_BLOCK_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    DMA_BLOCK_SIZE, 1,		/* maxsize, nsegments */
	    DMA_BLOCK_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rtsx_dma_tag);
	if (error) {
                device_printf(sc->rtsx_dev,
			      "couldn't create parent DMA tag\n");
		return error;
	}
	
	error = bus_dmamem_alloc(sc->rtsx_dma_tag,
				 &sc->rtsx_dmamem_cmd,
				 BUS_DMA_NOWAIT,
				 &sc->rtsx_dmamap_cmd);
	if (error) {
                device_printf(sc->rtsx_dev,
			      "couldn't create DMA map for command transfer\n");
		return error;
	}
	/*
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
	return error;
	/*---
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
	---*/
	return error;
}

void
rtsx_dma_free(struct rtsx_pci_softc *sc) {

	if (sc->rtsx_dma_tag != NULL)
		/*
		if (sc->rtsx_dmap_adma != NULL)
                        bus_dmamap_unload(sc->rl_cdata.rl_rx_tag,
                            sc->rl_cdata.rl_rx_dmamap);
                if (sc->rtsx_dmap_adma != NULL)
                        bus_dmamem_free(,
                            sc->rl_cdata.rl_rx_buf_ptr,
                            sc->rl_cdata.rl_rx_dmamap);
                sc->rl_cdata.rl_rx_buf_ptr = NULL;
                sc->rl_cdata.rl_rx_buf = NULL;
                sc->rl_cdata.rl_rx_buf_paddr = 0;
		*/
                bus_dma_tag_destroy(sc->rtsx_dma_tag);
                sc->rtsx_dma_tag = NULL;
        }
	
/*
 *
 * OpenBSD rtsx.c function declarations
 *
 */
int rtsx_bus_power_on(struct rtsx_pci_softc *);
int rtsx_read(struct rtsx_pci_softc *, u_int16_t, u_int8_t *);
int rtsx_write(struct rtsx_pci_softc *, u_int16_t, u_int8_t, u_int8_t);
int rtsx_write_phy(struct rtsx_pci_softc *sc, u_int8_t addr, u_int16_t val);
int rtsx_init(struct rtsx_pci_softc *sc, int attaching);
int rtsx_card_detect(struct rtsx_pci_softc *sch);
int rtsx_read_cfg(struct rtsx_pci_softc *sc, u_int8_t func, u_int16_t addr, u_int32_t *val);
void rtsx_card_insert(struct rtsx_pci_softc *);
void rtsx_card_eject(struct rtsx_pci_softc *);
int rtsx_led_enable(struct rtsx_pci_softc *);
int rtsx_led_disable(struct rtsx_pci_softc *);
void rtsx_intr(void *arg);


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
 * Return non-zero if the card is currently inserted.
 */
int
rtsx_card_detect(struct rtsx_pci_softc *sch)
{
	struct rtsx_pci_softc *sc = sch;
	return ISSET(sc->rtsx_flags, RTSX_F_CARD_PRESENT);
}

int
rtsx_read(struct rtsx_pci_softc *sc, u_int16_t addr, u_int8_t *val)
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
rtsx_read_cfg(struct rtsx_pci_softc *sc, u_int8_t func, u_int16_t addr, u_int32_t *val)
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
rtsx_write(struct rtsx_pci_softc *sc, u_int16_t addr, u_int8_t mask, u_int8_t val)
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
rtsx_write_phy(struct rtsx_pci_softc *sc, u_int8_t addr, u_int16_t val)
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
rtsx_bus_power_on(struct rtsx_pci_softc *sc)
{
	u_int8_t enable3;
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

	DELAY(200);

	/* Full power. */
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	if (sc->rtsx_flags & RTSX_F_5209)
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);

	/* Enable SD card output. */
	RTSX_WRITE(sc, RTSX_CARD_OE, RTSX_SD_OUTPUT_EN);

	return 0;
}


int
rtsx_init(struct rtsx_pci_softc *sc, int attaching)
{
	u_int32_t status;
	u_int8_t version;
	int error;
	
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
			return 1;
		}
	}

	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(sc, RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);

	/* Clear any pending interrupts. */
	status = READ4(sc, RTSX_BIPR);
	WRITE4(sc, RTSX_BIPR, status);

	/* Check for cards already inserted at attach time. */
	if (attaching==1 && (status & RTSX_SD_EXIST)) {
	  sc->rtsx_flags |= RTSX_F_CARD_PRESENT;
	  device_printf(sc->rtsx_dev, "Card inserted at attach time\n");

	}
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
		device_printf(sc->rtsx_dev, "couldn't write phy register\n");
		return 1;
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

	return 0;
}

void
rtsx_card_insert(struct rtsx_pci_softc *sc)
{
	device_printf(sc->rtsx_dev, "Card inserted\n");

	sc->rtsx_flags |= RTSX_F_CARD_PRESENT;
	(void)rtsx_led_enable(sc);

	/* Schedule card discovery task. */
//	sdmmc_needs_discover(sc->sdmmc);
}

void
rtsx_card_eject(struct rtsx_pci_softc *sc)
{
	device_printf(sc->rtsx_dev, "Card ejected\n");

	sc->rtsx_flags &= ~RTSX_F_CARD_PRESENT;
	(void)rtsx_led_disable(sc);

	/* Schedule card discovery task. */
//	sdmmc_needs_discover(sc->sdmmc);
}

int
rtsx_led_enable(struct rtsx_pci_softc *sc)
{
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
		RTSX_WRITE(sc, RTSX_CARD_AUTO_BLINK,
		    RTSX_LED_BLINK_EN | RTSX_LED_BLINK_SPEED);
	} else {
		RTSX_SET(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		RTSX_SET(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
	}

	return 0;
}

int
rtsx_led_disable(struct rtsx_pci_softc *sc)
{
	if (sc->rtsx_flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_AUTO_BLINK, RTSX_LED_BLINK_EN);
		RTSX_WRITE(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
	} else {
		RTSX_CLR(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
		RTSX_CLR(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
	}

	return 0;
}


void
rtsx_intr(void *arg)
{
	struct rtsx_pci_softc *sc = arg;
	u_int32_t enabled, status;

	enabled = READ4(sc, RTSX_BIER);
	status = READ4(sc, RTSX_BIPR);

	/* Ack interrupts. */
	WRITE4(sc, RTSX_BIPR, status);

	if (((enabled & status) == 0) || status == 0xffffffff)
		return;

	if (status & RTSX_SD_INT) {
		if (status & RTSX_SD_EXIST) {
			if (!ISSET(sc->rtsx_flags, RTSX_F_CARD_PRESENT))
				rtsx_card_insert(sc);
		} else {
			rtsx_card_eject(sc);
		}
	}

	if (status & (RTSX_TRANS_OK_INT | RTSX_TRANS_FAIL_INT)) {
		sc->intr_status |= status;
		wakeup(&sc->intr_status);
	}
}

MODULE_VERSION(rtsx, 1);
