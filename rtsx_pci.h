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
struct rtsx_pci_softc {
	device_t	rtsx_dev;
	struct cdev	*rtsx_cdev;
	int		rtsx_flags;
	struct resource *rtsx_irq_res;
	int		rtsx_irq_res_id;
	void		*rtsx_irq_cookie;
	struct resource *rtsx_res;
	int		rtsx_res_id;
	int		rtsx_res_type;
	bus_space_tag_t	rtsx_btag;		/* host register set tag */
	bus_space_handle_t rtsx_bhandle;	/* host register set handle */
	//	struct resource	*sdmmc;		/* generic SD/MMC device */
	bus_dma_tag_t	rtsx_dma_tag;		/* DMA tag from attachment driver */
	void		*rtsx_cmd_dmamem;	/* DMA mem for command transfer */
	bus_dmamap_t	rtsx_cmd_dmamap;	/* DMA map for command transfer */
	bus_addr_t	*rtsx_cmd_buffer;	/* Command buffer */
	void		*rtsx_data_dmamem;	/* DMA mem for data transfer */
	bus_dmamap_t	rtsx_data_dmamap;	/* DMA map for data transfer */
	void		*rtsx_adma_dmamem;	/* DMA mem for ADMA SG descriptors */
	bus_dmamap_t	rtsx_adma_dmamap;	/* DMA map for ADMA SG descriptors */
	bus_dma_segment_t adma_segs[1];		/* segments for ADMA SG buffer */
	u_int32_t 	intr_status;		/* soft interrupt status */
	u_int8_t	regs[RTSX_NREG];	/* host controller state */
	u_int32_t	regs4[6];		/* host controller state */
};

static const struct rtsx_device {
	uint16_t	vendor;
	uint16_t	device;
	int		flags;	
	const char	*desc;
} rtsx_devices[] = {
	{ 0x10ec,	0x5287,	RTSX_F_DEFAULT, "Realtek RTL8411B PCI Express Card Reader"},
	{ 0, 		0,	0,		NULL}
};

/* Host controller functions called by the attachment driver. */
void	rtsx_intr(void *arg);
int	rtsx_init(struct rtsx_pci_softc *sc, int attaching);
int	rtsx_read_cfg(struct rtsx_pci_softc *sc, u_int8_t func, u_int16_t addr, u_int32_t *val);
int	rtsx_dma_alloc(struct rtsx_pci_softc *sc);
void	rtsx_dma_free(struct rtsx_pci_softc *sc);
