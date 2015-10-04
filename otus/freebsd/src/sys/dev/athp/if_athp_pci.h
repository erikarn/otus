#ifndef	__IF_ATHP_PCI_H__
#define	__IF_ATHP_PCI_H__

/*
 * Temporary - do register accesses here.
 */
#define	OS_REG_WRITE(_ah, _reg, _val)					\
	bus_space_write_4((bus_space_tag_t)(_ah)->sc_st,		\
	    (bus_space_handle_t)(_ah)->sc_sh, (_reg), (_val))
#define	OS_REG_READ(_ah, _reg)						\
	bus_space_read_4((bus_space_tag_t)(_ah)->sc_st,			\
	    (bus_space_handle_t)(_ah)->sc_sh, (_reg))

#define	MS(_v, _f)	(((_v) & _f##_MASK) >> _f##_LSB)
#define	SM(_v, _f)	(((_v) << _f##_LSB) & _f##_MASK)
#define	WO(_f)		((_f##_OFFSET) >> 2)

/*
 * Array of PCI chip version information.
 */
struct athp_pci_supp_chip {
	int dev_id;
	int rev_id;
};

/*
 * PCI specific glue for athp/ath10k.
 */
struct athp_pci_softc {
	struct athp_softc	sc_sc;
	struct resource		*sc_sr;         /* memory resource */
	struct resource		*sc_irq;        /* irq resource */
	void			*sc_ih;         /* interrupt handler */
	bus_dma_tag_t		sc_dmat;        /* bus DMA tag */
	bus_space_tag_t		sc_st;          /* bus space tag */
	bus_space_handle_t	sc_sh;          /* bus handle tag */
	int			sc_vendorid;
	int			sc_deviceid;

	/* XXX TODO: this doesn't belong here! */

	/* Register mapping, etc */
	enum ath10k_hw_rev	sc_hwrev;
	const struct ath10k_hw_regs	*sc_hwregs;	/* register mapping */
};

#endif	/* __IF_ATHP_PCI_H__ */
