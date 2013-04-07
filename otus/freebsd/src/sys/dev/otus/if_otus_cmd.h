#ifndef	__IF_OTUS_CMD_H__
#define	__IF_OTUS_CMD_H__

struct otus_softc;

#define	OTUS_CMD_LIST_COUNT	60
#define	OTUS_MAX_CMDSZ		512

struct otus_cmd {
	struct otus_softc	*sc;
	uint32_t		flags;
	uint32_t		msgid;
	uint8_t			*buf;
	uint16_t		buflen;
	void			*odata;	/* NB: tx only */
	int			olen;	/* space in odata */
	STAILQ_ENTRY(otus_cmd)	next;
};
typedef STAILQ_HEAD(, otus_cmd) otus_cmdhead;

extern	int otus_alloc_cmd_list(struct otus_softc *sc,
	    struct otus_cmd cmds[], int ncmd, int maxsz);
extern	void otus_free_cmd_list(struct otus_softc *sc,
	    struct otus_cmd cmds[], int ncmd);
extern	struct otus_cmd * otus_get_cmdbuf(struct otus_softc *sc);

#endif /* __IF_OTUS_CMD_H__ */
