# $FreeBSD$

KMOD=	otus-init
FIRMWS=	otus-init:otus-init:1

CLEANFILES=	otus-init

otus-init: ${.CURDIR}/../../../contrib/dev/otus/otus-init
	cp ${.CURDIR}/../../../contrib/dev/otus/otus-init ${.TARGET}

.include <bsd.kmod.mk>
