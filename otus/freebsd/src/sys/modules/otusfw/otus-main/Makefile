# $FreeBSD$

KMOD=	otus-main
FIRMWS=	otus-main:otus-main:1

CLEANFILES=	otus-main

otus-main: ${.CURDIR}/../../../contrib/dev/otus/otus-main
	cp ${.CURDIR}/../../../contrib/dev/otus/otus-main ${.TARGET}

.include <bsd.kmod.mk>
