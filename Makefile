
MCU=atmega8
FREQ=8000000
AVRPROG=usbtiny

PROG=	mcgps

CPPFLAGS+=-DDEBUG
LDFLAGS	= -Wl,-Map=$(PROG).map,--cref
#LDADD += -Wl,-u,vfscanf -lscanf_min
CLEANFILES+=${PROG}.map

.include "../bsd.avr.mk"
