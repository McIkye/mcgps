/*
 * Copyright (c) 2011 Michael Shalayeff
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF MIND, USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
pins:
pb0/icp1
pb1/oc1a	- led gps
pb2/ss/oc1b	- led i2c
pb3/mosi
pb4/miso
pb5/sck

pb6/xtal
pb7/xtal

pc0/adc0
pc1/adc1
pc2/adc2
pc3/adc3
pc4/adc4/sda	- i2c slave iface
pc5/adc5/scl
pc6/reset

pd0/rxd		- gps
pd1/txd		- gps
pd2/int0	- pps?
pd3/int1
pd4/t0
pd5/t1
pd6/ain0
pd7/ain1

 */

#define	GPSON	(PORTB |= _BV(PORTB1))
#define	GPSOFF	(PORTB &= ~_BV(PORTB1))
#define	I2CON	(PORTB |= _BV(PORTB2))
#define	I2COFF	(PORTB &= ~_BV(PORTB2))

#include <stdint.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include <util/twi.h>
#include <util/crc16.h>

struct {
	int longit;
	int latit;
	int speed;
	int course;
	char sec;
	char min;
	char hour;
	char day;
	char month;
	char year;
	char flags;
#define	GPS_VALID	0x01
#define	GPS_STALE	0x02
#define	GPS_SAVED	0x04
	char crc;	/* use ibutton 8bit crc */
} d[3];

int pos2ffnum(char *, char **);
int str2ffnum(char *, char **);

signed char gpst;
char eeaddr;

const char hexd[] PROGMEM = "0123456789abcdef";

int
main(void)
{
	cli();
	MCUSR = 0;	/* clear the reset bits */

	/* reinit */
	d[0].flags = d[1].flags = 0;
	gpst = -1;

	// SFIOR = 0;
	ADCSRA = 0;		/* disable ADC */
	ACSR = _BV(ACD);
	SPCR = 0;		/* disable SPI */
	TCCR2 = 0;		/* reset timer-2 */

	PORTB = 0;
	DDRB = _BV(DDB1) | _BV(DDB2);
	PORTC = 0;
	DDRC = 0;
	PORTD = 0;
	DDRD = 0;

#if 0
	/* setup PWM for leds */
	OCR1A = 0x7f7f;				/* fastPWM mode */
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR1B = _BV(CS12) | _BV(WGM12);	/* / 256 */
	TIMSK = 0;				/* no interrupts */
#endif

	TWCR = _BV(TWIE) | _BV(TWEA) | _BV(TWEN) | _BV(TWINT);
	TWAR = 0x42;				/* our address */
	TWSR = 0;
	TWBR = 0;

#define	BRR	(F_CPU / 16 / 9600 - 1)
	UCSRB = _BV(RXEN) | _BV(RXCIE);
	UBRRH = BRR >> 8;
	UBRRL = BRR;				/* 9600 */
	UCSRC = _BV(UCSZ1) | _BV(UCSZ0);	/* 8n1 */

	/* check EEPROM for last gps data */
	{
		char c, *p = (char *)&d[2];
		int i;

		for (c = 0, i = sizeof d[2]; i--; p++) {
			EEAR = p - (char *)&d[2];
			EECR |= _BV(EERE);
			*p = EEDR;
			c = _crc_ibutton_update(c, *p);
		}

		if (c == 0 && (d[2].flags & GPS_VALID)) {
			GPSON;
/* TODO upload last data into gps */
			GPSOFF;
		}
	}

	/* keep sleeping and let ints do all the works */
	while (1) {
		// MCUCR = _BV(BODS) | _BV(BODSE);
		// MCUCR = _BV(BODS);
		MCUCR = _BV(SE) | _BV(SM2) | _BV(SM1);	/* standby */
		sei();
		sleep_cpu();
		cli();
		MCUCR = 0;
		if ((d[0].flags & d[1].flags & GPS_VALID) &&
		    d[0].hour != d[1].hour) {
			d[2].longit = d[1].longit;
			d[2].latit = d[1].latit;
			d[2].speed = d[1].speed;
			d[2].course = d[1].course;
			d[2].sec = d[1].sec;
			d[2].min = d[1].min;
			d[2].hour = d[1].hour;
			d[2].day = d[1].day;
			d[2].month = d[1].month;
			d[2].year = d[1].year;
			d[2].flags = d[1].flags;
			d[2].crc = d[1].crc;
			eeaddr = 0;
			EECR = EERIE;	/* start the write */
		}
	}

	return 0;
}

ISR(EE_RDY_vect)	/* vector 15 */
{
	static char *p;

	if (!(d[2].flags & GPS_SAVED) && !eeaddr)
		p = (char *)&d[2];

	if (eeaddr == sizeof d[2]) {
		EECR = 0;
		d[2].flags |= GPS_SAVED;
	}

	EEAR = eeaddr++;
	EEDR = *p++;
	EECR |= _BV(EEMWE);
	EECR |= _BV(EEWE);
}

/*
 * spi: chip-select has come
 */
ISR(TWI_vect)	/* vector 17 */
{
	static char *p, c;
	char cr;

	I2CON;

	cr = _BV(TWIE) | _BV(TWEN);
	switch (TWSR & 0xf8) {
	case TW_ST_SLA_ACK:
	case TW_ST_ARB_LOST_SLA_ACK:
		p = (char *)&d[0];
		c = sizeof d[0] - 1;
		TWDR = *p++;
		TWCR = cr | _BV(TWINT) | _BV(TWEA);
		I2COFF;
		return;

	case TW_ST_DATA_ACK:
		TWDR = *p++;
		if (--c > 0)
			c |= _BV(TWEA);
		TWCR = cr | _BV(TWINT);
		I2COFF;
		return;

	default:
	case TW_ST_DATA_NACK:
		TWCR = cr | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO);
		I2COFF;
		return;

	case TW_ST_LAST_DATA:
		TWCR = cr | _BV(TWINT) | _BV(TWEA);
		/* FALLTHROUGH */
	}

	if (d[1].flags & GPS_VALID) {
		d[0].longit = d[1].longit;
		d[0].latit = d[1].latit;
		d[0].speed = d[1].speed;
		d[0].course = d[1].course;
		d[0].sec = d[1].sec;
		d[0].min = d[1].min;
		d[0].hour = d[1].hour;
		d[0].day = d[1].day;
		d[0].month = d[1].month;
		d[0].year = d[1].year;
		d[0].flags = d[1].flags;
		d[0].crc = d[1].crc;
	} else {
		char *p = (char *)&d[0];
		int i = sizeof d[0];

		d[0].flags |= GPS_STALE;
		for (d[0].crc = c = 0; i--; p++)
			c = _crc_ibutton_update(c, *p);

		d[0].crc = c;
	}

	I2COFF;
}

/*
 * uart: char received
 */
ISR(USART_RXC_vect)	/* vector 11 */
{
	static char bbb[12], *bp, cksum;
	int *var;
	char c, *buf = &bbb[1];

	if (!(UCSRA & _BV(RXC)))
		return;

	GPSON;

	c = UDR;
	if (gpst >= 0) {
		cksum ^= c;
		/* even state only read into the buffer */
		if (!(gpst & 1)) {
			if (c != ',' && c != '\r') {
				if (bp - buf >= 10)
					goto fail;
				*bp++ = c;
				GPSOFF;
				return;
			} else {
				gpst++;
				*bp = '\0';
			}
		}
	}

	switch (gpst) {
	case -1:	/* need a dollar! */
		if (c == '$') {
			cksum = 0;
			d[1].flags = 0;
			goto next;
		}
		break;
	case 1:		/* check if it's our msg type */
/* $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598,E,*10 */
/* $GPRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,W,n.nn,ddd.dd,ddmmyy,A,*cs */
		if (buf[0] == 'G' && buf[1] == 'P' &&
		    buf[2] == 'R' && buf[3] == 'M' && buf[4] == 'C')
			goto next;
		goto fail;
	case 3:		/* UTC time */
		if (bp - buf < 10)
			goto fail;
		/* only copy now; if data is valid update times */
		d[1].sec = buf[5] - '0' + (buf[4] - '0') * 10;
		d[1].min = buf[3] - '0' + (buf[2] - '0') * 10;
		d[1].hour = buf[1] - '0' + (buf[0] - '0') * 10;
		goto next;
	case 5:		/* data valid */
		if (buf[0] == 'A')
			goto next;
		else
			goto fail;
	case 7:		/* latitude */
		var = &d[1].latit;
		buf = bbb;
	cooin:
		*var = pos2ffnum(buf, &bp);
		if (*bp != '\0')
			goto fail;
		else
			goto next;
	case 9:		/* N/S */
		if (buf[0] == 'N')
			d[1].latit = -d[1].latit;
		else if (buf[0] != 'S')
			goto fail;
		goto next;
	case 11:	/* longitude */
		var = &d[1].longit;
		goto cooin;
	case 13:	/* E/W */
		if (buf[0] == 'W')
			d[1].longit = -d[1].longit;
		else if (buf[0] != 'E')
			goto fail;
		goto next;
	case 15:	/* speed */
		var = &d[1].speed;
	numin:
		*var = str2ffnum(buf, &bp);
		if (*bp != '\0')
			goto fail;
		goto next;
	case 17:	/* course */
		var = &d[1].course;
		goto numin;
	case 19:	/* date */
		if (bp - buf < 6)
			goto fail;
		d[1].day = buf[1] - '0' + (buf[0] - '0') * 10;
		d[1].month = buf[3] - '0' + (buf[2] - '0') * 10;
		d[1].year = buf[5] - '0' + (buf[4] - '0') * 10;
		goto next;
	case 23:	/* checksum */
		if (bp - buf < 3 || buf[0] != '*')
			goto fail;
		/* compensate for the buffer */
		c ^= buf[0];
		c ^= buf[1];
		c ^= buf[2];
		c ^= '\r';
#define	tolower(c)	((c) + 0x20)
		buf[4] = pgm_read_byte(&hexd[(int)c & 0xf]);
		buf[3] = pgm_read_byte(&hexd[(int)c >> 4]);
		if (buf[4] != buf[2] || buf[3] != buf[1])
			goto fail;
		else {
			/* skip buffering */
			gpst++;
			goto next;
		}
		break;
	case 25:
		if (c != '\n')
			goto fail;
		d[1].flags |= GPS_VALID;
		{
			char *p = (char *)&d[1];
			int i = sizeof d[1];

			for (d[1].crc = c = 0; i--; p++)
				c = _crc_ibutton_update(c, *p);

			d[1].crc = c;
		}
		/* FALLTHROUGH */
	default:
		if (0) {
	fail:
			/* DEBUG FAIL */
			;
		}
		gpst = -1;
		break;
	case 21:	/* magnetic variation(EW)/mode(ADEN) */
	next:
		bbb[0] = '0';
		bp = buf;
		gpst++;
	}
	GPSOFF;
}

int
pos2ffnum(char *buf, char **bp)
{
	char *p, rif;
	int rv;

	for (rif = 0, rv = 0, p = buf; *p != '\0'; p++) {
		if (*p == '.') {
			rif = 4;
			continue;
		}
		if (*p < '0' || '9' < *p) {
			*bp = p;
			return 0;
		}
		rv *= p - buf == 3? 6 : 10;
		rv += *p - '0';
		if (rif)
			rif--;
	}

	while (rif-- > 0)
		rv *= 10;

	*bp = p;
	return rv;
}

int
str2ffnum(char *buf, char **bp)
{
	char *p, rif;
	int rv;

	for (rif = 0, rv = 0, p = buf; *p != '\0'; p++) {
		if (*p == '.') {
			rif = 3;
			continue;
		}
		if (*p < '0' || '9' < *p) {
			*bp = p;
			return 0;
		}
		rv *= 10;
		rv += *p - '0';
		if (rif)
			rif--;
	}

	while (rif-- > 0)
		rv *= 10;

	*bp = p;
	return rv;
}
