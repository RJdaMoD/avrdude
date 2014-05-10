/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2014 Roger John
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Created on: 03.05.2014
 *      Author: Roger John
 */

#include "stellarisSPI.h"

//#include "ac_cfg.h"

#include "avrdude.h"
#include "avr.h"
#include "pindefs.h"
#include <termios.h>
#include <asm/ioctls.h>
#include <linux/serial.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * Data for the programmer
 */

struct pdata {
	int fd;
	unsigned int baudrate;
	unsigned int bitrate;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))
#define IMPORT_PDATA(pgm) struct pdata *pdata = PDATA(pgm)

/**
 * Function Prototypes
 */

//stellarisSPI specific functions
static void stellarisSPI_setReset(PROGRAMMER* pgm, int value);
static void stellarisSPI_pulseReset(PROGRAMMER* pgm);
static size_t stellarisSPI_write(PROGRAMMER* pgm, const char *s, size_t len);
static size_t stellarisSPI_writeString(PROGRAMMER* pgm, const char *s);
static size_t stellarisSPI_writeChar(PROGRAMMER* pgm, const char c);
static int stellarisSPI_read(PROGRAMMER* pgm, char *s, size_t len);
static int stellarisSPI_readChar(PROGRAMMER* pgm);
static int stellarisSPI_duplex(PROGRAMMER* pgm, unsigned char* tx,
		unsigned char* rx, int len);
//interface - management
static void stellarisSPI_setup(PROGRAMMER* pgm);
static void stellarisSPI_teardown(PROGRAMMER* pgm);
//interface - prog
static int stellarisSPI_open(PROGRAMMER* pgm, char* port);
static void stellarisSPI_close(PROGRAMMER* pgm);
// dummy functions
static void stellarisSPI_disable(PROGRAMMER * pgm);
static void stellarisSPI_enable(PROGRAMMER * pgm);
static void stellarisSPI_display(PROGRAMMER * pgm, const char * p);
//universal
static int stellarisSPI_initialize(PROGRAMMER* pgm, AVRPART* p);
// SPI specific functions
static int stellarisSPI_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
		unsigned char res[4]);
static int stellarisSPI_program_enable(PROGRAMMER * pgm, AVRPART * p);
static int stellarisSPI_chip_erase(PROGRAMMER * pgm, AVRPART * p);

static void stellarisSPI_setup(PROGRAMMER* pgm) {
	if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
		fprintf(stderr,
				"%s: stellarisSPI_setup(): Unable to allocate private memory.\n",
				progname);
		exit(1);
	}
	memset(pgm->cookie, 0, sizeof(struct pdata));
}

static void stellarisSPI_teardown(PROGRAMMER* pgm) {
	free(pgm->cookie);
}
// for serial speed setting, but not needed here
//#define s(x) {x,B##x}
//static struct {
//	int speed;
//	int id;
//} serialSpeedMap[] = { s(0), s(50), s(75), s(110), s(134), s(200), s(300),
//		s(600),
//s(1200), s(1800), s(2400), s(4800), s(9600), s(19200), s(38400), s(57600),
//s(115200), s(230400), s(460800), s(500000), s(576000), s(921600),
//		s(1000000),
//s(1152000), s(1500000), s(2000000), s(2500000), s(3000000), s(3500000),
//s(4000000) };
//#undef s
//static int mapSerialSpeed(int speed) {
//	int i;
//	for (i = 0; i < sizeof(serialSpeedMap) / (2 * sizeof(int)); i++)
//		if (serialSpeedMap[i].speed == speed)
//			return serialSpeedMap[i].id;
//	return -1;
//}
static int stellarisSPI_open(PROGRAMMER* pgm, char* port) {
	if (port == 0 || strcmp(port, "unknown") == 0) //unknown port
			{
		fprintf(stderr,
				"%s: error: No port specified. Port should point to an StellarisSPI interface.\n",
				progname);
		exit(1);
	}
	unsigned int pin = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;
	if (pin == 0) {
		fprintf(stderr, "%s: error: No pin assigned to AVR RESET.\n", progname);
		exit(1);
	} else if (pin != 1 && pin != 2) {
		fprintf(stderr, "%s: error: wrong pin assigned to AVR RESET: %d.\n",
				progname, pin);
		exit(1);
	}

	//save the port to our data
	strcpy(pgm->port, port);

	int fd = open(pgm->port, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "\n%s: error: Unable to open stellarisSPI port %s",
				progname, pgm->port);
		exit(1); //error
	}
	fcntl(fd, F_SETFL, 0);
	struct termios options;
	tcgetattr(fd, &options);
	cfsetspeed(&options, B115200);
	cfmakeraw(&options);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CSTOPB;
	tcsetattr(fd, TCSANOW, &options);

	IMPORT_PDATA(pgm);
	pdata->fd = fd;
	pdata->baudrate = pgm->baudrate == 0 ? 115200 : pgm->baudrate;
	pdata->bitrate =
			pgm->bitclock > 0.0 ?
					(unsigned int) (pgm->bitclock * 1000000) :
					pdata->baudrate;
	stellarisSPI_writeString(pgm, "$Fb");
// changing baudrate seems not be supported by the icdi of the launchpad, so disable it for now
//	if (pdata->baudrate != 115200) {
//		char buf[16];
//		sprintf(buf, "$S*%d*", pdata->baudrate);
//		stellarisSPI_writeString(pgm, buf);
//		int speedId = mapSerialSpeed(pdata->baudrate);
//		if (speedId < 0) {
//			struct serial_struct ser;
//			ioctl(fd, TIOCGSERIAL, &ser);
//			ser.custom_divisor = ser.baud_base / pdata->baudrate;
//			if (!(ser.custom_divisor))
//				ser.custom_divisor = 1;
//			if (pdata->baudrate != (ser.baud_base / ser.custom_divisor)) {
//				fprintf(stderr,
//						"%s: warning: using baudrate %d instead of %d due to rounding (baud_base is %d)",
//						progname, ser.baud_base / ser.custom_divisor,
//						pdata->baudrate, ser.baud_base);
//				pdata->baudrate = ser.baud_base / ser.custom_divisor;
//			}
//			ser.flags &= ~ASYNC_SPD_MASK;
//			ser.flags |= ASYNC_SPD_CUST;
//			ioctl(fd, TIOCSSERIAL, &ser);
//		} else {
//			close(fd);
//			tcgetattr(fd, &options);
//			cfsetspeed(&options, speedId);
//			cfmakeraw(&options);
//			options.c_cflag |= (CLOCAL | CREAD);
//			options.c_cflag &= ~CSTOPB;
//			tcsetattr(fd, TCSANOW, &options);
//		}
//	}
	if (pdata->bitrate != 115200) {
		char buf[16];
		sprintf(buf, "$s*%d*", pdata->bitrate);
		stellarisSPI_writeString(pgm, buf);
	}
	stellarisSPI_pulseReset(pgm);

	return 0;
}
static void stellarisSPI_close(PROGRAMMER* pgm) {
	stellarisSPI_pulseReset(pgm);
	stellarisSPI_writeString(pgm, "$Fh$sf$Sf");
	close(PDATA(pgm)->fd);
}
static size_t stellarisSPI_write(PROGRAMMER* pgm, const char *s, size_t len) {
	return write(PDATA(pgm)->fd, s, len);
}
static size_t stellarisSPI_writeString(PROGRAMMER* pgm, const char *s) {
	return write(PDATA(pgm)->fd, s, strlen(s));
}
static size_t stellarisSPI_writeChar(PROGRAMMER* pgm, const char c) {
	return write(PDATA(pgm)->fd, &c, 1);
}
static int stellarisSPI_read(PROGRAMMER* pgm, char *s, size_t len) {
	int i = 0;
	while (i < len) {
		int t = read(PDATA(pgm)->fd, s + i, len - i);
		if (t < 0) {
			fprintf(stderr,
					"%s: stellarisSPI_read(): Unable to read from stellarisSPI.\n",
					progname);
			exit(1);
		}
		i += t;
	}
	return i;
}
static int stellarisSPI_readChar(PROGRAMMER* pgm) {
	int c, len;
	do {
		len = read(PDATA(pgm)->fd, &c, 1);
	} while (len == 0);
	if (len != 1) {
		fprintf(stderr,
				"%s: stellarisSPI_readChar(): Unable to read from stellarisSPI.\n",
				progname);
		exit(1);
	}
	return c;
}
static void stellarisSPI_setReset(PROGRAMMER* pgm, int value) {
	char buf[4];
	unsigned int pin = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;
	int invert = (pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE) ? 1 : 0;
	char c = (1 << (pin + 1)) | (((value ^ invert) & 1) << (pin - 1));
	sprintf(buf, "$g%X", c & 0xF);
	stellarisSPI_writeString(pgm, buf);
}
static void stellarisSPI_pulseReset(PROGRAMMER* pgm) {
	stellarisSPI_setReset(pgm, 0);
	usleep(1000);
	stellarisSPI_setReset(pgm, 1);
	usleep(1000);
	stellarisSPI_setReset(pgm, 0);
	usleep(1000);
}
static int stellarisSPI_duplex(PROGRAMMER* pgm, unsigned char* tx,
		unsigned char* rx, int len) {
	if (PDATA(pgm)->fd < 0) {
		fprintf(stderr, "\n%s: error: stellarisSPI port not open: %s", progname,
				pgm->port);
		return -1; //error
	}
	int i, len2 = len;
//    for(i=0;i<len;i++) {
//    	if(tx[i]=='$')
//    		stellarisSPI_write(pgm,"$$");
//    	else
//    		stellarisSPI_writeChar(pgm,tx[i]);
//    	//rx[i] = stellarisSPI_readChar(pgm);
//    }
	for (i = 0; i < len; i++)
		if (tx[i] == '$')
			len2++;
	if (len2 > len) {
		unsigned char* tx2 = malloc(len2 * sizeof(unsigned char));
		int j = 0;
		for (i = 0; i < len; i++) {
			tx2[i + j] = tx[i];
			if (tx[i] == '$') {
				j++;
				tx2[i + j] = '$';
			}
		}
		stellarisSPI_write(pgm, tx2, len2);
		free(tx2);
	} else
		stellarisSPI_write(pgm, tx, len);
	stellarisSPI_read(pgm, rx, len);
	return len;
}
static void stellarisSPI_disable(PROGRAMMER* pgm) {
	//do nothing
}

static void stellarisSPI_enable(PROGRAMMER* pgm) {
	//do nothing
}

static void stellarisSPI_display(PROGRAMMER* pgm, const char* p) {
	//do nothing
}

static int stellarisSPI_initialize(PROGRAMMER* pgm, AVRPART* p) {
	int tries, rc;

	if (p->flags & AVRPART_HAS_TPI) {
		//we do not support tpi..this is a dedicated SPI thing
		fprintf(stderr, "%s: error: Programmer %s does not support TPI\n",
				progname, pgm->type);
		return -1;
	}

	//enable programming on the part
	tries = 0;
	do {
		rc = pgm->program_enable(pgm, p);
		if (rc == 0 || rc == -1)
			break;
		stellarisSPI_pulseReset(pgm);
		tries++;
	} while (tries < 65);

	if (rc) {
		fprintf(stderr, "%s: error: AVR device not responding\n", progname);
		return -1;
	}

	return 0;
}

static int stellarisSPI_cmd(PROGRAMMER* pgm, unsigned char cmd[4],
		unsigned char res[4]) {
	return stellarisSPI_duplex(pgm, cmd, res, 4);
}

static int stellarisSPI_program_enable(PROGRAMMER* pgm, AVRPART* p) {
	unsigned char cmd[4];
	unsigned char res[4];

	if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
		fprintf(stderr,
				"%s: error: program enable instruction not defined for part \"%s\"\n",
				progname, p->desc);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));
	avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd); //set the cmd
	pgm->cmd(pgm, cmd, res);

	if (res[2] != cmd[1])
		return -2;

	return 0;
}

static int stellarisSPI_chip_erase(PROGRAMMER* pgm, AVRPART* p) {
	unsigned char cmd[4];
	unsigned char res[4];

	if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
		fprintf(stderr,
				"%s: error: chip erase instruction not defined for part \"%s\"\n",
				progname, p->desc);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));

	avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
	pgm->cmd(pgm, cmd, res);
	usleep(p->chip_erase_delay);
	pgm->initialize(pgm, p);

	return 0;
}

void stellarisSPI_initpgm(PROGRAMMER * pgm) {
	strcpy(pgm->type, "stellarisSPI");

	pgm_fill_old_pins(pgm); // TODO to be removed if old pin data no longer needed

	/*
	 * mandatory functions
	 */

	pgm->initialize = stellarisSPI_initialize;
	pgm->display = stellarisSPI_display;
	pgm->enable = stellarisSPI_enable;
	pgm->disable = stellarisSPI_disable;
	pgm->program_enable = stellarisSPI_program_enable;
	pgm->chip_erase = stellarisSPI_chip_erase;
	pgm->cmd = stellarisSPI_cmd;
	pgm->spi = stellarisSPI_duplex;
	pgm->open = stellarisSPI_open;
	pgm->close = stellarisSPI_close;
	pgm->read_byte = avr_read_byte_default;
	pgm->write_byte = avr_write_byte_default;

	/*
	 * optional functions
	 */
	pgm->setup = stellarisSPI_setup;
	pgm->teardown = stellarisSPI_teardown;
}

const char stellarisSPI_desc[] = "SPI using Stellaris launchpad with stellarisSPI";
