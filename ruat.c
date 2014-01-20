/*
 * Copyright (c) 2014 Pete Zaitcev <zaitcev@yahoo.com>
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. See file COPYING
 * for details.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Unable to find pi in math.h. Strange... */
#define PI 3.1415926536

#include <rtl-sdr.h>
#define BUF_MAX  256

#define UAT_FREQ  978000000	/* carrier or center frequency, Doc 9861 2.2 */
#define UAT_MOD      312500	/* notional modulation */
#define UAT_RATE (2*1041667)

#define TAG "ruat"

struct ss_stat {
	unsigned long calls;
	unsigned long goodbits;
	unsigned long maxfill;
	unsigned long goodsync;
};

static int to_phi(double *fbuf, unsigned char *buf, int len);
static int search_sync(struct ss_stat *stp, double *fbuf, int flen);

int main(int argc, char **argv)
{
	size_t bsize = 40960;	/* 1/100th of a second worth of samples XXX */
	unsigned char *rbuf;
	double *fbuf;
	unsigned int device_count, devx;
	char manuf[BUF_MAX], prod[BUF_MAX], sernum[BUF_MAX];
	int ppm_error = 0;
	unsigned int real_rate;
	rtlsdr_dev_t *dev;
	struct ss_stat stats;
	int blen, rlen;
	int flen, fleft;
	int rc;

	rbuf = malloc(bsize);
	if (!rbuf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}
	fbuf = malloc(bsize * sizeof(double) / 2);
	if (!fbuf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}
	memset(&stats, 0, sizeof(struct ss_stat));

	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, TAG ": No supported devices found\n");
		exit(1);
	}
	printf("Devices found: %u\n", device_count);

	devx = 0;
	printf("Using device: %s\n", rtlsdr_get_device_name(devx));

	rc = rtlsdr_open(&dev, devx);
	if (rc < 0) {
		fprintf(stderr, TAG ": Error opening device %d: %d\n",
		    devx, rc);
		exit(1);
	}

	rc = rtlsdr_get_usb_strings(dev, manuf, prod, sernum);
	if (rc < 0) {
		fprintf(stderr, TAG ": Error getting strings: %d\n", rc);
		exit(1);
	}
	printf("  %d:  %s, %s, SN: %s\n", devx, manuf, prod, sernum);

	rc = rtlsdr_set_tuner_gain_mode(dev, 0);
	if (rc < 0) {
		fprintf(stderr, TAG ": Error setting auto gain: %d\n", rc);
		exit(1);
	}

	rtlsdr_set_agc_mode(dev, 1);

	rc = rtlsdr_set_center_freq(dev, UAT_FREQ);
	if (rc < 0) {
		fprintf(stderr,
		    TAG ": Error setting center frequency: %d\n", rc);
		exit(1);
	}

	rc = rtlsdr_set_freq_correction(dev, ppm_error);
	if (rc == -2) {
		; /* same correction already in effect */
	} else if (rc < 0) {
		fprintf(stderr, TAG ": Error setting correction: %d\n", rc);
		exit(1);
	} else {
		printf("Correction set to %d\n", rc);
	}

	rc = rtlsdr_set_sample_rate(dev, UAT_RATE);
	if (rc < 0) {
		fprintf(stderr, TAG ": Error setting rate: %d\n", rc);
		exit(1);
	}
	real_rate = rtlsdr_get_sample_rate(dev);
	printf("Sample rate set to %u (desired %d)\n", real_rate, UAT_RATE);
	if (real_rate < UAT_RATE - UAT_RATE/544 ||
	    real_rate > UAT_RATE + UAT_RATE/544) {
		fprintf(stderr, TAG ": Set rate %u not acceptable, need %d\n",
		    real_rate, UAT_RATE);
		exit(1);
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	rc = rtlsdr_reset_buffer(dev);
	if (rc < 0) {
		fprintf(stderr, TAG ": Error resetting: %d\n", rc);
		exit(1);
	}

	/* Flushing is cargo-culted from rtl_adsb. */
	sleep(1);
	rtlsdr_read_sync(dev, NULL, 4096, NULL);

	flen = 0;
	for (;;) {
		/*
		 * Read simply hangs unless the requested size is aligned.
		 */
		blen = (bsize - flen*2) & ~0xFFF;
		rc = rtlsdr_read_sync(dev, rbuf, blen, &rlen);
		if (rc < 0) {
			fprintf(stderr, TAG ": Read error: %d\n", rc);
			exit(1);
		}
		if (rlen > blen) {
			fprintf(stderr,
			    TAG ": Internal error 1: %d (%d %d) %d\n",
			    blen, (int)bsize, flen, rlen);
			exit(1);
		}

		flen += to_phi(fbuf + flen, rbuf, rlen);

		fleft = search_sync(&stats, fbuf, flen);

		memmove(fbuf, fbuf+flen-fleft, fleft * sizeof(double));
		flen = fleft;

		if (stats.calls >= 1000) {
			printf("Calls %lu Bits %lu Maxfill %lu Syncs %lu\n",
			    stats.calls, stats.goodbits, stats.maxfill,
			    stats.goodsync);
			break;
		}
	}

	rtlsdr_close(dev);
	return 0;
}

/*
 * As it turns out, all these arc-tangets are hugely expensive. When doing
 * nothing else, ruat consumes exactly half of an 800 MHz AMD Turion.
 * So, we convert I/Q samples to angles once and stash them in a buffer
 * where sync searches can access them several times.
 *  fbuf: return buffer
 *  buf: I/Q samples
 *  len: length of samples in bytes (2 bytes per sample)
 *  returns: number of numbers in fbuf
 */
static int to_phi(double *fbuf, unsigned char *buf, int len)
{
	int cnt;
	double vi, vq;
	double phi;

	cnt = 0;
	while (len >= 2) {
		/*
		 * No need to normalize to 1.0 because we're about to divide.
		 */
		vi = (double) *buf++ - 127;
		if (vi != (double) (buf[-1] - 127)) {	/* P3 */
			fprintf(stderr, "%d\n", buf[-1]);
		}
		vq = (double) *buf++ - 127;
		if (vq == 0.0) {
			phi = (vi < 0) ? -PI/2 : PI/2;
		} else {
			phi = atan(vi / vq);
		}
		fbuf[cnt++] = phi;
		len -= 2;
	}
	return cnt;
}

/*
 * Search for the sync bit sequence.
 * XXX Experimental function
 *  stp: persistent stats
 *  fbuf: angles
 *  flen: number of angles
 *  returns: number of leftover angles
 *
 * XXX Only searching half of signals for now!
 */
static int search_sync(struct ss_stat *stp, double *fbuf, int flen)
{
	const char sync_bits[] = "111010101100110111011010010011100010";
	enum { NBITS = (sizeof(sync_bits)-1)/sizeof(char) };
	int n;
	double delta_phi;
	char bits[NBITS+1];
	int bfill;

	stp->calls++;

	bfill = 0;
	for (n = 0; n < flen; n += 2) {
		/*
		 * Let's find if the frequency went lower or higher than
		 * the center, accounting for the modulo 2*pi.
		 *
		 * Chris Moody writes: "In practice, however, because the
		 * filtering for bandwidth limitation introduces some
		 * overshoot in the deviation, the maximum deviation is
		 * closer to ±450 kHz."
		 */
		delta_phi = fbuf[n+1] - fbuf[n];
		if (delta_phi < -PI) {
			delta_phi += 2*PI;
		} else if (delta_phi > PI) {
			delta_phi -= 2*PI;
		}
		if (delta_phi < (100000.0/(float)UAT_RATE) * 2*PI) {
			bfill = 0;
			continue;
		}
		if (delta_phi > (500000.0/(float)UAT_RATE) * 2*PI) {
			bfill = 0;
			continue;
		}
		stp->goodbits++;

		if (bfill >= NBITS) {
			bits[NBITS] = 0;
			fprintf(stderr,
			    TAG ": Internal error 2: %s\n", bits);
			exit(1);
		}
		bits[bfill++] = (delta_phi < 0) ? '0' : '1';
		if (bfill > stp->maxfill) stp->maxfill = bfill;
		if (bfill == NBITS) {
			bits[bfill] = 0;
			if (strcmp(bits, sync_bits) == 0) {
				stp->goodsync++;
				bfill = 0;
			} else {
				/*
				 * Okay, so these are bits, but they do not
				 * match. May be frame body, may be noise.
				 * XXX inefficient
				 */
				memmove(bits, bits+1, --bfill);
			}
		}
	}

	return bfill;
}
