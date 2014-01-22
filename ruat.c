/*
 * Copyright (c) 2014 Pete Zaitcev <zaitcev@yahoo.com>
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. See file COPYING
 * for details.
 */
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Unable to find pi in math.h. Strange... */
#define PI 3.1415926536

#include <rtl-sdr.h>
#define BUF_MAX  256

#define TAG "ruat"

#define UAT_FREQ  978000000	/* carrier or center frequency, Doc 9861 2.2 */
#define UAT_MOD      312500	/* notional modulation */
#define UAT_RATE (2*1041667)

#define SAMPLE_HIST_SIZE  64
#define ANGLE_HIST_SIZE   20

struct ss_stat {
	unsigned long mark;
	unsigned long samples;
	unsigned long goodbits;
	unsigned long maxfill;
	unsigned long goodsync;

	unsigned int is_dist[SAMPLE_HIST_SIZE];
	unsigned int qs_dist[SAMPLE_HIST_SIZE];
	unsigned int a_dist[ANGLE_HIST_SIZE];
};

static void rx_callback(unsigned char *buf, uint32_t len, void *ctx);
static void *rx_worker(void *arg);
static int to_phi(struct ss_stat *, double *fbuf, unsigned char *buf, int len);
static int search_sync(struct ss_stat *stp, double *fbuf, int flen);

struct mbuf {
	unsigned char *buf;
	unsigned int len;
};

#define NBUFS_MAX 10

/* Could easily pass these as an argument to rx_worker, but meh. */
static pthread_mutex_t rx_mutex;
static pthread_cond_t rx_cond;
static int rx_die;
static int rx_nbufs, rx_in, rx_out;
static struct mbuf rx_bufs[NBUFS_MAX];

int main(int argc, char **argv)
{
	unsigned int device_count, devx;
	char manuf[BUF_MAX], prod[BUF_MAX], sernum[BUF_MAX];
	int ppm_error = 0;
	unsigned int real_rate;
	rtlsdr_dev_t *dev;
	pthread_t rx_thread;
	int rc;

	pthread_mutex_init(&rx_mutex, NULL);
	pthread_cond_init(&rx_cond, NULL);

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

	rc = pthread_create(&rx_thread, NULL, rx_worker, NULL);
	if (rc != 0) {
		fprintf(stderr, TAG ": Error in pthread_create: %d\n", rc);
		exit(1);
	}

#if 1
	/* Flushing is cargo-culted from rtl_adsb. */
	sleep(1);
	rtlsdr_read_sync(dev, NULL, 4096, NULL);
#endif

	rtlsdr_read_async(dev, rx_callback, NULL, 0, 0);

	rx_die = 1;
	rtlsdr_close(dev);
	pthread_cond_signal(&rx_cond);

	return 0;
}

static void rx_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	unsigned char *tmp;
	struct mbuf *p;

	tmp = malloc(len);
	memcpy(tmp, buf, len);

	pthread_mutex_lock(&rx_mutex);

	if (rx_nbufs >= NBUFS_MAX) {

		pthread_mutex_unlock(&rx_mutex);
		fprintf(stderr, TAG ": No buffs\n");
		free(tmp);
		return;
	}

	p = &rx_bufs[rx_in];
	if (++rx_in == NBUFS_MAX) rx_in = 0;
	p->buf = tmp;
	p->len = len;
	rx_nbufs++;

	pthread_cond_signal(&rx_cond);
	pthread_mutex_unlock(&rx_mutex);
}

static void *rx_worker(void *arg)
{
	double *fbuf;
	unsigned int fsize = 40960;
	struct ss_stat stats;
	struct timeval now;
	struct mbuf *p;
	unsigned char *rbuf;
	int flen, fleft;
	int new_flen;
	int rlen;
	unsigned long t;
	int i;
	int rc;

	memset(&stats, 0, sizeof(struct ss_stat));

	fbuf = malloc(fsize);
	if (!fbuf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}

	gettimeofday(&now, NULL);
	stats.mark = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;
	flen = 0;
	for (;;) {
		pthread_mutex_lock(&rx_mutex);
		if (rx_die) {
			pthread_mutex_unlock(&rx_mutex);
			break;
		}

		if (rx_nbufs == 0) {
			rc = pthread_cond_wait(&rx_cond, &rx_mutex);
			if (rc != 0) {
				fprintf(stderr,
				    TAG ": Internal error 3: %d\n", rc);
				pthread_mutex_unlock(&rx_mutex);
				break;
			}
			if (rx_die) {
				pthread_mutex_unlock(&rx_mutex);
				break;
			}

			if (rx_nbufs == 0) {
				pthread_mutex_unlock(&rx_mutex);
				continue;
			}
		}

		--rx_nbufs;
		p = &rx_bufs[rx_out];
		if (++rx_out == NBUFS_MAX) rx_out = 0;
		rbuf = p->buf;
		rlen = p->len;
		p->buf = NULL;

		pthread_mutex_unlock(&rx_mutex);

		stats.samples += rlen/2;

		new_flen = flen + rlen/2;
		if (new_flen * sizeof(double) > fsize) {
			fsize = new_flen * sizeof(double);
			fbuf = realloc(fbuf, fsize);
			if (!fbuf) {
				fprintf(stderr, "No core: %u\n", fsize);
				exit(1);
			}
		}

		flen += to_phi(&stats, fbuf + flen, rbuf, rlen);
		if (flen != new_flen) {
			fprintf(stderr,
			    TAG ": Internal error 4: %d %d\n", flen, new_flen);
			exit(1);
		}

		free(rbuf);
		rbuf = NULL;

		fleft = search_sync(&stats, fbuf, flen);

		memmove(fbuf, fbuf+flen-fleft, fleft * sizeof(double));
		flen = fleft;

		if (stats.samples >= 20000000) {
			gettimeofday(&now, NULL);
			t = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;

			printf("Samples %lu dT %lu"
			    " Bits %lu Maxfill %lu Syncs %lu\n",
			    stats.samples, t - stats.mark,
			    stats.goodbits, stats.maxfill, stats.goodsync);

			printf("I");
			for (i = 0; i < SAMPLE_HIST_SIZE; i++) {
				printf(" %d", stats.is_dist[i]);
			}
			printf("\n");
			printf("Q");
			for (i = 0; i < SAMPLE_HIST_SIZE; i++) {
				printf(" %d", stats.qs_dist[i]);
			}
			printf("\n");
			printf("Phi");
			for (i = 0; i < ANGLE_HIST_SIZE; i++) {
				printf(" %d", stats.a_dist[i]);
			}
			printf("\n");

			memset(&stats, 0, sizeof(struct ss_stat));
			stats.mark = t;
		}
	}
	return NULL;
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
static int to_phi(struct ss_stat *stp,
    double *fbuf, unsigned char *buf, int len)
{
	int cnt;
	int v;
	double vi, vq;
	double phi;

	cnt = 0;
	while (len >= 2) {
		/*
		 * No need to normalize to 1.0 because we're about to divide.
		 */
		v = *buf++;
		stp->is_dist[v * SAMPLE_HIST_SIZE / 256]++;
		vi = (double) (v - 127);
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
