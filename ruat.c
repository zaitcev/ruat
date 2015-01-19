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

#include <rtl-sdr.h>
#define BUF_MAX  256
/*
 * The built-in default is not visible to rtlsdr clients. So, hardcode.
 */
#define DEFAULT_BUF_LENGTH	(16 * 32 * 512)

#define TAG "ruat"

#define UAT_FREQ  978000000	/* carrier or center frequency, Doc 9861 2.2 */
#define UAT_MOD      312500	/* notional modulation */
#define UAT_RATE (2*1041667)

#define NBITS 36		/* sync length for both Active and Uplink */

struct ss_stat {
	unsigned long mark;
	unsigned long samples;
	unsigned long goodbits;
	unsigned long goodlen;
	unsigned int goodsynca, goodsyncu;	/* ADS-B and Uplink */
};

struct param {
	int gain;
	int verbose;		/* XXX Currently does nothing */
	int dump_interval;	/* seconds */
};

struct scan {
	int runlen;		/* Run length for statistic */

	char bits[NBITS+1];
	int bfill;
};

struct fbuf {
	double *buf;
	unsigned int len;
};

static void preload_phi(void);
static void alloc_fbuf(struct fbuf bufv[]);
static void rx_callback(unsigned char *buf, uint32_t len, void *ctx);
static void *rx_worker(void *arg);
static void stats_dump(struct ss_stat *sp, struct param *par, unsigned long t);
static void stats_reset(struct ss_stat *sp, unsigned long t);
static int to_phi(double *fbuf, unsigned char *buf, int len);
static void search_sync(struct ss_stat *stp, struct scan *ssp, int _rx_out);
static void params(struct param *, int argc, char **argv);
static void Usage(void);
static int nearest_gain(int target_gain, rtlsdr_dev_t *dev);

#define NBUFS 10
#define FBUF_DIM  (DEFAULT_BUF_LENGTH / 2)

/* Could easily pass these as an argument to rx_worker, but meh. */
static pthread_mutex_t rx_mutex;
static pthread_cond_t rx_cond;
static int rx_die;
static int rx_nbufs, rx_in, rx_out;
static struct fbuf rx_bufs[NBUFS];

#if 1
static double iq_to_phi[256][256];
#endif

int main(int argc, char **argv)
{
	struct param par;
	unsigned int device_count, devx;
	char manuf[BUF_MAX], prod[BUF_MAX], sernum[BUF_MAX];
	int ppm_error = 0;
	unsigned int real_rate;
	int gain;
	rtlsdr_dev_t *dev;
	pthread_t rx_thread;
	int rc;

	pthread_mutex_init(&rx_mutex, NULL);
	pthread_cond_init(&rx_cond, NULL);

	params(&par, argc, argv);

	preload_phi();
	alloc_fbuf(rx_bufs);

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
	if ((gain = par.gain) == (~0)) {
		rc = rtlsdr_set_tuner_gain_mode(dev, 0);
		if (rc < 0) {
			fprintf(stderr,
			    TAG ": Error setting auto gain: %d\n", rc);
			exit(1);
		}
	} else {
		rc = rtlsdr_set_tuner_gain_mode(dev, 1);
		gain = nearest_gain(gain * 10, dev);
		rc = rtlsdr_set_tuner_gain(dev, gain);
		if (rc < 0) {
			fprintf(stderr,
			    TAG ": Error setting gain %d/10: %d\n", gain, rc);
			exit(1);
		}
		printf("Gain set to %d\n", gain/10);
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

	rc = pthread_create(&rx_thread, NULL, rx_worker, &par);
	if (rc != 0) {
		fprintf(stderr, TAG ": Error in pthread_create: %d\n", rc);
		exit(1);
	}

#if 1
	/* Flushing is cargo-culted from rtl_adsb. */
	sleep(1);
	rtlsdr_read_sync(dev, NULL, 4096, NULL);
#endif

	rtlsdr_read_async(dev, rx_callback, NULL, 0, DEFAULT_BUF_LENGTH);

	pthread_mutex_lock(&rx_mutex);
	rx_die = 1;
	pthread_mutex_unlock(&rx_mutex);
	rtlsdr_close(dev);
	pthread_cond_signal(&rx_cond);

	return 0;
}

static void rx_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct fbuf *p;

	pthread_mutex_lock(&rx_mutex);

	if (rx_nbufs >= NBUFS) {
		pthread_mutex_unlock(&rx_mutex);
		fprintf(stderr, TAG ": No buffs\n");
		return;
	}

	p = &rx_bufs[rx_in];
	if (++rx_in == NBUFS) rx_in = 0;

	p->len = to_phi(p->buf, buf, len);

	rx_nbufs++;

	pthread_cond_signal(&rx_cond);
	pthread_mutex_unlock(&rx_mutex);
}

#if 1
/*
 * Computing phi in-place: CPU 44%, RES 7742.
 * Precomputing phi here: CPU 9%, RES 17372.
 * You'd think that the program would be larger by 524288 bytes or so, but no.
 * It's actually 9.3 MB bigger. But you cannot argue with the CPU savings.
 * So much for the effects of caches and the memory wall.
 */
static void preload_phi(void)
{
	int vi, vq;
	double phi;

	/*
	 * No need to normalize to 1.0 because we're about to divide.
	 * XXX What about 00 = -1.0, 0xff = +1.0, thus zero at 127.5?
	 */

	for (vi = 0; vi < 256; vi++) {
		for (vq = 0; vq < 256; vq++) {
			/* V = Inphase + j*Quadrature; atan2(y,x) */
			phi = atan2((double) (vq - 127), (double) (vi - 127));
			iq_to_phi[vi][vq] = phi;
		}
	}
}
#endif

/*
 * We pass know the size of buffers in rtlsdr_read_async(),
 * sp we never run outside of the preallocated phi buffers.
 */
static void alloc_fbuf(struct fbuf bufv[])
{
	struct fbuf *p;
	int i;

	p = bufv;
	for (i = 0; i < NBUFS; i++) {
		p->buf = malloc(sizeof(double) * FBUF_DIM);
		if (p->buf == NULL) {
			fprintf(stderr, TAG ": No core\n");
			exit(1);
		}
		p->len = 0;
		p++;
	}
}

static void *rx_worker(void *arg)
{
	struct param *par = arg;
	double *fbuf;
	unsigned int fsize = 40960;
	struct scan sstate;
	struct ss_stat stats;
	struct timeval now;
	int _rx_out;
	// struct fbuf *p;
	unsigned long t;
	int rc;

	fbuf = malloc(fsize);
	if (!fbuf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}

	gettimeofday(&now, NULL);
	t = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;
	stats_reset(&stats, t);

	memset(&sstate, 0, sizeof(struct scan));

	pthread_mutex_lock(&rx_mutex);
	for (;;) {
		if (rx_die)
			break;
		if (rx_nbufs == 0) {
			rc = pthread_cond_wait(&rx_cond, &rx_mutex);
			if (rc != 0) {
				pthread_mutex_unlock(&rx_mutex);
				fprintf(stderr,
				    TAG ": Internal error 3: %d\n", rc);
				// break;
				exit(1);
			}
			if (rx_die)
				break;
			if (rx_nbufs == 0)
				continue;
		}

		_rx_out = rx_out;
		pthread_mutex_unlock(&rx_mutex);

		search_sync(&stats, &sstate, _rx_out);

		gettimeofday(&now, NULL);
		t = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;
		if (t - stats.mark >= par->dump_interval*1000000) {
			stats_dump(&stats, par, t);
			stats_reset(&stats, t);
		}

		pthread_mutex_lock(&rx_mutex);
		if (rx_nbufs == 0) {
			pthread_mutex_unlock(&rx_mutex);
			fprintf(stderr, TAG ": Internal error 1\n");
			exit(1);
		}

		if (++rx_out == NBUFS) rx_out = 0;
		--rx_nbufs;
	}
	pthread_mutex_unlock(&rx_mutex);
	return NULL;
}

static void stats_dump(struct ss_stat *sp, struct param *par, unsigned long t)
{

	printf("Samples %lu dT %lu"
	    " Bits %lu Maxlen %lu Syncs a:%u u:%u\n",
	    sp->samples, t - sp->mark,
	    sp->goodbits, sp->goodlen, sp->goodsynca, sp->goodsyncu);
}

static void stats_reset(struct ss_stat *sp, unsigned long t)
{
	memset(sp, 0, sizeof(struct ss_stat));
	sp->mark = t;
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
	double phi;

	cnt = 0;
	while (len >= 2) {
#if 0
		int v;
		double vi, vq;

		/*
		 * No need to normalize to 1.0 because we're about to divide.
		 * XXX What about 00 = -1.0, 0xff = +1.0, thus zero at 127.5?
		 */
		v = *buf++;
		vi = (double) (v - 127);
		v = *buf++;
		vq = (double) (v - 127);
		phi = atan2(vq, vi);		/* V = Inphase + j*Quadrature */
#endif
#if 1
		int vi, vq;

		vi = *buf++;
		vq = *buf++;
		phi = iq_to_phi[vi][vq];
#endif
		fbuf[cnt++] = phi;
		len -= 2;
	}
	return cnt;
}

/*
 * Search for the sync bit sequence.
 *
 * Input is the list of phi buffers. We take samples from rx_bufs and
 * other associated globals, which is highly improper, but meh.
 * We adjust corrent rx_out & rx_nbufs as a side effect, obviously.
 *
 *  stp: persistent stats
 *  ssp: persistent state
 *  _rx_out: copy of global rx_out - current index into rx_bufs[]
 *
 * XXX Only searching half of signals for now!
 */
static void search_sync(struct ss_stat *stp, struct scan *ssp, int _rx_out)
{
	const char sync_bits_a[] = "111010101100110111011010010011100010";
	const char sync_bits_u[] = "000101010011001000100101101100011101";
	double delta_phi, mod_dphi;
	double phi1, phi2;
	struct fbuf *p;
	int x;

	p = &rx_bufs[_rx_out];
	x = 0;

	stp->samples += p->len;

	for (;;) {
		/* XXX This only works as long as length is always even. */
		if (x >= p->len)
			return;
		phi1 = p->buf[x++];

		if (x >= p->len)
			return;
		phi2 = p->buf[x++];

		/*
		 * Let's find if the frequency went lower or higher than
		 * the center, accounting for the modulo 2*pi.
		 *
		 * Chris Moody writes: "In practice, however, because the
		 * filtering for bandwidth limitation introduces some
		 * overshoot in the deviation, the maximum deviation is
		 * closer to ±450 kHz."
		 *
		 * So, for now we use made-up constraints instead of UAT_MOD.
		 */
		delta_phi = phi2 - phi1;
		if (delta_phi < -M_PI) {
			delta_phi += 2*M_PI;
		} else if (delta_phi > M_PI) {
			delta_phi -= 2*M_PI;
		}
		mod_dphi = fabs(delta_phi);
		if (mod_dphi < (150000.0/(float)UAT_RATE) * 2*M_PI) {
			ssp->bfill = 0;
			ssp->runlen = 0;
			continue;
		}
		if (mod_dphi > (500000.0/(float)UAT_RATE) * 2*M_PI) {
			ssp->bfill = 0;
			ssp->runlen = 0;
			continue;
		}
		stp->goodbits++;
		if (++(ssp->runlen) > stp->goodlen) stp->goodlen = ssp->runlen;

		if (ssp->bfill >= NBITS) {
			ssp->bits[NBITS] = 0;
			fprintf(stderr,
			    TAG ": Internal error 2: %s\n", ssp->bits);
			exit(1);
		}
		ssp->bits[ssp->bfill++] = (delta_phi < 0) ? '0' : '1';
		if (ssp->bfill == NBITS) {
			ssp->bits[ssp->bfill] = 0;
			if (strcmp(ssp->bits, sync_bits_a) == 0) {
				stp->goodsynca++;
				ssp->bfill = 0;
			} else if (strcmp(ssp->bits, sync_bits_u) == 0) {
				stp->goodsyncu++;
				ssp->bfill = 0;
			} else {
				/*
				 * Okay, so these are bits, but they do not
				 * match. May be frame body, may be noise.
				 * XXX inefficient
				 */
				memmove(ssp->bits, ssp->bits+1, --(ssp->bfill));
			}
		}
	}
}

static void params(struct param *par, int argc, char **argv)
{
	char *arg;
	long n;

	par->gain = (~0);
	par->verbose = 0;
	par->dump_interval = 10;

	argv += 1;
	while ((arg = *argv++) != NULL) {
		if (arg[0] == '-') {
			if (arg[1] == 'd') {
				if ((arg = *argv++) == NULL)
					Usage();
				n = strtol(arg, NULL, 10);
				if (n < 1)
					n = 1;
				if (n >= 24*60*60)
					n = 24*60*60;
				par->dump_interval = n;
			} else if (arg[1] == 'g') {
				if ((arg = *argv++) == NULL)
					Usage();
				n = strtol(arg, NULL, 10);
				if (n < -10000 || n >= 10000) {
					fprintf(stderr,
					    TAG ": Invalid gain `%s'\n", arg);
					exit(1);
				}
				par->gain = n;
			} else if (arg[1] == 'v') {
				par->verbose = 1;
			} else {
				Usage();
			}
		} else {
			Usage();
		}
	}
}

static void Usage(void)
{
	printf("Usage: " TAG " [-v] [-d interval] [-g gain]\n");
	exit(1);
}

/* taken from rtl_rm */
static int nearest_gain(int target_gain, rtlsdr_dev_t *dev)
{
	int i, err1, err2, count, close_gain;
	int* gains;
	count = rtlsdr_get_tuner_gains(dev, NULL);
	if (count <= 0) {
		return 0;
	}
	gains = malloc(sizeof(int) * count);
	count = rtlsdr_get_tuner_gains(dev, gains);
	close_gain = gains[0];
	for (i=0; i<count; i++) {
		err1 = abs(target_gain - close_gain);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			close_gain = gains[i];
		}
	}
	free(gains);
	return close_gain;
}
