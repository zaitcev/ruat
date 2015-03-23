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

#include "fec.h"

#define TAG "ruat"

#define UAT_FREQ  978000000	/* carrier or center frequency, Doc 9861 2.2 */
#define UAT_MOD      312500	/* notional modulation */
#define UAT_RATE (2*1041667)

#define NBITS 36		/* sync length for both Active and Uplink */

/*
 * The code should work as long as BITS_LEN > NBITS and BITS_LEN > BITS_UPLINK.
 * Considering how big the phi buffers are, could as well use a megabyte. But
 * let's not blow caches so easily. So, maybe experiment with the size one day.
 */
#define BITS_LEN      10000

#define BITS_ACTIVE_S   240	/* 144 bits data + 96 bits FEC */
#define BITS_ACTIVE_L   384	/* 272 bits data + 112 bits FEC */
#define BITS_UPLINK    4416	/* 3456 bits data + 960 bits FEC */
#define BITS_U_STEP     736	/* or as they say, 92 8-bit codewords */

/*
 * The primitive polynomial of UAT is defined by ICAO Annex 10 Volume III,
 * 12.4.4.1.3.1 and 12.4.4.2.2.2.1 as p(x) = x^8 + x^7 + x^2 + x + 1, or 0x187.
 */
#define GF256_POLY_UAT 0x187

struct ss_stat {
	unsigned long mark;
	unsigned long samples;
	unsigned long goodbits;
	unsigned long goodlen;
	unsigned int goodsynca, goodsyncu;	/* ADS-B and Uplink */
};

struct param {
	int gain;
	int raw;
	int dump_interval;	/* seconds */
};

/*
 * Pack a string of 8 bit-per-byte bits into a byte.
 *
 * Stricly speaking, Anx.10 V.3 12.4.4.2.2.2.2 only talks about the bit
 * order of bytes of FEC, but it's unthinkable to have it different from
 * the data bytes, right? So, MSB first it is.
 */
#define PICK_BYTE(p)	\
	( (((p)[0] & 1) << 7) | \
	  (((p)[1] & 1) << 6) | \
	  (((p)[2] & 1) << 5) | \
	  (((p)[3] & 1) << 4) | \
	  (((p)[4] & 1) << 3) | \
	  (((p)[5] & 1) << 2) | \
	  (((p)[6] & 1) << 1) | \
	   ((p)[7] & 1) )

struct scan {
	int runlen;		/* Run length for statistic */

	char *bits;
	int bfill;		/* Total bits in bits[] */
	int bwanted;		/* Target amount to complete packet */
};

struct fbuf {
	double *buf;
	unsigned int len;
};

static void preload_phi(void);
static void init_field(void);
static void alloc_fbuf(struct fbuf bufv[]);
static void rx_callback(unsigned char *buf, uint32_t len, void *ctx);
static void *rx_worker(void *arg);
static void stats_dump(struct ss_stat *sp, unsigned long t);
static void stats_reset(struct ss_stat *sp, unsigned long t);
static int to_phi(double *fbuf, unsigned char *buf, int len);
static void scan_init(struct scan *ssp);
static void scan_fbuf(struct scan *ssp, struct ss_stat *stp, struct fbuf *p);
static void scan_spill(struct scan *ssp, struct ss_stat *stp, int ended);
static void scan_endbuf_save(struct scan *ssp, char *s, unsigned int wanted);
static void packet_active_short(char *bits);
static void packet_active_long(char *bits);
static void packet_uplink(char *bits);
static void params(struct param *, int argc, char **argv);
static void Usage(void);
static int nearest_gain(int target_gain, rtlsdr_dev_t *dev);

/*
 * Phi buffers are very large, so we keep NBUFS as low as possible
 * without getting the "No buffs" message. The number of USB buffers
 * in librtlsdr.c is 15, but it's unclear if we need to match that.
 */
#define NBUFS  5
#define FBUF_DIM  (DEFAULT_BUF_LENGTH / 2)

/* Could easily pass these as an argument to rx_worker, but meh. */
static pthread_mutex_t rx_mutex;
static pthread_cond_t rx_cond;
static int rx_die;
static int rx_nbufs, rx_in, rx_out;
static struct fbuf rx_bufs[NBUFS];

static struct param par;
#if 1
static double iq_to_phi[256][256];
#endif
static struct gf field;
static unsigned char gpoly_up[21];
static unsigned char gpoly_as[12];
static unsigned char gpoly_al[14];

int main(int argc, char **argv)
{
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
	init_field();
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

static void init_field(void)
{
	int rc;

	rc = gf_init(&field, GF256_POLY_UAT);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	/*
	 * Ann 10 vol III, 12.4.4.2.2.2.1:
	 * The generator polynomial shall be as follows:
	 *   (x - alpha^120)*(x - alpha^121)* ... *(x - alpha^139)
	 *
	 * For some odd reason we supply 140 instead of 139 to p_gen_gen().
	 */
	rc = p_gen_gen(&field, gpoly_up, 120, 140);
	if (rc != 0) {
		/* This should not occur if the builder has run "make check". */
		fprintf(stderr, TAG ": gf_gen_gen(0x%x,120,140) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	rc = p_gen_gen(&field, gpoly_as, 120, 132);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_gen_gen(0x%x,120,132) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	rc = p_gen_gen(&field, gpoly_al, 120, 134);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_gen_gen(0x%x,120,134) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}
}

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
	struct scan sstate;
	struct ss_stat stats;
	struct timeval now;
	struct fbuf *p;
	unsigned long t;
	int rc;

	gettimeofday(&now, NULL);
	t = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;
	stats_reset(&stats, t);

	scan_init(&sstate);

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

		p = &rx_bufs[rx_out];
		pthread_mutex_unlock(&rx_mutex);

		scan_fbuf(&sstate, &stats, p);

		gettimeofday(&now, NULL);
		t = (unsigned long)now.tv_sec * 1000000 + now.tv_usec;
		if (t - stats.mark >= par.dump_interval*1000000) {
			stats_dump(&stats, t);
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

static void stats_dump(struct ss_stat *sp, unsigned long t)
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
 * Convert I/Q samples to angles and stash them in a buffer
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

static void scan_init(struct scan *ssp)
{
	memset(ssp, 0, sizeof(struct scan));
	ssp->bits = malloc(BITS_LEN);
	if (ssp->bits == NULL) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}
}

/*
 * Scan a phi buffer for the sync bit sequence, push packets downchain
 *
 *  ssp: persistent state
 *  stp: persistent stats
 *  p: the input buffer
 *
 * XXX Only searching half of signals for now!
 */
static void scan_fbuf(struct scan *ssp, struct ss_stat *stp, struct fbuf *p)
{
	double delta_phi, mod_dphi;
	double phi1, phi2;
	int x;

	x = 0;

	stp->samples += p->len;

	for (;;) {
		/* XXX This only works as long as length is always even. */
		if (x >= p->len)
			break;
		phi1 = p->buf[x++];

		if (x >= p->len)
			break;
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
		if (mod_dphi < (150000.0/(float)UAT_RATE) * 2*M_PI ||
		    mod_dphi > (500000.0/(float)UAT_RATE) * 2*M_PI) {
			if (ssp->bfill != 0)
				scan_spill(ssp, stp, 1);
			ssp->runlen = 0;
			continue;
		}
		stp->goodbits++;
		if (++(ssp->runlen) > stp->goodlen) stp->goodlen = ssp->runlen;

		if (ssp->bfill >= BITS_LEN) {
			scan_spill(ssp, stp, 0);
			if (ssp->bfill >= BITS_LEN) {
				/* Never happens because we spilled */
				fprintf(stderr, TAG ": Internal error 2\n");
				exit(1);
			}
		}
		ssp->bits[ssp->bfill++] = (delta_phi < 0) ? '0' : '1';
	}
	// scan_spill(ssp, stp, 0);
}

/*
 * Spill the scan buffer
 *
 *  ssp: persistent state
 *  stp: persistent stats
 *  ended: this spill was invoked by a bad bit, no more contiguous bits
 *
 * At this point, the bit buffer contains a contiguous string of bits.
 * It may be long enough to contain several packets.
 *
 * Before returning, we must reduce bfill (else we loop or crash).
 */
static void scan_spill(struct scan *ssp, struct ss_stat *stp, int ended)
{
	const char sync_bits_a[] = "111010101100110111011010010011100010";
	const char sync_bits_u[] = "000101010011001000100101101100011101";
	char *end = ssp->bits + ssp->bfill;
	char *s;
	size_t off;

	off = 0;
	if (ssp->bwanted) {
		if (ssp->bfill < ssp->bwanted) {
			if (!ended) {
				/*
				 * The packet is not ended, but we spilled.
				 * This is an internal error, because we only
				 * should spill when the bit buffer overflows,
				 * and the size of it should be longer than
				 * the longest packet.
				 */
				fprintf(stderr,
				    TAG ": Internal error 4: %d %d\n",
				    ssp->bfill, ssp->bwanted);
				exit(1);
				// return;
			}
			/* A packet is truncated, restart scan */
			ssp->bwanted = 0;
			ssp->bfill = 0;
			return;
		}
		/* We have a packet, print it, spill its bits, restart scan */
		if (ssp->bwanted == BITS_UPLINK) {
			packet_uplink(ssp->bits);
			off = BITS_UPLINK;
		} else if (ssp->bwanted == BITS_ACTIVE_L) {
			packet_active_long(ssp->bits);
			off = BITS_ACTIVE_L;
		} else if (ssp->bwanted == BITS_ACTIVE_S) {
			if (memcmp(ssp->bits, "00000", 5) == 0) {
				packet_active_short(ssp->bits);
				off = BITS_ACTIVE_S;
			} else {
				if (ssp->bfill >= BITS_ACTIVE_L) {
					packet_active_long(ssp->bits);
					off = BITS_ACTIVE_L;
				} else {
					/* junk bits */
					off = 0;
				}
			}
		} else {
			fprintf(stderr, TAG ": Internal error 6: %d\n",
			    ssp->bwanted);
			exit(1);
		}
		ssp->bwanted = 0;
	}

	s = ssp->bits + off;
	for (;;) {
		if (s + NBITS > end) {
			if (ended) {
				/* A few bits of junk, drop */
				ssp->bfill = 0;
			} else {
				ssp->bfill = end - s;
				if (s != ssp->bits && ssp->bfill != 0)
					memmove(ssp->bits, s, ssp->bfill);
			}
			break;
		}

		if (memcmp(s, sync_bits_a, NBITS) == 0) {
			stp->goodsynca++;
			s += NBITS;
			if (s + BITS_ACTIVE_S > end) {
				if (ended) {
					/* not even a short one - truncated */
					ssp->bfill = 0;
					break;
				}
				/*
				 * We don't know which one this is yet,
				 * ask short.
				 */
				scan_endbuf_save(ssp, s, BITS_ACTIVE_S);
				break;
			}
			/*
			 * Now we have to peek inside a packet that
			 * is not error-corrected yet. Good job, ICAO.
			 * See Doc.9861 2.1.2.
			 */
			if (memcmp(s, "00000", 5) == 0) {	/* short */
				packet_active_short(s);
				s += BITS_ACTIVE_S;
			} else {				/* long */
				if (s + BITS_ACTIVE_L > end) {
					if (ended) {
						/* truncated */
						ssp->bfill = 0;
						break;
					}
					scan_endbuf_save(ssp, s, BITS_ACTIVE_L);
					break;
				}
				packet_active_long(s);
				s += BITS_ACTIVE_L;
			}
		} else if (memcmp(s, sync_bits_u, NBITS) == 0) {
			stp->goodsyncu++;
			s += NBITS;
			if (s + BITS_UPLINK > end) {
				if (ended) {
					/* An uplink packet is truncated */
					ssp->bfill = 0;
					break;
				}
				scan_endbuf_save(ssp, s, BITS_UPLINK);
				break;
			}
			packet_uplink(s);
			s += BITS_UPLINK;
		} else {
			s++;
		}
	}
}

/*
 * We end here when we need more bits, but the bit buffer does not have them,
 * and the transmission has not ended yet. So, save the remaining bits
 * into the head of the buffer, and set ssp->wanted for the next time.
 */
static void scan_endbuf_save(struct scan *ssp, char *s, unsigned int wanted)
{
	char *end = ssp->bits + ssp->bfill;

	/*
	 * This looks like we are checking for overlap, and this condition
	 * looks like it may be valid. But the real objective here is to
	 * guard against looping, which may only happen if we save more
	 * than we process. Looping is a pain to debug. We make sure this
	 * can never happen by keeping the bit buffer size longer than
	 * 2 times the size of longest packet.
	 */
	if (end-s >= ssp->bfill) {
		fprintf(stderr, TAG ": Internal error 5: %d %ld\n",
		    ssp->bfill, (long)(end-s));
		exit(1);
	}

	ssp->bfill = end - s;
	memmove(ssp->bits, s, ssp->bfill);
	ssp->bwanted = wanted;
}

static void packet_active_short(char *bits)
{
	unsigned char packet[BITS_ACTIVE_S/8];
	int i;
	unsigned char buf[12];

	for (i = 0; i < BITS_ACTIVE_S/8; i++) {
		packet[i] = PICK_BYTE(bits); bits += 8;
	}
	if (par.raw) {
		printf("-");
		for (i = 0; i < 18; i++)
			printf("%02x", packet[i]);
		printf(";");
#if 1 /* P3 */
		printf(" fec=");
		for (i = 0; i < 12; i++) {
			printf("%02x", packet[18 + i]);
		}
#endif
		printf("\n");
		fflush(stdout);
	} else {
		p_rem(&field, buf, 12, 18, packet, gpoly_as);
		if (memcmp(buf, packet + 18, 12) == 0) {
			printf("-");
			for (i = 0; i < 18; i++)
				printf("%02x", packet[i]);
			printf(";\n");
			fflush(stdout); /* needed for timely updates in Glie */
		} else {
			printf("as\n");
		}
	}
}

static void packet_active_long(char *bits)
{
	unsigned char packet[BITS_ACTIVE_L/8];
	int i;
	unsigned char buf[14];

	for (i = 0; i < BITS_ACTIVE_L/8; i++) {
		packet[i] = PICK_BYTE(bits); bits += 8;
	}
	if (par.raw) {
		printf("-");
		for (i = 0; i < 34; i++)
			printf("%02x", packet[i]);
		printf(";");
#if 1 /* P3 */  
		printf(" fec=");
		for (i = 0; i < 14; i++) {
			printf("%02x", packet[34 + i]);
		}
#endif
		printf("\n");
		fflush(stdout);
	} else {
		p_rem(&field, buf, 14, 34, packet, gpoly_al);
		if (memcmp(buf, packet + 34, 14) == 0) {
			printf("-");
			for (i = 0; i < 34; i++)
				printf("%02x", packet[i]);
			printf(";\n");
			fflush(stdout); /* needed for timely updates in Glie */
		} else {
			printf("al\n");
		}
	}
}

/*
 * See Annex 10 Volume III 12.4.4.2.2.3 for the interleaving procedure.
 *
 *  bits: A bit buffer of length BITS_UPLINK
 */
static void packet_uplink(char *bits)
{
	unsigned char packet[BITS_UPLINK/8], *p;
	unsigned char d;
	int ecnt;
	int i, j;
	unsigned char buf[20];

	for (i = 0; i < BITS_UPLINK/8; i++) {
		d = PICK_BYTE(bits); bits += 8;
		packet[(i%6) * (BITS_U_STEP/8) + (i/6)] = d;
	}

	if (par.raw) {
		printf("+");
		for (i = 0; i < 6; i++) {
			p = packet + i*92;
			for (j = 0; j < 72; j++) {
				printf("%02x", p[j]);
			}
		}
		printf(";");
#if 1 /* P3 */
		printf(" fec=");
		for (i = 0; i < 6; i++) {
			p = packet + i*92 + 72;
			for (j = 0; j < 20; j++) {
				printf("%02x", p[j]);
			}
		}
#endif
		printf("\n");
	} else {
		ecnt = 0;
		for (i = 0; i < 6; i++) {
			p = packet + i*92;
			p_rem(&field, buf, 20, 72, p, gpoly_up);
			if (memcmp(buf, p + 72, 20) != 0) {
				ecnt += 1;
			}
		}

		if (ecnt == 0) {
			printf("+");
			for (i = 0; i < 6; i++) {
				p = packet + i*92;
				for (j = 0; j < 72; j++)
					printf("%02x", p[j]);
			}
			printf(";\n");
			fflush(stdout);
		} else {
			printf("u %d\n", ecnt); /* P3 */
		}
	}
}

static void params(struct param *par, int argc, char **argv)
{
	char *arg;
	long n;

	par->gain = (~0);
	par->raw = 0;
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
			} else if (arg[1] == 'r') {
				par->raw = 1;
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
	fprintf(stderr, "Usage: " TAG " [-r] [-d interval] [-g gain]\n");
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
