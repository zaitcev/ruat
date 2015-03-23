/*
 * test
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fec.h"

#define TAG "tester"

static void test_gen_lc(void);
static void test_field_lc(struct gf *f);
static void test_gen_uat(void);
static void test_gen_gen_qrc(void);
static void test_gen_gen_uat(void);
static void test_gen_gen(unsigned int poly, int rpow, int len,
    const unsigned char *sample);
static void test_rem_qrc(void);
static void test_rem_uat1(void);
static void test_rem_uat2(void);
static void test_rem_uat3(void);
static void test_rem(struct gf *f, int mlen, const unsigned char *msg,
    unsigned int ppoly, int gplen, const unsigned char *gpoly,
    const unsigned char *sample);

/*
 * This is the sample GF(2^8) taken from 1983 Lin & Costello.
 * Its primitive polynomial is p(x)=x^8+x^4+x^3+x^2+1, or 0x11d.
 * Note that in the book, the alpha^0 is the leftmost bit!
 * We keep that notation for the convenience of matching by eye.
 */
#define GF256_POLY_LC  0x11d
static char gf256_8_4_3_2_0[256][8] = {
	"00000000",	/* index 0  power -: element 0 */
	"10000000",	/*       1        0: element 1 */
	"01000000",	/*       2        1: element alpha */
	"00100000",
	"00010000",
	"00001000",
	"00000100",
	"00000010",	/*       7        6: element alpha^6 */

	"00000001",	/* power 7  alpha^7 */
	"10111000",
	"01011100",
	"00101110",
	"00010111",
	"10110011",
	"11100001",
	"11001000",

	"01100100",	/* power 15 */
	"00110010",
	"00011001",
	"10110100",
	"01011010",
	"00101101",
	"10101110",
	"01010111",

	"10010011",	/* power 23 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 31 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 39 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 47 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 55 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 63 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 71 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 79 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 87 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 95 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 103 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 111 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 119 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 127 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 135 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 143 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 151 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 159 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 167 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 175 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 183 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 191 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",


	"",	/* power 199 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 207 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 215 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 223 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 231 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"",	/* power 239 */
	"",
	"",
	"",
	"",
	"",
	"",
	"",

	"11000001",	/* power 247 */
	"",
	"",
	"",
	"",
	"",
	"",
	"01110001" 	/* power 254 */
};

/*
 * The primitive polynomial of UAT is defined by ICAO Annex 10 Volume III,
 * 12.4.4.1.3.1 and 12.4.4.2.2.2.1 as p(x) = x^8 + x^7 + x^2 + x + 1, or 0x187.
 */
#define GF256_POLY_UAT 0x187

int main(int argc, char **argv)
{
	test_gen_lc();
	test_gen_uat();
	test_gen_gen_qrc();
	test_gen_gen_uat();
	test_rem_qrc();
	test_rem_uat1();
	test_rem_uat2();
	test_rem_uat3();
	return 0;
}

static void test_gen_lc(void)
{
	unsigned char sample[256];
	struct gf field;
	char *p;
	unsigned int v;
	int i, j;
	int error;
	int rc;

	/*
	 * Test #1
	 * Test if gen256 with a known poly generates a known field.
	 *
	 * Since the sample field is not complete, we also test the generated
	 * field for uniqueness.
	 */

	/*
	 * Convert the L&C GF(256) from the book form to computer form.
	 */
	for (i = 0; i < 256; i++) {
		p = gf256_8_4_3_2_0[i];
		if (p[0] != 0) {
			v = 0;
			for (j = 8; j-- != 0;) {
				v <<= 1;
				v |= p[j] & 1;
			}
			sample[i] = v;
		} else {
			/*
			 * We do not have an unused value for a sentiel,
			 * so we set a certain valid value for debugging.
			 */
			sample[i] = 0xee;
		}
	}

	rc = gf_init(&field, GF256_POLY_LC);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_LC, rc);
		exit(1);
	}

	error = 0;
	for (i = 0; i < 256; i++) {
		/*
		 * We were too lazy to type in the whole sample field from
		 * the book, so we only compare to the known elements.
		 */
		if (gf256_8_4_3_2_0[i][0] != 0) {
			if (sample[i] != field.field[i]) {
				fprintf(stderr, TAG ": "
				    "mismatch [%d] 0x%02x 0x%02x\n",
				    i, sample[i], field.field[i]);
				error = 1;
			}
		}
	}
	if (error)
		exit(1);

	test_field_lc(&field);

	gf_fin(&field);
}

/*
 * Since we care about the binary/tuple representation, our tests for
 * anything but identity, zero, and alpha^1 are dependent on the field.
 * So, although f is an argument, this can only test the LC field.
 */
static void test_field_lc(struct gf *f)
{
	struct test1 {
		unsigned int a;
		unsigned int b;
		unsigned int a_by_b;
	};
	static struct test1 test_v[] = {
		{ 0, 0, 0 },
		{ 0, 1, 0 },
		{ 1, 0, 0 },
		{ 1, 1, 1 },
		{ 0, 2, 0 },		/* 0 * alpha = 0 */
		{ 2, 0, 0 },
		{ 1, 2, 2 },		/* 1 * alpha = alpha */
		{ 2, 1, 2 },		/* 1 * alpha = alpha */
		{ 2, 2, 4 },		/* alpha * alpha = alpha^2 */
		{ 0x47, 2, 0x8e },	/* alpha^253 * alpha = alpha^254 */
		{ 0x8e, 1, 0x8e },
		{ 0x8e, 2, 1 },		/* alpha^254 * alpha = alpha^0 = 1 */
		{ 0x8e, 4, 2 }		/* alpha^254 * alpha^2 = alpha */
	};
	struct test1 *p;
	int error = 0;
	int i;
	unsigned int res;

	p = test_v;
	for (i = 0; i < sizeof(test_v)/sizeof(struct test1); i++) {
		res = gf_mult(f, p->a, p->b);
		if (res != p->a_by_b) {
			fprintf(stderr, TAG ": "
			    "gf_mult(0x%x, 0x%x) expected 0x%x actual 0x%x\n",
			    p->a, p->b, p->a_by_b, res);
			error = 1;
		}
		p++;
	}
	if (error)
		exit(1);
}

static void test_gen_uat(void)
{
	struct gf field;
	int rc;

	/*
	 * Test if generation of UAT's field succeeds. Obviously, we don't
	 * have a sample, but at least this runs validity checks.
	 */
	rc = gf_init(&field, GF256_POLY_UAT);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}
	gf_fin(&field);
}

static void test_gen_gen_qrc(void)
{
	/*
	 * For some reason, the poly from Lin&Costello is popular,
	 * and samples for its generator are easy to find.
	 * This particular poly is used by QR-codes, apparently.
	 */
	static unsigned char sample[5] = { 0x01, 0x0f, 0x36, 0x78, 0x40 };
	test_gen_gen(0x11d, 0, 5, sample);
}

static void test_gen_gen_uat(void)
{
	/*
	 * Finding the poly sample for UAT is basically impossible,
	 * so we're using the actual poly that David Carr's receiver
	 * calculates in its operation. Its length is 21, even though
	 * the spec calls for roots for power of alpha from 120 to 139
	 * inclusive (20 roots). It's a mystery why this works.
	 */
	static unsigned char sample[21] = {
	    0x01, 0x9b, 0x91, 0x8c, 0x91, 0xe1, 0x4f, 0x0c, 0x7c, 0x91,
	    0x6c, 0x3a, 0xa2, 0x8e, 0x42, 0xcb, 0x37, 0x80, 0x7b, 0xb4,
	    0x62
	};
	test_gen_gen(GF256_POLY_UAT, 120, 21, sample);
}

static void test_gen_gen(unsigned int poly, int rpow, int len,
    const unsigned char *sample)
{
	int rpow_end = rpow + len - 1;
	struct gf field;
	unsigned char *buf;
	int rc;

	buf = malloc(len + 2);
	if (!buf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}

	rc = gf_init(&field, poly);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n", poly, rc);
		exit(1);
	}

	/*
	 * Surround the buffer with tripwire. Obviously it's not going to
	 * catch every concievable memory scribble, but better than nothing.
	 */
	memset(buf, 0xe5, len+2);

	rc = p_gen_gen(&field, buf+1, rpow, rpow_end);
	if (rc != 0) {
		fprintf(stderr, TAG ": p_gen_gen(0x%x,%d,%d) error: %d\n",
		    poly, rpow, rpow_end, rc);
		exit(1);
	}

	if (buf[0] != 0xe5 || buf[len+1] != 0xe5) {
		fprintf(stderr, TAG ": "
		    "p_gen_gen(0x%x,%d,%d) destination overflow\n",
		    poly, rpow, rpow_end);
		exit(1);
	}
#if 0 /* P3 */
	{
		int n;
		for (n = 0; n < len+2; n++) {
			printf(" %02x", buf[n]);
		}
		printf("\n");
	}
#endif
	if (memcmp(buf+1, sample, len) != 0) {
		fprintf(stderr, TAG ": "
		    "p_gen_gen(0x%x,%d,%d) sample mismatch\n",
		    poly, rpow, rpow_end);
		exit(1);
	}

	gf_fin(&field);
	free(buf);
}

static void test_rem_qrc(void)
{
	static unsigned int sample_pp = 0x11d;
	static unsigned char sample_gp[5] = { 0x01, 0x0f, 0x36, 0x78, 0x40 };
	static unsigned char sample_msg[3] = { 0x12, 0x34, 0x56 };
	static unsigned char sample_rem[4] = { 0x37, 0xe6, 0x78, 0xd9 };

	struct gf field;
	int rc;

	rc = gf_init(&field, sample_pp);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	test_rem(&field, sizeof(sample_msg), sample_msg,
	    sample_pp, sizeof(sample_gp), sample_gp, sample_rem);

	gf_fin(&field);
}

static void test_rem_uat1(void)
{
	static unsigned int sample_pp = GF256_POLY_UAT;
	static unsigned char sample_gp[21] = {
	    0x01, 0x9b, 0x91, 0x8c, 0x91, 0xe1, 0x4f, 0x0c, 0x7c, 0x91,
	    0x6c, 0x3a, 0xa2, 0x8e, 0x42, 0xcb, 0x37, 0x80, 0x7b, 0xb4,
	    0x62
	};
	static unsigned char sample_msg[72] = {
	    0x32, 0x15, 0xfb, 0x68, 0x9a, 0x02, 0xa1, 0x90, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00
	};
	static unsigned char sample_rem[20] = {
	    0xc5, 0xb7, 0x01, 0x05, 0x22, 0xee, 0x4e, 0xcf, 0x7e, 0xb9,
	    0xf7, 0x97, 0xce, 0x8c, 0xf7, 0x7e, 0xcb, 0x7e, 0x99, 0xcc
	};

	struct gf field;
	int rc;

	rc = gf_init(&field, sample_pp);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	test_rem(&field, sizeof(sample_msg), sample_msg,
	    sample_pp, sizeof(sample_gp), sample_gp, sample_rem);

	gf_fin(&field);
}

static void test_rem_uat2(void)
{
	static unsigned int sample_pp = GF256_POLY_UAT;
	static unsigned char sample_gp[21] = {
	    0x01, 0x9b, 0x91, 0x8c, 0x91, 0xe1, 0x4f, 0x0c, 0x7c, 0x91,
	    0x6c, 0x3a, 0xa2, 0x8e, 0x42, 0xcb, 0x37, 0x80, 0x7b, 0xb4,
	    0x62
	};
	static unsigned char sample_msg[72] = {
	    0x32, 0x15, 0xfb, 0x68, 0x9a, 0x02, 0xb3, 0x90, 0x51, 0x00,
	    0x00, 0x2d, 0x0f, 0xc5, 0x68, 0x82, 0x10, 0x00, 0x00, 0x00,
	    0xff, 0x25, 0xce, 0x81, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xef,
	    0xd3, 0x15, 0x01, 0x1f, 0x02, 0x2d, 0x01, 0x1f, 0x09, 0x00,
	    0xb9, 0xb9, 0x23, 0xb5, 0x88, 0xdc, 0xb7, 0x7f, 0x63, 0xae,
	    0x54, 0xdc, 0xb7, 0x75, 0x03, 0x4a, 0xdc, 0xdc, 0xb6, 0xbd,
	    0xa3, 0x4b, 0x74, 0xdc, 0xb6, 0xc2, 0xa3, 0x09, 0xf0, 0xdc,
	    0xb9, 0xf4
	};
	static unsigned char sample_rem[20] = {
	    0x51, 0xd9, 0x87, 0xd6, 0x34, 0xbe, 0xec, 0x6d, 0x7e, 0xdf,
	    0xd4, 0xb9, 0x45, 0x4a, 0x92, 0xc7, 0x5a, 0xa2, 0x04, 0x8f
	};

	struct gf field;
	int rc;

	rc = gf_init(&field, sample_pp);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	test_rem(&field, sizeof(sample_msg), sample_msg,
	    sample_pp, sizeof(sample_gp), sample_gp, sample_rem);

	gf_fin(&field);
}

/*
 * This is the ADS-B Long packet, for which we do not have a sample
 * generator polynomial, so we generate it.
 */
static void test_rem_uat3(void)
{
	static unsigned int sample_pp = GF256_POLY_UAT;
	static unsigned char sample_gp[15];
	static unsigned char sample_msg[34] = {
	    0x0b, 0x9a, 0x08, 0x6f, 0x32, 0x1c, 0x29, 0x68, 0x6a, 0xd0,
	    0x20, 0x66, 0x02, 0xf8, 0x13, 0xc1, 0x51, 0x05, 0xc4, 0xe6,
	    0xc4, 0xe6, 0xc4, 0x0a, 0x12, 0x82, 0x03, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00
	};
	static unsigned char sample_rem[14] = {
	    0x9e, 0x5f, 0x81, 0xe2, 0x2b, 0x70, 0xd8, 0x8a, 0x3b, 0x0f,
	    0x3e, 0x2c, 0xec, 0x7d
	};

	struct gf field;
	int rc;

	rc = gf_init(&field, sample_pp);
	if (rc != 0) {
		fprintf(stderr, TAG ": gf_init(0x%x) error: %d\n",
		    GF256_POLY_UAT, rc);
		exit(1);
	}

	rc = p_gen_gen(&field, sample_gp, 120, 134);
	if (rc != 0) {
		fprintf(stderr, TAG ": p_gen_gen(0x%x,%d,%d) error: %d\n",
		    GF256_POLY_UAT, 120, 134, rc);
		exit(1);
	}

	test_rem(&field, sizeof(sample_msg), sample_msg,
	    sample_pp, sizeof(sample_gp), sample_gp, sample_rem);

	gf_fin(&field);
}

/*
 * Since we have tested gen_gen above, we should be able to generate
 * what is taken by gpoly[] argument from ppoly and gplen. But we focus
 * on testing p_rem() for now and use pre-cooked poly from samples above.
 */
static void test_rem(struct gf *f, int mlen, const unsigned char *msg,
    unsigned int ppoly, int gplen, const unsigned char *gpoly,
    const unsigned char *sample)
{
	int rlen = gplen - 1;
	unsigned char *buf;

	buf = malloc(rlen + 2);
	if (!buf) {
		fprintf(stderr, TAG ": No core\n");
		exit(1);
	}

	/*
	 * Surround the buffer with tripwire. Obviously it's not going to
	 * catch every concievable memory scribble, but better than nothing.
	 */
	memset(buf, 0xe5, rlen+2);

	p_rem(f, buf+1, rlen, mlen, msg, gpoly);

	if (buf[0] != 0xe5 || buf[rlen+1] != 0xe5) {
		fprintf(stderr, TAG ": "
		    "p_rem(0x%x,%d,...) destination overflow\n",
		    ppoly, rlen);
		exit(1);
	}
#if 0 /* P3 */
	{
		int n;
		for (n = 0; n < rlen+2; n++) {
			printf(" %02x", buf[n]);
		}
		printf("\n");
	}
#endif
	if (memcmp(buf+1, sample, rlen) != 0) {
		fprintf(stderr, TAG ": "
		    "p_rem(0x%x,%d,...) sample mismatch\n",
		    ppoly, rlen);
		exit(1);
	}

	free(buf);
}
