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
static void test_gen_gen(void);

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
#if 0
	char buf[9];

	printf("%ld %ld %ld\n",
	    sizeof(gf256_8_4_3_2_0),
	    sizeof(gf256_8_4_3_2_0[0]),
	    sizeof(gf256_8_4_3_2_0[0][0]));

	memcpy(buf, gf256_8_4_3_2_0[8], 8);
	buf[8] = 0;
	printf("%s\n", buf);
#endif
	test_gen_lc();
	test_gen_uat();
	test_gen_gen();
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

static void test_gen_gen(void)
{
	/*
	 * For some reason, the poly from Lin&Costello is popular,
	 * and samples for its generator are easy to find.
	 */
	static unsigned char sample[5] = { 0x01, 0x0f, 0x36, 0x78, 0x40 };

	struct gf field;
	unsigned char buf[7];
	int rc;

	gf_init(&field, 0x11d);

	/*
	 * Surround the buffer with tripwire. Obviously it's not going to
	 * catch every concievable memory scribble, but better than nothing.
	 */
	memset(buf, 0xe5, 7);

	rc = p_gen_gen(&field, buf+1, 0, 4);
	if (rc != 0) {
		fprintf(stderr, TAG ": p_gen_gen(0x%x,0,4) error: %d\n",
		    GF256_POLY_LC, rc);
		exit(1);
	}

	if (buf[0] != 0xe5 || buf[6] != 0xe5) {
		fprintf(stderr, TAG ": "
		    "p_gen_gen(0x%x,0,4) destination overflow\n",
		    GF256_POLY_LC);
		exit(1);
	}
	if (memcmp(buf+1, sample, 5) != 0) {
		fprintf(stderr, TAG ": "
		    "p_gen_gen(0x%x,0,4) sample mismatch\n",
		    GF256_POLY_LC);
#if 0
		int i;
		for (i = 1; i < 6; i++) {
			printf(" %02x", buf[i]);
		}
		printf("\n");
#endif
		exit(1);
	}

	gf_fin(&field);
}
