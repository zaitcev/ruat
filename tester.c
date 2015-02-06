/*
 * test
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "tester"

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
 * Generate a GF(256) from the supplied primitive polynomial
 */
static void gen256(unsigned char dst[256], unsigned int prim_poly)
{
	unsigned int identity;
	int i;
	unsigned int v;

	/*
	 * The primitive polynomial must have x^8 set.
	 */
	assert((prim_poly & 0x100) != 0);

	/*
	 * First, precompute the identity for alpha^8.
	 * Apparently, this is done by adding alpha^8 (0x100 modulo 2) to
	 * both sides of the poly, and then using alpha^i + alpha^i == 0.
	 * Result is, we just chop off the extra bit.
	 */
	identity = prim_poly & 0xff;

	/*
	 * Then, set the elements of GF(2), which also serve as certain
	 * elements for addition and multiplication. Either way, constants.
	 */
	dst[0] = 0;
	dst[1] = 1;

	/*
	 * Now, run the generator machinery for each extension element.
	 *
	 * The key observation is that each next step can be a multiplication:
	 *   alpha^(i+1) == alpha * alpha^i
	 * But, if we open the parenthesii, it amounts to incrementing the
	 * power of each member of the poly by 1. Which is shift left by 1.
	 * And, if the shift produces an alpha^8, we substitute identity
	 * and add (xor, of course).
	 *
	 * Finally, shift happens to work for smaller powers of alpha, too.
	 * So, we start from i=2 instead of i=8.
	 */
	v = 1;
	for (i = 2; i < 256; i++) {
		v <<= 1;
		if (v & 0x100) {
			v &= 0xff;
			v ^= identity;
		}
		dst[i] = v;
	}
}

/*
 * Check a GF(256) field for uniqueness
 *
 * Returns true if unique, false otherwise.
 */
static int check_unique_256(unsigned char field[256])
{
	int error = 0;
	short int index[256];	/* indices (or alpha power + 1) by element */
	int i;
	unsigned int e;
	int x;

	for (i = 0; i < 256; i++)
		index[i] = -1;

	for (i = 0; i < 256; i++) {
		e = field[i];
		x = index[e];
		if (x != -1) {
			fprintf(stderr, TAG ": dup [%d]:0x%02x [%d]:0x%02x\n",
			    i, e, x, field[x]);
			error = 1;
		}
		index[e] = i;
	}

	return error == 0;
}

int main(int argc, char **argv)
{
	unsigned char field[256], sample[256];
	char *p;
	unsigned int v;
	int i, j;
	int error;

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

	/*
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

	gen256(field, GF256_POLY_LC);

	error = 0;
	for (i = 0; i < 256; i++) {
		/*
		 * We were too lazy to type in the whole sample field from
		 * the book, so we only compare to the known elements.
		 */
		if (gf256_8_4_3_2_0[i][0] != 0) {
			if (sample[i] != field[i]) {
				fprintf(stderr, TAG ": "
				    "mismatch [%d] 0x%02x 0x%02x\n",
				    i, sample[i], field[i]);
				error = 1;
			}
		}
	}
	if (error)
		exit(1);

	if (!check_unique_256(field)) {
		fprintf(stderr,
		    TAG ": Generated GF(256) over 0x%03x failed uniqueness\n",
		    GF256_POLY_LC);
		exit(1);
	}

	return 0;
}
