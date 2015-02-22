/*
 * fec.c: implementation of FEC, including GF arithmetic
 */
#include <stdlib.h>

#include "fec.h"

static int gen256(unsigned char dst[256], unsigned int prim_poly);
static int genx(unsigned char dst[256], unsigned char field[256]);

/*
 * gf_init: construct the GF(256) field and its index
 *
 *   f: pointer to the field
 *   prim_poly: the primitive polynomial in binary form, x^0 == 1 (LSB)
 *   return: 0 if successfuly created
 *
 * As long as a suitable polynomial is supplied, gf_init() cannot fail
 * thanks to the magic of math. However, we check the argument and
 * self-check in case of bugs. Failing those checks produces an error.
 */
int gf_init(struct gf *f, unsigned short prim_poly)
{
	unsigned char *p;
	int ret;

	ret = -1;
	p = malloc(256);
	if (!p)
		goto no_core_1;
	f->field = p;
	p = malloc(256);
	if (!p)
		goto no_core_2;
	f->index = p;

	if (gen256(f->field, prim_poly) != 0) {
		ret = -2;
		goto err_gen256;
	}

	if (genx(f->index, f->field) != 0) {
		ret = -3;
		goto err_genx;
	}

	return 0;

err_genx:
err_gen256:
	free(f->index);
no_core_2:
	free (f->field);
no_core_1:
	return ret;
}

void gf_fin(struct gf *f)
{
	free(f->index);
	free(f->field);
}

/*
 * Generate a GF(256) from the supplied primitive polynomial
 */
static int gen256(unsigned char dst[256], unsigned int prim_poly)
{
	unsigned int identity;
	int i;
	unsigned int v;

	/*
	 * The primitive polynomial must have x^8 set.
	 */
	if ((prim_poly & 0x100) == 0)
		return -1;

	/*
	 * XXX Verify and assert that alpha (that is, 0x02) is a root
	 * of the primitive polynomial.
	 */

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

	return 0;
}

/*
 * Generate an index for GF(256) field, check for uniqueness
 *
 * Returns true if unique, false otherwise.
 */
static int genx(unsigned char dst[256], unsigned char field[256])
{
	int error = 0;
	short int index[256];	/* indices (or alpha power + 1) by element */
	int i;
	unsigned int e;
	int x;

	/* pre-fill with sentiel */
	for (i = 0; i < 256; i++)
		index[i] = -1;

	/* place index values with checking */
	for (i = 0; i < 256; i++) {
		e = field[i];
		x = index[e];
		if (x != -1) {
			// fprintf(stderr,
			//     TAG ": dup [%d]:0x%02x [%d]:0x%02x\n",
			//     i, e, x, field[x]);
			error = 1;
		} else {
			index[e] = i;
		}
	}

	/* We do not need to check if any sentiels are left in. */

	/* compress into destination */
	for (i = 0; i < 256; i++)
		dst[i] = index[i];

	return (error == 0)? 0: -1;
}