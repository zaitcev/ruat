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

/*
 * Both a and b are in binary (tuple) representation, of course.
 */
unsigned int gf_add(struct gf *f, unsigned int a, unsigned int b)
{
	return a ^ b;
}

unsigned int gf_mult(struct gf *f, unsigned int a, unsigned int b)
{
	unsigned int a_power, b_power, sum_power;

	if (a == 0 || b == 0)
		return 0;
	a_power = f->index[a] - 1;
	b_power = f->index[b] - 1;

	sum_power = a_power + b_power;
	if (sum_power < 255)
		return f->field[sum_power + 1];
	sum_power -= 255;
	return f->field[sum_power + 1];
}

/*
 * Generate a generator polynomial. It is usually described as a multiplication
 * sequence of g(x) = mult{i=1..2t}(X - alpha^i). However, UAT specifies roots
 * starting with alpha^120 instead of 1.
 *
 *   f: pointer to the field
 *   dst: output of size [end_power-start_power+1]
 *   start_power: power of first root - may be 0
 *   end_power: power of last root
 */
int p_gen_gen(struct gf *f, unsigned char *dst, int start_power, int end_power)
{
	int i, j;
	unsigned char arg1[255];
	unsigned char arg2[2];

	if (start_power < 0 ||
	    start_power >= 255 ||
	    end_power < start_power ||
	    end_power >= 255) {
		return -1;
	}

	/* Seed the first member of the polynomial, (X - alpha^0). */
	for (j = 0; j < (end_power-start_power)+1; j++) {
		dst[j] = 0xff;
	}
	dst[end_power-start_power] = 1;

	for (i = start_power; i < end_power; i++) {
		for (j = 0; j < (i-start_power)+1; j++) {
			arg1[j] = dst[(end_power-i) + j];
		}

		arg2[0] = 1;			/* X */
		arg2[1] = f->field[i+1];

		p_mul(f, &dst[end_power-1 - i],
		    1 + i-start_power, arg1,
		    2, arg2);
	}
	return 0;
}

/*
 * Multiply 2 polynomials. Note that these are polynomials with coefficients
 * that are members of GF(256). The power is the position, with leftmost
 * being the greatest. Thus p[len-1] is the constant (power=0). Kinda
 * backwards, but traditional. These polynomials have nothing to do with
 * the binary polynomials that underpin the field itself (e.g. the primitive).
 *
 *   f: pointer to the field
 *   dst: product of size [alen + blen - 1]
 *   alen: length of pa[]
 *   blen: length of pb[]
 *   pa: polynomial a with max power of X ^ (alen-1) at pa[0]
 *   pb: polynomial a with max power of X ^ (blen-1) at pb[0]
 */
void p_mul(struct gf *f, unsigned char *dst,
    int alen, unsigned char *pa, int blen, unsigned char *pb)
{
	int i, j;
	unsigned char pm;

	for (i = 0; i < alen+blen-1; i++)
		dst[i] = 0;
	for (i = 0; i < alen; i++) {
		for (j = 0; j < blen; j++) {
			pm = gf_mult(f, pa[i], pb[j]);
			dst[i+j] = gf_add(f, dst[i+j], pm);
		}
	}
}

/*
 * Over GF(256) f, divide polynomial a[alen] by polynomial b[len],
 * and store the remainder in r[len].
 */
void p_rem(struct gf *f, unsigned char *r, int len, int alen,
    const char *a, const char *b)
{
	int i;
	/* XXX */
	for (i = 0; i < len; i++)
		r[0] = 0xff;
	return;
}
