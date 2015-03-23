/*
 * fec.h: definitions for the FEC, including GF arithmetic
 */

/*
 * Our field is always the size GF(265), so both field[] and index[] are
 * byte arrays of 265 elements.
 */
struct gf {
	unsigned char *field;	/* "power" representation */
	unsigned char *index;	/* index of field[] */
};

int gf_init(struct gf *f, unsigned short gen_poly);
void gf_fin(struct gf *f);
unsigned int gf_add(struct gf *f, unsigned int a, unsigned int b);
unsigned int gf_mult(struct gf *f, unsigned int a, unsigned int b);
int p_gen_gen(struct gf *f, unsigned char *dst, int start_power, int end_power);
void p_mul(struct gf *f, unsigned char *dst,
    int alen, unsigned char *pa, int blen, unsigned char *pb);
void p_rem(struct gf *f, unsigned char *rem, int len,
    int alen, const unsigned char *pa, const unsigned char *div);
