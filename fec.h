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
