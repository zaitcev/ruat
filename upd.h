/*
 * The "update thing": a device used to keep an average
 *
 * This is slightly more generic but a slightly slower version
 * with a division by a variable size.
 */

#define AVG_UPD_P(pcur, sub, p)  { *(pcur) -= (sub);  *(pcur) += (p); }

struct upd {
	int *vec;
	int cur;
	int len;		// number of entries in vec[]
	unsigned int x;
};

#define UPD_CUR(up) ((up)->cur / (up)->len)

int upd_init(struct upd *up, int length);
int upd_ate(struct upd *up, int p);
void upd_fini(struct upd *up);
