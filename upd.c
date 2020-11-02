/*
 * The "update thing": a device used to keep an average
 */

#include <stdlib.h>
#include <string.h>

#include "upd.h"

int upd_init(struct upd *up, int length)
{
	void *p;

	p = malloc(length * sizeof(int));
	if (!p)
		return -1;
	memset(up, 0, sizeof(struct upd));
	up->len = length;
	up->vec = p;
	return 0;
}

int upd_ate(struct upd *up, int p)
{
	int sub;
	unsigned int x;

	x = up->x;
	sub = up->vec[x];
	up->vec[x] = p;
	up->x = (x + 1) % up->len;

	// AVG_UPD_P(&up->cur, sub, p);
	up->cur -= sub;
	up->cur += p;

	return up->cur / up->len;
}

void upd_fini(struct upd *up)
{
	free(up->vec);
	up->vec = NULL;
}
