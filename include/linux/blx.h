/* include/linux/blx.h */

#ifndef _LINUX_BLX_H
#define _LINUX_BLX_H

#define MAX_CHARGINGLIMIT 100

// is 10 - (charginglimit / 10) -> 0..10
int get_cap_level(void);

int get_charginglimit(void);

#endif
