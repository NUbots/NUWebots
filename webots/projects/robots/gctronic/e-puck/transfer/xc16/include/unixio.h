#ifndef _UNIXIO_H_
#define _UNIXIO_H_

/*
 *	Declarations for Unix style low-level I/O functions.
 */

#ifndef _STDDEF_H_
#include <stddef.h>
#endif /* _STDDEF_H_ */

extern int	errno;			/* system error number */

//extern int	open(const char *, int, ...);
extern int open(const char *, unsigned int, unsigned int);
extern int	close(int);
extern long	lseek(int, long, int);
extern size_t	read(int, void *, size_t);
extern size_t	write(int, const void *, size_t);
extern int creat(const char *, int);

#endif /* _UNIXIO_H_ */
