#ifndef _ERRNO_H_
#define _ERRNO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define	EINVAL	1
#define	ENOENT	2
#define	NOPATH	3
#define	EMFILE	4
#define	EACCESS	5
#define	EBADF	6
#define	EMCBD	7
#define	ENOMEM	8
#define	EIMBA	9
#define	EINVENV	10
#define	ENOEXEC	11
#define	EPERM	12
#define	EDATA	13
#define	EDRIVE	15
#define	ECURDIR	16
#define	EXDEV	17
#define	ENFILE	18

#define	EDOM	33	/* Domain error */
#define	ERANGE	34	/* Range error */

extern int errno;

#ifdef __cplusplus
}
#endif

#endif /* _ERRNO_H_ */
