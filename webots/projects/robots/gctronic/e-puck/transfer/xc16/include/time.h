#ifndef	_TIME_H_
#define _TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _STDDEF_H_
#include <stddef.h>
#endif /* _STDDEF_H_ */

typedef	long	time_t;		/* for representing times in seconds */
typedef	long	clock_t;
struct tm {
	int	tm_sec;
	int	tm_min;
	int	tm_hour;
	int	tm_mday;
	int	tm_mon;
	int	tm_year;
	int	tm_wday;
	int	tm_yday;
	int	tm_isdst;
};

#define	CLOCKS_PER_SEC	1
#define	difftime(t1, t0)	((double)((time_t)(t1)-(time_t)(t0)))

extern int	time_zone;	/* minutes WESTWARD of Greenwich */
				/* this value defaults to 0 since with
				   operating systems like MS-DOS there is
				   no time zone information available */

extern clock_t clock(void);
extern time_t	time(time_t *);	/* seconds since 00:00:00 Jan 1 1970 */
extern int	stime(time_t *);	/* set time */
extern char *	asctime(const struct tm *);	/* converts struct tm to ascii time */
extern char *	ctime(const time_t *);	/* current local time in ascii form */
extern struct tm *	gmtime(const time_t *);	/* Universal time */
extern struct tm *	localtime(const time_t *);	/* local time */
extern time_t		mktime(struct tm *);
extern size_t		strftime(char *, size_t, const char *, const struct tm *);

#ifdef __cplusplus
}
#endif

#endif /* _TIME_H_ */
