#ifndef _SIGNAL
#define _SIGNAL

#ifndef _FULL_SIGNAL_IMPLEMENTATION
#define SIGABRT	1
#define	SIGIOT	1
#define SIGINT	2
#define SIGILL	3
#define SIGFPE	4
#define SIGSEGV	5
#define SIGTERM	6
#define _NSIG	6
#else		//full signal implementation
#define	SIGHUP	1	/* hangup    (not used by terminal driver) */
#define	SIGINT	2	/* interrupt (^C or BREAK) */
#define	SIGQUIT	3	/* quit      (^\) */
#define	SIGILL	4	/* illegal instruction (not reset when caught) */
#define	SIGTRAP	5	/* trace trap (not reset when caught) */
#define	SIGIOT	6	/* IOT instruction */
#define SIGABRT	6	/* a synonym of SIGIOT */
#define	SIGEMT	7	/* EMT instruction */
#define	SIGMSG	7	/* message received */
#define	SIGFPE	8	/* floating point exception */
#define	SIGKILL	9	/* kill (cannot be caught or ignored) */
#define	SIGBUS	10	/* bus error */
#define	SIGSEGV	11	/* segmentation violation */
#define	SIGSYS	12	/* bad argument to system call */
#define	SIGPIPE	13	/* write on a pipe with no one to read it */
#define	SIGALRM	14	/* alarm clock */
#define	SIGTERM	15	/* software termination signal from kill */
#define _NSIG	15
#endif

#define	SIG_DFL	((void (*)(int))0)	/* default action is to exit */
#define	SIG_IGN	((void (*)(int))1)	/* ignore them */
#define SIG_ERR ((void (*)(int))-1)

#ifdef __cplusplus
extern "C" {
#endif

typedef int	sig_atomic_t;

extern void (*	signal(int, void (*)(int)))(int);
extern int raise(int);

#ifdef __cplusplus
}
#endif
#endif
