#ifndef	_SETJMP_H_
#define	_SETJMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __C30__
#define _JB_LEN 18
typedef unsigned int jmp_buf[_JB_LEN];
#elif defined(__C32_VERSION__) || defined(__XC32) 
#define _JB_LEN 24
typedef int jmp_buf[_JB_LEN];
#endif

extern	int	setjmp(jmp_buf);
extern void	longjmp(jmp_buf, int);

#ifdef __cplusplus
}
#endif
#endif	/* _SETJMP_H_ */
