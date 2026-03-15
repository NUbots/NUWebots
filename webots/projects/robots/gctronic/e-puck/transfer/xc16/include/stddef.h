#ifndef	_STDDEF_H_
#define	_STDDEF_H_

typedef	__PTRDIFF_TYPE__ ptrdiff_t;	/* result type of pointer difference */
typedef	__SIZE_TYPE__ size_t;		/* type yielded by sizeof */
typedef __WCHAR_TYPE__ wchar_t;		/* wide char type */

#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)

#ifndef	NULL
#define NULL (0)
#endif	/* NULL */

extern int	errno;			/* system error number */

#endif	/* _STDDEF_H_ */
