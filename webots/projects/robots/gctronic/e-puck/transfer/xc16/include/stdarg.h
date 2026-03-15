#ifndef _STDARG_H_
#define _STDARG_H_

/*	Macros for accessing variable arguments */

typedef void *va_list;

#define va_arg(AP, TYPE)	\
	(AP = (void *) ((char *) (AP) - __va_rounded_size (TYPE)), \
        *((TYPE *) (void *) ((char *) (AP) )))
#define va_copy(apd, aps)	(void)((apd) = (aps))
#define va_end(ap)		(void)0
#define va_start(AP, LASTARG) AP=(void *) __builtin_next_arg (LASTARG)
#define __va_rounded_size(TYPE) \
        (((sizeof (TYPE) + sizeof (int) - 1) / sizeof (int)) * sizeof (int))

#endif /* _STDARG_H_ */
