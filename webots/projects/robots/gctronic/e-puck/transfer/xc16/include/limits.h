#ifndef _LIMITS_H_
#define _LIMITS_H_

#define MB_LEN_MAX 1

/* char properties */
#define CHAR_BIT   8
#define SCHAR_MAX  0x7f
#define SCHAR_MIN  (-SCHAR_MAX -1)
#define UCHAR_MAX  0xff
#define CHAR_MIN   SCHAR_MIN
#define CHAR_MAX   SCHAR_MAX

/* int properties */
#define SHRT_MAX   0x7fff
#define SHRT_MIN   (-SHRT_MAX -1)
#define USHRT_MAX  0xffff
#define INT_MAX    SHRT_MAX
#define INT_MIN    SHRT_MIN
#define UINT_MAX   USHRT_MAX

/* long properties */
#define LONG_MAX   0x7fffffff
#define LONG_MIN   (-LONG_MAX -1)
#define ULONG_MAX  0xffffffffU

/* long long properties */
#define LLONG_MAX  0x7fffffffffffffffLL
#define LLONG_MIN  (-LLONG_MAX -1)
#define ULLONG_MAX 0xffffffffffffffffULL

#endif
