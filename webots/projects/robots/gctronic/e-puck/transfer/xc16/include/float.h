#ifndef _FLOAT_H_
#define _FLOAT_H_

/* Characteristics of floating types */

#define FLT_RADIX       2
#define FLT_ROUNDS      1

/*
 *  These values provide information pertaining to the float type.
 */
#define FLT_EPSILON     __FLT_EPSILON__
#define FLT_MAX         __FLT_MAX__
#define FLT_MIN         __FLT_MIN__
#define FLT_DIG         __FLT_DIG__
#define FLT_MANT_DIG    __FLT_MANT_DIG__
#define FLT_MAX_10_EXP  __FLT_MAX_10_EXP__
#define FLT_MAX_EXP     __FLT_MAX_EXP__
#define FLT_MIN_10_EXP  __FLT_MIN_10_EXP__
#define FLT_MIN_EXP     __FLT_MIN_EXP__

/*
 *  These values provide information pertaining to the double type
 *    The values are dependant upon the presence of the -fno-short-double
 *    compiler option.
 */
#define DBL_EPSILON     __DBL_EPSILON__
#define DBL_MAX         __DBL_MAX__
#define DBL_MIN         __DBL_MIN__
#define DBL_DIG         __DBL_DIG__
#define DBL_MANT_DIG    __DBL_MANT_DIG__
#define DBL_MAX_10_EXP  __DBL_MAX_10_EXP__
#define DBL_MAX_EXP     __DBL_MAX_EXP__
#define DBL_MIN_10_EXP  __DBL_MIN_10_EXP__
#define DBL_MIN_EXP     __DBL_MIN_EXP__

/*
 *  These values provide information pertaining to the long double type.
 */
#define LDBL_EPSILON    __LDBL_EPSILON__
#define LDBL_MAX        __LDBL_MAX__
#define LDBL_MIN        __LDBL_MIN__
#define LDBL_DIG        __LDBL_DIG__
#define LDBL_MANT_DIG   __LDBL_MANT_DIG__
#define LDBL_MAX_10_EXP __LDBL_MAX_10_EXP__
#define LDBL_MAX_EXP    __LDBL_MAX_EXP__
#define LDBL_MIN_10_EXP __LDBL_MIN_10_EXP__
#define LDBL_MIN_EXP    __LDBL_MIN_EXP__

#endif
