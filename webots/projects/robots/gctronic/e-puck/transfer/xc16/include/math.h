#ifndef _MATH_H_
#define _MATH_H_

#define HUGE_VAL (__DBL_MAX__ * 2.0)

/*
 *   prototypes for the standard C maths functions.  
 */

#if (__DBL_MANT_DIG__ != __FLT_MANT_DIG__)
double acos(double);
double asin(double);
double atan(double);
double atan2(double, double);
double cos(double);
double sin(double);
double tan(double);
double cosh(double);
double sinh(double);
double tanh(double);
double exp(double);
double frexp(double, int *);
double ldexp(double, int);
double log(double);
double log10(double);
double modf(double, double *);
double pow(double, double);
double sqrt(double);
double ceil(double);
double fabs(double);
double floor(double);
double fmod(double, double);
#endif

/*
 *   prototypes for the float maths functions. 
 */
float acosf(float);
float asinf(float);
float atanf(float);
float atan2f(float, float);
float cosf(float);
float sinf(float);
float tanf(float);
float coshf(float);
float sinhf(float);
float tanhf(float);
float expf(float);
float frexpf(float, int *);
float ldexpf(float, int);
float logf(float);
float log10f(float);
#if (__DBL_MANT_DIG__ == __FLT_MANT_DIG__)
/*
 *  Without -fno-short-double, float * and double * are compatable, but a
 *  warning can still be produced.
 */
float modff(float, void *);
#else
float modff(float, float *);
#endif
float powf(float, float);
float sqrtf(float);
float ceilf(float);
float fabsf(float);
float floorf(float);
float fmodf(float, float);

/*
 *    prototypes for long double maths functions.  On dsPIC30, long double
 *      is always 64bits regardless of the setting of the -fno-short-double
 *      and -fshort-double options.
 */
long double acosl(long double);
long double asinl(long double);
long double atanl(long double);
long double atan2l(long double, long double);
long double cosl(long double);
long double sinl(long double);
long double tanl(long double);
long double coshl(long double);
long double sinhl(long double);
long double tanhl(long double);
long double expl(long double);
long double frexpl(long double, int *);
long double ldexpl(long double, int);
long double logl(long double);
long double log10l(long double);
long double modfl(long double, long double *);
long double powl(long double, long double);
long double sqrtl(long double);
long double ceill(long double);
long double fabsl(long double);
long double floorl(long double);
long double fmodl(long double, long double);

/*
 *   macro definitions to ensure that the default functions are correct
 *     for the chosen size of double:  see -fno-short-double and -fshort-double
 */

#if (__DBL_MANT_DIG__ == __FLT_MANT_DIG__)
#define __MPROTO(x)  x ## f
#else
#define __MPROTO(x)  x
#endif


#define acos __MPROTO(acos)
#define asin __MPROTO(asin)
#define atan __MPROTO(atan)
#define atan2 __MPROTO(atan2)
#define cos __MPROTO(cos)
#define sin __MPROTO(sin)
#define tan __MPROTO(tan)
#define cosh __MPROTO(cosh)
#define sinh __MPROTO(sinh)
#define tanh __MPROTO(tanh)
#define exp __MPROTO(exp)
#define frexp __MPROTO(frexp)
#define ldexp __MPROTO(ldexp)
#define log __MPROTO(log)
#define log10 __MPROTO(log10)
#define modf __MPROTO(modf)
#define pow __MPROTO(pow)
#define sqrt __MPROTO(sqrt)
#define ceil __MPROTO(ceil)
#define fabs __MPROTO(fabs)
#define floor __MPROTO(floor)
#define fmod __MPROTO(fmod)

/*
 *  Prototypes for non-standard maths functions; these are part of libm
 *    and will be satisfied by libm-<omf>.a which is automatically included
 *    by the compiler.
 */

unsigned long __udiv3216(unsigned long, unsigned int);
         long __div3216(long, int);

#endif 
