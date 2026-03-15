/*
 *	Assertion - use liberally for debugging. Defining NDEBUG
 *	turns assertions off.
 *	assert(exp) where exp is non-zero does nothing, while
 *	assert(exp) where exp evaluates to zero aborts the program
 *	with a message.
 *
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifndef NDEBUG

#ifndef __mkstr__
#define __mkstr__(exp)  #exp
#endif /* __mkstr__ */

#undef assert

#ifndef __ASSERT_FUNC
#if defined __cplusplus && defined __XC32
#define __ASSERT_FUNC __PRETTY_FUNCTION__
#elif __STDC_VERSION__ >= 199901L
#define __ASSERT_FUNC __func__
#elif (__XC32_VERSION >= 1000) || (__XC30_VERSION >= 1000)
#define __ASSERT_FUNC __FUNCTION__
#else
#define __ASSERT_FUNC ((char *)0)
#endif
#endif /* __ASSERT_FUNC */

extern void __attribute__((noreturn)) _fassert(int, const char *, const char *, const char*);
#define __assert(line,file,expression,func)  \
  _fassert(line,file,expression,func)
#define assert(expr)     \
  ((void)((expr) ? 0 : (__assert (__LINE__, __FILE__, #expr, __ASSERT_FUNC), 0)))

#else /* NDEBUG */
#define assert(exp) ((void)0)
#endif /* NDEBUG */

#ifdef __cplusplus
}
#endif


