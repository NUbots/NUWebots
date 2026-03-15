#ifndef _STRING_H_
#define _STRING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* String functions */

#ifndef _STDDEF_H_
#include <stddef.h>
#endif /* _STDDEF_H_ */

extern void *	memcpy(void *, const void *, size_t);
extern void *	memmove(void *, const void *, size_t);
extern void *	memset(void *, int, size_t);

extern char *	strcat(char *, const char *);
extern char *	strcpy(char *, const char *);
extern char *	strncat(char *, const char *, size_t);
extern char *	strncpy(char *, const char *, size_t);
extern char *	strdup(const char *);
extern char *	strtok(char *, const char *);

extern int	memcmp(const void *, const void *, size_t);
extern int	strcmp(const char *, const char *);
extern int	strncmp(const char *, const char *, size_t);
extern  void *	memchr(const void *, int, size_t);
extern size_t	strcspn(const char *, const char *);
extern  char *	strpbrk(const char *, const char *);
extern size_t	strspn(const char *, const char *);
extern  char *	strstr(const char *, const char *);
extern char *	strerror(int);
extern size_t	strlen(const char *);
extern  char *	strchr(const char *, int);
extern  char *	strrchr(const char *, int);

#ifdef __cplusplus
}
#endif
#endif /* _STRING_H_ */
