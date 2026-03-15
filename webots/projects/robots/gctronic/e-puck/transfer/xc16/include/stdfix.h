/* Standard Fixed Point include file */

/* Copyright 2013 Microchip Technology Inc */


#ifndef _STDFIX_H_
#define _STDFIX_H_

/* signed short _Fract */
#define SFRACT_FBIT     (15)
#define SFRACT_MIN      (-1.0hr)
#define SFRACT_MAX      (.999969482421875hr)
#define SFRACT_EPSILON  (.000030517578125hr)

/* unsigned short _Fract */
#define USFRACT_FBIT    (15)
#define USFRACT_MIN     (0.0uhr)
#define USFRACT_MAX     (.99969482421875uhr)
#define USFRACT_EPSILON (.00030517578125uhr)

/* signed _Fract */
#define FRACT_FBIT      SFRACT_BIT
#define FRACT_MIN       SFRACT_MIN
#define FRACT_MAX       SFRACT_MAX
#define FRACT_EPSILON   SFRACT_EPSILON

/* unsigned _Fract */
#define UFRACT_FBIT     USFRACT_BIT
#define UFRACT_MIN      USFRACT_MIN
#define UFRACT_MAX      USFRACT_MAX
#define UFRACT_EPSILON  USFRACT_EPSILON

/* signed long _Fract */
#define LFRACT_FBIT     (31)
#define LFRACT_MIN      (-1.0r)
#define LFRACT_MAX      (0.9999999995343387126922607421875lr)
#define LFRACT_EPSILON  (0.0000000004656612873077392578125lr)

/* unsigned long _Fract */
#define ULFRACT_FBIT    (31)
#define ULFRACT_MIN     (0.0ur)
#define ULFRACT_MAX     (0.9999999995343387126922607421875ulr)
#define ULFRACT_EPSILON (0.0000000004656612873077392578125ulr)

/* signed short _Accum */
#define SACCUM_FBIT     (31)
#define SACCUM_IBIT     (9)
#define SACCUM_MIN      (-256.0hk)
#define SACCUM_MAX      (255.9999999995343387126922607421875hk)
#define SACCUM_EPSILON  (0.0000000004656612873077392578125hk)

/* unsigned short _Accum */
#define USACCUM_FBIT    (31)
#define USACCUM_IBIT    (9)
#define USACCUM_MIN     (0.0uhk)
#define USACCUM_MAX     (255.9999999995343387126922607421875uhk)
#define USACCUM_EPSILON (0.0000000004656612873077392578125uhk)

/* signed _Accum */
#define ACCUM_FBIT      SACCUM_FBIT
#define ACCUM_IBIT      SACCUM_IBIT
#define ACCUM_MIN       SACCUM_MIN
#define ACCUM_MAX       SACCUM_MAX
#define ACCUM_EPSILON   SACCUM_EPSILON

/* unsigned _Accum */
#define UACCUM_FBIT     USACCUM_FBIT
#define UACCUM_IBIT     USACCUM_IBIT
#define UACCUM_MIN      USACCUM_MIN
#define UACCUM_MAX      USACCUM_MAX
#define UACCUM_EPSILON  USACCUM_EPSILON

/* signed long _Accum */
#define LACCUM_FBIT     SACCUM_FBIT
#define LACCUM_IBIT     SACCUM_IBIT
#define LACCUM_MIN      SACCUM_MIN
#define LACCUM_MAX      SACCUM_MAX
#define LACCUM_EPSILON  SACCUM_EPSILON

/* unsigned long _Accum */
#define ULACCUM_FBIT    USACCUM_FBIT
#define ULACCUM_IBIT    USACCUM_IBIT
#define ULACCUM_MIN     USACCUM_MIN
#define ULACCUM_MAX     USACCUM_MAX
#define ULACCUM_EPSILON USACCUM_EPSILON

#endif

/* FYI - Fixed Point Constants        */
/* -----------------------------------*/
/* Suffix - Type                      */
/* hr       short _Fract              */
/* uhr      unsigned shoft _Fract     */
/* r        _Fract                    */
/* ur       unsigned _Fract           */
/* lr       long _Fract               */
/* ulr      unsigned long _Fract      */
/* hk       short _Accum              */
/* uhk      unsigned short _Accum     */
/* k        _Accum                    */
/* uk       unsigned _Accum           */
 
