/* ========================================================================== */
/* === SuiteSparse_config =================================================== */
/* ========================================================================== */

/* Configuration file for SuiteSparse: a Suite of Sparse matrix packages
 * (AMD, COLAMD, CCOLAMD, CAMD, CHOLMOD, UMFPACK, CXSparse, and others).
 *
 * SuiteSparse_config.h provides the definition of the long integer.  On most
 * systems, a C program can be compiled in LP64 mode, in which long's and
 * pointers are both 64-bits, and int's are 32-bits.  Windows 64, however, uses
 * the LLP64 model, in which int's and long's are 32-bits, and long long's and
 * pointers are 64-bits.
 *
 * SuiteSparse packages that include long integer versions are
 * intended for the LP64 mode.  However, as a workaround for Windows 64
 * (and perhaps other systems), the long integer can be redefined.
 *
 * If _WIN64 is defined, then the __int64 type is used instead of long.
 *
 * The long integer can also be defined at compile time.  For example, this
 * could be added to SuiteSparse_config.mk:
 *
 * CFLAGS = -O -D'SuiteSparse_long=long long' \
 *  -D'SuiteSparse_long_max=9223372036854775801' -D'SuiteSparse_long_idd="lld"'
 *
 * This file defines SuiteSparse_long as either long (on all but _WIN64) or
 * __int64 on Windows 64.  The intent is that a SuiteSparse_long is always a
 * 64-bit integer in a 64-bit code.  ptrdiff_t might be a better choice than
 * long; it is always the same size as a pointer.
 *
 * This file also defines the SUITESPARSE_VERSION and related definitions.
 *
 * Copyright (c) 2012, Timothy A. Davis.  No licensing restrictions apply
 * to this file or to the SuiteSparse_config directory.
 * Author: Timothy A. Davis.
 */

#ifndef SUITESPARSE_CONFIG_H
#define SUITESPARSE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "thirdParty/JCQP/glob_opts.h"
#include <limits.h>
#include <stdlib.h>

/* ========================================================================== */
/* === SuiteSparse_long ===================================================== */
/* ========================================================================== */

#ifndef SuiteSparse_long

#define SuiteSparse_long long long
#define SuiteSparse_long_max LONG_MAX
#define SuiteSparse_long_idd "ld"

#define SuiteSparse_long_id "%" SuiteSparse_long_idd
#endif

/* ========================================================================== */
/* === SuiteSparse_config parameters and functions ========================== */
/* ========================================================================== */

/* SuiteSparse-wide parameters are placed in this struct.  It is meant to be
   an extern, globally-accessible struct.  It is not meant to be updated
   frequently by multiple threads.  Rather, if an application needs to modify
   SuiteSparse_config, it should do it once at the beginning of the application,
   before multiple threads are launched.

   The intent of these function pointers is that they not be used in your
   application directly, except to assign them to the desired user-provided
   functions.  Rather, you should use the
 */

struct SuiteSparse_config_struct
{
    void *(*malloc_func) (size_t) ;             /* pointer to malloc */
    void *(*realloc_func) (void *, size_t) ;    /* pointer to realloc */
    void (*free_func) (void *) ;                /* pointer to free */
#if defined PYTHON
    void (*printf_func) (const char *, ...) ;    /* pointer to printf (in Python it returns void)*/
#elif defined R_LANG
    void (*printf_func) (const char *, ...) ;    /* pointer to printf (in R it returns void)*/
#else
    int (*printf_func) (const char *, ...) ;    /* pointer to printf */
#endif
    c_float (*hypot_func) (c_float, c_float) ;     /* pointer to hypot */
    int (*divcomplex_func) (c_float, c_float, c_float, c_float, c_float *, c_float *);
} ;

extern struct SuiteSparse_config_struct SuiteSparse_config ;

void *SuiteSparse_malloc    /* pointer to allocated block of memory */
(
    size_t nitems,          /* number of items to malloc (>=1 is enforced) */
    size_t size_of_item     /* sizeof each item */
) ;

void *SuiteSparse_calloc    /* pointer to allocated block of memory */
(
    size_t nitems,          /* number of items to calloc (>=1 is enforced) */
    size_t size_of_item     /* sizeof each item */
) ;

void *SuiteSparse_realloc   /* pointer to reallocated block of memory, or
                               to original block if the realloc failed. */
(
    size_t nitems_new,      /* new number of items in the object */
    size_t nitems_old,      /* old number of items in the object */
    size_t size_of_item,    /* sizeof each item */
    void *p,                /* old object to reallocate */
    int *ok                 /* 1 if successful, 0 otherwise */
) ;

void *SuiteSparse_free      /* always returns NULL */
(
    void *p                 /* block to free */
) ;

void SuiteSparse_tic    /* start the timer */
(
    c_float tic [2]      /* output, contents undefined on input */
) ;

c_float SuiteSparse_toc  /* return time in seconds since last tic */
(
    c_float tic [2]      /* input: from last call to SuiteSparse_tic */
) ;

c_float SuiteSparse_time  /* returns current wall clock time in seconds */
(
    void
) ;

/* returns sqrt (x^2 + y^2), computed reliably */
c_float SuiteSparse_hypot (c_float x, c_float y) ;

/* complex division of c = a/b */
int SuiteSparse_divcomplex
(
    c_float ar, c_float ai,	/* real and imaginary parts of a */
    c_float br, c_float bi,	/* real and imaginary parts of b */
    c_float *cr, c_float *ci	/* real and imaginary parts of c */
) ;

/* OSQP: disabling this timing code */
#define NTIMER

/* determine which timer to use, if any */
#ifndef NTIMER
#ifdef _POSIX_C_SOURCE
#if    _POSIX_C_SOURCE >= 199309L
#define SUITESPARSE_TIMER_ENABLED
#endif
#endif
#endif

/* SuiteSparse printf macro */
#define SUITESPARSE_PRINTF(params) \
{ \
    if (SuiteSparse_config.printf_func != NULL) \
    { \
        (void) (SuiteSparse_config.printf_func) params ; \
    } \
}

/* ========================================================================== */
/* === SuiteSparse version ================================================== */
/* ========================================================================== */

/* SuiteSparse is not a package itself, but a collection of packages, some of
 * which must be used together (UMFPACK requires AMD, CHOLMOD requires AMD,
 * COLAMD, CAMD, and CCOLAMD, etc).  A version number is provided here for the
 * collection itself.  The versions of packages within each version of
 * SuiteSparse are meant to work together.  Combining one package from one
 * version of SuiteSparse, with another package from another version of
 * SuiteSparse, may or may not work.
 *
 * SuiteSparse contains the following packages:
 *
 *  SuiteSparse_config version 4.5.3 (version always the same as SuiteSparse)
 *  AMD             version 2.4.6
 *  BTF             version 1.2.6
 *  CAMD            version 2.4.6
 *  CCOLAMD         version 2.9.6
 *  CHOLMOD         version 3.0.11
 *  COLAMD          version 2.9.6
 *  CSparse         version 3.1.9
 *  CXSparse        version 3.1.9
 *  GPUQREngine     version 1.0.5
 *  KLU             version 1.3.8
 *  LDL             version 2.2.6
 *  RBio            version 2.2.6
 *  SPQR            version 2.0.7
 *  SuiteSparse_GPURuntime  version 1.0.5
 *  UMFPACK         version 5.7.6
 *  MATLAB_Tools    various packages & M-files
 *  xerbla          version 1.0.3
 *
 * Other package dependencies:
 *  BLAS            required by CHOLMOD and UMFPACK
 *  LAPACK          required by CHOLMOD
 *  METIS 5.1.0     required by CHOLMOD (optional) and KLU (optional)
 *  CUBLAS, CUDART  NVIDIA libraries required by CHOLMOD and SPQR when
 *                  they are compiled with GPU acceleration.
 */

int SuiteSparse_version     /* returns SUITESPARSE_VERSION */
(
    /* output, not defined on input.  Not used if NULL.  Returns
       the three version codes in version [0..2]:
       version [0] is SUITESPARSE_MAIN_VERSION
       version [1] is SUITESPARSE_SUB_VERSION
       version [2] is SUITESPARSE_SUBSUB_VERSION
       */
    int version [3]
) ;

/* Versions prior to 4.2.0 do not have the above function.  The following
   code fragment will work with any version of SuiteSparse:

   #ifdef SUITESPARSE_HAS_VERSION_FUNCTION
   v = SuiteSparse_version (NULL) ;
   #else
   v = SUITESPARSE_VERSION ;
   #endif
*/
#define SUITESPARSE_HAS_VERSION_FUNCTION

#define SUITESPARSE_DATE "May 4, 2016"
#define SUITESPARSE_VER_CODE(main,sub) ((main) * 1000 + (sub))
#define SUITESPARSE_MAIN_VERSION 4
#define SUITESPARSE_SUB_VERSION 5
#define SUITESPARSE_SUBSUB_VERSION 3
#define SUITESPARSE_VERSION \
    SUITESPARSE_VER_CODE(SUITESPARSE_MAIN_VERSION,SUITESPARSE_SUB_VERSION)

#ifdef __cplusplus
}
#endif
#endif
