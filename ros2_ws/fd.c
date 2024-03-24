/* This file was automatically generated by CasADi 3.6.5.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) fd_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};

/* fd:(i0[3],i1)->(o0[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5;
  a0=arg[0]? arg[0][1] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=-1.9187435146469209e+01;
  a1=-6.7853808000000015e-01;
  a2=arg[0]? arg[0][0] : 0;
  a3=sin(a2);
  a3=(a1*a3);
  a0=(a0*a3);
  a3=1.9187435146469209e+01;
  a4=arg[1]? arg[1][0] : 0;
  a5=(a3*a4);
  a0=(a0-a5);
  if (res[0]!=0) res[0][1]=a0;
  a2=sin(a2);
  a1=(a1*a2);
  a3=(a3*a1);
  a1=2.1918743514646923e+02;
  a1=(a1*a4);
  a3=(a3+a1);
  if (res[0]!=0) res[0][2]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int fd(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fd_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fd_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fd_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fd_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fd_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fd_incref(void) {
}

CASADI_SYMBOL_EXPORT void fd_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fd_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int fd_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real fd_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fd_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fd_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fd_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fd_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fd_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int fd_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
