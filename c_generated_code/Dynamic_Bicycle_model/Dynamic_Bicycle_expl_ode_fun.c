/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Dynamic_Bicycle_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* Dynamic_Bicycle_expl_ode_fun:(i0[7],i1,i2[])->(o0[7]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3, w4, w5, w6;
  /* #0: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #1: @1 = cos(@0) */
  w1 = cos( w0 );
  /* #2: @2 = input[0][4] */
  w2 = arg[0] ? arg[0][4] : 0;
  /* #3: @3 = sin(@0) */
  w3 = sin( w0 );
  /* #4: @3 = (@2*@3) */
  w3  = (w2*w3);
  /* #5: @1 = (@1-@3) */
  w1 -= w3;
  /* #6: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #7: @1 = sin(@0) */
  w1 = sin( w0 );
  /* #8: @0 = cos(@0) */
  w0 = cos( w0 );
  /* #9: @0 = (@2*@0) */
  w0  = (w2*w0);
  /* #10: @1 = (@1+@0) */
  w1 += w0;
  /* #11: output[0][1] = @1 */
  if (res[0]) res[0][1] = w1;
  /* #12: @1 = 1 */
  w1 = 1.;
  /* #13: output[0][2] = @1 */
  if (res[0]) res[0][2] = w1;
  /* #14: output[0][3] = @2 */
  if (res[0]) res[0][3] = w2;
  /* #15: @1 = 0.359195 */
  w1 = 3.5919540229885061e-01;
  /* #16: @0 = -6.932 */
  w0 = -6.9320000000000004e+00;
  /* #17: @3 = 0.205 */
  w3 = 2.0499999999999999e-01;
  /* #18: @4 = input[0][6] */
  w4 = arg[0] ? arg[0][6] : 0;
  /* #19: @3 = (@3*@4) */
  w3 *= w4;
  /* #20: @3 = (@2+@3) */
  w3  = (w2+w3);
  /* #21: @5 = 1 */
  w5 = 1.;
  /* #22: @3 = atan2(@3,@5) */
  w3  = atan2(w3,w5);
  /* #23: @5 = input[1][0] */
  w5 = arg[1] ? arg[1][0] : 0;
  /* #24: @3 = (@3-@5) */
  w3 -= w5;
  /* #25: @0 = (@0*@3) */
  w0 *= w3;
  /* #26: @5 = cos(@5) */
  w5 = cos( w5 );
  /* #27: @5 = (@0*@5) */
  w5  = (w0*w5);
  /* #28: @3 = -6.918 */
  w3 = -6.9180000000000001e+00;
  /* #29: @6 = 0.199 */
  w6 = 1.9900000000000001e-01;
  /* #30: @6 = (@6*@4) */
  w6 *= w4;
  /* #31: @2 = (@2-@6) */
  w2 -= w6;
  /* #32: @6 = 1 */
  w6 = 1.;
  /* #33: @2 = atan2(@2,@6) */
  w2  = atan2(w2,w6);
  /* #34: @3 = (@3*@2) */
  w3 *= w2;
  /* #35: @5 = (@5+@3) */
  w5 += w3;
  /* #36: @1 = (@1*@5) */
  w1 *= w5;
  /* #37: @1 = (@1-@4) */
  w1 -= w4;
  /* #38: output[0][4] = @1 */
  if (res[0]) res[0][4] = w1;
  /* #39: output[0][5] = @4 */
  if (res[0]) res[0][5] = w4;
  /* #40: @4 = 11.976 */
  w4 = 1.1976047904191615e+01;
  /* #41: @1 = 0.205 */
  w1 = 2.0499999999999999e-01;
  /* #42: @1 = (@1*@0) */
  w1 *= w0;
  /* #43: @0 = 0.199 */
  w0 = 1.9900000000000001e-01;
  /* #44: @0 = (@0*@3) */
  w0 *= w3;
  /* #45: @1 = (@1-@0) */
  w1 -= w0;
  /* #46: @4 = (@4*@1) */
  w4 *= w1;
  /* #47: output[0][6] = @4 */
  if (res[0]) res[0][6] = w4;
  return 0;
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Dynamic_Bicycle_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int Dynamic_Bicycle_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real Dynamic_Bicycle_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Dynamic_Bicycle_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Dynamic_Bicycle_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Dynamic_Bicycle_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Dynamic_Bicycle_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 7;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
