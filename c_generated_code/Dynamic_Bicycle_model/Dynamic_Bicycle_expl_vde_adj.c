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
  #define CASADI_PREFIX(ID) Dynamic_Bicycle_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

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

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[8] = {8, 1, 0, 4, 4, 5, 6, 7};

/* Dynamic_Bicycle_expl_vde_adj:(i0[7],i1[7],i2,i3[])->(o0[8x1,4nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, *w3=w+3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18;
  /* #0: @0 = 6.918 */
  w0 = 6.9180000000000001e+00;
  /* #1: @1 = 0.199 */
  w1 = 1.9900000000000001e-01;
  /* #2: @2 = 11.976 */
  w2 = 1.1976047904191615e+01;
  /* #3: @3 = input[1][0] */
  casadi_copy(arg[1], 7, w3);
  /* #4: {@4, @5, NULL, @6, @7, @8, @9} = vertsplit(@3) */
  w4 = w3[0];
  w5 = w3[1];
  w6 = w3[3];
  w7 = w3[4];
  w8 = w3[5];
  w9 = w3[6];
  /* #5: @2 = (@2*@9) */
  w2 *= w9;
  /* #6: @1 = (@1*@2) */
  w1 *= w2;
  /* #7: @9 = 0.359195 */
  w9 = 3.5919540229885061e-01;
  /* #8: @9 = (@9*@7) */
  w9 *= w7;
  /* #9: @1 = (@1-@9) */
  w1 -= w9;
  /* #10: @0 = (@0*@1) */
  w0 *= w1;
  /* #11: @1 = 1 */
  w1 = 1.;
  /* #12: @10 = input[0][4] */
  w10 = arg[0] ? arg[0][4] : 0;
  /* #13: @11 = 0.199 */
  w11 = 1.9900000000000001e-01;
  /* #14: @12 = input[0][6] */
  w12 = arg[0] ? arg[0][6] : 0;
  /* #15: @13 = (@11*@12) */
  w13  = (w11*w12);
  /* #16: @13 = (@10-@13) */
  w13  = (w10-w13);
  /* #17: @13 = sq(@13) */
  w13 = casadi_sq( w13 );
  /* #18: @1 = (@1+@13) */
  w1 += w13;
  /* #19: @0 = (@0/@1) */
  w0 /= w1;
  /* #20: @1 = 6.932 */
  w1 = 6.9320000000000004e+00;
  /* #21: @13 = 0.205 */
  w13 = 2.0499999999999999e-01;
  /* #22: @13 = (@13*@2) */
  w13 *= w2;
  /* #23: @2 = input[2][0] */
  w2 = arg[2] ? arg[2][0] : 0;
  /* #24: @14 = cos(@2) */
  w14 = cos( w2 );
  /* #25: @14 = (@14*@9) */
  w14 *= w9;
  /* #26: @13 = (@13+@14) */
  w13 += w14;
  /* #27: @13 = (@1*@13) */
  w13  = (w1*w13);
  /* #28: @14 = 1 */
  w14 = 1.;
  /* #29: @15 = 0.205 */
  w15 = 2.0499999999999999e-01;
  /* #30: @12 = (@15*@12) */
  w12  = (w15*w12);
  /* #31: @12 = (@10+@12) */
  w12  = (w10+w12);
  /* #32: @16 = sq(@12) */
  w16 = casadi_sq( w12 );
  /* #33: @16 = (@14+@16) */
  w16  = (w14+w16);
  /* #34: @16 = (@13/@16) */
  w16  = (w13/w16);
  /* #35: @17 = (@0-@16) */
  w17  = (w0-w16);
  /* #36: @17 = (@17+@6) */
  w17 += w6;
  /* #37: @6 = input[0][5] */
  w6 = arg[0] ? arg[0][5] : 0;
  /* #38: @18 = cos(@6) */
  w18 = cos( w6 );
  /* #39: @18 = (@18*@5) */
  w18 *= w5;
  /* #40: @17 = (@17+@18) */
  w17 += w18;
  /* #41: @18 = sin(@6) */
  w18 = sin( w6 );
  /* #42: @18 = (@18*@4) */
  w18 *= w4;
  /* #43: @17 = (@17-@18) */
  w17 -= w18;
  /* #44: output[0][0] = @17 */
  if (res[0]) res[0][0] = w17;
  /* #45: @17 = cos(@6) */
  w17 = cos( w6 );
  /* #46: @17 = (@17*@5) */
  w17 *= w5;
  /* #47: @18 = sin(@6) */
  w18 = sin( w6 );
  /* #48: @5 = (@10*@5) */
  w5  = (w10*w5);
  /* #49: @18 = (@18*@5) */
  w18 *= w5;
  /* #50: @17 = (@17-@18) */
  w17 -= w18;
  /* #51: @18 = cos(@6) */
  w18 = cos( w6 );
  /* #52: @10 = (@10*@4) */
  w10 *= w4;
  /* #53: @18 = (@18*@10) */
  w18 *= w10;
  /* #54: @17 = (@17-@18) */
  w17 -= w18;
  /* #55: @6 = sin(@6) */
  w6 = sin( w6 );
  /* #56: @6 = (@6*@4) */
  w6 *= w4;
  /* #57: @17 = (@17-@6) */
  w17 -= w6;
  /* #58: output[0][1] = @17 */
  if (res[0]) res[0][1] = w17;
  /* #59: @8 = (@8-@7) */
  w8 -= w7;
  /* #60: @11 = (@11*@0) */
  w11 *= w0;
  /* #61: @8 = (@8-@11) */
  w8 -= w11;
  /* #62: @15 = (@15*@16) */
  w15 *= w16;
  /* #63: @8 = (@8-@15) */
  w8 -= w15;
  /* #64: output[0][2] = @8 */
  if (res[0]) res[0][2] = w8;
  /* #65: @8 = sin(@2) */
  w8 = sin( w2 );
  /* #66: @12 = atan2(@12,@14) */
  w12  = atan2(w12,w14);
  /* #67: @2 = (@2-@12) */
  w2 -= w12;
  /* #68: @1 = (@1*@2) */
  w1 *= w2;
  /* #69: @1 = (@1*@9) */
  w1 *= w9;
  /* #70: @8 = (@8*@1) */
  w8 *= w1;
  /* #71: @13 = (@13-@8) */
  w13 -= w8;
  /* #72: output[0][3] = @13 */
  if (res[0]) res[0][3] = w13;
  return 0;
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void Dynamic_Bicycle_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Dynamic_Bicycle_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int Dynamic_Bicycle_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real Dynamic_Bicycle_expl_vde_adj_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Dynamic_Bicycle_expl_vde_adj_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Dynamic_Bicycle_expl_vde_adj_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Dynamic_Bicycle_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Dynamic_Bicycle_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Dynamic_Bicycle_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 8;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 25;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
