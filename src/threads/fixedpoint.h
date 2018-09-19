#ifndef __THREAD_FIXED_POINT_H
#define __THREAD_FIXED_POINT_H

#define P 17
#define Q 14
#define F (1 << Q)

// x and y are fixed point
// n is integer

fixed_point fp(int n);
int fp_to_int(fixed_point x);
int fp_to_int_round (fixed_point x);
fixed_point fp_add(fixed_point x, fixed_point y);
fixed_point fp_sub(fixed_point x, fixed_point y);
fixed_point fp_mul(fixed_point x, fixed_point y);
fixed_point fp_div(fixed_point x, fixed_point y);
fixed_point mixed_add(fixed_point x, int n);
fixed_point mixed_sub(fixed_point x, int n);
fixed_point mixed_mul(fixed_point x, int n);
fixed_point mixed_div(fixed_point x, int n);

// convert int to fixed point
fixed_point fp (int n){ return n*F; }

// get integer part of fp
int fp_to_int (fixed_point x){ return x/F; }

// rounds to nearest
int fp_to_int_round (fixed_point x){
    if (x >= 0){ return ((x+F/2)/F); }
    else{ return ((x-F/2)/F); }
}

fixed_point fp_add(fixed_point x, fixed_point y){ return x + y; }

=fixed_point fp_sub(fixed_point x, fixed_point y){ return x-y; }

fixed_point fp_mul(fixed_point x, fixed_point y){ return ((int64_t) x) * y / F; }

fixed_point fp_div(fixed_point x, fixed_point y){ return ((int64_t) x) * F / y; }

fixed_point mixed_add(int x, int n){ return (x + n*F); }

fixed_point mixed_sub(fixed_point x, int n){ return (x - n*F); }

fixed_point mixed_mul(fixed_point x, int n){ return x * n; }

fixed_point mixed_div(fixed_point x, int n){ return (x / n); }

#endif