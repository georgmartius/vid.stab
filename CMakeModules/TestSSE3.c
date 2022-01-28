#include <stdio.h>

//include sse and sse2 headers
#include <xmmintrin.h>
#include <emmintrin.h>

/* __m128 is ugly to write */
typedef __m128d v2df;  // vector of 2 double (sse2)

int main(int argc, char **argv)
{
  v2df calcx = _mm_setr_pd(2.0, 3.0);
  v2df xx = _mm_mul_pd(calcx, calcx);
  double d;
  _mm_storel_pd(&d, xx);
  printf("%f\n", d);
  _mm_storeh_pd(&d, xx);
  printf("%f\n", d);
  return 0;
}
