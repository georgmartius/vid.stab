#include <stdio.h>

//include sse, sse2 and ssse3 headers
#include <xmmintrin.h>
#include <emmintrin.h>
#include <tmmintrin.h>

int main(int argc, char **argv)
{
  __m128i calcx = _mm_set_epi32(1, 2, 3, 4);
  __m128i xx = _mm_hadd_epi32(calcx, calcx);
  int i;
  _mm_storeu_si32(&i, xx);
  printf("%d\n", i);
  return 0;
}
