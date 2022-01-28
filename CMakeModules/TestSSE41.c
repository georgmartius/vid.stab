#include <stdio.h>

//include sse, sse2 and sse4.1 headers
#include <xmmintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>

int main(int argc, char **argv)
{
  __m128i a = _mm_set_epi32(1, 2, 3, 4);
  __m128i b = _mm_set_epi32(1, 2, 3, 4);
  int i = _mm_testz_si128(a, b);
  printf("%d\n", i);
  return 0;
}
