#ifndef COMPAT_H
#define COMPAT_H

#if defined(_MSC_VER)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
  #include <math.h>

  #ifndef M_PI
    #define M_PI 3.14159265358979323846
  #endif
#else
  #include <math.h>
#endif


#endif //COMPAT_H
