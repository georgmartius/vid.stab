#ifndef VIDSTAB_API_H
#define VIDSTAB_API_H

#if defined(_WIN32) && defined(_MSC_VER)
  #ifdef VIDSTAB_EXPORTS
    #define VS_API __declspec(dllexport)
  #else
    #define VS_API __declspec(dllimport)
  #endif
#else
  #define VS_API
#endif

#endif // VIDSTAB_API_H