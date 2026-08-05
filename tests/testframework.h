#ifndef __TESTFRAMEWORK_H
#define __TESTFRAMEWORK_H

#include <stdio.h>

#if defined(__linux__)
#include <features.h>
#endif

int contains(char **list, int len,  const char *str, const char* descr);
void unittest_init();
int unittest_summary();
void unittest_help_mode();

long timeOfDayinMS();


#define test_bool(expr)   \
  ((expr)                 \
   ? tests_success++     \
   : test_fails (__STRING(expr), __FILE__, __LINE__, ___FUNCTION))

#define UNIT(func)                                                               \
  if(!help_mode){tests_init();                                              \
   fprintf(stderr,"\033[1;34m*** UNIT TEST %s ***\033[0m\n",__STRING(func));     \
   (func);                                                                       \
   fprintf(stderr,"---->\t");                                                     \
   if(test_summary()){ fprintf(stderr, "\t\t\033[1;32m PASSED\033[0m\n");         \
     units_success++; }                                                           \
   else { fprintf(stderr, "\t\t\033[1;31m FAILED\033[0m !!!!!\n");               \
     units_failed++;  }                                                           \
   }

/* __STRING comes from glibc's <sys/cdefs.h> and does not exist elsewhere; it
   is a plain stringification, so provide it where it is missing. */
#ifndef __STRING
# define __STRING(x) #x
#endif

#if defined __STDC_VERSION__ && __STDC_VERSION__ >= 199901L
# define ___FUNCTION  __func__
#elif defined(_MSC_VER)
/* MSVC only defines __STDC_VERSION__ when told to compile a specific C
   standard, and offers __FUNCTION__ regardless. */
# define ___FUNCTION  __FUNCTION__
#else
# define ___FUNCTION  ((const char *) 0)
#endif


// INTERNALS
extern int units_success;
extern int units_failed;
extern int tests_success;
extern int tests_failed;
extern int help_mode;


void tests_init();

int test_summary();

void test_fails (const char *assertion, const char *file,
                 unsigned int line, const char *function);



#endif
