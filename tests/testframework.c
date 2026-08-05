#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "testframework.h"

int help_mode=0;

void unittest_help_mode(){
  help_mode=1;
}

// returns 0 of not found and otherwise the index of the next element (possible argument)
int contains(char **list, int len,  const char *str, const char* descr) {
  if(help_mode) {
    printf("\t%s:\t%s\n",str, descr);
    return 0;
  }
  int i;
  for(i=0; i<len; i++) {
    if(strcmp(list[i],str) == 0)
      return i+1;
  }
  return 0;
}

int units_success;
int units_failed;

void unittest_init(){
  units_success=0;
  units_failed=0;
}

int unittest_summary(){
  fprintf(stderr, "*********** SUMMARY **************\n");
  fprintf(stderr, "UNIT TESTs succeeded:\t %s%i/%i\033[0m\n",
          units_failed>0 ? "\033[1;31m" : "\033[1;32m",
          units_success, units_success + units_failed);
  return units_failed!=0;

}

/* Milliseconds from an origin nobody should rely on: only differences between
   two calls mean anything.  Monotonic on both branches, so a clock adjustment
   in the middle of a measurement cannot turn an interval negative -- which the
   gettimeofday() this replaces allowed, besides not existing on Windows. */
long timeOfDayinMS() {
#ifdef _WIN32
  return (long)GetTickCount64();
#else
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec*1000 + t.tv_nsec/1000000;
#endif
}

//// INTERNALS
int tests_success;
int tests_failed;

void tests_init(){
  tests_success=0;
  tests_failed=0;
}


int test_summary(){
  fprintf(stderr, "Tests checks succeeded: %i/%i",
          tests_success, tests_success + tests_failed);
  return tests_failed==0;
}

void test_fails (const char *assertion, const char *file,
                 unsigned int line, const char *function){
  fprintf(stderr, "%s:%i: Test Failed: %s\n in Function %s", file,line,assertion,function);
  tests_failed++;
}
