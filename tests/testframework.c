#include <string.h>
#include <sys/time.h>

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

long timeOfDayinMS() {
  struct timeval t;
  gettimeofday(&t, 0);
  return t.tv_sec*1000 + t.tv_usec/1000;
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

void test_fails (__const char *__assertion, __const char *__file,
                 unsigned int __line, __const char *__function){
  fprintf(stderr, "%s:%i: Test Failed: %s\n in Function %s", __file,__line,__assertion,__function);
  tests_failed++;
}
