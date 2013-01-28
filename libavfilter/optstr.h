#ifndef OPTSTR_H
#define OPTSTR_H

#define ARG_MAXIMUM (4)
#define ARG_SEP ':'
#define ARG_CONFIG_LEN 8192

/*
 * optstr_lookup:
 *     Finds the _exact_ 'needle' in 'haystack' (naming intentionally 
 *     identical to the 'strstr' (3) linux man page)
 *
 * Parameters:
 *     needle: substring to be searched
 *     haystack: string which is supposed to contain the substring
 * Return Value:
 *     constant pointer to first substring found, or NULL if substring
 *     isn't found.
 * Side effects:
 *     none
 * Preconditions:
 *     none
 * Postconditions:
 *     none
 */
const char * optstr_lookup(const char *haystack, const char *needle);

/*
 * optstr_get:
 *     extract values from option string
 *
 * Parameters:
 *     options: a null terminated string of options to parse,
 *              syntax is "opt1=val1:opt_bool:opt2=val1-val2"
 *              where ':' is the seperator.
 *     name: the name to look for in options; eg "opt2"
 *     fmt: the format to scan values (printf format); eg "%d-%d"
 *     (...): variables to assign; eg &lower, &upper
 * Return value:
 *     -2 internal error
 *     -1 `name' is not in `options'
 *     0  `name' is in `options'
 *     >0 number of arguments assigned
 * Side effects:
 *     none
 * Preconditions:
 *     none
 * Postconditions:
 *     none
 */
int optstr_get(const char *options, const char *name, const char *fmt, ...)
#ifdef HAVE_GCC_ATTRIBUTES
__attribute__((format(scanf,3,4)))
#endif
;

#endif /* OPTSTR_H */
