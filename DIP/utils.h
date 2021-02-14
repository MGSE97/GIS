#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "defines.h"


#define ALLOCTEST( x ) if ( ( !x ) ) utils_out_of_memory( __FILE__, __LINE__ )

void utils_out_of_memory( const char *filename, const int lineno );

void make_binary_file( const char *input_filename, const char *out_filename );

#endif //_UTILS_H_
