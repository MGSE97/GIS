#include "utils.h"

void utils_out_of_memory( const char *filename, const int lineno ) {

    fprintf( stderr, "Oops! Out of memory (%s:%i)\n", filename, lineno );
    exit( -1 );

} //utils_out_of_memory

void make_binary_file( const char *input_filename, const char *out_filename ) {
    FILE *f_in, *f_out;
    char line[ LINE_SIZE ];
    char *no = NULL;
    long line_no = 0;
    float x, y, z;

    f_in = fopen( input_filename, "r" );
    f_out = fopen( out_filename, "wb" );

    while ( fgets( line, LINE_SIZE, f_in ) != NULL ) {
        if ( line_no % 1000000 == 0 ) {
            printf( "Processing line: %ld Million\n", line_no / 1000000 );
        }

        no = (char*)strtok( line, " " );
        SKIP_LINE( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no );
        x = (float)atof( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no )
        y = (float)atof( no );
        no = (char*)strtok( NULL, " " );
        SKIP_LINE( no );
        z = (float)atof( no );

        fwrite( &x, sizeof( x ), 1, f_out );
        fwrite( &y, sizeof( y ), 1, f_out );
        fwrite( &z, sizeof( z ), 1, f_out );

        line_no++;
    }

    printf( "Closing file: %s\n", input_filename );
    printf( "Closing file: %s\n", out_filename );
    fclose( f_in );
    fclose( f_out );
}
