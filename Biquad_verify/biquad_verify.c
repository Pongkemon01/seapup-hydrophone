#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Constant
// Coefficient
uint64_t b1_1 = (uint64_t)( (int64_t)-128356LL );
uint64_t minus_a1_1 = (uint64_t)( (int64_t)127922LL );
uint64_t minus_a2_1 = (uint64_t)( (int64_t)-62632LL );
uint64_t b1_2 = (uint64_t)( (int64_t)-115996LL );
uint64_t minus_a1_2 = (uint64_t)( (int64_t)123486LL );
uint64_t minus_a2_2 = (uint64_t)( (int64_t)-58208LL );

uint64_t verifying_mask = 0xFFFFFFFFFFE00000LL;

// biquad state
typedef struct
{
    uint64_t b1, minus_a1, minus_a2;    // Coefficients
    uint64_t fir_delay[2];  // Q14.2
    uint64_t iir_delay[2];  // Q28.2
    uint64_t epoch_sum;     // Q30.18
}BIQUAD;

// Sampling data
struct
{
    uint64_t total_sampling;
    uint64_t* hydrophone[4];    // Q14.2 sign extended to 64 bits
}input_data;

// Functions

void u2txt( uint64_t data, char* out )  // out must have enough space for 81 chars
{
    uint64_t mask = 0x8000000000000000LL;
    int j;
    j = 4;
    for( int i = 0; i < 79; i++ )
    {
        // Add dash for easy reading
        if( j == 0 )
        {
            out[i] = '-';
            i++;
            j = 3;
        }
        else
            j--;

        if( ( data & mask ) != 0 )
            out[i] = '1';
        else
            out[i] = '0';
        
        mask >>= 1;
    }
    out[79] = '\0';
}

uint64_t RoundShiftBits( uint64_t data, int LSB_bit )
{
    uint64_t rounding_bit_num;

    // If not start from bit 0, then we have some bits for rounding
    if( LSB_bit != 0 )
    {
        rounding_bit_num = 1LL;
        for( int i = 1; i <= LSB_bit; i++ )
            rounding_bit_num <<= 1;
        
        data += rounding_bit_num;
    }

    // Reduce bits
    for( int i = 1; i <= LSB_bit; i++ )
    {
        data >>= 1;
        if( ( data & 0x4000000000000000LL ) != 0 )
            data |= 0x8000000000000000LL;   // Sign extended
    }

    return( data );
}

int ReadSampling( char* filename )
{
    FILE* f;
    long file_size, total_sampling;
    int16_t rawdata[4];

    if( ( f = fopen( filename, "rb" ) ) == NULL )
        return -1;
    
    // Get file size
    fseek( f, 0L, SEEK_END );
    file_size = ftell( f );
    fseek( f, 0L, SEEK_SET );

    total_sampling = file_size / 8;     // Each sampling contains 4 head of 16-bit signed data
    
    printf( "Input file has %ld sampling with total size %ld bytes\n", total_sampling, file_size );

    // Initialize input_data
    input_data.hydrophone[0] = malloc( total_sampling * sizeof(uint64_t) );
    input_data.hydrophone[1] = malloc( total_sampling * sizeof(uint64_t) );
    input_data.hydrophone[2] = malloc( total_sampling * sizeof(uint64_t) );
    input_data.hydrophone[3] = malloc( total_sampling * sizeof(uint64_t) );

    if( input_data.hydrophone[0] == NULL || input_data.hydrophone[1] == NULL ||
        input_data.hydrophone[2] == NULL || input_data.hydrophone[3] == NULL )
    {
        printf( "Failed to allocate buffer\n" );
        fclose( f );
        for( int i = 0; i < 4; i++ )
        {
            if( input_data.hydrophone[i] != NULL )
            {
                free( input_data.hydrophone[i] );
                input_data.hydrophone[i] = NULL;
            }
        }
        return -2;
    }
    
    input_data.total_sampling = (uint64_t)total_sampling;

    // Reading data
    for( long l = 0L; l < total_sampling; l++ )
    {
        if( fread( rawdata, sizeof(int16_t), 4, f ) != 4 )
        {
            printf( "Error reading data at %ld \n", l );
            fclose( f );
            for( int i = 0; i < 4; i++ )
            {
                free( input_data.hydrophone[i] );
                input_data.hydrophone[i] = NULL;
            }
            return -3;
        }

        // Read a sampling successfully, store it
        *(input_data.hydrophone[0]) = (uint64_t)( (int64_t)( rawdata[0] ) );
        *(input_data.hydrophone[1]) = (uint64_t)( (int64_t)( rawdata[1] ) );
        *(input_data.hydrophone[2]) = (uint64_t)( (int64_t)( rawdata[2] ) );
        *(input_data.hydrophone[3]) = (uint64_t)( (int64_t)( rawdata[3] ) );
    }

    // All reading completed
    fclose( f );
    return 0;
}

uint64_t BiquadEpoch( BIQUAD* filter, uint64_t input )
{
    // Sum of current epoch
    filter->epoch_sum = input + ( filter->b1 * filter->fir_delay[0] ) + filter->fir_delay[1];
    filter->epoch_sum += ( filter->minus_a1 * filter->iir_delay[0] ) + ( filter->minus_a2 * filter->iir_delay[1] );

    // Advance delay lines
    filter->fir_delay[1] = filter->fir_delay[0];
    filter->fir_delay[0] = input;
    filter->iir_delay[1] = filter->iir_delay[0];
    filter->iir_delay[0] = RoundShiftBits( filter->epoch_sum, 16 ); // Q30.18 => Q28.2
}

int main(void)
{
    int ret;
    FILE *f1, *f2;
    char buf[100];
    uint64_t t;

    BIQUAD filter1 = 
    {
        .b1 = b1_1, .minus_a1 = minus_a1_1, .minus_a2 = minus_a2_1,
        .fir_delay = { 0ULL, 0ULL }, .iir_delay = { 0ULL, 0ULL },
        .epoch_sum = 0ULL
    };
    BIQUAD filter2 = 
    {
        .b1 = b1_2, .minus_a1 = minus_a1_2, .minus_a2 = minus_a2_2,
        .fir_delay = { 0ULL, 0ULL }, .iir_delay = { 0ULL, 0ULL },
        .epoch_sum = 0ULL
    };

    printf( "Reading input\n" );
    ret = ReadSampling( "file_01.bin" );
    if( ret != 0 )
        return ret;

    // Open output files
    //f1 = fopen( "filter1.txt", "w" );
    //f2 = fopen( "filter2.txt", "w" );

    // Epoch advancing
    for( uint64_t i = 0; i < input_data.total_sampling; i++ )
    {
        BiquadEpoch( &filter1, *(input_data.hydrophone[0]) );
        t = filter1.iir_delay[0] & verifying_mask;
        if( t != verifying_mask && t != 0LL )
        {
            u2txt( filter1.iir_delay[0], buf );
            printf( "Filter 1 overflow at %lu with data %s\n", i, buf );
        }
        BiquadEpoch( &filter2, RoundShiftBits( filter1.iir_delay[0], 6 ) ); // Q28.2 => Q14.2
        t = filter2.iir_delay[0] & verifying_mask;
        if( t != verifying_mask && t != 0LL )
        {
            u2txt( filter2.iir_delay[0], buf );
            printf( "Filter 2 overflow at %lu with data %s\n", i, buf );
        }
    }

    printf( "Finished all epoch\n" );
    free( input_data.hydrophone[0] );
    free( input_data.hydrophone[1] );
    free( input_data.hydrophone[2] );
    free( input_data.hydrophone[3] );

    //fclose( f1 );
    //fclose( f2 );
}