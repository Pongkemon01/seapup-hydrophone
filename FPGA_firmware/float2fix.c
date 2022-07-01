#include <stdio.h>
#include <stdint.h>


/// Fixed-point Format: 11.5 (16-bit)
typedef uint16_t fixed_point_t;

#define FIXED_POINT_FRACTIONAL_BITS 5

inline double fixed_to_double(fixed_point_t input)
{
    return ((double)input / (double)(1 << FIXED_POINT_FRACTIONAL_BITS));
}

inline fixed_point_t double_to_fixed(double input)
{
    return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

int main()
{
	double coefs[] = { -1.951934814453125, 0.9556884765625, -1.95855712890625, 
			   -1.884246826171875, 0.88818359375, -1.76995849609375 };
}
