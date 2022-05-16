#include "varia.h"

const int32_t MAX_INT32 = 2147483647;
const int32_t MIN_INT32 = -2147483648;

int16_t q31_degree_to_degree(q31_t q31_degree)
{
    return ((q31_degree >> 23) * 180) >> 8;
}

/**
   * @brief Linear interpolation y = y(x).
   * @param x               input
   * @param x_min, x_max    input range: x_max has to be bigger than x_min
   * @param y_min, y_max    output range: y_max can be smaller than y_min
   * @return y              output
   */
int32_t map(int32_t x, int32_t x_min, int32_t x_max, int32_t y_min, int32_t y_max)
{
    int32_t y;
    if (x < x_min)
    {
        y = y_min;
    }
    else if (x > x_max)
    {
        y = y_max;
    }
    //else if ((x_max - x_min) > (y_max - y_min))
    //{
    //    // round up if mapping bigger ranges to smaller ranges
    //    y = (x - x_min) * (y_max - y_min + 1) / (x_max - x_min + 1) + y_min;
    //}
    else
    {
        // round down if mapping smaller ranges to bigger ranges
        y = (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
    }
    return y;
}

uint32_t limit_uint32(uint32_t min, uint32_t max, uint32_t value)
{
    if(value < min)
    {
        value = min;
    }
    if(value > max)
    {
        value = max;
    }
    return value;
}

int32_t limit_int32(int32_t min, int32_t max, int32_t value)
{
    if(value < min)
    {
        value = min;
    }
    if(value > max)
    {
        value = max;
    }
    return value;
}

uint32_t isqrt(uint32_t x) 
{
    // http://rosettacode.org/wiki/Isqrt_(integer_square_root)_of_X#C
    uint32_t q = 1, r = 0;
    while (q <= x) 
    {
        q <<= 2;
    }
    while (q > 1) 
    {
        int32_t t;
        q >>= 2;
        t = x - r - q;
        r >>= 1;
        if (t >= 0) 
        {
            x = t;
            r += q;
        }
    }
    return r;
}
