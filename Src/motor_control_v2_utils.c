#include "stdint.h"
#include "varia.h"

/**
    @brief          Compute d/q components of space vector from magnitude and angle.
    @param[in]      i32_mag
    @param[in]      i32_angle   in q31 format
    @param[out]     p_i32_xd
    @param[out]     p_i32_xq
    Tested for the following value ranges:
        i32_angle from -1073741824 (-90 deg) to 1073741824 (+90 deg)
        i32_mag from 0 to 10000
*/
void get_dq_coordinates(int32_t i32_mag, int32_t i32_angle, int32_t *p_i32_xd, int32_t *p_i32_xq)
{
    int32_t i32_sin, i32_cos;
    int64_t i64_mag = (int64_t)i32_mag;
    //
    arm_sin_cos_q31(i32_angle, &i32_sin, &i32_cos);
    //
    *p_i32_xd = (int32_t)((i64_mag * i32_sin) >> 31);
    *p_i32_xq = (int32_t)((i64_mag * i32_cos) >> 31);
}

/**
    @brief          Compute angle of space vector from d/q components.
    @param[in]      xd
    @param[in]      xq
    @return         angle   angle in q31 format
    The atan approximation is accurate in the interval -45 to 45 degree, that is -1 < (xd / xq) < +1
    No overflow if xd and xq < 30000.
*/
int32_t get_angle(int32_t xd, int32_t xq)
{
    int64_t i64_num, i64_denom;
    int32_t angle;

    // http://www.olliw.eu/2014/fast-functions/#atan2
    // https://de.wikipedia.org/wiki/Arkustangens_und_Arkuskotangens#N.C3.A4herungsweise_Berechnung
    //
    if (xq > 0)
    {
        // max i32 divided by pi
        i64_num = 683565275;
        i64_num = ((int64_t)xd * xq) * i64_num;
        i64_denom = ((int64_t)(xq * xq)) * 1024 + ((int64_t)(xd * xd)) * 287;
        angle = (int32_t)((i64_num / i64_denom) * 1024);
    }
    else
    {
        angle = 0;
    }

    return angle;
}

/**
    @brief          Compute magnitude of space vector from d/q components.
    @param[in]      xd
    @param[in]      xq
    @return         i32_mag
*/
int32_t get_magnitude(int32_t xd, int32_t xq)
{
    int32_t i32_mag;
    uint32_t ui32_mag_sqr = xd * xd + xq * xq;
    i32_mag = (int32_t)isqrt(ui32_mag_sqr);
    return i32_mag;
}