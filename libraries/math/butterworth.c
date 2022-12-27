#include <math.h>
#include <uvos_math.h>
#include "butterworth.h"

/**
 * Initialization function for coefficients of a second order Butterworth biquadratic filter in direct from 2.
 * Note that b1  = 2 * b0 and b2  = b0 is use here and in the sequel.
 * @param[in]  ff Cut-off frequency ratio
 * @param[out] filterPtr Pointer to filter coefficients
 * @returns Nothing
 */
void InitButterWorthDF2Filter(const float ff, struct ButterWorthDF2Filter *filterPtr)
{
    const float ita = 1.0f / tanf(M_PI_F * ff);
    const float b0  = 1.0f / (1.0f + M_SQRT2_F * ita + ita * ita);
    const float a1  = 2.0f * b0 * (ita * ita - 1.0f);
    const float a2  = -b0 * (1.0f - M_SQRT2_F * ita + ita * ita);

    filterPtr->b0 = b0;
    filterPtr->a1 = a1;
    filterPtr->a2 = a2;
}


/**
 * Initialization function for intermediate values of a second order Butterworth biquadratic filter in direct from 2.
 * Obtained by solving a linear equation system.
 * @param[in]  x0 Prescribed value
 * @param[in]  filterPtr Pointer to filter coefficients
 * @param[out] wn1Ptr Pointer to first intermediate value
 * @param[out] wn2Ptr Pointer to second intermediate value
 * @returns Nothing
 */
void InitButterWorthDF2Values(const float x0, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
{
    const float b0   = filterPtr->b0;
    const float a1   = filterPtr->a1;
    const float a2   = filterPtr->a2;

    const float a11  = 2.0f + a1;
    const float a12  = 1.0f + a2;
    const float a21  = 2.0f + a1 * a1 + a2;
    const float a22  = 1.0f + a1 * a2;
    const float det  = a11 * a22 - a12 * a21;
    const float rhs1 = x0 / b0 - x0;
    const float rhs2 = x0 / b0 - x0 + a1 * x0;

    *wn1Ptr = (a22 * rhs1 - a12 * rhs2) / det;
    *wn2Ptr = (-a21 * rhs1 + a11 * rhs2) / det;
}


/**
 * Second order Butterworth biquadratic filter in direct from 2, such that only two values wn1=w[n-1] and wn2=w[n-2] need to be stored.
 * Function takes care of updating the values wn1 and wn2.
 * @param[in]  xn New raw value
 * @param[in]  filterPtr Pointer to filter coefficients
 * @param[out] wn1Ptr Pointer to first intermediate value
 * @param[out] wn2Ptr Pointer to second intermediate value
 * @returns Filtered value
 */
float FilterButterWorthDF2(const float xn, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
{
    const float wn  = xn + filterPtr->a1 * (*wn1Ptr) + filterPtr->a2 * (*wn2Ptr);
    const float val = filterPtr->b0 * (wn + 2.0f * (*wn1Ptr) + (*wn2Ptr));

    *wn2Ptr = *wn1Ptr;
    *wn1Ptr = wn;
    return val;
}
