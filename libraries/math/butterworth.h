#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

// Coefficients of second order Butterworth biquadratic filter in direct from 2
struct ButterWorthDF2Filter {
    float b0;
    float a1;
    float a2;
};

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Function declarations
void InitButterWorthDF2Filter(const float ff, struct ButterWorthDF2Filter *filterPtr);
void InitButterWorthDF2Values(const float x0, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr);
float FilterButterWorthDF2(const float xn, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif
