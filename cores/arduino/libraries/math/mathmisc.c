#include <mathmisc.h>

void pseudo_windowed_variance_init(pw_variance_t *variance, int32_t window_size)
{
    variance->new_sma  = 0.0f;
    variance->new_smsa = 0.0f;
    variance->p1 = 1.0f / (float)window_size;
    variance->p2 = 1.0f - variance->p1;
}

float pseudo_windowed_variance_get(pw_variance_t *variance)
{
    return variance->new_smsa - variance->new_sma * variance->new_sma;
}
