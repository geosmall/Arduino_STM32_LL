#ifndef SIN_LOOKUP_H
#define SIN_LOOKUP_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

int sin_lookup_initalize();
float sin_lookup_deg(float angle);
float cos_lookup_deg(float angle);
float sin_lookup_rad(float angle);
float cos_lookup_rad(float angle);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif
