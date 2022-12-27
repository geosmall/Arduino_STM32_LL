#ifndef VECTORS_H_
#define VECTORS_H_

#include <math.h>
#include <stdint.h>

#define DECLAREVECTOR3(suffix, datatype) \
    typedef struct Vector3##suffix##_t { \
        datatype x; \
        datatype y; \
        datatype z; \
    } Vector3##suffix

#define DECLAREVECTOR2(suffix, datatype) \
    typedef struct Vector2##suffix##_t { \
        datatype x; \
        datatype y; \
    } Vector2##suffix


DECLAREVECTOR3(i16, int16_t);
DECLAREVECTOR3(i32, int32_t);
DECLAREVECTOR3(u16, uint16_t);
DECLAREVECTOR3(u32, uint32_t);
DECLAREVECTOR3(f, float);

DECLAREVECTOR2(i16, int16_t);
DECLAREVECTOR2(i32, int32_t);
DECLAREVECTOR2(u16, uint16_t);
DECLAREVECTOR2(u32, uint32_t);
DECLAREVECTOR2(f, float);

#endif /* VECTORS_H_ */
