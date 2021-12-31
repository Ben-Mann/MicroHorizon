#ifndef MICROHORIZON_VECTOR_H
#define MICROHORIZON_VECTOR_H

#include <math.h>

typedef struct {
    float x, y, z;
} Vector;

#define vecLengthSq(v) ((v).x*(v).x + (v).y*(v).y + (v).z*(v).z)
#define vecLength(v) (sqrtf(vecLengthSq(v)))
#define dotProduct(v1, v2) ((v1).x*(v2).x + (v1).y*(v2).y + (v1).z*(v2).z)

void vecNormalise(Vector &v);
Vector vecCrossProduct(Vector &v1, Vector &v2);

#endif //MICROHORIZON_VECTOR_H
