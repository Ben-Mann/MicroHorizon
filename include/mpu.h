#ifndef MICROHORIZON_MPU_H
#define MICROHORIZON_MPU_H

#include "vector.h"

extern Vector mpuAccel;
extern float mpuTemp;

void mpuSetup();
void mpuGet();

#endif //MICROHORIZON_MPU_H
