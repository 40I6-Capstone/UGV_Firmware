#include <math.h>
#define WHEEL_DIA 0.075 // 7.5 cm

#define WHEEL_CIRCUMFERENCE WHEEL_DIA*M_PI //3.14159265358979

#define TICKS_PER_REV 3840.

#define M_PER_REV WHEEL_CIRCUMFERENCE/TICKS_PER_REV