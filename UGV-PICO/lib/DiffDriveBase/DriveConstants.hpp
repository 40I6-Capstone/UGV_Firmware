#include <math.h>
#define WHEEL_DIA 0.075 // 7.5 cm

#define WHEEL_CIRCUMFERENCE WHEEL_DIA*M_PI //3.14159265358979

#define TICKS_PER_REV 2880. //3840.

#define M_PER_REV WHEEL_CIRCUMFERENCE/TICKS_PER_REV

#define KF 1.


#define TURN_KP 0.001
#define TURN_KI 0.0
#define TURN_KD 0.00


#define RIGHT_KP 4.2
// #define RIGHT_KP 0.0
#define RIGHT_KI 0.00
#define RIGHT_KD 0.015
// #define RIGHT_KD 0.0


#define LEFT_KP 4.2
// #define LEFT_KP 0.0
#define LEFT_KI 0.0
#define LEFT_KD 0.01
// #define LEFT_KD 0.0