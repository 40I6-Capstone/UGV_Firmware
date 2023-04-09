#pragma once

#include <cmath>

#define DEG_TO_RAD(deg) (deg*M_PI/180.)
#define RAD_TO_DEG(rad) (rad*180./M_PI)

typedef struct
{
    double x; // X coord m
    double y; // Y coord m
    double theta = 0; // Angle in Radians
} Pose;



double headingToPoint(Pose from, Pose to);

double distToPoint(Pose from, Pose to);

double inputModulus(double input, double minInput, double maxInput);

double dotProd(Pose a, Pose b);

Pose relativeTo(Pose base, Pose to);