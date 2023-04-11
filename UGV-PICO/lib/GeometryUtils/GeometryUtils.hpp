#pragma once
#include <cmath>
#include <iostream>



class GeometryUtils{
    public:
    typedef struct
    {
        double x; // X coord m
        double y; // Y coord m
        double theta = 0; // Angle in Radians
    } Pose;
    static double headingToPoint(Pose from, Pose to);
    static double distToPoint(Pose from, Pose to);
    static double inputModulus(double input, double minInput, double maxInput);
    static double dotProd(Pose a, Pose b);
    static Pose relativeTo(Pose base, Pose to);
    static double flipAngle(double angle); 
    static double degToRad(double);
    static double radToDeg(double);

};









