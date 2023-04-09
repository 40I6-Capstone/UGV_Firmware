#include "GeometryUtils.hpp"

double distToPoint(Pose from, Pose to){
    double deltaX = to.x - from.x;
    double deltaY = to.y - from.y;

    double xSqr, ySqr;
    xSqr = deltaX*deltaX;
    ySqr = deltaY*deltaY;

    return sqrt(xSqr + ySqr);
}


double headingToPoint(Pose from, Pose to){
    double deltaX = to.x - from.x;
    double deltaY = to.y - from.y;
    double headingRad = atan2(deltaY, deltaX);
    
    return RAD_TO_DEG(headingRad);
}

double inputModulus(double input, double minInput, double maxInput){
    double ret = input;
    double modulus = maxInput - minInput;

    int numMax = (int)((ret-minInput)/modulus);

    ret -= numMax*modulus;

    int numMin = (int)((ret-maxInput)/modulus);
    ret -= numMin*modulus;

    return ret;
}


double dotProd(Pose a, Pose b){
    return a.x * b.x + a.y * b.y;
}

Pose relativeTo(Pose base, Pose to){
    double relativeX = to.x - base.x;
    double relativeY = to.y - base.y;

    return {.x = relativeX, .y = relativeY, .theta = 0};
}