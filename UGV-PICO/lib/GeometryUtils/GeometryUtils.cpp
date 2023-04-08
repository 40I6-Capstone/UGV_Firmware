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