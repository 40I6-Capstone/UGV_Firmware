#include "GeometryUtils.hpp"

double GeometryUtils::distToPoint(Pose from, Pose to){
    double deltaX = to.x - from.x;
    double deltaY = to.y - from.y;

    double xSqr, ySqr;
    xSqr = deltaX*deltaX;
    ySqr = deltaY*deltaY;

    return sqrt(xSqr + ySqr);
}


double GeometryUtils::headingToPoint(Pose from, Pose to){
    double deltaX = to.x - from.x;
    double deltaY = to.y - from.y;
    double headingRad = atan2(deltaY, deltaX);
    
    return GeometryUtils::radToDeg(headingRad);
}

double GeometryUtils::inputModulus(double input, double minInput, double maxInput){
    double modulus = maxInput - minInput;

    int numMax = (int)((input-minInput)/modulus);

    input -= numMax*modulus;

    int numMin = (int)((input-maxInput)/modulus);
    input -= numMin*modulus;

    return input;
}


double GeometryUtils::dotProd(Pose a, Pose b){
    return a.x * b.x + a.y * b.y;
}

GeometryUtils::Pose GeometryUtils::relativeTo(Pose base, Pose to){
    double relativeX = to.x - base.x;
    double relativeY = to.y - base.y;

    return {.x = relativeX, .y = relativeY, .theta = 0};
}

double GeometryUtils::flipAngle(double angle){
    return inputModulus(angle+180, -180, 180);
}

double GeometryUtils::degToRad(double deg){
    return deg*M_PI/180.;
}


double GeometryUtils::radToDeg(double rad){
    return rad*180./M_PI;
}