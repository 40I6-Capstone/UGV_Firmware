#pragma once
#include "GeometryUtils/GeometryUtils.hpp"
#include "../../UGV-ESP/UGV-ESP/network_defines.hpp"
#include <stdlib.h>
#include <limits>


class PurePursuit{

    public:
        PurePursuit(double, Pose, bool);
        double getLookAheadHeading(Pose);
        Pose getLookAheadPose(Pose);
        Pose getLookAheadPose();
        void setPath(packet_path_point*,int);
        bool isLookAheadLast();
        Pose poseFromPacket(packet_path_point);
        void setReversed(bool);
        Pose getLastPose();


    private:
        packet_path_point* path;
        int pathSize;
        int nextPoseIndex;
        bool isReversed;
        double lookAheadDist;
        void updateLookAhead(Pose);
        int getClosestPoseIndex(Pose);
        int getLastIndex();
};


PurePursuit::PurePursuit(double lookAheadDist, Pose currentPose, bool isReversed){
    this->lookAheadDist = lookAheadDist;
    this->isReversed = isReversed;
    // Update next pose index and the next pose
    // this->updateLookAhead(currentPose);
    this->nextPoseIndex = 0;
}

int PurePursuit::getLastIndex(){
    return (this->isReversed? 0: this->pathSize-1);
}

void PurePursuit::setPath(packet_path_point *path, int length){
    this->path = path;
    this->pathSize = length;
    this->nextPoseIndex = isReversed? this->pathSize - 1 : 0; // Set nextIndex to be at the beginning
}


int PurePursuit::getClosestPoseIndex(Pose currentPose){
    int closestIndex = -1;
    double minDist = std::numeric_limits<double>::max();
    for(int i = 0; i < this->pathSize; i++ ){
        int checkIndex = this->isReversed? this->pathSize-1-i : i; // Current point to check against
        Pose checkPose = this->poseFromPacket(this->path[checkIndex]); // Create a pose from packet point
        double dist = distToPoint(currentPose,checkPose); // Compute distance to point
        if(dist < minDist){ // Check if distance is smaller than current minimum
            // Update Index
            closestIndex = checkIndex;
            // Update Min Distance
            minDist = dist;
        }
    }

    return closestIndex;   
}


/**
 * @brief updates the lookahead index given the current pose
*/
void PurePursuit::updateLookAhead(Pose currentPose){
    // Check if next pose is last pose
    // if(isLookAheadLast()){
    //     std::cout << " LOOKING AT END" << std::endl;
    //     return;
    // }

    // Get closest point
    int closestIndex = getClosestPoseIndex(currentPose);
    if(closestIndex == -1){ // If for some reason theres an error, start from the beginning
        closestIndex = isReversed ? this->pathSize-1 : 0; 
        std::cout << "invalid closest" << std::endl;
        this->nextPoseIndex = closestIndex;
        return;
    }

    std::cout << "CloseX: " << path[closestIndex].x << " Close Y: " << path[closestIndex].y << " Index: " << closestIndex <<std::endl; 

    // Check if closest point is last point
    if(closestIndex == getLastIndex()){
        this->nextPoseIndex = closestIndex; // Set the next index as this index
        return;
    }

    // Closest point isn't the last point, iterate along the path until a point is larger than lookahead
    int direction = this->isReversed? -1:1;
    Pose closestPose = poseFromPacket(this->path[closestIndex]);
    for(int i = 0; i < abs(closestIndex-getLastIndex()); i++){
        int checkIndex = i*direction + closestIndex;
        Pose testPose = poseFromPacket(this->path[checkIndex]);
        Pose nextTestPose = poseFromPacket(this->path[checkIndex+direction]);
        if(distToPoint(currentPose, testPose) > this->lookAheadDist 
            && dotProd(relativeTo(currentPose,testPose),relativeTo(currentPose,nextTestPose)) > 0.){ 
            this->nextPoseIndex = checkIndex; 
            return;
        }
    }

    // No point is greater than lookahead distance away, select last point
    this->nextPoseIndex = getLastIndex();

}


/**
 * @brief returns if the pose being pointed to is the last pose in the path
*/
bool PurePursuit::isLookAheadLast(){
    return this->nextPoseIndex == getLastIndex();
}


/**
 * @brief returns the lookahead pose
*/
Pose PurePursuit::getLookAheadPose(Pose currentPose){
    // Update next pose
    this->updateLookAhead(currentPose);
    return getLookAheadPose();
}

Pose PurePursuit::getLookAheadPose(){
    return poseFromPacket(this->path[this->nextPoseIndex]);
}



double PurePursuit::getLookAheadHeading(Pose current){
    return headingToPoint(current, this->getLookAheadPose());
}


Pose PurePursuit::poseFromPacket(packet_path_point packet){
    Pose ret = {.x = packet.x, .y = packet.y };
    return ret;
}

void PurePursuit::setReversed(bool isReversed){
    this->isReversed = isReversed;
}

Pose PurePursuit::getLastPose(){
    return poseFromPacket(this->path[getLastIndex()]);
}