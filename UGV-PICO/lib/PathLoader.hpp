#include "../../UGV-ESP/UGV-ESP/network_defines.hpp"
#include <iostream>

#define PATH_MAX_POINTS 56
// #define PATH_MAX_POINTS 3

class PathLoader
{
private:
    packet_path_point paths[2][PATH_MAX_POINTS]; // Swap buffer
    int activePathIndex;
    int loadingIndex;
    void (*swapCallback)();
    

public:
    PathLoader(void (*swapCallback)());
    void load(packet_path_point point);
    packet_path_point *getActivePath();
};

PathLoader::PathLoader(void (*swapCallback)())
{
    this->activePathIndex = 0;
    this->loadingIndex = 0;
    for(int i = 0; i < PATH_MAX_POINTS; i++){
        this->getActivePath()[i] = {.code = PACKET_PATH, .x = 0.0, .y = 0.0};
    }
    this->swapCallback = swapCallback;
}

packet_path_point *PathLoader::getActivePath()
{
    return this->paths[activePathIndex];
}

void PathLoader::load(packet_path_point point)
{
    int inactiveIndex = (this->activePathIndex + 1) % 2;
    // std::cout << "edit path index: " << inactiveIndex << std::endl;
    // std::cout << "edit point index: " << loadingIndex << std::endl;
    // std::cout << "code: " << (int)(point.code) << std::endl;
    // std::cout << "x: " << point.x << std::endl;
    // std::cout << "y: " << point.y << std::endl;
    this->paths[inactiveIndex][loadingIndex++] = point;
    if (loadingIndex >= PATH_MAX_POINTS)
    {
        // std::cout << "Full Buffer" << std::endl;

        // for(int i = 0; i < PATH_MAX_POINTS; i++){
        //     std::cout<< " x: " << this->paths[inactiveIndex][i].x;
        // }
        // std::cout << std::endl;

        loadingIndex = 0;
        this->activePathIndex = (this->activePathIndex + 1) % 2;
        this->swapCallback();
    }
}
