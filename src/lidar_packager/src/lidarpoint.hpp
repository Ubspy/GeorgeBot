#pragma once
#include <cstdint>

struct LidarPoint
{
    float x;
    float y;
    float z;
    uint32_t rgba;
    
    LidarPoint(){}
    LidarPoint(float x, float y, float z, uint32_t rgba)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->rgba = rgba;
    }
};
