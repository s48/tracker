// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef DISTANCE_MAP_HPP
#define DISTANCE_MAP_HPP

#include "constant.hpp"
#include "cruft.hpp"

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <queue>

class DistanceMap
{
public:
    DistanceMap() {}

    void reinitialize();
    void setOccupied(const uint32_t index);
    uint16_t distance(const uint32_t index);
    uint16_t distance(const uint8_t x, const uint8_t y);
    void propagateDistances();
    std::tuple<uint16_t, uint16_t> reversePath(uint32_t index);
    bool empty() { return mEmpty; }
    void setOpenDefault(bool defaultValue) { mOpenDefault = defaultValue; }
    void setOpen(const uint32_t index);
    void print();

private:
    bool mOpenDefault = true;
    Floor16 mDistances;
    std::queue<uint16_t> mToDo;
    bool mEmpty;
  
    void propagateDistance(uint16_t index);
};

#endif // DISTANCE_MAP_HPP

