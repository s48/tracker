// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "pixel.hpp"
#include "distance-map.hpp"

#include <cstdint>
#include <cstring>
#include <tuple>

#include <assert.h>

// We use the low-order bit of the mDistances[] values to mark squares
// that are in the mToDo queue.  Using the low-order bit lets use compare
// distance values without first clearing the bit.
static const uint16_t queuedFlag = 1;


// Boundary squares are ignored and cannot be marked open.
static const uint16_t boundary = 0xFFFE;
// Blocked squares are ignored but can be marked open.
static const uint16_t blocked = 0xFFFD;
// Distances propagate into open squares.
static const uint16_t open = 0xFFFC;
// None of the special values have the queued flag set.

void DistanceMap::reinitialize()
{
    mDistances.fill(mOpenDefault ? open : blocked);

    // Block the borders to avoid having to do range checks
    // when propagating values.
    // This hack cuts the propagation time in half, but means that
    // the borders are off limits.  Could they be marked in some 
    // other way?  Set the high bit and check that for a slower
    // path?
    for (uint32_t i = 0; i < floorSize; i++) {
        mDistances[floorIndex(i, 0)] = boundary;
        mDistances[floorIndex(i, floorSize - 1)] = boundary;
        mDistances[floorIndex(0, i)] = boundary;
        mDistances[floorIndex(floorSize - 1, i)] = boundary;
    }
    // Clear out anything in the queue.
    while (!mToDo.empty()) {
        mToDo.pop();
    }
    mEmpty = true;
}

void DistanceMap::setOccupied(const uint32_t index)
{
    mEmpty = false;
    if (!(mDistances[index] & queuedFlag)) {
        mToDo.push(index);
    }
    mDistances[index] = queuedFlag; // distance is zero
}

void DistanceMap::setOpen(const uint32_t index)
{
    if (mDistances[index] == blocked) {
        mDistances[index] = open;
    }
}

uint16_t DistanceMap::distance(const uint32_t index)
{
    return mDistances[index] >> 1;
}

uint16_t DistanceMap::distance(const uint8_t x, const uint8_t y)
{
    return mDistances[floorIndex(x, y)] >> 1;
}

void  DistanceMap::propagateDistances()
{
    while (!mToDo.empty()) {
        uint16_t next = mToDo.front();
        mToDo.pop();
        propagateDistance(next);
    }
}

void DistanceMap::propagateDistance(uint16_t index)
{
    uint16_t value = mDistances[index];
    mDistances[index] = value & ~queuedFlag;
    value |= queuedFlag;
    value += 2;
    auto offsets = directionOffsets(index);
    for (uint16_t i = 0; i < 6; i++) {
        uint16_t j = index + offsets[i];
        if (j < floorSize * floorSize
            && value < mDistances[j]
            && mDistances[j] < blocked) {
            if (! (mDistances[j] & queuedFlag)) {
                mToDo.push(j);
            }
            mDistances[j] = value;
        }
    }
}

// Finds the nearest occupied square to 'index' by repeatedly
// moving to the neighbor with the lowest distance.

std::tuple<uint16_t, uint16_t> DistanceMap::reversePath(uint32_t index)
{
    assert(mDistances[index] != blocked
            && mDistances[index] != open);
    uint32_t startIndex = index;
    while (0 < mDistances[index]) {
        uint32_t next = index;
        auto offsets = directionOffsets(index);
        for (uint16_t i = 0; i < 6; i++) {
            uint16_t j = index + offsets[i];
            if (mDistances[j] < mDistances[next]) {
                next = j;
            }
        }
        if (next == index) {
            fprintf(stderr, "[no reverse path from %d %d, at %d %d]\n",
                    startIndex % floorSize, startIndex / floorSize,
                    index % floorSize, index / floorSize);
            for (int i = 0; i < floorSize; i++) {
                for (int j = 0; j < floorSize; j++) {
                    fprintf(stderr, " %3d", mDistances[i + j * floorSize] >> 1);
                }
                fprintf(stderr, "\n");
            }
        }
        assert(next != index);
        index = next;
    }
    return std::make_tuple(index % floorSize, index / floorSize);
}

void DistanceMap::print()
{
    for (int i = 0; i < floorSize; i++) {
        for (int j = 0; j < floorSize; j++) {
            uint16_t value = mDistances[floorIndex(i, j)];
            if (value == boundary) {
                fprintf(stderr, "##");
            } else if (value == blocked) {
                fprintf(stderr, "==");
            } else if (value == open) {
                fprintf(stderr, "  ");
            } else {
                fprintf(stderr, "%2X", value >> 1);
            }
        }
        fprintf(stderr, "\n");
    }
}

void distanceTest()
{
    DistanceMap map;
    map.setOpenDefault(false);
    map.reinitialize();
    for (uint32_t x = 5; x < 30; x++) {
        for (uint32_t y = 5; y < 30; y++) {
            map.setOpen(floorIndex(x, y));
        }
    }
    map.setOccupied(floorIndex(10, 10));
    map.setOccupied(floorIndex(20, 8));
    fprintf(stderr, "occupied\n");
    map.print();
    map.propagateDistances();
    fprintf(stderr, "distances\n");
    map.print();
}
