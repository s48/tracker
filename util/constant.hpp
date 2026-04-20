// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef CONSTANT_HPP
#define CONSTANT_HPP

#include <cstdint>

const int maxCameras = 8;

extern float voxelSizeInM;
extern float voxelSizeInCm;
extern float voxelsPerMeter;

const float maxCameraRangeInM = 8.0f;

const uint16_t floorSize = 200;  // 10 meters * 20 voxels per meter
const uint16_t cellHeight = 40;  // 2 meters * 20 voxels per meter

#endif // CONSTANT_HPP
