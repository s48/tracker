// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef CONSTANT_HPP
#define CONSTANT_HPP

#include <cstdint>

const int maxCameras = 8;

const float voxelSizeInM = 0.05f;
const float voxelSizeInCm = (voxelSizeInM * 100.0f);
const float voxelsPerMeter = (1 / voxelSizeInM);

const float maxCameraRangeInM = 8.0f;

const uint16_t floorSize = 200;  // 10 meters * 20 voxels per meter
const uint16_t cellHeight = 40;  // 2 meters * 20 voxels per meter

#endif // CONSTANT_HPP
