// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef FUSION_HPP
#define FUSION_HPP

#include "blob.hpp"
#include "distance-map.hpp"
#include "robot.hpp"
#include "camera.hpp"

#include <nlohmann/json.hpp>
#include "ThreadPool.h"

#include <array>
#include <cstdint>

// Debugging options for what to display in the floor map.
extern bool examineColumns;
extern bool printColumn;
extern bool printRays;
extern int16_t cursorX;
extern int16_t cursorY;

// The floor in whatever color scheme the current mode calls for.
extern Floor32 floorImage;
void clearFloorImage(void);
void setFloorPixelLevel(const uint32_t x, const uint32_t y, const uint8_t level);
void setFloorPixelLevel(const uint32_t index, const uint8_t level);
void setFloorPixel(const uint32_t index, uint32_t color);
void setFloorPixel(const uint32_t x, const uint32_t y, uint32_t color);

// The distance from each floor square to the nearest floor square
// that the robot is above.
extern DistanceMap robotDistances;

void initializeEngine(const nlohmann::json& config, const std::string& configDirectory);

//----------------------------------------------------------------
// Image processing steps:

// 1. Use the depth images to determine the empty/occupied/unobserved state of each voxel.
//void traceRays(ThreadPool& threadPool, const Image16 * const * depthImages);
void traceRays(ThreadPool& threadPool, const std::vector<Image16>& depthImages);

// 2. Use the robots joint positions to mark the voxels occupied by the robot.
void renderRobot(const Robot& robot, const std::vector<float>& pose);

// 3. Merge adjacent voxels of the same type in to blobs.
Blob *mergeBlobs(ThreadPool& threadPool);

// 4. Find the distance from each blob to the robot.
uint32_t checkRobotDistances(Blob *allBlobs);

// 5. Display the final result.
void showFloor(Blob *allBlobs, std::vector<uint32_t>& floorImageData);

#endif // FUSION_HPP
