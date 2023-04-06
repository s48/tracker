// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef ROBOT_SPRITE_HPP
#define ROBOT_SPRITE_HPP

#include "robot.hpp"
#include "pixel.hpp"

#include <cstdint>
#include <vector>

#include <math.h>

// Use a step size of pi/32, which is about six degrees.
const float angleDelta = M_PI / 32.0f;

extern std::vector<float> minControlRadians;
extern std::vector<float> maxControlRadians;
// extern std::vector<float> maxJointVelocityRadiansPerSec;

void renderSprite(Canvas&  canvas, const Mesh& cloud, const Eigen::Affine3f& transform);

void addSprite(std::vector<uint32_t>& offsets,
               std::vector<uint16_t>& data,
               const Canvas&  canvas);

void makeRobotSprite(const Robot& robot,
                     const std::vector<float>& robotPose,
                     std::vector<uint16_t>& data);

void drawBoundary(Canvas&  canvas,
                  const std::vector<uint16_t>& data,
                  uint32_t index);

void drawRobot(Canvas&  canvas,
               const Robot& robot,
               const std::vector<float>& baseRobotPose,
               std::vector<uint32_t>& positionCounts,
               std::vector<float>& stepSizes);

#endif // ROBOT_SPRITE_HPP
