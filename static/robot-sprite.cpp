// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "directions.hpp"
#include "mesh.hpp"
#include "pixel.hpp"
#include "raster.hpp"
#include "robot.hpp"
#include "util.hpp"
#include "robot-sprite.hpp"

//#include <unordered_set>
#include <vector>

#include <math.h>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

std::vector<float> minControlRadians;
std::vector<float> maxControlRadians;
// std::vector<float> maxJointVelocityRadiansPerSec;

void renderSprite(Canvas&  canvas, const Mesh& cloud, const Eigen::Affine3f& transform)
{
    for (const auto& point : cloud.getPoints()) {
        const Eigen::Vector3f& vCam = transform * point;
        int16_t y = floatToInt32u(vCam.y() * voxelsPerMeter + 100.5);
        float vCamX = vCam.x();
        if ((y & 0) == 0) {
            vCamX -= voxelSizeInM / 2.0f;
        }
        int16_t x = floatToInt32u(vCamX * voxelsPerMeter + 100.5);
        if (x < 0 || floorSize <= x || y < 0 || floorSize <= y) {
            fprintf(stderr, "vCam (%.2f, %.2f) x %d y %d\n", vCamX, vCam.y(), x, y);
            assert(false);
        } else {
            canvas[x + y * floorSize] = 1;
        }
    }
}

void drawBoundary(Canvas&  canvas,
                  const std::vector<uint16_t>& data,
                  uint32_t index)
{
//    fprintf(stderr, "[draw %d %d boundary ", data[index], data[index + 1]);
    uint32_t pixelIndex = data[index];
    uint16_t count = data[index + 1];
    index += 2;
    uint16_t bits = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (i % 5 == 0) {
            bits = data[index++];
        }
//        fprintf(stderr, "%d", bits % 8);
        pixelIndex += directionOffsets(pixelIndex)[bits % 8];
        canvas[pixelIndex] = 1;
        bits >>= 3;
    }
//    fprintf(stderr, "]\n");
}

void addSprite(std::vector<uint32_t>& offsets,
               std::vector<uint16_t>& data,
               const Canvas&  canvas)
{
    // walk the y = 100 row
    uint16_t firstPixel = 100 * floorSize;
    while (canvas[firstPixel] == 0) {
        firstPixel += 1;
    }
    assert(firstPixel - (100 * floorSize) < floorSize);
    std::vector<uint8_t> path;
    firstPixel = walkPixelBoundary(canvas,
                                   firstPixel,
                                   [&path](uint32_t index, uint8_t direction)
                                       {
                                           path.push_back(direction);
                                       });
    offsets.push_back(data.size());
    data.push_back(firstPixel);
    data.push_back(path.size());
    uint16_t count = 0;
    uint16_t accumulator = 0;
    for (auto& direction : path) {
        accumulator |= direction << (count * 3);
        count += 1;
        if (count == 5) {
            data.push_back(accumulator);
            accumulator = 0;
            count = 0;
        }
    }
    if (0 < count) {
        data.push_back(accumulator);
    }
}

// Test routine that creates a sprite of the full robot.

void makeRobotSprite(const Robot& robot,
                     const std::vector<float>& robotPose,
                     std::vector<uint16_t>& data)
{
    const auto& transforms = robot.makeLinkTransforms(robotPose);
    Canvas canvas;
    canvas.fill(0);
    for (uint32_t i = 0; i < transforms.size(); i++) {
        if (robot.getLink(i).cloud != nullptr) {
            renderSprite(canvas, *robot.getLink(i).cloud, transforms[i]);
        }
    }
    std::vector<uint32_t> offsets;
    addSprite(offsets, data, canvas);
}

// Draw a sprite with links 1 and 2 in their position in the pose and links
// 3, 4, and 5 with their full range of motion.  Link 0 is the robot base and
// doesn't move and link 6 moves with whatever the robot is holding.
// We do it this way because links 3, 4, and 5 are small, have a wide range, and
// there may not be any stopping distance data for them.  Showing them with
// their full range of motion is saves a lot of time and does not increase the
// footprint of the robot significantly, it at all.
//
// This will need to be updated to handle dependent links.  For now we assume
// that pose[i] affects link[i] + 1.

void drawRobot(Canvas&  canvas,
               const Robot& robot,
               const std::vector<float>& baseRobotPose,
               std::vector<uint32_t>& positionCounts,
               std::vector<float>& stepSizes)
{
    std::vector<float> robotPose = baseRobotPose;

    // Needs to be part of the robot.
    uint16_t fixedControls = 3;
    uint16_t sweptControls = 2;
    uint16_t maxSweptControl = fixedControls + sweptControls - 1;

    // Draw the basic pose.
    const auto& transforms = robot.makeLinkTransforms(robotPose);
    for (uint16_t i = 0; i < robot.getLinkCount(); i++) {
        renderSprite(canvas, *robot.getLink(i).cloud, transforms[i]);
    }

    // This is N nested loops that hits all combinations of the swept controls.
    std::vector<uint16_t> indexes(robot.getControlCount(), 0);
    uint16_t control = fixedControls;
    do {
        robotPose[control] = minControlRadians[control] + indexes[control] * stepSizes[control];
        indexes[control] += 1;
        // should do this for every link controlled by the control
        renderSprite(canvas,
                     *robot.getLink(control).cloud,
                     robot.makeLinkTransforms(robotPose)[control]);
        if (control < maxSweptControl) {
            control += 1;
        }
        while (indexes[control] == positionCounts[control]) {
            positionCounts[control] = 0;
            robotPose[control] = minControlRadians[control];
            control -= 1;
        }
    } while (fixedControls <= control);
}
