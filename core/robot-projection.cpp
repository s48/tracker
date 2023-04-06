// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// Generate an outline on the floorImage of the possible future positions
// of the robot.
//
// My original approach to this was too complicated, so I'm starting over.

#include "cruft.hpp"
#include "robot.hpp"
#include "directions.hpp"
#include "distance-map.hpp"
#include "pixel.hpp"
#include "fusion.hpp"
#include "robot-sprite.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <unistd.h>

extern void setFloorPixelLevel(const uint32_t x,
                               const uint32_t y,
                               const uint8_t level);
void setFloorPixel(const uint32_t x,
                   const uint32_t y,
                   uint32_t color);
extern bool debugPrint;
extern uint8_t robotSpriteLevel;

//static std::vector<float> maxJointVelocityRadiansPerSec;
//static std::vector<float> maxJointStopRadians;

static uint32_t drawSprite(Canvas&  canvas, const std::vector<uint16_t> sprite, uint32_t index)
{
    uint32_t pixelIndex = sprite[index];
    uint16_t count = sprite[index + 1];
    index += 2;
    uint16_t bits = 0;
    canvas[pixelIndex] = 1;
    for (uint32_t i = 0; i < count; i++) {
        if (i % 5 == 0) {
            bits = sprite[index++];
        }
        pixelIndex += directionOffsets(pixelIndex)[bits % 8];
        canvas[pixelIndex] = 1;
        bits >>= 3;
    }
    return pixelIndex;
}

// Two dimensional arrays for sprites.  For robot links the dimensions
// are the angles of joints 1 and 2.  For end effectors they are the
// index of the direction in which it points and the rotation.
struct SpriteSet {
    uint16_t dimension0;
    uint16_t dimension1;
    float step0;
    float step1;
    std::vector<uint32_t> offsets;
    std::vector<uint16_t> sprites;

    SpriteSet() {
    }

    // Robot link version.
    uint32_t offset(float angle0, float angle1) const {
        return offsets[angleToIndex(angle0, dimension0, step0) * dimension1
                       + angleToIndex(angle1, dimension1, step1)];
    }

    // End effector version
    uint32_t direction(const Eigen::Affine3f& transform, uint32_t& angleIndex) const
    {
        Eigen::Vector3f dir = Eigen::Vector3f(transform(0, 0), transform(1, 0), transform(2, 0));
        dir.normalize();
        uint32_t directionIndex = closestDirection(dir);
        bool xzx = fabs(directions[directionIndex].x()) < sqrt(0.5f);
        float angle = (xzx
                       ? atan2f(transform(0, 2), -transform(0, 1))
                       : atan2f(-transform(1, 2), transform(1, 1)));
        angleIndex = angleToIndex(angle, 64, angleDelta);
        return directionIndex;
    }

    uint32_t offset(uint32_t directionIndex, uint32_t angleIndex) const
    {
        return offsets[angleIndex + directionIndex * dimension0];
    }

    uint32_t offset(const Eigen::Affine3f& transform) const
    {
        uint32_t angleIndex;
        uint32_t directionIndex = direction(transform, angleIndex);
        return offset(directionIndex, angleIndex);
    }

    uint32_t draw(uint32_t index, Canvas& canvas) const
    {
        drawSprite(canvas, sprites, index);
        return sprites[index];
    }

    uint32_t drawLinkSprites(float lowAngle0,
                             float highAngle0,
                             float lowAngle1,
                             float highAngle1,
                             Canvas& canvas) const {
        uint32_t low0 = angleToIndex(lowAngle0, dimension0, step0);
        uint32_t high0 = angleToIndex(highAngle0, dimension0, step0);
        uint32_t low1 = angleToIndex(lowAngle1, dimension1, step1);
        uint32_t high1 = angleToIndex(highAngle1, dimension1, step1);
        for (uint32_t i0 = low0; i0 <= high0; i0++) {
            for (uint32_t i1 = low1; i1 <= high1; i1++) {
                drawSprite(canvas, sprites, offsets[i0 * dimension1 + i1]);
            }
        }
        return sprites[offsets[low0 + dimension1 + low1]];
    }

private:
    uint32_t angleToIndex(float angle, uint16_t limit, float step) const
    {
        while (angle < 0.0f) {
            angle += 2 * M_PI;
        }
        angle = fmod(angle, 2 * M_PI);
        uint32_t index = std::floor((angle + step / 2) / step);
        return std::min(index, (uint32_t) (limit - 1));
    }
    uint32_t angleToIndexP(float angle, uint16_t limit, float step) const
    {
        while (angle < 0.0f) {
            angle += 2 * M_PI;
        }
        angle = fmod(angle, 2 * M_PI);
        uint32_t index = std::floor((angle + step / 2) / step);
        fprintf(stderr, "[angle %.2f limit %d step %.2f -> %d]\n",
                angle, limit, step,
                (uint32_t) std::min(index, (uint32_t) (limit - 1)));
        return std::min(index, (uint32_t) (limit - 1));
    }
};

// Joint 0 we handle by rotating the sprite.
// Joints 1 and 2 index into the sprite set.
// The remaining links are small and each sprite includes the full
// range of each.

static SpriteSet robotSprites;
static SpriteSet endEffectorSprites;

static void readSpriteSet(const std::string& filename,
                          SpriteSet& sprites)
{
    std::ifstream fin(filename, std::ios::in | std::ios::binary);
    if (!fin.is_open()) {
        fprintf(stderr, "Can't open sprite data file '%s'\n", filename.c_str());
        exit(0);
    }
    char header[80];
    fin.read(header, 80);
    fin.read((char *) &sprites.dimension0, sizeof(uint16_t));
    fin.read((char *) &sprites.dimension1, sizeof(uint16_t));
    fin.read((char *) &sprites.step0, sizeof(float));
    fin.read((char *) &sprites.step1, sizeof(float));
    uint32_t dataSize;
    fin.read((char *) &dataSize, sizeof(uint32_t));

    sprites.offsets.resize(sprites.dimension0 * sprites.dimension1);
    fin.read((char *) sprites.offsets.data(), sprites.offsets.size() * sizeof(uint32_t));
    sprites.sprites.resize(dataSize);
    fin.read((char *) sprites.sprites.data(), dataSize * sizeof(uint16_t));
    fprintf(stderr, "[link2 %d link3 %d read in %ld offsets %ld sprite values]\n",
            sprites.dimension0,
            sprites.dimension1,
            sprites.offsets.size(),
            sprites.sprites.size());
    fin.close();
}

void initializeRobotProjection(const nlohmann::json& config,
                               const std::string& configDirectory)
{
    const nlohmann::json& robotJson = config["robot"];
    Robot robot(configDirectory
                + "/"
                + robotJson["urdf"].get<std::string>());

    minControlRadians.resize(robot.getControlCount());
    maxControlRadians.resize(robot.getControlCount());
    for (uint16_t i = 0; i < robot.getControlCount(); i++) {
        minControlRadians[i] = robot.getLink(i).minRadians;
        maxControlRadians[i] = robot.getLink(i).maxRadians;
    }

    for (uint16_t i = 0; i < minControlRadians.size(); i++) {
        // First 33ms is the time until the next image arrives, second
        // is the safety processor's timeout plus latency between the
        // safety processor and the robot controller.
//        maxJointStopRadians[i] += maxJointVelocityRadiansPerSec[i] * (0.033 + 0.033);
    }

    // fprintf(stderr, "[min %.4f %.4f %.4f]\n", minControlRadians[0], minControlRadians[1], minControlRadians[2]);
    // fprintf(stderr, "[max %.4f %.4f %.4f]\n", maxControlRadians[0], maxControlRadians[1], maxControlRadians[2]);

    readSpriteSet(configDirectory + "/" + robotJson["robot_sprites"].get<std::string>(), robotSprites);
//    readSpriteSet(configDirectory + "/" + robotJson["end_effector_sprites"].get<std::string>(), endEffectorSprites);
}

// Making this more accurate just costs time at file generation, so we
// could make this smaller.
static const float angleStopDelta = M_PI / 32.0f;

static bool shown = false;

static uint32_t drawRobotSprites(const SpriteSet& linkSprites,
                                 const SpriteSet& endEffectorSprites,
                                 const Robot& robot,
                                 const std::vector<float>& minPose,
                                 const std::vector<float>& maxPose,
                                 Canvas& canvas)
{
    if (robotSpriteLevel == 1) {
        std::vector<uint16_t> data;

        std::vector<float>pose = minPose;
        for (uint32_t i = 0; i < pose.size(); i++) {
            pose[i] = (pose[i] + maxPose[i]) / 2;
        }
        float angle = pose[0] - minControlRadians[0];
        pose[0] = minControlRadians[0];

        std::vector<uint32_t> positionCounts(minPose.size(), 1);
        std::vector<float> stepSizes(minPose.size(), 0);
        drawRobot(canvas, robot, pose, positionCounts, stepSizes);
        uint32_t pixelIndex = 100 + 100 * floorSize;

        std::vector<uint8_t> path;
        pixelIndex = walkPixelBoundary(canvas,
                                       pixelIndex,
                                       [&path](uint32_t index, uint8_t direction) {
                                           path.push_back(direction);
                                       });
        canvas.fill(0);

        rotatePath(canvas,
                   path,
                   pixelIndex % floorSize,
                   pixelIndex / floorSize,
                   angle,
                   100,        // center of rotation
                   100);
        path.resize(0);
        uint32_t rowStart = pixelIndex - pixelIndex % floorSize;
        for (uint32_t i = rowStart; i < rowStart + floorSize; i++) {
            if (canvas[i]) {
                pixelIndex = i;
                break;
            }
        }
        pixelIndex = walkPixelBoundary(canvas,
                                       pixelIndex,
                                       [&path](uint32_t index, uint8_t direction) {
                                           path.push_back(direction);
                                       });
        canvas.fill(0);
        drawPixelBoundary(canvas, pixelIndex, path);
        return pixelIndex;
    } else if (robotSpriteLevel == 2) {
        uint32_t pixelIndex =
            linkSprites.drawLinkSprites(minPose[1] - minControlRadians[1],
                                        maxPose[1] - minControlRadians[1],
                                        minPose[2] - minControlRadians[2],
                                        maxPose[2] - minControlRadians[2],
                                        canvas);

//        drawEndEffectorSprites(canvas, robot, pose);
        std::vector<uint8_t> path;
        pixelIndex = walkPixelBoundary(canvas,
                                       pixelIndex,
                                       [&path](uint32_t index, uint8_t direction) {
                                           path.push_back(direction);
                                       });
        canvas.fill(0);
        float minAngle = minPose[0] - minControlRadians[0];
        float maxAngle = maxPose[0] - minControlRadians[0];
        float diff = fmod((maxAngle - minAngle) + 2 * M_PI, 2 * M_PI);
        if (false && debugPrint) {
            fprintf(stderr, "[min %.4f max %.4f diff %.4f]\n", minAngle, maxAngle, diff);
            debugPrint = false;
        }

        if (diff < M_PI) {
            rotatePath(canvas,
                       path,
                       pixelIndex % floorSize,
                       pixelIndex / floorSize,
                       maxAngle,
                       100,        // center of rotation
                       100);
            for (float delta = 0.0f; delta < diff; delta += angleDelta) {
                rotatePath(canvas,
                           path,
                           pixelIndex % floorSize,
                           pixelIndex / floorSize,
                           minAngle + delta,
                           100,        // center of rotation
                           100);
            }
        } else {
            rotatePath(canvas,
                       path,
                       pixelIndex % floorSize,
                       pixelIndex / floorSize,
                       minAngle,
                       100,        // center of rotation
                       100);
            for (float delta = 0.0f; delta < ((2 * M_PI) - diff); delta += angleDelta) {
                rotatePath(canvas,
                           path,
                           pixelIndex % floorSize,
                           pixelIndex / floorSize,
                           maxAngle + delta,
                           100,        // center of rotation
                           100);
            }
        }
        return pixelIndex;
    } else if (robotSpriteLevel == 3) {
//        drawEndEffectorSprites(canvas, robot, pose);
    }
    return 0;
}

static void markBoundary(uint32_t index, uint8_t direction)
{
    // Convert from static coords (center is 100,100) to cell coords.
    int32_t x = index % floorSize - cellMin.x() - 100;
    int32_t y = index / floorSize - cellMin.y() - 100;
    if (0 <= x && x < cellSize.x() && 0 <= y && y < cellSize.y()) {
        robotDistances.setOccupied(floorIndex(x, y));
        setFloorPixel(x, y, 0xFFFFFF);
    }
}

void addRobotProjection(const Robot& robot,
                        const std::vector<float>& pose)
{
    if (robotSpriteLevel == 0) {
        return;
    }
    Canvas canvas;
    canvas.fill(0);

    // Set the first join to one end of its range.
    if (false && debugPrint) {
        std::vector<float> basePose(pose);
        basePose[0] = 0.0;
        const Eigen::Affine3f& transform = robot.makeLinkTransforms(basePose)[3];
        fprintf(stderr, "[joint 3 angle %.4f %.4f pose[1] %.4f pose[2] %.4f pose[3] %.4f]\n",
                atan2f(transform(0, 2), transform(0, 0)),
                atan2f(transform(0, 2), transform(0, 0)) * 180.0f / M_PI,
                pose[1], pose[2], pose[3]);
        debugPrint = false;
    }

    uint16_t maxRadius[2] = { 0, 0 };
    uint16_t maxRadiusX[2];
    uint16_t maxRadiusY[2];

    // 0.4 is a stand in for the stopping angles of the first three joints
    std::vector<float> minPose = pose;
    std::vector<float> maxPose = pose;
    for (uint32_t i = 0; i < 3; i++) {
        minPose[i] = std::max(pose[i] - 0.4f, minControlRadians[i]);
        maxPose[i] = std::min(pose[i] + 0.4f, maxControlRadians[i]);
    }
    uint32_t pixelIndex =
        drawRobotSprites(robotSprites, endEffectorSprites, robot, minPose, maxPose, canvas);

    if (false && debugPrint) {
        printPixels(canvas);
        debugPrint = false;
    }

    // Faster to walk in along y = 100.
    if (false && debugPrint) {
        for (uint32_t index = 0; index < sizeof(canvas); index ++) {
            if (canvas[index]) {
                setFloorPixel(index % floorSize, index / floorSize, 0xFFFFFF);
            }
        }
        return;
    }

    robotDistances.reinitialize();
    for (int i = 0; i < sizeof(canvas); i++) {
        if (canvas[i]) {
            walkPixelBoundary(canvas, i, markBoundary);
            break;
        }
    }
    robotDistances.propagateDistances();
}
