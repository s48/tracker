// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// Experimenting with how a camera's positions might be determined
// from the images it produces.
//
// The idea is to use the robot, which has a known shape and position,
// to determine a camera's position and direction.  We first take a
// series of images while rotating the robot.  Pixels that don't
// changed are considered background, and removing them leaves an
// image of the robot.  This can then be compared against the robot
// model to determine the location of the camera.  In theory.
//
// The code here generates the background image and writes it to a file.
// It also writes camera images to files as collections of 3D points.

#include "camera.hpp"
#include "command.hpp"
#include "robot.hpp"
#include "socket.hpp"
#include "cruft.hpp"
#include "display.hpp"
#include "render.hpp"
#include "raster.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <functional>

#include <stdio.h>

extern int coreToSimFd;

// Number of images used for background detection.  The robot has a
// different rotation in each image.
#define backgroundImageCount 8

// For each pixel in the background images, this is the minimum number
// of instances of that pixel that have to be close to the maximum depth
// for that pixel for it to be considered background.
#define minBackgroundCount 3

static Image16 backgroundImages[backgroundImageCount];
// Can be displayed.
Image16 backgroundImage;
Image16 robotImage;

static void saveRegisterImage(uint32_t robotRotation, Image16& depthImage)
{
    backgroundImages[robotRotation] = depthImage;
}

static uint16_t maxCameraRangeInCm = maxCameraRangeInM * 100.0f;

static void makeBackgroundImage(Robot& robot, std::vector<float>& pose)
{
    backgroundImage.fill(0);

    for (uint32_t iPixel = 0; iPixel < imagePixelCount; iPixel++) {
        uint16_t max = 0;

        // Find the maximum distance for this pixel in any image.
        for (uint32_t i = 0; i < backgroundImageCount; i++) {
            uint16_t depthCm = backgroundImages[i][iPixel];
            if (depthCm < 4000
                && max < depthCm) {
                max = depthCm;
            }
        }

        // Find the number of images in which the pixel is at the maximum.
        uint32_t backgroundCount = 0;
        for (uint32_t i = 0; i < backgroundImageCount; i++) {
            uint16_t depthCm = backgroundImages[i][iPixel];
            if (depthCm < 4000
                && max - depthCm < 4) {
                backgroundCount += 1;
            }
        }

        // If enough images have the maximum depth then that depth is
        // considered to be the background.  In the other images the
        // pixel is presumably obscured by the robot.
        if (minBackgroundCount < backgroundCount) {
            backgroundImage[iPixel] = max;
        }
    }

    // Pick out all non-background pixels from image seven.
    robotImage.fill(0);
    for (uint32_t iPixel = 0; iPixel < imagePixelCount; iPixel++) {
        uint16_t backgroundDepthCm = backgroundImage[iPixel];
        uint16_t robotDepthCm = backgroundImages[7][iPixel];
        if (robotDepthCm < 4000
            && 10 < abs(backgroundDepthCm - robotDepthCm)) {
            robotImage[iPixel] = robotDepthCm;
        }
    }

    // Zero out all robot pixels that are not adjacent to at least
    // two other robot pixels.  This removes a lot of noise from the
    // robot image.
    for (uint32_t i = 0; i < 2; i++) {
        for (uint32_t iPixel = imageWidth; iPixel < imagePixelCount - imageWidth; iPixel++) {
            if (0 < robotImage[iPixel]
                && (((robotImage[iPixel - 1] != 0 ? 1 : 0)
                     + (robotImage[iPixel + 1] != 0 ? 1 : 0)
                     + (robotImage[iPixel - imageWidth] != 0 ? 1 : 0)
                     + (robotImage[iPixel + imageWidth] != 0 ? 1 : 0))
                    < 2)) {
                robotImage[iPixel] = 0;
            }
        }
    }

    // Write out the robot image as a point cloud file.
    Mesh robotCloud;
    for (uint32_t iPixel = imageWidth; iPixel < imagePixelCount - imageWidth; iPixel++) {
        if (0 < robotImage[iPixel]) {
            // Convert depths to meters.
            robotCloud.addPoint(pixelNormal[iPixel] * (robotImage[iPixel] / 100.0f));
        }
    }
    robotCloud.writePlyFile("robot-image.ply");

    // This writes out three point cloud files for the robot, each with a
    // different 20% of the robots points.  This could just as well be done
    // by a stand-alone program as it does not use the image files.
    const std::vector<Eigen::Affine3f>& transforms = robot.makeLinkTransforms(pose);
    for (uint32_t j = 0; j < 3; j++) {
        robotCloud.clear();
        uint32_t count = j;
        for (uint32_t i = 0; i < transforms.size(); i++) {
            if (robot.getLink(i).cloud == nullptr) {
                continue;
            }
            Eigen::Affine3f transform =
                // Center the point cloud around the origin.
                Eigen::Translation3f(Eigen::Vector3f(-cellMinM.x(), -cellMinM.y(), 0.3)) * transforms[i];
            for (const auto& point : robot.getLink(i).cloud->getPoints()) {
                if (count % 5 == 0) {
                    robotCloud.addPoint(transform * point);
                }
                count += 1;
            }
        }
        robotCloud.writePlyFile("robot" + std::to_string(j) + ".ply");
    }
}

static int32_t robotRotation = -1; // How many steps the robot has been rotated.
static uint32_t backgroundScanDelay = 0; // How long to wait before recording an image.

void startBackgroundScan(void)
{
  robotRotation = 0;
  sendCommand(coreToSimFd, "rotate", {robotRotation});
  backgroundScanDelay = 2;

}

// Record a background scan and rotate the robot for the next scan.

void backgroundScan(Robot& robot, std::vector<float>& pose, Image16& depthImage)
{
    if (robotRotation == -1) {
        // do nothing
    } else if (0 < backgroundScanDelay) {
        backgroundScanDelay -= 1;
    } else {
        saveRegisterImage(robotRotation, depthImage);
        robotRotation += 1;
        if (robotRotation < backgroundImageCount) {
            sendCommand(coreToSimFd, "rotate", {robotRotation});
            backgroundScanDelay = 2;
        } else {
            makeBackgroundImage(robot, pose);
            robotRotation = -1;
        }
    }
}

// Write a file containing the depth image translated into a set of
// 3D points.
void writePointCloud(uint32_t index, Image16& depthImage)
{
    uint32_t count = 0;
    for (uint32_t iPixel = 0; iPixel < imagePixelCount; iPixel++) {
        uint16_t rawDepth = depthImage[iPixel];
        if (rawDepth != 0 and rawDepth != 1000) {
            count += 1;
        }
    }
    std::string file = "camera-cloud" + std::to_string(index) + ".dat";
    std::ofstream fout(file, std::ios::out | std::ios::binary);
    char header[80] = "point cloud";
    fout.write(header, 80);
    fout.write((char*)&count, 4);
    for (uint32_t iPixel = 0; iPixel < imagePixelCount; iPixel++) {
        uint16_t rawDepth = depthImage[iPixel];
      if (rawDepth != 0 and rawDepth != 1000) {
        float distanceM = ((float) rawDepth) / 100.0f;
        Eigen::Vector3f point = pixelNormal[iPixel] * distanceM;
        fout.write((char*)point.data(), 12);
      }
    }
    fout.close();
}
