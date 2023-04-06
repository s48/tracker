// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"

void startBackgroundScan();
void writePointCloud(uint32_t index, Image16& depthImage);
void backgroundScan(Robot& robot, std::vector<float>& pose, Image16& depthImage);

// Images of just the background or just the robot.
extern Image16 backgroundImage;
extern Image16 robotImage;
