// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"

#include <array>
#include <cstdint>

// Modes set by user input.
extern bool printClouds;
extern bool printBlobs;

enum class Show {
    combined,      // combine voxel types in each column
    maybeOccupied, // unknown and maybe occupied
    layer,         // the voxel type for each voxel in the current layer
    layerBlobs,    // which blob each voxel belongs to
    layerCoverage  // which voxels can be seen
};

void updateDepthTexture(Image16& depthImage, uint32_t index);

bool initDisplay(void);
bool checkDisplay(void);
void update_display(const std::vector<uint32_t>& floorImageData);
