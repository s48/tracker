// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef COLUMN_DATA_HPP
#define COLUMN_DATA_HPP

#include "camera.hpp"
#include "voxel-data-file.hpp"
#include "constant.hpp"
#include "cruft.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>

// Status for voxels that are neither background or unseen.  In the
// data file this is replaced by the number of rays that must see
// through a voxel to consider it empty.
static const uint8_t voxelIsEmpty = 0;

// Indicates that a camera ray can see a particular voxel.
struct Intersection {
    int32_t voxelIndex;     // which voxel
    float depthInM;  // the voxel's distance from the camera
    Intersection(uint32_t voxelIndex,
                 float depthInM) :
        voxelIndex(voxelIndex),
        depthInM(depthInM)
        {
        }
    Intersection() :
        voxelIndex(0),
        depthInM(0)
        {
        }
};

struct CameraRays {
    uint32_t mIndex; // for debugging
    bool mBackgroundOnly; // true if this camera is not used at runtime
    Eigen::Affine3f mWorldToCamera;
    // Background depth image for this camera.  Read from a file.
    std::vector<float> mBackgroundImage;

    // Shared between all cameras - marks each voxel as one of
    // voxelIsUnseen, voxelIsBackground, or voxelIsEmpty
    std::vector<uint8_t>& mVoxelStatus;

    // How far each voxel is from the camera.
    std::vector<uint16_t> mVoxelDistances;

    // Which voxels each camera ray sees.
    std::array<std::vector<Intersection>, imagePixelCount> mIntersections;

    // Just the voxel indexes from mIntersections, minus any voxels
    // that turn out to be background.
    std::array<std::vector<uint32_t>, imagePixelCount> mHitVoxels;

    // A per-voxel array with the number of times each voxel appears in
    // an intersection arry.
    std::vector<uint8_t> mVoxelHitCounts;

    CameraRays(uint32_t index,
               nlohmann::json config,
               std::vector<float>,
               std::vector<uint8_t>& voxelStatus);
    void makeCameraRays();
    void makeVoxelData();
};

void makeVoxels(void);

const uint8_t realPass = 0;
const uint8_t emptyPass = 1;
const uint8_t intersectPass = 2;

#endif // COLUMN_DATA_HPP
