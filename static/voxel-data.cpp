// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "voxel-data.hpp"
#include "voxel-data-file.hpp"
#include "cruft.hpp"
#include "raster.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <array>
#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstdlib>
#include <cstring>

static bool debug = false;
bool debugPrint = false;

// Giving rays priority.  The idea is that we get more accuracy if the same ray
// is used through a sequence of voxels rather than randomly picking which rays
// are checked for each voxel.  At runtime we trace each ray in towards the
// camera marking voxels as hit or hidden.  For one scene, these are the stats
// for:
//  - hit voxels, where a ray's depth ended in the voxel
//  - hidden voxels, where a ray's depth ended before reaching the voxel
//  - skipped voxels: voxels where a ray's depth ended somewhere past the voxel
//   random selection: hits 26038 hidden 1516182 skipped 1166446
//   using priority:   hits 28533 hidden 1612789 skipped  901059
// Using the priority to pick which rays to check for each voxel made
// a small increase in the number of marked voxels and reduced the number
// of unmarked ones.
//
// The code for bit interleaving and reversal are from
// https://graphics.stanford.edu/~seander/bithacks.html

// We cache all the pixelRank() values.
static std::array<uint32_t, imagePixelCount> pixelRanks;

static uint32_t pixelRank(uint16_t x, uint16_t y)
{
  // Interleave the bits from the two coordinates.  This gives the
  // Z-order curve which has the property that pixels that are close
  // to each other generally have nearby values.
  uint32_t xy = 0;
  for (int i = 0; i < 16; i++) {
    xy |= (((x & 1) << i) << i
                    | ((y & 1) << i) << (i + 1));
  }

  // Reverse the bits.  Now pixels that are close to each other
  // will generally have values that are far apart.
  uint32_t rank = xy;
  int zeros = 31; // will be the number of high-order zeros in xy
  for (xy >>= 1; xy; xy >>= 1) {
    rank <<= 1;
    rank |= xy & 1;
    zeros -= 1;
  }
  rank <<= zeros; // shift when xy's highest bits are zero

  // Invert the result to get my origial conception of the rank to
  // use.  This probably doesn't matter.
  return ~rank;
}

// The centers of the voxels in configuration space.
static std::vector<Eigen::Vector3f> voxelCenters;
// A cube centered on the origin rendered as triangles.  This is a little
// smaller than an voxel so that any ray intersecting it must go through
// a substantial portion of the voxel.
static std::vector<Eigen::Vector3f> voxelTriangles;

void makeVoxels(void)
{
  // *0.8 because we don't want to count rays that just clip the outside edge of
  // a voxel.
  voxelTriangles = makeVoxelTriangles(voxelSizeInM * 0.8);

  fprintf(stderr, "(%d %d %d) (%d %d %d)\n",
          cellMin.x(), cellMin.y(), cellMin.z(),
          cellMax.x(), cellMax.y(), cellMax.z());
    for (int y = cellMin.y(); y < cellMax.y(); y++) {
        for (int x = cellMin.x(); x < cellMax.x(); x++) {
            float yf = y;
            if (((x - cellMin.x()) & 1) == 0) {
                yf += 0.5;
            }
            for (int z = cellMin.z(); z < cellMax.z(); z++) {
                voxelCenters.push_back(
                    Eigen::Vector3f((float) x * voxelSizeInM + voxelSizeInM / 2,
                                           yf * voxelSizeInM + voxelSizeInM / 2,
                                    (float) z * voxelSizeInM + voxelSizeInM / 2));
            }
        }
    }

    uint32_t pixelIndex = 0;
    for (int y = 0; y < imageHeight; y++) {
      for (int x = 0; x < imageWidth; x++) {
        pixelRanks[pixelIndex] = pixelRank(x, y);
        pixelIndex += 1;
      }
    }
}

// There is some horrible confusion about whether the Y axis is at the
// top or bottom of an image.  This swaps from one to the other.
inline uint32_t reverseY(uint32_t i)
{
  return (i % imageWidth) + (imageHeight - (i / imageWidth) - 1) * imageWidth;
}

CameraRays::CameraRays(uint32_t index,
                       nlohmann::json config,
                       std::vector<float>backgroundImage,
                       std::vector<uint8_t>& voxelStatus) :
    mIndex(index),
    mBackgroundImage(backgroundImage),
    mVoxelStatus(voxelStatus),
    mBackgroundOnly(config.count("port") == 0)
{
  if (! mBackgroundOnly) {
    mVoxelHitCounts.resize(voxelCount);
    mVoxelDistances.resize(voxelCount);
  }

  Eigen::Vector3f position = toVec<Eigen::Vector3f>(config["position"]);
  Eigen::Vector3f rpy = toVec<Eigen::Vector3f>(config["rpy"]);
  mWorldToCamera = makeRpyXyzTransform(rpy, position).inverse();
}

const float voxelRadiusInCm = 3.0f;

void CameraRays::makeCameraRays(void)
{
    // Statistics: pixelCounts[n] is the of voxels that were projected
    // onto n pixels.
    std::vector<int32_t> pixelCounts(10000);

    // Find which rays intersect each voxel.
    for (int iVoxel = 0; iVoxel < voxelCount; ++iVoxel) {

        std::vector<int32_t> voxelPixels;
        const Eigen::Vector3f voxCam = mWorldToCamera * voxelCenters[iVoxel];
        const float distance = voxCam.norm();

        // Ignore pixels closer than half a meter to the camera and
        // within 10cm of the camera's maximum.  The 10cm avoids
        // voxels near the limit showing as background due to rounding
        // errors.
        //
        // This also needs to ignore pixels that are greater than the
        // background distance.

        if (distance < 0.5f
            || maxCameraRangeInM - 0.10f < distance) {
          continue; 
        }

        // Project the voxel onto the pixel array, noting which indexes
        // get hit.
        renderVoxel(voxelTriangles,
                    mWorldToCamera * Eigen::Translation3f(voxelCenters[iVoxel]),
                    voxelPixels);

        uint32_t count = static_cast<int32_t>(voxelPixels.size());
        pixelCounts[count] += 1;  // statistics

        uint32_t hits = 0;
        std::vector<int32_t> emptyPixels;
        for (auto& pixel : voxelPixels) {
          auto backgroundDepth = mBackgroundImage[reverseY(pixel)];
          if (abs(distance - backgroundDepth) <= voxelRadiusInCm / 100.0f) {
            hits += 1;
          } else if (distance + (voxelSizeInCm / 100.0f) / 2 < backgroundDepth) {
            emptyPixels.push_back(pixel);
          }
        }

        uint32_t emptyCount = static_cast<int32_t>(emptyPixels.size());

        if (count < hits * 2) {
          mVoxelStatus[iVoxel] = voxelIsBackground;
        } else if (8 <= emptyCount
                   && count * 2 <= emptyCount * 3) {
            if (mVoxelStatus[iVoxel] != voxelIsBackground) {
                mVoxelStatus[iVoxel] = voxelIsEmpty;
            }
            // At runtime we need at least eight rays and 2/3 of the
            // possible rays to see through the voxel in order to
            // consider it empty.  For run-time efficiency we limit
            // the intersections to 12 per camera per voxel.  It also
            // helps that it fits in a nibble.
            if (true) {
                // Use pixel ranking.
                if (12 < emptyCount) {
                    std::sort(emptyPixels.begin(),
                              emptyPixels.end(),
                              [](uint32_t a, uint32_t b) {
                                  return pixelRanks[b] < pixelRanks[a]; });
                    emptyCount = 12;
                }
            } else {
                // Choose randomly.
                for ( ; 12 <= emptyCount; emptyCount--) {
                    uint32_t loser = random() % emptyCount;
                    emptyPixels[loser] = emptyPixels[emptyCount - 1];
                }
            }
            for (uint32_t i = 0; i < emptyCount; i++) {
                mIntersections[emptyPixels[i]].emplace_back(iVoxel, distance);
            }
        }
    }

    fprintf(stderr, "[voxel ray counts");
    for (int i = 0; i <= 10; i++) {
        fprintf(stderr, " %d", pixelCounts[i]);
    }
    fprintf(stderr, " [10s]");
    for (int i = 0; i < pixelCounts.size(); i += 10) {
        uint32_t sum = 0;
        for (int j = 0; j < 10 && i + j < pixelCounts.size(); j++) {
            sum += pixelCounts[i + j];
        }
        fprintf(stderr, " %d", sum);
    }
    fprintf(stderr, "]\n");

    // Sort by voxel depth going from far to near, which is the order
    // they are used at runtime.
    for (size_t j = 0; j < imagePixelCount; ++j) {
      auto& intersections = mIntersections[j];
      std::sort(mIntersections[j].begin(),
                mIntersections[j].end(),
                [](const Intersection& a, const Intersection& b) {
                  return b.depthInM < a.depthInM; });
    }
}

// This is done as a separate pass so that the background data
// from all of the cameras is available.
void CameraRays::makeVoxelData()
{
    for (uint32_t iPixel = 0; iPixel < imagePixelCount; iPixel++) {
        auto& hitVoxels = mHitVoxels[reverseY(iPixel)];
        for (auto& intersection : mIntersections[iPixel]) {
            uint32_t voxelIndex = intersection.voxelIndex;
            if (!isBackgroundValue(mVoxelStatus[voxelIndex])) {
                hitVoxels.emplace_back(voxelIndex);
                mVoxelHitCounts[voxelIndex] += 1;
                mVoxelDistances[voxelIndex] = 100.0f * intersection.depthInM;
            }
        }
    }
}
