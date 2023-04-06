// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef COLUMN_DATA_FILE_HPP
#define COLUMN_DATA_FILE_HPP

#include <cstdint>

// The first value in a voxel data file is an array:
//   uint8_t counts[voxelIndex][cameraIndex]
// For each voxel and camera this has the number rays from that
// camera that must to see through that voxel for the voxel to
// be considered empty.
// If a voxel is unobserved or considered background it will have
// one of the special values voxelIsUnseed or voxelIsBackground
// for all cameras.
//
// The counts array is followed by three arrays per camera.  For each
// camera these are:
//   pixelData[pixelIndex]
//   voxelIndexes[]
//   voxelDistances[voxelIndex]
// The first two combine to list the voxels that each pixel can
// see and the third gives the distance to each voxel from the
// camera.  Each pixelData has the starting index and number of
// entries for that pixel in the voxelIndexes array.  Those
// entries are the indexes of the voxels that can be seen from
// that pixel, ordered from furthest to nearest.

// Special values for intersection count array.
static const uint8_t voxelIsUnseen = 255;     // no camera can see this voxel
static const uint8_t voxelIsBackground = 254; // this voxel is occupied in the background images

inline bool isBackgroundValue(uint8_t intersectionCount)
{
  return voxelIsBackground <= intersectionCount;
}

struct PixelData {
    uint32_t offset; // start index for this pixel in the intersection data
    uint32_t count;  // how many entries there are
    PixelData(uint32_t offset,
              uint32_t count) :
        offset(offset),
        count(count)
        {
        }
    PixelData() :
        offset(0),
        count(0)
        {
        }
};

#endif // COLUMN_DATA_FILE_HPP
