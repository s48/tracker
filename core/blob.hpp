// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef BLOB_HPP
#define BLOB_HPP

#include "constant.hpp"
#include "cruft.hpp"

#include <cstdint>

#include <Eigen/Core>

// The types of voxels (and intervals and blobs) are:
const uint8_t voxelRobot      = 0x01; // a voxel expected to be occupied by the robot
const uint8_t voxelBackground = 0x02; // part of the fixed background
const uint8_t voxelOccupied   = 0x04; // a camera sees something here
const uint8_t voxelEmpty      = 0x08; // a camera can see that this is empty
const uint8_t voxelUnknown    = 0x10; // not visible to any camera
const uint8_t voxelOutside    = 0x20; // outside the volume we care about
const uint8_t voxelTypeCount = 6;
// A blob is set of connected voxels of the same type. A blob can
// contain both occupied and unknown voxels; no other combinations are
// allowed.

// Voxels of these types are not combined into intervals or blobs.
inline bool isIntervalType(uint8_t type)
{
    return (type & (voxelOutside | voxelEmpty | voxelBackground)) == 0;
}

using Vector3ui16 = Eigen::Vector3<uint16_t>;

struct Blob; // a set of connected voxels of the same type

// An interval is a vertical section of the (x, y) column of voxels all
// the same type, extending from mMinZ to mMaxZ inclusive.  Every interval
// has one of the types listed above.
// Adjoining intervals of the same type are grouped into a blob.

struct Interval {
    uint8_t mType;
    uint16_t mX;
    uint16_t mY;
    uint16_t mMinZ;
    uint16_t mMaxZ;
    Blob *mBlob;      // the blob this interval belongs to
    Interval *mNext;  // the next interval in the blob's circular list of intervals

    Interval() {}

    Blob *getBlob(); // follows the parent pointers up from mBlob to get the root
    void color(uint32_t color, uint16_t level);
    void color(const uint32_t *, uint16_t level);

    // Used to calculate a blob's ccenter of mass.
    void addToMoments(Eigen::Vector3f& moments)
    {
        float weight = mMaxZ - mMinZ + 1;
        moments.x() += weight * mX;
        moments.y() += weight * mY;
        moments.z() += weight * (float) (mMaxZ + mMinZ) / 2.0f;
    }

    void addData(std::vector<uint16_t>& data);
};

// Intervals are allocated from a static array.
const uint32_t maxIntervals = 100000;
extern Interval intervals[maxIntervals];
// For floor square I = floorIndex(x, y)
//   first interval: intervals[intervalStarts[I]]
//   how many in total: intervalCounts[I]
// The intervals for each square are sorted by increasing Z
extern Floor8 intervalCounts;
extern Floor32 intervalStarts;

// (aX, aY) and (bX, bY) are adjacent floor squares.  This merges any
// adjacent intervals into a single blob.
void mergeIntervals(uint16_t& nextBlobIndex, uint16_t aX, uint16_t aY, uint16_t bX, uint16_t bY);

// Print out 'tag' followed by the intervals above (x, y).
void printIntervals(const char *tag, uint16_t x, uint16_t y);

//----------------------------------------------------------------
// A set of connected voxels of the same type.

struct Blob {
    // A list of the intervals that make up this blob, linked through
    // the intervals' 'mNext' fields.
    Interval *mIntervals;
    uint8_t mType; // The type of the blob (and of its intervals).

    Blob *mParent; // The larger blob this one has been merged into.

    // Used to create a linked list of all of the top-level blobs
    // (those with no parents).
    Blob *mNext;

    uint32_t mVolume; // the number of voxels in the blob

    // The number of intervals in the blob.  When merging two
    // blobs we add the smaller one to the larger.
    uint16_t mCount;

    // The bounding box of the blob's voxels.
    Vector3ui16 mMin;
    Vector3ui16 mMax;

    Blob():
        mParent(nullptr),
        mIntervals(nullptr),
        mCount(0),
        mVolume(0),
        mType(0)
    {
    }

    // Blobs are preallocated and reused, so we need to reinitialize them.
    void init()
    {
        mParent = nullptr;
        mIntervals = nullptr;
        mCount = 0;
        mVolume = 0;
        mType = 0;
        for (uint16_t i = 0; i < 3; i++) {
            mMin[i] = 0xFFFF;
            mMax[i] = 0;
        }
    }

    void addInterval(Interval *interval);

    void merge(Blob *other);

    void walkIntervals(std::function<void(Interval&)> proc)
    {
        proc(*mIntervals);
        for (Interval* n = mIntervals->mNext; n != mIntervals; n = n->mNext) {
            proc(*n);
        }
    }

    // Color all intervals with 'color'.
    void color(uint32_t color)
    {
        walkIntervals([color](Interval& interval) { interval.color(color, interval.mMinZ); });
    }

    // Color all intervals with 'colors[mType]'.
    void color(const uint32_t* colors)
    {
        walkIntervals([colors](Interval& interval) { interval.color(colors, interval.mMinZ); });
    }

    // Ditto, but only color intervals that contain 'level'.
    void color(uint32_t color, uint16_t level)
    {
        walkIntervals([color, level](Interval& interval) { interval.color(color, level); });
    }

    void color(const uint32_t* colors, uint16_t level)
    {
        walkIntervals([colors, level](Interval& interval) { interval.color(colors, level); });
    }

    // The blob's center of mass.  Not currently used for anything.
    Eigen::Vector3i centerOfMass();

    // Collect the list of blobs' intervals into a vector.
    void writeToVector(std::vector<uint16_t>& data);
};


static const uint32_t maxBlobs = 10000;  // not a clue as to how many will be needed

// Verify that all possible merging has been done.
bool checkMergingComplete(uint16_t& nextBlobIndex);

// Link all the top blobs (those with mParent == NULL) from 'start' to 'end'
// 'mNext' pointers.
Blob* linkBlobs(const uint16_t start, uint16_t end, Blob *previous);

#endif // BLOB_HPP
