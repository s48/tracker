// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "display.hpp"
#include "fusion.hpp" // setFloorPixel
#include "blob.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <cstdio>

// For printing intervals.
// R - robot
// B - background
// X - occupied
// E - empty
// ? - unknown
// O - outside
static const char * intervalTypeChars = "-RB-X---E-------?---------------O";

Floor8 intervalCounts;
Floor32 intervalStarts;
Interval intervals[maxIntervals];

static inline Interval * intervalBase(const uint32_t x, const uint32_t y)
{
    return intervals + intervalStarts[floorIndex(x, y)];
}

// Blobs are allocated out of a static array.
Blob blobs[maxBlobs];

static Blob* addBlob(uint16_t &nextBlobIndex, Interval *interval)
{
    assert(nextBlobIndex < maxBlobs);
    Blob *blob = blobs + nextBlobIndex;
    nextBlobIndex += 1;
    blob->init();
    blob->addInterval(interval);
    return blob;
}

// Link all the top blobs from 'start' to 'end' using the 'mParent' and
// 'mNext' pointers.
Blob* linkBlobs(const uint16_t start, uint16_t end, Blob *previous)
{
    for (uint16_t i = start; i < end; i++) {
        Blob* blob = blobs + i;
        if (blob->mParent == nullptr) {
            blob->mNext = previous;
            previous = blob;
        }
    }
    return previous;
}

// Walk up the parent links to get the blob struct that represents
// this blob.
Blob *Interval::getBlob()
{
    if (mBlob == nullptr) {
        return nullptr;
    }
    while (mBlob->mParent != nullptr) {
        mBlob = mBlob->mParent;
    }
    return mBlob;
}

void Interval::color(uint32_t color, uint16_t level)
{
    if (mMinZ <= level && level <= mMaxZ) {
        setFloorPixel(mX, mY, color);
    }
}

void Interval::color(const uint32_t* colors, uint16_t level)
{
    color(colors[mType], level);
}

void Interval::addData(std::vector<uint16_t>& data)
{
    data.emplace_back((mX << 8) | mY);
    data.emplace_back((mMaxZ << 8) | mMinZ);
}

static bool bug = false;
static bool allowMerge = true;
static uint32_t mergeCount = 0;

void Blob::addInterval(Interval *interval)
{
    mCount += 1;
    mVolume += interval->mMaxZ - interval->mMinZ + 1;
    mType |= interval->mType;
    if (mType == (voxelOccupied | voxelUnknown)) {
        mType = voxelOccupied;
    }

    mMin.x() = std::min(interval->mX, mMin.x());
    mMin.y() = std::min(interval->mY, mMin.y());
    mMin.z() = std::min(interval->mMinZ, mMin.z());

    mMax.x() = std::max(interval->mX, mMax.x());
    mMax.y() = std::max(interval->mY, mMax.y());
    mMax.z() = std::max(interval->mMaxZ, mMax.z());

    if (mIntervals == nullptr) {
        mIntervals = interval;
        interval->mNext = interval;
    } else {
        interval->mNext = mIntervals->mNext;
        mIntervals->mNext = interval;
    }

    interval->mBlob = this;
}

void Blob::merge(Blob *other)
{
    if (this == other) {
        // do nothing
    } else if (mCount < other->mCount) {
        other->merge(this);
    } else {
        other->mParent = this;
        mCount += other->mCount;
        mVolume += other->mVolume;
        mType |= other->mType;
        if (mType == (voxelOccupied | voxelUnknown)) {
            mType = voxelOccupied;
        }

        for (uint16_t i = 0; i < 3; i++) {
            mMin[i] = std::min(other->mMin[i], mMin[i]);
            mMax[i] = std::max(other->mMax[i], mMax[i]);
        }

        Interval *temp = mIntervals->mNext;
        mIntervals->mNext = other->mIntervals->mNext;
        other->mIntervals->mNext = temp;
        mergeCount += 1;
    }
}

// The blob's center of mass.  Not currently used for anything.
Eigen::Vector3i Blob::centerOfMass()
{
    Eigen::Vector3f moments(0.0f, 0.0f, 0.0f);
    walkIntervals([&moments](Interval& interval) { interval.addToMoments(moments); });
    return Eigen::Vector3i(moments.x() / (float) mVolume,
                           moments.y() / (float) mVolume,
                           moments.z() / (float) mVolume);
}

void Blob::writeToVector(std::vector<uint16_t>& data)
{
    data.clear();
    for (Blob *blob = this; blob != nullptr; blob = blob->mNext) {
        data.emplace_back(blob->mType);
        uint16_t endIndexLoc = data.size();
        data.emplace_back(0); // space for the end index
        blob->walkIntervals([&data](Interval& interval) { interval.addData(data); });
        data[endIndexLoc] = data.size();
    }
}

void printIntervals(const char *tag, uint16_t x, uint16_t y)
{
    Interval *interval = intervalBase(x, y);
    uint8_t count = intervalCounts[floorIndex(x, y)];
    fprintf(stderr, "[%s%d %d", tag, x, y);
    uint8_t nextLow = 0;
    for (int i = 0; i < count; i++, interval++) {
        if (nextLow < interval->mMinZ) {
            fprintf(stderr, " (E %d)", interval->mMinZ - nextLow);
        }
        if (interval->mBlob == nullptr) {
            fprintf(stderr, " (%c %d(%d %d))",
                    intervalTypeChars[interval->mType],
                    interval->mMaxZ - interval->mMinZ + 1,
                    interval->mMinZ,
                    interval->mMaxZ);
        } else {
            fprintf(stderr, " (%c %d(%d %d) blob %ld(%d %d))",
                    intervalTypeChars[interval->mType],
                    interval->mMaxZ - interval->mMinZ + 1,
                    interval->mMinZ,
                    interval->mMaxZ,
                    interval->mBlob - blobs,
                    interval->mBlob->mType,
                    interval->mBlob->mVolume);
        }
        nextLow = interval->mMaxZ + 1;
    }
    fprintf(stderr, "]\n");
}

// Verify that the intervals above (X, Y) are properly constructed.
// Not currently used.
static void checkIntervals(uint16_t x, uint16_t y)
{
    Interval *interval = intervalBase(x, y);
    uint8_t count = intervalCounts[floorIndex(x, y)];
    if (count < 2) {
        return;
    }
    Interval *previous = interval;
    interval += 1;
    for (int i = 1; i < count; i++, interval++) {
        bool good = previous->mMaxZ < interval->mMinZ
                || previous->mType != interval->mType;
        if (! good) {
            printIntervals("", x, y);
        }
        assert(good);
        previous = interval;
    }
}

// Merge all adjoining intervals in the (aX, aY) and (bX, bY) columns.
//  (aX, aY) and (bX, bY) are adjacent floor locations.
void mergeIntervals(uint16_t& nextBlobIndex, uint16_t aX, uint16_t aY, uint16_t bX, uint16_t bY)
{
    Interval *intervalA = intervalBase(aX, aY);
    Interval *intervalB = intervalBase(bX, bY);
    uint8_t countA = intervalCounts[floorIndex(aX, aY)];
    uint8_t countB = intervalCounts[floorIndex(bX, bY)];
    // fprintf(stderr, "[merge %d %d %d and %d %d %d]\n",
    //         aX, aY, countA, bX, bY, countB);
    while (0 < countA && 0 < countB) {
        if (((intervalA->mMinZ <= intervalB->mMaxZ + 1
              && intervalB->mMaxZ <= intervalA->mMaxZ)
             || (intervalB->mMinZ <= intervalA->mMaxZ + 1
                 && intervalA->mMaxZ <= intervalB->mMaxZ))
            && (intervalA->mType == intervalB->mType
                || ((intervalA->mType | intervalB->mType)
                    == (voxelOccupied | voxelUnknown)))) {
            // intervalA and intervalB overlap and have the same
            // or compatible types.  Merge them into a blob.

            Blob *blobA = intervalA->getBlob();
            Blob *blobB = intervalB->getBlob();

            // An error-checking pass done after all merges should have been found.
            if (! allowMerge
                    && (blobA == nullptr
                        || blobA != blobB)) {
                fprintf(stderr, "[late merge (%d %d %d-%d) (%d %d %d-%d)]\n",
                        aX, aY, intervalA->mMinZ, intervalA->mMaxZ,
                        bX, bY, intervalB->mMinZ, intervalB->mMaxZ);
                printIntervals("A: ", aX, aY);
                printIntervals("B: ", bX, bY);
            }

            // Merge the two intervals into a blob.
            if (blobA == nullptr) {
                if (blobB == nullptr) {
                    blobB = addBlob(nextBlobIndex, intervalB);
                }
                blobB->addInterval(intervalA);
            } else if (blobB == nullptr) {
                blobA->addInterval(intervalB);
            } else if (blobA != blobB) {
                blobA->merge(blobB);
            }
        }

        // Advance the interval with the lower maxZ value.
        if (intervalA->mMaxZ < intervalB->mMaxZ) {
            intervalA += 1;
            countA -= 1;
        } else {
            intervalB += 1;
            countB -= 1;
        }
    }
}

// Test function - see if any merges have been missed.
bool checkMergingComplete(uint16_t& nextBlobIndex)
{
    mergeCount = 0;
    allowMerge = false;

    for (uint32_t i = 0; i < cellSize.x(); i++) {
      for (uint32_t j = 0; j < cellSize.y(); j++) {
        if (i != 0) {
          mergeIntervals(nextBlobIndex, i, j, i - 1, j);
        }
        if (j != 0) {
          mergeIntervals(nextBlobIndex, i, j, i, j - 1);
        }
      }
    }
    allowMerge = true;
    return mergeCount == 0;
}
