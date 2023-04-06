// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"
#include "voxel-data-file.hpp"
#include "distance-map.hpp"
#include "display.hpp"
#include "fusion.hpp"
#include "pixel.hpp"
#include "robot.hpp"
#include "cruft.hpp"
#include "blob.hpp"

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include "ThreadPool.h"

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <tuple>

#include <unistd.h>

extern void clearRobotLine();
extern void setRobotLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

DistanceMap robotDistances;

//----------------------------------------------------------------
// Debugging flags set by the user interface.

extern Show floorShow;
extern uint32_t layerToShow;
extern uint8_t cameraIndex;

bool examineColumns = false;
bool printColumn = false;
bool printRays = false;
int16_t cursorX = 10;
int16_t cursorY = 10;

extern bool debugPrint;
extern uint8_t robotSpriteLevel;

//----------------------------------------------------------------
// Density of people is the same as water, so a 50kg person has a volume
// of 50,000 ccs.  A voxel is 5x5x5cm = 125ccs, so a 50kg person would
// occupy 50000 / 125 = 400 voxels.  But the reconstruction rounds out
// so much that this doesn't come close to working.  What's needed is to
// run it with real cameras and real people and see what the volumes
// come out to.
// A breadbox. A 30cm cube.
const uint32_t ccPerVoxel = voxelSizeInCm * voxelSizeInCm * voxelSizeInCm;
const uint32_t sizeOfABreadbox = (30 * 30 * 30) / ccPerVoxel;

//----------------------------------------------------------------
// Per camera data.

// The bytes in the voxels vector hold two counts, the number of
// rays that can see through that voxel and the number of rays
// that cannot.  At most fifteen rays from a single camera are
// checked against each voxel.
const uint8_t hiddenIncrement = 0x01;
const uint8_t occupiedIncrement = 0x10;

uint32_t hitVoxels;
uint32_t hiddenVoxels;
uint32_t skippedVoxels;
uint32_t deadVoxels;
uint32_t livePixels;
uint32_t deadPixels;

static std::vector<uint8_t> intersectionCounts;
static uint16_t cameraCount;

// The minium distance from the center of a voxel to its surface.  The
// maximum distance is sqrt(2*2 + 2*2 + 2*2) = 3.46cm.  Using a larger
// radius here would allow hits that were in the voxel but outside the
// sphere of radius two, at the cost of false hits where the larger
// sphere extended outside the voxel.  The false hits add a lot of
// noise, so we use 2cm.  With a radius of 2cm the missed hits will
// get caught by the next voxel.
const uint16_t voxelRadiusInCm = 2;

struct Camera {
    uint32_t mIndex; // debugging
    // The first three fields are read from the column data file.
    std::array<PixelData, imagePixelCount> mPixelData;
    std::vector<uint16_t> mIntersectionDepthCms;
    std::vector<uint32_t> mIntersectionVoxelIndexes;
    // The high nibble has the number of rays that ended in this voxel,
    // the low nibble has the number that passed through it.
    std::vector<uint8_t> mVoxels;
    // For each pixel this has the index into the intersection data
    // of the voxel where the ray ended on the previous image.
    // These are signed, as -1 is a legitimate value.
    std::array<int16_t, imagePixelCount> mPreviousMaxMarked;
    Image8 mPreviousHitCounts;

    Camera(uint32_t index, std::ifstream& fin) :
        mIndex(index)
    {
        uint32_t pixelDataCount;
        fin.read((char *) &pixelDataCount, 4);
        fin.read((char *) mPixelData.data(), pixelDataCount * sizeof(PixelData));

        uint32_t intersectionCount;
        fin.read((char *) &intersectionCount, 4);
        mIntersectionVoxelIndexes.resize(intersectionCount);
        fin.read((char *) mIntersectionVoxelIndexes.data(), intersectionCount * 4);

        mIntersectionDepthCms.resize(voxelCount);
        fin.read((char *) mIntersectionDepthCms.data(), voxelCount * 2);

        mVoxels.resize(voxelCount);
        memset(mVoxels.data(), 0, voxelCount);
        // No marks, so maxMarked is -1.
        mPreviousMaxMarked.fill(-1);
        mPreviousHitCounts.fill(0);
    }
    void traceRays(const Image16& image);
};

// Trace the rays from the camera incrementing the hidden and occupied counts
// of the voxels it passes through.  For each pixel it only updates voxels
// affected by the change in depth from the previous image.  This saves time
// in general and avoids any dependence on the distance to objects.  If you
// start at either end of the ray you lose time if the object is at the other
// end.  This way the time is dependent only on the speed at which objects move,
// which is less variable.  Camera foibles can also cause big changes in
// measurements, but those are hopefully not common.

void Camera::traceRays(const Image16& image)
{
    for (uint32_t j = 0; j < imagePixelCount; j++) {
        uint16_t depthCm = image[j];
        uint32_t baseVoxelIndex = mPixelData[j].offset;
        uint32_t *voxelIndexes = mIntersectionVoxelIndexes.data() + baseVoxelIndex;
        uint32_t count = mPixelData[j].count;
        int32_t maxMarked = mPreviousMaxMarked[j];

        // Clear out any previous hits.
        uint32_t hits = mPreviousHitCounts[j];
        for ( ; 0 < hits; hits--) {
            // hitsRemoved += 1;
            uint32_t voxelIndex = voxelIndexes[maxMarked];
            mVoxels[voxelIndex] -= occupiedIncrement;
            maxMarked -= 1;
        }

        // Move away from the camera clearing out invalid marks.
        for (; 0 <= maxMarked; maxMarked--) {
            uint32_t voxelIndex = voxelIndexes[maxMarked];
            uint32_t voxelDepthCm = mIntersectionDepthCms[voxelIndex];
            if (voxelDepthCm < depthCm + 2 * voxelRadiusInCm) {
                // movedAway += 1;
                mVoxels[voxelIndex] -= hiddenIncrement;
            } else {
                break;
            }
        }

        int32_t firstUnmarked = maxMarked + 1;

        // Move towards the camera marking voxels.
        for (; firstUnmarked < count; firstUnmarked++) {
            uint32_t voxelIndex = voxelIndexes[firstUnmarked];
            // according to perf, 28% of runtime is in this fetch.
            // why here and not above?
            uint32_t voxelDepthCm = mIntersectionDepthCms[voxelIndex];
            // and another 16% in this line
            if (abs((int) voxelDepthCm - (int) depthCm) <= voxelRadiusInCm) {
                // We can see something here.
                // hitVoxels += 1;
                mVoxels[voxelIndex] += occupiedIncrement;
                hits += 1;
            } else if (0 < hits) {
                // We reached the end of the hits and can see through
                // everything closer.
                // unobservedVoxels += 1;
                break;
            } else if (voxelDepthCm + voxelRadiusInCm < depthCm) {
                // The voxel data has skipped past the voxel where
                // we would have hit.  We can see through this and
                // all closer voxels, but that's all we know.
                break;
            } else {
                // we can't see through to this voxel
                mVoxels[voxelIndex] += hiddenIncrement;
                // movedToward += 1;
            }
        }
        mPreviousMaxMarked[j] = firstUnmarked - 1;
        mPreviousHitCounts[j] = hits;
    }
}

static std::vector<Camera> cameras;

static uint32_t startTimes[8];
static uint32_t endTimes[8];

static int traceCameraRays(uint8_t startCameraIndex, uint8_t step, const std::vector<Image16>& depthImages)
{
    for (uint32_t i = startCameraIndex; i < cameraCount; i += step) {
        startTimes[i] = timeMs();
        cameras[i].traceRays(depthImages[i]);
        endTimes[i] = timeMs();
    }
    return 0;
}

void traceRays(ThreadPool& threadPool, const std::vector<Image16>& depthImages)
{
    if (false) { // single thread version
        traceCameraRays(0, 1, depthImages);
        // fprintf(stderr, "hits %d hidden %d skipped %d dead %d pixels live %d dead %d\n",
        //         hitVoxels, hiddenVoxels, skippedVoxels, deadVoxels, livePixels, deadPixels);
        // hitVoxels = 0;
        // hiddenVoxels = 0;
        // skippedVoxels = 0;
        // deadVoxels = 0;
        // livePixels = 0;
        // deadPixels = 0;
    } else { // multi thread version
        uint16_t numThreads = 4;
        std::vector<std::future<int>> futures;
        uint32_t startTime = timeMs();
        for (uint16_t i = 0; i < numThreads - 1; i++) {
            futures.emplace_back(threadPool.enqueue(traceCameraRays, i, numThreads, depthImages));
        }
        traceCameraRays(numThreads - 1, numThreads, depthImages);

        for (auto& future : futures) {
            future.get();
        }
        // fprintf(stderr, "[traceRays %d", timeMs() - startTime);
        // for (uint32_t i = 0; i < 8; i++) {
        //     fprintf(stderr, " %d:%d", startTimes[i] - startTime, endTimes[i] - startTimes[i]);
        // }
        // fprintf(stderr, "]\n");
    }
}

//----------------------------------------------------------------

void initializeEngine(const nlohmann::json& config, const std::string& configDirectory)
{
    std::string columnDataFile = configDirectory
            + "/" 
            + config["column_data_file"].get<std::string>();

    std::ifstream fin(columnDataFile, std::ios::in | std::ios::binary);
    if (!fin.is_open()) {
        fprintf(stderr, "Can't open column data file '%s'\n", columnDataFile.c_str());
        exit(0);
    }
    char header[80];
    fin.read(header, 80);

    cameraCount = 0;
    for (auto& camera : config["cameras"]) {
        if (camera.count("port") != 0) {
            cameraCount += 1;
        }
    }

    uint32_t backgroundVoxelSize;
    fin.read((char *) &backgroundVoxelSize, 4);
    intersectionCounts.resize(backgroundVoxelSize * cameraCount);
    fin.read((char *) intersectionCounts.data(), backgroundVoxelSize * cameraCount);

    for (uint32_t i = 0; i < cameraCount; i++) {
      cameras.emplace_back(i, fin);
    }
}

//----------------------------------------------------------------
// To save time, rather than doing individual voxels for the robot we
// mark the vertical extent of the robot for each column.  Anything
// that has parts of the robot both above and below it will be marked
// as robot.  This doesn't change any of the horizontal distances, which
// are what we care about.

static Floor8 robotMinHeight;
static Floor8 robotMaxHeight;

void renderRobot(const Robot& robot, const std::vector<float>& pose)
{
    robotMinHeight.fill(0xFF);
    robotMaxHeight.fill(0);
    const std::vector<Eigen::Affine3f>& transforms = robot.makeLinkTransforms(pose);

    for (uint32_t i = 0; i < transforms.size(); i++) {
        if (robot.getLink(i).cloud == nullptr) {
            continue;
        }
        Eigen::Affine3f transform =
            Eigen::Translation3f(Eigen::Vector3f(-cellMinM.x(), -cellMinM.y(), 0.3)) * transforms[i];
        for (const auto& vCloud : robot.getLink(i).cloud->getPoints()) {
            const Eigen::Vector3f& vCam = transform * vCloud;
            uint16_t x = (uint16_t) std::round(vCam.x() * voxelsPerMeter);
            float vCamY = vCam.y();
            if (x & 1 == 1) {
                vCamY -= voxelSizeInM / 2.0f;
            }
            uint16_t y = (uint16_t) std::round(vCamY * voxelsPerMeter);
            // Round up and down by one voxel to minimize noise blobs.
            int16_t zLow = (int16_t) std::floor(vCam.z() * voxelsPerMeter - 1.5f);
            int16_t zHigh = (int16_t) std::ceil(vCam.z() * voxelsPerMeter + 1.5f);
            zHigh = std::min(zHigh, (int16_t) cellSize.z());
            if (0 <= x && x < cellSize.x()
                && 0 <= y && y < cellSize.y()) {
                uint32_t pixelIndex = floorIndex(x, y);
                if (0 <= zLow && zLow < robotMinHeight[pixelIndex]) {
                    robotMinHeight[pixelIndex] = zLow;
                }
                if (zHigh <= cellSize.z() && robotMaxHeight[pixelIndex] < zHigh) {
                    robotMaxHeight[pixelIndex] = zHigh;
                }
            }
        }
    }
}

// Splice in a robot interval if 'interval' intersects the robot.
Interval* finishInterval(Interval* interval)
{
    uint32_t index = floorIndex(interval->mX, interval->mY);
    uint8_t robotMin = robotMinHeight[index];
    uint8_t robotMax = robotMaxHeight[index];

    if (interval->mMaxZ < robotMin
            || robotMax < interval->mMinZ) {
        return interval + 1;
    }

    Interval *start = interval;
    uint8_t type = interval->mType;
    uint8_t low = interval->mMinZ;
    uint8_t high = interval->mMaxZ;

    // Interval extends below the robot.  Split the non-robot
    // part off as a separate interval.
    if (low < robotMin) {
        interval->mMaxZ = robotMin - 1;
        interval += 1;
        memcpy(interval, start, sizeof(Interval));
        interval->mType = voxelRobot;
        interval->mMinZ = robotMin;
        interval->mMaxZ = high;
    } else {
        interval->mType = voxelRobot;
    }

    // Interval extends above the robot.  Split the non-robot
    // part off as a separate interval.
    if (robotMax < high) {
        interval->mMaxZ = robotMax;
        interval += 1;
        memcpy(interval, start, sizeof(Interval));
        interval->mType = type;
        interval->mMinZ = robotMax + 1;
        interval->mMaxZ = high;
    }

    // Bump past the top interval.
    return interval + 1;
}

//----------------------------------------------------------------
// Walk up each column of voxels finding contiguous intervals of
// occupied and occupied and unkown voxels.  Each interval is merged
// into a blob with any adjacent intervals from columns that have
// already been processed.

// The merging order is such that when we get to a square all squares
// with lower x or with the same x and lower y have already been done.
// The staggered squares look like this |x y|:
//
// The staggered squares look like this:
//
//   +---+   +---+
//   |0 2|---|2 2|---+    +---+
//   |---|1 2|---|3 2|    |x y|
//   |0 1|---|2 1|---|    +---+
//   |---|1 1|---|3 1|
//   |0 0|---|2 0|---|
//   +---|1 0|---|3 0|
//       +---+   +---+
//
// Thus we always merge with (x, y-1) and (x-1, y).  There are two
// neighbors on the previous column, so if x is even we need to also
// merge with (x-1, y-1) and if x is odd then merge with (x-1, y+1).

static void mergeBack(uint16_t x,
                      uint16_t y,
                      uint16_t minX,
                      uint16_t& nextBlobIndex)
{
    if (y != 0) {
        mergeIntervals(nextBlobIndex, x, y, x, y - 1);
    }
    if (x != minX) {
        mergeIntervals(nextBlobIndex, x, y, x - 1, y);
        if ((x & 1) == 0) {
            if (y != 0) {
                mergeIntervals(nextBlobIndex, x, y, x - 1, y - 1);
            }
        } else {
            if (y != cellSize.y() - 1) {
                mergeIntervals(nextBlobIndex, x, y, x - 1, y + 1);
            }
        }
    }
}

// Find the non-empty intervals in the column at (x, y).
static Interval *findIntervals(uint16_t x, uint16_t y, Interval* nextInterval)
{
    bool inInterval = false;
    uint8_t previousVoxelState = 0;  // no state

    const bool printIt = false && debugPrint && x == 68 && y == 149;

    for (uint32_t height = 0; height < cellSize.z(); height++) {
        uint32_t voxel = voxelIndex(x, y, height);
        const uint8_t* counts = &intersectionCounts[voxel * cameraCount];
        uint8_t voxelState = voxelUnknown;

        if (printIt) {
            fprintf(stderr, "[height %d", height);
        }

        if (isBackgroundValue(*counts)) {
            voxelState = voxelBackground;
        } else {
            for (uint32_t i = 0; i < cameraCount; i++) {
                uint8_t next = cameras[i].mVoxels[voxel];
                if (printIt) {
                    fprintf(stderr, " %d:%d:%d", i, next, counts[i]);
                }
                if (occupiedIncrement <= next) {
                    voxelState = voxelOccupied;
                    break;
                } else if (next < counts[i]) {
                    // If enough rays see through the voxel we
                    // consider it empty.
                    voxelState = voxelEmpty;
                }
            }
        }

        if (printIt) {
            fprintf(stderr, " state %02X previous %02X]\n",
                    voxelState, previousVoxelState);
        }

        if (voxelState != previousVoxelState) {
            if (inInterval) {
                nextInterval->mMaxZ = height;
                nextInterval = finishInterval(nextInterval);
                inInterval = false;
            }
            if (isIntervalType(voxelState)) {
                inInterval = true;
                nextInterval->mType = voxelState;
                nextInterval->mX = x;
                nextInterval->mY = y;
                nextInterval->mMinZ = height;
                nextInterval->mBlob = nullptr;
            }
            previousVoxelState = voxelState;
        }
    }
    if (printIt) {
        debugPrint = false;
    }
    // close off current interval
    if (inInterval) {
        nextInterval->mMaxZ = cellSize.z();
        nextInterval = finishInterval(nextInterval);
    }
    return nextInterval;
}

static int mergeSlice(uint16_t minX,
                      uint16_t maxX,
                      uint32_t intervalIndex,
                      uint16_t& nextBlobIndex)
{
    Interval *nextInterval = intervals + intervalIndex;
    for (uint32_t x = minX; x < maxX; x++) {
        for (uint32_t y = 0; y < cellSize.y(); y++) {
            Interval *start = nextInterval;
            intervalStarts[floorIndex(x, y)] = start - intervals;
            nextInterval = findIntervals(x, y, start);
            intervalCounts[floorIndex(x, y)] = nextInterval - start;

            if (false && x == 20 && y == 20) {
                fprintf(stderr, "[intervalCount %ld]\n", nextInterval - start);
            }
            // checkIntervals(x, y);
            mergeBack(x, y, minX, nextBlobIndex);
            // checkIntervals(x, y);
        }
    }
    // fprintf(stderr, "[%d x %d]\n", timeMs() - startMs, minX);

    // Needs to return a value to be run in a future.
    return 0;
}

// Merge adjacent voxels of the same type into blobs.

Blob* mergeBlobs(ThreadPool& threadPool)
{
    Blob *allBlobs = nullptr;

    // Single thread version and multi-thread version.

    if (false) {
        uint16_t nextBlobIndex = 0;
        mergeSlice(0, cellSize.y(), 0, nextBlobIndex);
        allBlobs = linkBlobs(0, nextBlobIndex, allBlobs);
        assert(checkMergingComplete(nextBlobIndex));
    } else {
        uint16_t numThreads = 4;
        std::vector<uint16_t> blobBlocks;
        std::vector<std::future<int>> futures;

        blobBlocks.resize(numThreads);
        uint32_t blockSize = cellSize.y() / numThreads;
        uint32_t column = 0;
        for (uint16_t i = 0; i < numThreads; i++) {
            blobBlocks[i] = (i * maxBlobs) / numThreads;
            uint32_t nextColumn = column + blockSize;
            if (i < numThreads - 1) {
                futures.emplace_back(threadPool.enqueue(mergeSlice,
                                column,
                                nextColumn,
                                (i * maxIntervals) / numThreads,
                                std::ref(blobBlocks[i])));
                column = nextColumn;
            }
        }
        
        mergeSlice(column,
                   cellSize.x(),
                   ((numThreads - 1) * maxIntervals) / numThreads,
                   std::ref(blobBlocks[numThreads - 1]));
        
        for (auto& future : futures) {
            future.get();
        }

        // At this point the blobs within each slice have been merged.
        // Now we stitch together the blobs along the boundaries.
        auto blocks1 = blobBlocks[1];
        for (uint16_t y = 0; y < cellSize.y(); y++) {
            for (uint16_t x = blockSize; x < cellSize.x(); x += blockSize) {
                mergeIntervals(blocks1, x, y, x - 1, y);
                if ((x & 1) == 0) {
                    if (y != 0) {
                        mergeIntervals(blocks1, x, y, x - 1, y - 1);
                    }
                } else {
                    if (y != cellSize.y() - 1) {
                        mergeIntervals(blocks1, x, y, x - 1, y + 1);
                    }
                }
            }
        }
        assert(checkMergingComplete(blobBlocks[1]));

        // Create a list of all the top-level blobs.
        for (uint16_t i = 0; i < numThreads; i++) {
            allBlobs = linkBlobs((i * maxBlobs) / numThreads,
                                 blobBlocks[i],
                                 allBlobs);
        }
    }
    return allBlobs;
}

//----------------------------------------------------------------

static const uint32_t smallColors[] = {
    0,
    0x0000FF, // robot is blue
    0,        // no background blobs
    0,
    0x800000, // occupied is reddish
    0, 0, 0,
    0,        // no empty blobs
    0, 0, 0, 0, 0, 0, 0,    
    0x008000  // unknown is greenish
};

static const uint32_t largeColors[] = {
    0,
    0x0000FF, // robot is blue
    0,        // no background blobs
    0,
    0xFF0000, // occupied is red
    0, 0, 0,
    0,        // no empty blobs
    0, 0, 0, 0, 0, 0, 0,    
    0x00FF00  // unknown is green
};

static void printColumnRays(uint16_t x, uint16_t y)
{
    fprintf(stderr, "Column %d %d\n", x, y);
    for (uint16_t z = 0; z < cellSize.z(); z++) {
        uint32_t voxel = voxelIndex(x, y, z);
        const uint8_t* counts = &intersectionCounts[voxel * cameraCount];
        fprintf(stderr, "  %2d:", z);
        if (isBackgroundValue(*counts)) {
            fprintf(stderr, " background\n");
        } else {
            for (uint32_t j = 0; j < cameraCount; j++) {
                uint8_t next = cameras[j].mVoxels[voxel];
                fprintf(stderr, " %d:", counts[j]);
                if (occupiedIncrement <= next) {
                    fprintf(stderr, "R");
                } else {
                    fprintf(stderr, "%d", next);
                    if (next < counts[j]) {
                        fprintf(stderr, "E");
                    }
                }
            }
            fprintf(stderr, "\n");
        }
    }
}

// A handy list of colors from
// https://sashat.me/2017/01/11/list-of-20-simple-distinct-colors/
// with black and white removed.
static const std::array<uint32_t, 20> palette = {
    0xE6194B, 0x3CB44B, 0xFFE119, 0x4363D8, 0xF58231, 0x911EB4, 0x46F0F0, 0xF032E6,
    0xBCF60C, 0xFABEBE, 0x008080, 0xE6BEFF, 0x9A6324, 0xFFFAC8, 0x800000, 0xAAFFC3,
    0x808000, 0xFFD8B1, 0x000075, 0x808080
};

static const char* voxelTypeNames[] = {
    "robot", "background", "occupied", "empty", "unknown", "outside"
};

// Various ways of displaying the current state.
void showFloor(Blob *allBlobs, std::vector<uint32_t>& floorImageData)
{
    if (printBlobs) {
        for (uint8_t i = 0; i < voxelTypeCount; i++) {
            uint16_t voxelType = 1 << i;
            if (voxelType != voxelEmpty) {
                fprintf(stderr, "%s blobs:\n", voxelTypeNames[i]);
                for (Blob *blob = allBlobs; blob != nullptr; blob = blob->mNext) {
                    if (blob->mType & voxelType && 10 < blob->mVolume) {
                        fprintf(stderr, "[blob %d vol %d min %s max %s center %s]\n",
                                blob->mCount,
                                blob->mVolume,
                                vectorToString<uint16_t>(blob->mMin).c_str(),
                                vectorToString<uint16_t>(blob->mMax).c_str(),
                                vectorToString<int>(blob->centerOfMass()).c_str());
                    }
                }
            }
        }
        printBlobs = false;
    }

    if (printColumn) {
        printIntervals("", cursorX, cursorY);
        printColumn = false;
    }

    if (printRays) {
        printColumnRays(cursorX, cursorY);
        printRays = false;
    }

    switch (floorShow) {
    case Show::layer:
    case Show::layerBlobs: {
        uint16_t i = 0;
        for (Blob *blob = allBlobs; blob != nullptr; blob = blob->mNext, i++) {
            if (floorShow == Show::layerBlobs) {
                blob->color(palette[i % palette.size()], layerToShow);
            } else if (sizeOfABreadbox <= blob->mVolume) {
                blob->color(largeColors, layerToShow);
            } else {
                blob->color(smallColors, layerToShow);
            }
        }
        if (examineColumns) {
            setFloorPixel(cursorX, cursorY, 0xFFFFFF);
            // setFloorPixel(cursorX + 1, cursorY, 0xFFFFFF);
            // setFloorPixel(cursorX + 2, cursorY, 0xFFFFFF);
            // setFloorPixel(cursorX - 1, cursorY, 0xFFFFFF);
            // setFloorPixel(cursorX - 2, cursorY, 0xFFFFFF);
            // setFloorPixel(cursorX, cursorY - 1, 0xFFFFFF);
            // setFloorPixel(cursorX, cursorY - 2, 0xFFFFFF);
            // setFloorPixel(cursorX, cursorY + 1, 0xFFFFFF);
            // setFloorPixel(cursorX, cursorY + 2, 0xFFFFFF);
        }
    }
        break;
    case  Show::layerCoverage:
        // This is currently broken.
        for (uint32_t x = 0; x < cellSize.x(); ++x) {
            for (uint32_t y = 0; y < cellSize.y(); ++y) {
                bool hit = false;
                uint32_t index = voxelIndex(x, y, layerToShow);
                const uint8_t* counts = &intersectionCounts[index * cameraCount];
                if (!isBackgroundValue(*counts)) {
                    for (uint32_t i = 0; i < cameraCount; i++) {
                        if (counts[i]) {
                            setFloorPixel(x, y, i == cameraIndex ? 0x00FF00 : 0x007F00);
                            hit = true;
                        }
                    }
                }
                if (debugPrint && !hit) {
                    uint32_t voxelIndex = layerToShow + cellSize.z() * index;
                    const uint8_t* counts = &intersectionCounts[voxelIndex * cameraCount];
                    fprintf(stderr, "[[%d, %d, %d] %d]\n",
                            x, y, layerToShow, !isBackgroundValue(*counts));
                }
            }
        }
        debugPrint = false;
        setFloorPixel(cursorX, cursorY, 0xFFFFFF);
        break;
    default:
        for (Blob *blob = allBlobs; blob != nullptr; blob = blob->mNext) {
            if (blob->mType == voxelUnknown
                    && 200 < blob->mVolume) {
                uint8_t green = ((blob->mMax.z() - blob->mMin.z()) * 0xFF) / cellSize.z();
                blob->color(green << 8);
            } else if (sizeOfABreadbox <= blob->mVolume) {
                blob->color(largeColors);
            } else {
                blob->color(smallColors);
            }
        }
    }

    // Run-length encoding of the floor image.
    floorImageData.clear();
    uint32_t color = floorImage[0];
    uint32_t count = 0;
    for (uint32_t x = 0; x < cellSize.x(); x++) {
        for (uint32_t y = 0; y < cellSize.y(); y++) {
            uint32_t index = floorIndex(x, y);
            if (floorImage[index] != color
                || count == 256) {
                floorImageData.emplace_back((color << 8) | count - 1);
                color = floorImage[index];
                count = 1;
            } else {
                count += 1;
            }
        }
    }
    floorImageData.emplace_back((color << 8) | count - 1);
}

//----------------------------------------------------------------

Floor32 floorImage; // color of each floor square.

void clearFloorImage(void)
{
    floorImage.fill(0);
}

// These are all |= so that brighter colors have priority.

void setFloorPixelLevel(const uint32_t x, const uint32_t y, const uint8_t level)
{
    floorImage[floorIndex(x, y)] |= (level << 16) | (level << 8) | level;
}

void setFloorPixelLevel(const uint32_t index, const uint8_t level)
{
    floorImage[index] |= (level << 16) | (level << 8) | level;
}

void setFloorPixel(const uint32_t index, uint32_t color)
{
    floorImage[index] |= color;
}

void setFloorPixel(const uint32_t x, const uint32_t y, uint32_t color)
{
    floorImage[floorIndex(x, y)] |= color;
}
