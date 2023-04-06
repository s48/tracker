// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// Render a perspective view of the blobs.  This is simple and slow, but
// there is no particular reason to make it fast.

#include "blob.hpp"
#include "constant.hpp"
#include "cruft.hpp"
#include "mesh.hpp"
#include "raster.hpp"
#include "camera.hpp"
#include "display.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>

std::array<uint8_t, imagePixelCount * 3> perspectiveImage; // One byte each of RGB.
// The viewpoint can be changed interactively.
Eigen::Vector3f renderViewpoint = {0.2f, 0.0f, 4.0f}; // lat, long, radius
Eigen::Vector3f renderTarget = {0.0f, 0.0f, 1.0f}; // meters

static Mesh voxelMesh;  // a cube, drawn at a different position for each voxel
static Mesh background; // the background, drawn once

// The generated image.  This doesn't use the cosine data, but the renderer
// needs it to put the values into.
static ImageF depthBuffer;
static ImageF cosineBuffer;
static Image8 objectTypeBuffer;

// These map the blob types to their RGB components.  Only a few values are used.
static uint8_t typeRed[256];    // FF for voxelOccupied
static uint8_t typeGreen[256];  // FF for voxelRobot
static uint8_t typeBlue[256];   // FF for voxelUnknown
// All three have 0x7F for voxelBackground, so those come out grey.

void renderInit(const nlohmann::json& config,
                const std::string& configDirectory)
{
    initDepthCorrection();
    voxelMesh.addBox(Eigen::Vector3f{-voxelSizeInM/2, -voxelSizeInM/2, -voxelSizeInM/2},
                     Eigen::Vector3f{voxelSizeInM, voxelSizeInM, voxelSizeInM});
    background.readStlFile(configDirectory
                           + "/"
                           + config["layout"].get<std::string>());

    typeRed[voxelOccupied] = 0xFF;
    typeBlue[voxelRobot] = 0xFF;
    typeGreen[voxelUnknown] = 0xFF;
    typeRed[voxelBackground] = 0x7F;
    typeBlue[voxelBackground] = 0x7F;
    typeGreen[voxelBackground] = 0x7F;
}

// Globals to pass in values needed by renderInterval().
static uint8_t blobType;
static Eigen::Affine3f worldToCamera;

static void renderInterval(Interval& interval)
{
    for (uint32_t z = interval.mMinZ; z <= interval.mMaxZ; z++) {
        auto transform = worldToCamera 
            * Eigen::Translation3f(interval.mX * voxelSizeInM,
                                   interval.mY * voxelSizeInM,
                                   z * voxelSizeInM);
        render(voxelMesh, transform, depthBuffer, cosineBuffer, objectTypeBuffer, blobType);
    }
}

static void startRender()
{
    depthBuffer.fill(maxCameraRangeInM);
    objectTypeBuffer.fill(0xFF);
    perspectiveImage.fill(0);

    // lookFrom is the latitude, longitude, and radius of the viewpoint
    // with respect to the target point.
    Eigen::Vector3f lookFromXyz = {
        (cos(renderViewpoint.x()) * cos(renderViewpoint.y()) * renderViewpoint.z()),
        (sin(renderViewpoint.x()) * cos(renderViewpoint.y()) * renderViewpoint.z()),
        (sin(renderViewpoint.y()) * renderViewpoint.z()) };
    const Eigen::Affine3f cameraToWorld = lookAt(lookFromXyz + renderTarget,
                                                 renderTarget,
                                                 Eigen::Vector3f{0.0f, 0.0f, 1.0f});
    worldToCamera = cameraToWorld.inverse();
    render(background, worldToCamera, depthBuffer, cosineBuffer, objectTypeBuffer, voxelBackground);

    // The voxels have the origin in the lower left corner, so we
    // need to move everything over.
    worldToCamera *= Eigen::Translation3f(cellMinM.x(), cellMinM.y(), cellMinM.z());
}


static void finishRender()
{
    // There is GL_UNSIGNED_BYTE_3_3_2 type where each pixel is a single
    // byte.  If the blob types were set properly the objectTypeBuffer
    // could be the image, saving some time.
    // Alternatively, make the blob types bigger and use GL_UNSIGNED_SHORT_5_6_5.
    // That probably isn't where the time goes, though.
    // Could do the projections once when the camera point moves and just have
    // a table mapping voxel indexes to lists of pixels.  Might have to break
    // up writing the table to avoid latency hits.  The table is too big.  Too
    // many empty voxels hit too many pixels.
    // Cache the hit values for voxels, possibly even discarding those that get
    // old, if that's possible.  -> Have a marker in each list, then walk the
    // lists.
    // For a typical image there are 130k to 170k pixels written and
    // 6-7k voxels that get written, plus rendering the background.  The
    // background is easy to cache, but I doubt it adds much to the
    // time.  Having 10k cached voxel->pixel lists doesn't seem like a
    // big deal.  Note that on average a voxel hits 20 or so pixels.

    for (uint32_t i = 0; i < imagePixelCount; i++) {
        uint8_t type = objectTypeBuffer[i];
        if (type != 0xFF) {
            perspectiveImage[i * 3] = typeRed[type];
            perspectiveImage[i * 3 + 1] = typeGreen[type];
            perspectiveImage[i * 3 + 2] = typeBlue[type];
        }
    }
}

void renderBlobs(std::vector<uint16_t>& blobData)
{
//    printf("[%ld bytes of blob data]\n", blobData.size());
    startRender();
    size_t index = 0;
    while (index < blobData.size()) {
        uint8_t blobType = blobData[index++];
        uint16_t endIndex = blobData[index++];
        while (index <  endIndex) {
            uint16_t xy = blobData[index++];
            uint16_t x = xy >> 8;
            uint16_t y = xy & 0xFF;
            uint16_t zs = blobData[index++];
            uint16_t minZ = zs & 0xFF;
            uint16_t maxZ = zs >> 8;
            for (uint32_t z = minZ; z <= maxZ; z++) {
                auto transform = worldToCamera
                    * Eigen::Translation3f(x * voxelSizeInM,
                                           y * voxelSizeInM,
                                           z * voxelSizeInM);
                render(voxelMesh, transform, depthBuffer, cosineBuffer, objectTypeBuffer, blobType);
            }
        }
    }
    finishRender();
}
