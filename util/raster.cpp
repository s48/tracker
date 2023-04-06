// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"
#include "cruft.hpp"
#include "mesh.hpp"
#include "raster.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <iostream>
#include <limits>
#include <cstring>

// To get from the Z-depth to the range to the camera we need to divide by the
// cosine of the angle of the pixel off of the centerline.  This computes the
// inverse of the cosine so that the correction can be done with a
// multiplication (fast) instead of a division (slow).
//
// We also want the normal vector pointing towards each pixel.
ImageF depthCorrection;
std::array<Eigen::Vector3f, imagePixelCount> pixelNormal;
void initDepthCorrection()
{
  for (int x = 0; x < imageWidth; x++) {
    for (int y = 0; y < imageHeight; y++) {
      int pixelIndex = y * imageWidth + x;

      // The coordinates of the center of the pixel at the focal length.
      // x -> [0, imageWidth)
      // x/imageWidth -> [0, 1)
      // x/imageWidth - 0.5 -> [-0.5, 0.5)
      // (x/imageWidth - 0.5) * filmApertureWidth
      //   -> [-filmApertureWidth/2, filmApertureWidth/2)
      // The x+0.5 shifts the point to the middle of the pixel.
      double pixelDx =
        ((((double) x + 0.5) / (double) imageWidth) - 0.5) * filmApertureWidth;
      double pixelDy =
        ((((double) y + 0.5) / (double) imageHeight) - 0.5) * filmApertureHeight;
      
      // Now we need to get the inverse of the cosine of the angle between
      // the Z axis and the pixel.
      // The square of the opposite side (Pythagoras).
      double oppositeSquared = pixelDx * pixelDx + pixelDy * pixelDy;
      // The distance from the focal point to the pixel (Pythagoras again).
      double hypotenuse = sqrt(oppositeSquared + (focalLength * focalLength));
      // 1/cos = hypotenuse/adjacent
      depthCorrection[pixelIndex] = hypotenuse / focalLength;

      pixelNormal[pixelIndex] =
        Eigen::Vector3f(pixelDx, pixelDy, focalLength).normalized();
    }
  }
}

inline Eigen::Vector3f toRaster(const Eigen::Vector3f &vertexCamera,
                                const float toNdcFactorX,
                                const float toNdcFactorY)
{
  // convert to screen space by dividing by negative Z (the camera
  // points in the negative Z direction).
  Eigen::Vector2f vertexScreen(vertexCamera.x() / -vertexCamera.z(),
                               vertexCamera.y() / -vertexCamera.z());

  // convert to NDC (Normalized Device Coordinates) with X and Y
  // in the range [-1,1].
  Eigen::Vector2f vertexNDC(vertexScreen.x() * toNdcFactorX,
                            vertexScreen.y() * toNdcFactorY);

  // With inlining etc., is the compiler good enough to figure this out?
  //    x = (((vertexScreen.x() * xFactor) + 1) / 2) * imageWidth
  //      = ((vertexScreen.x() * xFactor) + 1)
  //      = (vertexScreen.x() * xFactor * imageWidth / 2) + (imageWidth / 2)
  //      = vertexScreen.x() * (xFactor * imageWidth / 2) + (imageWidth / 2)
  // It boils down to one multiplcation and one add.

  // Convert to raster space (where Y is down instead of up).
  return Eigen::Vector3f((vertexNDC.x() + 1) / 2 * imageWidth,
                         (1 - vertexNDC.y()) / 2 * imageHeight,
                         -1/vertexCamera.z()); // we actually need the reciprocal
}

// This computes the two-dimensional cross product of (B - A) and (C - A).
// The result is positive if C is on one side of the line A<->B and negative
// if it is on the there side.  Its magnitude is proportional to C's
// distance from the line A<->B.
static float abCrossAc(const Eigen::Vector3f &a, 
                       const Eigen::Vector3f &b, 
                       const Eigen::Vector3f &c)
{
  return ((c.x() - a.x()) * (b.y() - a.y())
          - (c.y() - a.y()) * (b.x() - a.x()));
}

// Draw the mesh, transformed by worldToCamera, into the depth, cosine, and
// objectType buffers.
//   depthBuffer - distance to closest object
//   cosineBuffer - the cosine of the angle of incidence of the ray to the surface
//   objectTypeBuffer - type of closest object
// The camera's ability to get the range to an surface depends on angle of the
// surface, among other things.  The cosineBuffer is used to simulate this effect.

void render(const Mesh& mesh,
            const Eigen::Affine3f& worldToCamera,
            ImageF& depthBuffer,
            ImageF& cosineBuffer,
            Image8& objectTypeBuffer,
            uint8_t objectType)
{
  // For translating from screen space (see toRaster()).
  float toNdcFactorX = focalLength / (filmApertureWidth / 2);
  float toNdcFactorY = focalLength / (filmApertureHeight / 2);

  // Translates the normal vectors to camera space.
  Eigen::Affine3f worldToCameraNormal(worldToCamera.matrix().inverse().transpose());

  const auto& triangles = mesh.getTriangles();
  const auto& normals = mesh.getNormals();
  for (int i = 0; i < triangles.size() / 3; i++) {
    Eigen::Vector3f normal = worldToCameraNormal * normals[i];
    const Eigen::Vector3f& v0Camera = worldToCamera * triangles[i * 3];

    if (0 <= normal.dot(v0Camera)) {
      continue;  // triangle faces away
    }

    const Eigen::Vector3f& v1Camera = worldToCamera * triangles[i * 3 + 1];
    const Eigen::Vector3f& v2Camera = worldToCamera * triangles[i * 3 + 2];

    // This can be added to worldToCamera, saving some operations.
    auto v0 = toRaster(v0Camera, toNdcFactorX, toNdcFactorY);
    auto v1 = toRaster(v1Camera, toNdcFactorX, toNdcFactorY);
    auto v2 = toRaster(v2Camera, toNdcFactorX, toNdcFactorY);

    // Get the bounding box of the triangle.
    float minXf = std::min(v0.x(), std::min(v1.x(), v2.x()));
    float minYf = std::min(v0.y(), std::min(v1.y(), v2.y()));
    float maxXf = std::max(v0.x(), std::max(v1.x(), v2.x()));
    float maxYf = std::max(v0.y(), std::max(v1.y(), v2.y()));
    float minZf = std::min(v0.z(), std::min(v1.z(), v2.z()));
    // We don't need maxZf.

    if (!std::isfinite(minXf)
        || !std::isfinite(maxXf)
        || !std::isfinite(minYf)
        || !std::isfinite(maxYf)
        || !std::isfinite(minZf)
        || imageWidth - 1 < minXf
        || maxXf < 0.0f
        || imageHeight - 1 < minYf
        || maxYf < 0.0f
        // The clipping doesn't work properly when you're too close to
        // the camera, so we drop any triangle that is too near.
        || minZf < (focalLength / 1000.0f)) { // focal length is in millimeters
      continue; // The triangle is outside of the visible area.
    }

    // Clip to within the screen and convert to integers.
    // The min max values can be negative and have to be compared as int32_t.
    uint32_t minX = floatToInt32u(std::max(int32_t(0), (int32_t)(std::floor(minXf))));
    uint32_t maxX = floatToInt32u(std::min(int32_t(imageWidth) - 1, (int32_t)(std::floor(maxXf))));
    uint32_t minY = floatToInt32u(std::max(int32_t(0), (int32_t)(std::floor(minYf))));
    uint32_t maxY = floatToInt32u(std::min(int32_t(imageHeight) - 1, (int32_t)(std::floor(maxYf))));

    // Twice the trangle's area, which is used in calculating the
    // barycentric coordinates.  Despite the fancy name, barycentric
    // coordinates are just a linear coordinate system based on the three
    // vertices of the triangle.
    float areaTimesTwo = abCrossAc(v0, v1, v2);

    // biStep is the corresponding increment in bi when X is incremented.
    // This saves us cost of redoing the crossproducts each time around
    // the inner loop.
    float b0Step = v2.y() - v1.y();
    float b1Step = v0.y() - v2.y();
    float b2Step = v1.y() - v0.y();

    for (uint32_t y = minY; y <= maxY; ++y) {
      // Calculate the barycentric coordinates of the first point on the scan line.
      // The actual coordinates are bi/(2*area), but we delay dividing until we know
      // that the pixel is visible.  For that we only need the signs, not the values.
      Eigen::Vector3f c(minX + 0.5f, y + 0.5f, 0.0f);
      float b0 = abCrossAc(v1, v2, c);
      float b1 = abCrossAc(v2, v0, c);
      float b2 = abCrossAc(v0, v1, c);
      uint32_t pixelIndex = y * imageWidth + minX;

      for (uint32_t x = minX; x <= maxX; ++x) {
        if (0 <= b0
            && 0 <= b1
            && 0 <= b2
            && minZf * depthCorrection[pixelIndex] < depthBuffer[pixelIndex]) {
          // We inverted the Z values when converting to raster, so we're
          // actually dividing by Z here and then have to invert the result.
          float depth = ((areaTimesTwo / (v0.z() * b0 + v1.z() * b1 + v2.z() * b2))
                         * depthCorrection[pixelIndex]);
          if (depth < depthBuffer[pixelIndex]) {
            depthBuffer[pixelIndex] = depth;
            cosineBuffer[pixelIndex] =
              std::max(0.f, normal.dot(pixelNormal[pixelIndex]));
            objectTypeBuffer[pixelIndex] = objectType;
          }
        }
        b0 += b0Step;
        b1 += b1Step;
        b2 += b2Step;
        pixelIndex += 1;
      }
    }
  }
}

// A voxel cube centered on the origin.
// 6 sides * 2 trianges/side * 3 points/triangle = 36 points
std::vector<Eigen::Vector3f> makeVoxelTriangles(float size)
{
  std::vector<Eigen::Vector3f> result = {
      Eigen::Vector3f(-0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f( 0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size, -0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size, -0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size,  0.5f * size),
      Eigen::Vector3f(-0.5f * size,  0.5f * size, -0.5f * size)};
  return result;
}

//----------------------------------------------------------------
// This is a copy of render() except that instead of drawing an image it
// produces a list of the pixels that would see the voxel.  It's used to
// generate the pixel->voxel mapping used to process the depth images.
//
// Exactly the same, except that here we do not reverse the Y coordinate.
// I need to figure out what's going on with the Y direction.
inline Eigen::Vector3f toRaster2(const Eigen::Vector3f &vertexCamera,
                                const float toNdcFactorX,
                                const float toNdcFactorY)
{
  Eigen::Vector2f vertexScreen(vertexCamera.x() / -vertexCamera.z(),
                               vertexCamera.y() / -vertexCamera.z());
  Eigen::Vector2f vertexNDC(vertexScreen.x() * toNdcFactorX,
                            vertexScreen.y() * toNdcFactorY);
  return Eigen::Vector3f((vertexNDC.x() + 1) / 2 * imageWidth,
                         (vertexNDC.y() + 1) / 2 * imageHeight,
                         -1/vertexCamera.z()); // we actually need the reciprocal
}

void renderVoxel(const std::vector<Eigen::Vector3f>& voxelTriangles,
                 const Eigen::Affine3f& worldToCamera,
                 std::vector<int32_t>& pixelIndexes)
{
  // For translating from screen space (see toRaster()).
  float toNdcFactorX = focalLength / (filmApertureWidth / 2);
  float toNdcFactorY = focalLength / (filmApertureHeight / 2);

  uint32_t start = static_cast<int32_t>(pixelIndexes.size());

  for (int i = 0; i < voxelTriangles.size(); i += 3) {
    const Eigen::Vector3f& v0Camera = worldToCamera * voxelTriangles[i];
    const Eigen::Vector3f& v1Camera = worldToCamera * voxelTriangles[i + 1];
    const Eigen::Vector3f& v2Camera = worldToCamera * voxelTriangles[i + 2];

    auto v0 = toRaster2(v0Camera, toNdcFactorX, toNdcFactorY);
    auto v1 = toRaster2(v1Camera, toNdcFactorX, toNdcFactorY);
    auto v2 = toRaster2(v2Camera, toNdcFactorX, toNdcFactorY);

    // Get the bounding box of the triangle.
    float minXf = std::min(v0.x(), std::min(v1.x(), v2.x()));
    float minYf = std::min(v0.y(), std::min(v1.y(), v2.y()));
    float maxXf = std::max(v0.x(), std::max(v1.x(), v2.x()));
    float maxYf = std::max(v0.y(), std::max(v1.y(), v2.y()));

    if (!std::isfinite(minXf)
        || !std::isfinite(maxXf)
        || !std::isfinite(minYf)
        || !std::isfinite(maxYf)
        || imageWidth - 1 < maxXf
        || minXf < 0.0f
        || imageHeight - 1 < maxYf
        || minYf < 0.0f) {
      // We only consider voxels that are entirely within the image.
      pixelIndexes.resize(start);
      return;
    }

    // Clip to within the screen and convert to integers.
    // The min max values can be negative and have to be compared as int32_t.
    uint32_t minX = std::max(int32_t(0), (int32_t)(std::floor(minXf)));
    uint32_t maxX = std::min(int32_t(imageWidth) - 1, (int32_t)(std::floor(maxXf)));
    uint32_t minY = std::max(int32_t(0), (int32_t)(std::floor(minYf)));
    uint32_t maxY = std::min(int32_t(imageHeight) - 1, (int32_t)(std::floor(maxYf)));

    // biStep is the corresponding increment in bi when X is incremented.
    // This saves us cost of redoing the crossproducts each time around
    // the inner loop.
    float b0Step = v2.y() - v1.y();
    float b1Step = v0.y() - v2.y();
    float b2Step = v1.y() - v0.y();

    for (uint32_t y = minY; y <= maxY; ++y) {
      // Calculate the barycentric coordinates of the first point on the scan line.
      // The actual coordinates are bi/(2*area), but we delay dividing until we know
      // that the pixel is visible.  For that we only need the signs, not the values.
      Eigen::Vector3f c(minX + 0.5f, y + 0.5f, 0.0f);
      float b0 = abCrossAc(v1, v2, c);
      float b1 = abCrossAc(v2, v0, c);
      float b2 = abCrossAc(v0, v1, c);
      uint32_t pixelIndex = y * imageWidth + minX;
      for (uint32_t x = minX; x <= maxX; ++x) {
        if (0 <= b0
            && 0 <= b1
            && 0 <= b2
            && (std::find(pixelIndexes.begin(), pixelIndexes.end(), pixelIndex)
                == pixelIndexes.end())) {
            pixelIndexes.push_back(pixelIndex);
        }
        b0 += b0Step;
        b1 += b1Step;
        b2 += b2Step;
        pixelIndex += 1;
      }
    }
  }
}
