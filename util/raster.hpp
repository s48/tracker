// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef RASTER_HPP
#define RASTER_HPP

#include "camera.hpp"
#include "mesh.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// Units are meters.
const float nearClippingPlane = 1;
const float farClippingPlane = 10;

void initDepthCorrection();

void render(const Mesh& mesh,
            const Eigen::Affine3f& worldToCamera,
            ImageF& depthBuffer,
            ImageF& cosineBuffer,
            Image8& objectTypeBuffer,
            uint8_t objectType);

std::vector<Eigen::Vector3f> makeVoxelTriangles(float size);

void renderVoxel(const std::vector<Eigen::Vector3f>& voxelTriangles,
                 const Eigen::Affine3f& worldToCamera,
                 std::vector<int32_t>& pixelIndexes);

extern std::array<Eigen::Vector3f, imagePixelCount> pixelNormal;

#endif // RASTER_HPP
