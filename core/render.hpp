// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// Render a perspective view of the blobs.

#include <blob.hpp>
#include <camera.hpp>

#include <Eigen/Core>

#include <cstdint>

// A reconstructed perspective view.  This makes it easier to see
// what is going on.
extern std::array<uint8_t, imagePixelCount * 3> perspectiveImage; // One byte each of RGB.
// The viewpoint and target point for the perspective view.
extern Eigen::Vector3f renderViewpoint;
extern Eigen::Vector3f renderTarget;

void renderInit(const nlohmann::json& config,
                const std::string& configDirectory);

void renderBlobs(std::vector<uint16_t>& blobData);
