// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "mesh.hpp"
#include "robot.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <math.h>
#include <string>
#include <vector>

std::vector<Eigen::Affine3f> Robot::makeLinkTransforms(std::vector<float> const& controls) const
{
  std::vector<Eigen::Affine3f> result;
  for (auto const& link : mLinks) {
    float angle = controls[link->controlIndex] * link->controlCoefficient;
    auto linkTransform = link->jointTransform * Eigen::AngleAxisf(angle, link->jointAxis);
    if (link->parent == nullptr) {
      result.emplace_back(linkTransform);
    } else {
      result.emplace_back(result[link->parent->linkIndex] * linkTransform);
    }
  }
  return result;
}

// Compute each joint's contribution to moving the TCP.
std::vector<Eigen::Vector3f> Robot::makeJointDerivatives(std::vector<float> const& controls,
                                                         Eigen::Vector3f const& tcpOffset) const
{
    auto transforms = makeLinkTransforms(controls);
    const Eigen::Vector3f tcp = transforms.back() * tcpOffset;
    std::vector<Eigen::Vector3f> result;
    // Link 0 has no joint, so we skip it.
    for (uint16_t i = 1; i < mLinks.size(); i++) {
        auto const& link = mLinks[i];
        auto const& transform = transforms[i];
        Eigen::Vector3f translation = transform * Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f jointAxis = (transform * link->jointAxis) - translation;
        result.emplace_back(jointAxis.cross(tcp - translation));
    }
    return result;
}
