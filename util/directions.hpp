// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <Eigen/Core>

// 162 nearly evenly-spaced unit vectors, about 16-19 degrees apart.
// No ray from the origin is more than 11 degrees from the nearest
// of these.
extern const std::array<Eigen::Vector3f, 162> directions;

uint16_t closestDirection(const Eigen::Vector3f& vector);
