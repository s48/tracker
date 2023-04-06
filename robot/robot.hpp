// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef ROBOT_H
#define ROBOT_H

#include "mesh.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

// The URDF XML format is described in http://wiki.ros.org/urdf/XML.
// There's lots more there than the minimum we implement here.
//
// The code does some format checking but there are gaps.  If you hit
// a segfault try running the URDF file through a verifier.

class Robot {
public:
  class Link {
  public:
    std::string name;

    std::unique_ptr<Mesh> mesh;  // loaded from file
    std::unique_ptr<Mesh> cloud; // created from mesh

    // The links form a tree, built from the <joint><parent> and
    // <joint><child> fields in the URDF.
    Link* parent;
    std::vector<Link*> children;
    Eigen::Affine3f jointTransform;

    // From the joint connecting this to its parent.
    float minRadians;          // <limit><lower>
    float maxRadians;          // <limit><upper>
    float maxVelocityRadians;  // <limit><velocity>
    Eigen::Vector3f jointAxis; // <axis><xyz>

    uint16_t linkIndex;        // index of this in mLinks.
    uint16_t controlIndex;     // which control determines the joint's angle
    // Multiplied times the control's value to get the joint angle.
    // This is almost always one, but for parallelogram
    // arrangements it is negative one for one set of parallels.
    float controlCoefficient;

    Link() :
      name(""),

      parent(nullptr),
      children(),

      minRadians(NAN),
      maxRadians(NAN),
      maxVelocityRadians(NAN),
      jointAxis(NAN, NAN, NAN){};
    ~Link(){};
  };

  // Need to use the control indexes and coefficients along with
  // the joint ranges to determine the control ranges.

  const Mesh& getBaseMesh() const { return *mBaseMesh; }
  const Link& getLink(size_t i) const { return *mLinks[i]; }
  size_t getLinkCount() const { return mLinks.size(); }
  uint16_t getControlCount() const { return mControlCount; }

  std::vector<Eigen::Affine3f> makeLinkTransforms(std::vector<float> const& controls) const;
  std::vector<Eigen::Vector3f> makeJointDerivatives(std::vector<float> const& controls,
                                                    Eigen::Vector3f const& tcpOffset) const;

  // The scale factor allows us to resize robots.
  Robot(const std::string& filename, float scaleFactor = 1.0f);
  ~Robot(){};

private:
  // parents come before their children
  // the first element is necessarily the root
  std::vector<Link*> mLinks;
  uint16_t mControlCount;
  // mesh for the unmoving base of the robot.
  std::unique_ptr<Mesh> mBaseMesh;
};

#endif // ROBOT_H
