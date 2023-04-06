// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "mesh.hpp"
#include "robot.hpp"
#include "util.hpp"
#include "cruft.hpp"

#include <tinyxml2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>
#include <iostream>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

  // Parse a string containing a series of floats into a vector.
  std::vector<float> splitFloats(const std::string& string)
  {
    std::stringstream stream(string);
    std::vector<float> result;
    std::string floatString;
    while (std::getline(stream, floatString, ' ')) {
      result.push_back(std::stof(floatString));
    }
    return result;
  }

  // Ditto, but there are three floats and you get a Vector3f.
  Eigen::Vector3f coordStringToVector3f(const std::string& str)
  {
    auto floats = splitFloats(str);
    if (floats.size() != 3) {
      throw fmtException("URDF: need 3 values, not {}", floats.size());
    }
    return {floats[0], floats[1], floats[2]};
  }

  // For some reason a joint's parent and child names are not just attributes.
  const char *jointParentName(tinyxml2::XMLElement* jointXml)
  {
    return jointXml->FirstChildElement("parent")->Attribute("link");
  }

  const char *jointChildName(tinyxml2::XMLElement* jointXml)
  {
    return jointXml->FirstChildElement("child")->Attribute("link");
  }

  // Fetch an XML link by name.
  tinyxml2::XMLElement* findLinkXml(tinyxml2::XMLElement* robotXml,
                                    const std::string& linkName)
  {
    for (auto next = robotXml->FirstChildElement("link");
         next != nullptr;
         next = next->NextSiblingElement("link")) {
      if (linkName.compare(next->Attribute("name")) == 0) {
        return next;
      }
    }
    throw fmtException("URDF: no link with name '{}'", linkName);
    return nullptr;
  }

  // Return child 'name' of 'element'.  'outerName' is used for exceptions.
  tinyxml2::XMLElement* getChildElement(tinyxml2::XMLElement* element,
                                        const char *name,
                                        const char *outerName)
  {
    auto result = element->FirstChildElement(name);
    if (result == nullptr) {
      throw fmtException("URDF: Can't find '{}' element in '{}'", name, outerName);
    }
    return result;
  }

  // Return attribute 'name' of 'element'.  The other arguments are only used
  // for exceptions.
  char const * const getAttribute(tinyxml2::XMLElement* element,
                                  const char *name,
                                  const char *elemName,
                                  const char *outerName)
  {
    auto result = element->Attribute(name);
    if (result == nullptr) {
      throw fmtException("URDF: Can't find '{}' attribute in '{}' in '{}'",
                         name,
                         elemName,
                         outerName);
    }
    return result;
  }

  // Parse a floating point attribute and convert it to radians.
  float getRadians(tinyxml2::XMLElement* element,
                   const char *name,
                   const char *elemName,
                   const char *outerName)
  {
    auto value = getAttribute(element, name, elemName, outerName);
    return std::stof(value) * M_PI / 180.0;
  }

  // Parse an attribute that is a series of floating points and return a vector.
  std::vector<float> getFloats(tinyxml2::XMLElement* element,
                               const char *name,
                               const char *elemName,
                               const char *outerName)
  {
    auto value = getAttribute(element, name, elemName, outerName);
    return splitFloats(value);
  }

  Robot::Link* parseLink(tinyxml2::XMLElement* linkXml,
                         const std::string& baseDirectory,
                         float scaleFactor)
  {
    auto link = new Robot::Link();

    const char *name = linkXml->Attribute("name");
    link->name = name;

    auto visualXml = linkXml->FirstChildElement("visual");
    auto geometryXml = getChildElement(visualXml, "geometry", name);
    auto meshXml = geometryXml->FirstChildElement("mesh");
    if (meshXml == nullptr) {
      fprintf(stderr, "no mesh for %s\n", name);
    } else {
      std::string filename = getAttribute(meshXml, "filename", "mesh", name);
      if (filename.empty()) {
        fprintf(stderr, "no file for %s\n", name);
      } else {
        if (filename[0] != '/') {
          filename = baseDirectory + "/" + filename;
        }
        link->mesh = std::make_unique<Mesh>();
        link->mesh->readStlFile(filename);
        auto scaleXml = meshXml->Attribute("scale");
        if (scaleXml != nullptr) {
          auto scaleFactors = coordStringToVector3f(scaleXml);
          // fprintf(stderr, "scale %s by %.5f\n", name, scaleFactors.x());
          link->mesh->scale(scaleFactors.x());
        }
        link->mesh->scale(scaleFactor);
        auto size = link->mesh->boundingBoxSize();
        // fprintf(stderr, "%s (%.3f %.3f %.3f)\n", name, size.x(), size.y(), size.z());

        auto originXml = getChildElement(visualXml, "origin", name);
        auto originRpy = coordStringToVector3f(getAttribute(originXml, "rpy", "origin", name));
        auto originXyz = coordStringToVector3f(getAttribute(originXml, "xyz", "origin", name));
        originXyz *= scaleFactor;
        link->mesh->transform(makeRpyXyzTransform(originRpy, originXyz));
        link->cloud = std::make_unique<Mesh>(link->mesh->makeSampledCloud(3000));
      }
    }
    return link;
  }

  void parseJoint(tinyxml2::XMLElement* jointXml,
                  Robot::Link* parentLink,
                  Robot::Link* childLink,
                  float scaleFactor)
  {
    const char *name = jointXml->Attribute("name");

    if (parentLink != nullptr) {
      parentLink->children.push_back(childLink);
    }
    childLink->parent = parentLink;

    auto originXml = getChildElement(jointXml, "origin", name);
    auto originRpy = coordStringToVector3f(getAttribute(originXml, "rpy", "origin", name));
    auto originXyz = coordStringToVector3f(getAttribute(originXml, "xyz", "origin", name));
    originXyz *= scaleFactor;
    childLink->jointTransform = makeRpyXyzTransform(originRpy, originXyz);

    auto axisXml = getChildElement(jointXml, "axis", name);
    childLink->jointAxis = coordStringToVector3f(getAttribute(axisXml, "xyz", "origin", name));

    auto limitXml = jointXml->FirstChildElement("limit");
    if (limitXml) {
      childLink->minRadians = getRadians(limitXml, "lower", "limit", name);
      childLink->maxRadians = getRadians(limitXml, "upper", "limit", name);
      childLink->maxVelocityRadians = getRadians(limitXml, "velocity", "limit", name);
    }
  }

} // anonymous namespace

Robot::Robot(const std::string& filename, float scaleFactor)
{
  std::string directory = std::filesystem::path{filename}.parent_path();
  tinyxml2::XMLDocument xmlDocument;
  tinyxml2::XMLError err = xmlDocument.LoadFile(filename.c_str());
  if (err != tinyxml2::XML_SUCCESS) {
    throw fmtException("URDF: Couldn't load file: '{}', xinyxml2 error: '{}'", filename, err);
  }

  auto robotXml = xmlDocument.FirstChildElement("robot");
  if (robotXml == nullptr) {
    throw fmtException("URDF: Can't find element 'robot'");
  }

  // Walk up the joints to find the root link.
  tinyxml2::XMLElement* rootXml = robotXml->FirstChildElement("link");
  while (true) {
    auto rootName = rootXml->Attribute("name");
    tinyxml2::XMLElement* jointXml;
    for (jointXml = robotXml->FirstChildElement("joint");
         jointXml != nullptr;
         jointXml = jointXml->NextSiblingElement("joint")) {
      if (strcmp(rootName, jointChildName(jointXml)) == 0) {
        break;
      }
    }
    if (jointXml == nullptr) {
      break;
    }
    rootXml = findLinkXml(robotXml, jointParentName(jointXml));
  }

  auto baseLink = parseLink(rootXml, directory, scaleFactor);

  // Walk down the joints to find all the links.
  mControlCount = 0;
  for (int i = -1; i < 0 || i < mLinks.size(); i++) {
    // Need two comparisons because -1 < mLinks.size() is always false.
    auto parent = i < 0 ? baseLink : mLinks[i];
    for (auto jointXml = robotXml->FirstChildElement("joint");
         jointXml != nullptr;
         jointXml = jointXml->NextSiblingElement("joint")) {
      if (parent->name.compare(jointParentName(jointXml)) == 0) {
        auto linkXml = findLinkXml(robotXml, jointChildName(jointXml));
        auto link = parseLink(linkXml, directory, scaleFactor);
        parseJoint(jointXml, parent == baseLink ? nullptr : parent, link, scaleFactor);
        link->linkIndex = mLinks.size();
        // We only handle simple robots at this point.
        link->controlIndex = mControlCount;
        link->controlCoefficient = 1.0f;
        mControlCount += 1;
        mLinks.push_back(link);
      }
    }
  }
  mBaseMesh = std::move(baseLink->mesh);
}
