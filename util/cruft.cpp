// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "constant.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <sys/timeb.h>

#include <nlohmann/json.hpp>
#include <optionparser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

int32_t timeMs(void)
{
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

Eigen::Vector3f cellMinM;
Eigen::Vector3f cellCenterM;
Eigen::Vector3i cellMin;
Eigen::Vector3i cellMax;
Eigen::Vector3i cellSize;
uint32_t voxelCount;
uint32_t floorCount;

void setCellDimensions(const nlohmann::json& config)
{
    cellMinM = toVec<Eigen::Vector3f>(config["min"]);
    Eigen::Vector3f fmax = toVec<Eigen::Vector3f>(config["max"]);
    cellCenterM = (cellMinM  + fmax) / 2.0f;
    voxelCount = 1;
    for (int i = 0; i < 3; i++) {
        cellMin[i] = cellMinM[i] / voxelSizeInM;
        cellMax[i] = fmax[i] / voxelSizeInM;
        cellSize[i] = cellMax[i] - cellMin[i];
        voxelCount *= cellSize[i];
    }
    floorCount = cellSize.x() * cellSize.y();
    // How much of the floor is actually used.
    fprintf(stderr, "[%d voxels]\n", voxelCount);
}

uint32_t voxelIndex(const Eigen::Vector3i& voxel)
{
  return voxel.z() + cellSize.z() * (voxel.x() + voxel.y() * cellSize.x());
}

Eigen::Vector3i voxelCoordinates(uint32_t voxelIndex)
{
  uint16_t xy = voxelIndex / cellSize.z();
  return Eigen::Vector3i(xy % cellSize.y(),
                         xy / cellSize.y(),
                         voxelIndex % cellSize.z());
}

CommandLine::CommandLine(int argc,
        char **argv, 
        const option::Descriptor *usage) :
    mStats(usage, argc + 1, argv + 1),
    mUsage(usage),
    mOptions(mStats.options_max),
    mBuffer(mStats.buffer_max),
    mParser(usage, argc + 1, argv + 1, &mOptions[0], &mBuffer[0])
{
}

bool CommandLine::verify(int unknownOption, int helpOption) 
{
    if (mParser.error()) {
        std::cout << "Option parser error.\n";
        return false;
    }
    
    for (option::Option* opt = mOptions[unknownOption]; opt; opt = opt->next()) {
        std::cout << "Unknown option: " << std::string(opt->name, opt->namelen) << ".\n";
        return false;
    }
    
    if (mParser.nonOptionsCount() != 0) {
        std::cout << "Propgram does not take non-option arguments.\n";
        return false;
    }
    
    if (mOptions[helpOption]) {
        option::printUsage(std::cout, mUsage);
        return  false;
    }
    return true;
}
    
const std::vector<option::Option>& CommandLine::options()
{
    return mOptions;
}

std::vector<float> readFloats(const nlohmann::json& config)
{
    std::vector<float> result;
    for (size_t i = 0; i < config.size(); ++i) { 
        result.push_back(config.at(i).get<float>());
    }
    return result;
}

void printTransform(const char *name, Eigen::Affine3f transform)
{
  auto matrix = transform.matrix();
  fprintf(stderr, "%s: (", name);
  for (uint32_t i = 0; i < 4; i++) {
    fprintf(stderr, "(%.2f", matrix(i, 0));
    for (uint32_t j = 1; j < 4; j++) {
      fprintf(stderr, " %.2f", matrix(i, j));
    }
    fprintf(stderr, ")");
  }
  fprintf(stderr, ")\n");
}

// RPY = Roll(Z), Pitch(Y), and Yaw(X), the order in which the three
// rotations are performed.  This is from aircraft: Z is forwards,
// Y is left-to-right through the wing, and X is vertical pointing down.
// In each case a positive rotation is clockwise when looking in the
// positive direction.

Eigen::Affine3f makeRpyXyzTransform(const Eigen::Vector3f& rpy, const Eigen::Vector3f& xyz)
{
  auto translate =  Eigen::Translation3f(xyz);
  auto roll = Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ());
  auto pitch = Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY());
  auto yaw = Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX());
  return Eigen::Affine3f(translate * roll * pitch * yaw);
}

// Returns the transformation for pointing the camera from 'from' to 'to'
// with 'cameraUp' to the top of the image.
Eigen::Affine3f lookAt(const Eigen::Vector3f& from,
                       const Eigen::Vector3f& to,
                       const Eigen::Vector3f& cameraUp)
{
  Eigen::Vector3f forward = (from - to).normalized();
  Eigen::Vector3f up = cameraUp.normalized();
  Eigen::Vector3f right = up.cross(forward).normalized();
  up = forward.cross(right).normalized();
  // At this point 'forward', 'right', and 'up' are unit vectors at right
  // angles to one another.
  Eigen::Matrix<float, 4, 4> resultMatrix;
  resultMatrix <<
    right.x(), up.x(), forward.x(), from.x(),
    right.y(), up.y(), forward.y(), from.y(),
    right.z(), up.z(), forward.z(), from.z(),
    0, 0, 0, 1;
  Eigen::Affine3f result(resultMatrix);
  return result;
}

char *vecToString(Eigen::Vector3f v)
{
  char *buffer = (char *)malloc(100);
  sprintf(buffer, "(%.6g, %.6g, %.6g)", v.x(), v.y(), v.z());
  return buffer;
}
