// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// All kinds of random little things.

#ifndef CRUFT_H
#define CRUFT_H

#include "constant.hpp"

#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
// For some reason cmake refuses to find this file.
//#include <optionparser.h>
#include </home/kelsey/me/micro/model2/external/optionparser/optionparser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Floor8 = std::array<uint8_t, floorSize * floorSize>;
using Floor16 = std::array<uint16_t, floorSize * floorSize>;
using Floor32 = std::array<uint32_t, floorSize * floorSize>;

const float degToRad = static_cast<float>(M_PI) / 180.0f;

nlohmann::json readConfig(std::string filename);

int32_t timeMs(void);

// Faster float->int conversion for smallish numbers.  It's a total hack,
// but can make substantial difference if you're doing a lot of these.
// The float must be less than 1<<23.  Adding 1<<23 sets hidden mantissa
// bit and leaves the orginal integer value in the low 23 bits.
// This saves any messing around the FPU would have to do to set the
// precision properly for a full float->int conversion.

inline uint32_t floatToInt32u(float f)
{
    f += (float) (1 << 23);
    return (*(uint32_t *)&f) & 0x007FFFFF;
}

extern Eigen::Vector3f cellMinM;
extern Eigen::Vector3f cellCenterM;
extern Eigen::Vector3i cellMin;
extern Eigen::Vector3i cellMax;
extern Eigen::Vector3i cellSize;
extern uint32_t voxelCount;
extern uint32_t floorCount;

// Floor arrays are always the same size because they don't take
// up a lot of space.
inline uint32_t floorIndex(uint16_t x, uint16_t y)
{
    return x + floorSize * y;
}

// Voxel arrays are sized to the cell.  Space matters.
inline uint32_t voxelIndex(uint16_t x, uint16_t y, uint16_t z)
{
    return z + cellSize.z() * (x + cellSize.x() * y);
}

void setCellDimensions(const nlohmann::json& config);
uint32_t voxelIndex(const Eigen::Vector3i& voxel);
Eigen::Vector3i voxelCoordinates(uint32_t voxelIndex);


extern const float hypersphereFivePoints[130][5];

template <typename T>
inline T toVec(const nlohmann::json& config)
{
    T result;
    for (size_t i = 0; i < config.size(); ++i) { result[i] = config.at(i).get<typename T::value_type>(); }
    return result;
}

template Eigen::Vector3f toVec<Eigen::Vector3f>(const nlohmann::json& config);

std::vector<float> readFloats(const nlohmann::json& config);

void printTransform(const char *name, Eigen::Affine3f transform);

template <typename T>
std::string vectorToString(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vector){
    std::stringstream ss;
    ss << vector;
    return ss.str();
}

Eigen::Affine3f makeRpyXyzTransform(const Eigen::Vector3f& rpy, const Eigen::Vector3f& xyz);

Eigen::Affine3f lookAt(const Eigen::Vector3f& from,
                       const Eigen::Vector3f& to,
                       const Eigen::Vector3f& cameraUp = Eigen::Vector3f(0, 1, 0));

class CommandLine {
    
public:
    CommandLine(int argc,
            char **argv, 
            const option::Descriptor *usage);
    bool verify(int unknownOption, int helpOption);
    const std::vector<option::Option>& options();

private:
    option::Stats mStats;
    const option::Descriptor *mUsage;
    std::vector<option::Option> mOptions;
    std::vector<option::Option> mBuffer;
    option::Parser mParser;
};

class RandomFloat {
public:
    RandomFloat(std::mt19937::result_type seed = std::mt19937::default_seed) :
        mSeed(seed),
        mGenerator(std::mt19937(mSeed)),
        mUniform(std::bind(std::uniform_real_distribution<float>(0, 1), mGenerator)),
        mNormal(std::bind(std::normal_distribution<float>(0, 1), mGenerator))
    {
    }

    float operator()(float min, float max)
    {
        float r = mUniform();
        return min + r * (max - min);
    };


    float normal()
    {
        return mNormal();
    }

private:
    int64_t mSeed;
    std::mt19937 mGenerator;
    std::function<float()> mUniform;
    std::function<float()> mNormal;
};

#endif // CRUFT_H
