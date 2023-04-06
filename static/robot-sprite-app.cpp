// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "directions.hpp"
#include "mesh.hpp"
#include "pixel.hpp"
#include "raster.hpp"
#include "robot.hpp"
#include "util.hpp"
#include "robot-sprite.hpp"

#include <iostream>
#include <fstream>
#include <unordered_set>
#include <vector>

#include <math.h>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

// Making this more accurate just costs time at file generation, so we
// could make this smaller.
static const float angleStopDelta = M_PI / 32.0f;

static float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return (a - b).norm();
}

static void makeJointSpriteFile(const std::string& filename, const Robot& robot)
{
    std::vector<uint32_t> offsets;
    std::vector<uint16_t> data;

    std::vector<float> robotPose(robot.getControlCount());
    std::vector<uint32_t> positionCounts(robot.getControlCount());
    std::vector<float> stepSizes(robot.getControlCount());

    for (uint32_t i = 0; i < robot.getControlCount(); i++) {
        robotPose[i] = minControlRadians[i];
        float range = maxControlRadians[i] - minControlRadians[i];
        if (M_PI * 2 <= range) {
            // Do the full circle.
            positionCounts[i] = 64;
            stepSizes[i] = angleStopDelta;
        } else {
            // Hit both min and max with a step size <= angleStopDelta.
            // (and no std::lciel)
            positionCounts[i] = std::max(2L, std::lround(std::ceil(range / angleStopDelta)));
            stepSizes[i] = range / (positionCounts[i] - 1);
        }
        printf("joint %d min %.2f max %.2f positions %d stepSize %.2f\n",
               i, minControlRadians[i], maxControlRadians[i], positionCounts[i], stepSizes[i]);
    }

    fprintf(stderr, "[limits %d %d]\n", positionCounts[1], positionCounts[2]);

    uint32_t count = 0;
    for (uint32_t l1 = 0; l1 < positionCounts[1]; l1++) {
        robotPose[1] = minControlRadians[1] + l1 * stepSizes[1];
        for (uint32_t l2 = 0; l2 < positionCounts[2]; l2++) {
            robotPose[2] = minControlRadians[2] + l2 * stepSizes[2];

            Canvas canvas;
            canvas.fill(0);
            drawRobot(canvas, robot, robotPose, positionCounts, stepSizes);
            count += 1;
            if ((count & (count - 1)) == 0) {
                fprintf(stderr, "[poses %d]\n", count);
            }
//            printf("%ld %.2f %.2f\n", data.size(), robotPose[1], robotPose[2]);
            addSprite(offsets, data, canvas);
        }
    }
    fprintf(stderr, "[data size %ld]\n", data.size());

    std::ofstream fout(filename, std::ios::out | std::ios::binary);
    char header[80] = "robot sprites";
    fout.write(header, 80);
    fout.write((char *) &positionCounts[1], sizeof(uint16_t));
    fout.write((char *) &positionCounts[2], sizeof(uint16_t));
    fout.write((char *) &stepSizes[1], sizeof(float));
    fout.write((char *) &stepSizes[2], sizeof(float));
    uint32_t temp = data.size();
    fout.write((char *) &temp, sizeof(uint32_t));
    fout.write((char *) offsets.data(), offsets.size() * sizeof(uint32_t));
    fout.write((char *) data.data(), data.size() * sizeof(uint16_t));
    fout.close();
}

//----------------------------------------------------------------
// The end effector sprite table is a 2d matrix of sprites.  Each row
// contains the sprites that correspond to one of the direction vectors.
// Each column has the sprites for a given rotation around the row's
// direction vector.
//
// The procedure for mapping the final transform of the robot pose
// to a sprite is:
//  - Apply the matrix to (1, 0, 0) and find the closest direction to the
//    result.  This gives the row of the sprite matrix.
//  - Decompose the matrix into its XZX or YZX Euler angles.  The third
//    angle is the column of the sprite matrix.
//
// The X axis and the __X Euler angles are used because the KR12 URDF has
// the last joint rotating around the x axis.  Thus rotations of the final
// joint affect only the third Euler angle, so a single sprite can contain
// a range of angle projections for that joint.
//
// To avoid gimbal lock problems we use either XZX or YZX Euler
// angles, depending on the cosine of Z rotation angle, which is also
// direction.x.
//
// https://www.geometrictools.com/Documentation/EulerAngles.pdf has
// rotation matrix <-> Euler angle formulae for all Euler variants.

static Eigen::Affine3f xzxEulerMatrix(const Eigen::Vector3f& direction, float x3);
static Eigen::Affine3f yzxEulerMatrix(const Eigen::Vector3f& direction, float x3);

static void makeEndEffectorSpriteFile(const std::string& filename,
                                      const Robot& robot)
{
    std::vector<uint32_t> offsets;
    std::vector<uint16_t> data;

    uint16_t steps = ((uint16_t) (2 * maxControlRadians[6] / angleStopDelta)) + 1;

    uint32_t xzxCount = 0;
    for (const auto& direction : directions) {
        bool xzx = fabs(direction.x()) < sqrt(0.5f);
        if (xzx) xzxCount += 1;
        for (uint16_t i = 0; i < 64; i++) {
            float x3 = i * (M_PI / 32) - maxControlRadians[6];
            Canvas canvas;
            canvas.fill(0);
            for (uint16_t j = 0; j < steps; j++) {
                float x = x3 + j * angleStopDelta;
                Eigen::Affine3f transform =
                    xzx
                    ? xzxEulerMatrix(direction, x)
                    : yzxEulerMatrix(direction, x);
                renderSprite(canvas, *robot.getLink(6).cloud, transform);
            }
            addSprite(offsets, data, canvas);
        }
    }
    fprintf(stderr, "[data size %ld]\n", data.size());
    fprintf(stderr, "[xzx count %d]\n", xzxCount);

    std::ofstream fout(filename, std::ios::out | std::ios::binary);
    char header[80] = "end-effector sprites";
    fout.write(header, 80);
    uint16_t temp = 64;
    fout.write((char *) &temp, sizeof(uint16_t));
    temp = directions.size();
    fprintf(stderr, "[directions.size() %ld]\n", directions.size());
    fout.write((char *) &temp, sizeof(uint16_t));
    uint32_t temp2 = data.size();
    fout.write((char *) &temp2, sizeof(uint32_t));
    fout.write((char *) offsets.data(), offsets.size() * sizeof(uint32_t));
    fout.write((char *) data.data(), data.size() * sizeof(uint16_t));
    fout.close();
}

// From the https://www.geometrictools.com/Documentation/EulerAngles.pdf
// the XZX matrix is:
// cz   -c3sz         szs3
// c1sz  c1czc3-s1s3 -c3s1-c1czs3
// s1sz  c1s3+czc3s1  c1c3-czs1s3
// cz and sz are the sine and cosine of the rotation around the Z axis
// and c1/c3 and s1/s3 are the sines and cosines of the two rotations
// around the X axis.
//
// With the final X rotation set to zero (s3 = 0, c3 = 1) the matrix
// applied to (1, 0, 0) needs to be the direction vector (x0, y0, z0).
// Working out the algebra gives:
// cz = x0
// sz = sqrt(1 - cz^2)
// c1 = y0/sz
// s1 = z0/sz

static Eigen::Affine3f xzxEulerMatrix(const Eigen::Vector3f& direction, float x3)
{
    float cosZ = direction.x();
    float sinZ = sqrt(1 - cosZ * cosZ);
    float cosX1 = direction.y() / sinZ;
    float sinX1 = direction.z() / sinZ;
    float cosX3 = cos(x3);
    float sinX3 = sin(x3);
    Eigen::Affine3f transform;
    transform.matrix() <<
        cosZ,         -cosX3 * sinZ,                          sinZ * sinX3,                         0.0f,
        cosX1 * sinZ,  cosX1 * cosZ * cosX3 - sinX1 * sinX3, -cosX3 * sinX1 - cosX1 * cosZ * sinX3, 0.0f,
        sinX1 * sinZ,  cosX1 * sinX3 + cosZ * cosX3 * sinX1,  cosX1 * cosX3 - cosZ * sinX1 * sinX3, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    // Check that the the transform maps 'direction' to itself (the dot
    // product is one).
    assert(0.999 < abs(direction.dot(transform * Eigen::Vector3f(1.0f, 0.0f, 0.0f))));
    return transform;
}

// From the https://www.geometrictools.com/Documentation/EulerAngles.pdf
// the YZX matrix is:
// cycz sxsy−cxcysz cxsy+cysxsz
// sz cxcz −czsx
// −czsy cysx+cxsysz cxcy−sxsysz
// cx, cy, cz are the cosines of the three rotation angles and sx, sy,
// sz are the sines.
//
// With the X rotation set to zero (sx = 0, cx = 1) the matrix applied
// to (1, 0, 0) needs to be the direction vector (x0, y0, z0).
// Working out the algebra gives:
// sz = y0
// cz = sqrt(1 - cz^2)
// sy = -z0/cz
// cy = x0/cz

static Eigen::Affine3f yzxEulerMatrix(const Eigen::Vector3f& direction, float x3)
{
    float sinZ = direction.y();
    float cosZ = sqrt(1 - sinZ * sinZ);
    float sinY = - direction.z() / cosZ;
    float cosY = direction.x() / cosZ;
    float cosX3 = cos(x3);
    float sinX3 = sin(x3);
    Eigen::Affine3f transform;
    transform.matrix() <<
        cosY * cosZ, sinX3 * sinY - cosX3 * cosY * sinZ, cosX3 * sinY + cosY * sinX3 * sinZ, 0.0f,
        sinZ, cosX3 * cosZ, -cosZ * sinX3, 0.0f,
        -cosZ * sinY, cosY * sinX3 + cosX3 * sinY * sinZ, cosX3 * cosY - sinX3 * sinY * sinZ, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    // Check that the the transform maps 'direction' to itself (the dot
    // product is one).
    assert(0.999 < abs(direction.dot(transform * Eigen::Vector3f(1.0f, 0.0f, 0.0f))));
    return transform;
}


//----------------------------------------------------------------

enum optionIndex { UNKNOWN, HELP, CONFIG_FILE, ROBOT, END_EFFECTOR, JACOBIAN, CLOUD, SAVE_CLOUD };

static const option::Descriptor usage[] = {
    {UNKNOWN, 0, "", "", option::Arg::None,
     "USAGE: foo [options]\n\n"
     "Options:"},
    {HELP,        0, "h", "help",       option::Arg::None, "--help, -h  \tPrint usage and exit."},
    {CONFIG_FILE, 0, "c", "configfile", option::Arg::Optional,
     "--configfile filename \tconfiguration file."},
    {ROBOT, 0, "r", "robot", option::Arg::None,
     "--robot \t generate robot sprite file"},
    {END_EFFECTOR, 0, "e", "end_effector", option::Arg::None,
     "--end_effector \t generate end effector sprite file"},
    {JACOBIAN, 0, "j", "jacobian", option::Arg::None,
     "--jacobian \t generate and test the Jacobian"},
    {0, 0, nullptr, nullptr, nullptr, nullptr}};

int main(int argc, char **argv)
{
    CommandLine cl(argc, argv, usage);
    if (!cl.verify(UNKNOWN, HELP)) {
        return 0;
    }
    const std::vector<option::Option> options = cl.options();

    if (options[CONFIG_FILE] == nullptr) {
        fprintf(stderr, "No config file specfied.\n");
        return 0;
    }

    const std::string configFileName = options[CONFIG_FILE].arg;
    const std::string configDirectory =
        configFileName.substr(0, configFileName.find_last_of("/"));
    nlohmann::json config = readConfig(configFileName);
    setCellDimensions(config);

    Robot robot(configDirectory
                + "/"
                + config["robot"]["urdf"].get<std::string>());
    std::string jointSpriteFile =
        configDirectory
        + "/"
        + config["robot"]["robot_sprites"].get<std::string>();
    std::string endEffectorSpriteFile =
        configDirectory
        + "/"
        + config["robot"]["end_effector_sprites"].get<std::string>();

    minControlRadians.resize(robot.getControlCount());
    maxControlRadians.resize(robot.getControlCount());
    for (uint16_t i = 0; i < robot.getControlCount(); i++) {
        minControlRadians[i] = robot.getLink(i).minRadians;
        maxControlRadians[i] = robot.getLink(i).maxRadians;

        // First 33ms is the time until the next image arrives, second
        // is the safety processor's timeout plus latency between the
        // safety processor and the robot controller.
//        maxJointStopRadians[i] += maxJointVelocityRadiansPerSec[i] * (0.033 + 0.033);
    }

    if (options[ROBOT] != nullptr) {
        fprintf(stderr, "[generating %s]\n", jointSpriteFile.c_str());
        makeJointSpriteFile(jointSpriteFile, robot);
    } else if (options[END_EFFECTOR] != nullptr) {
        fprintf(stderr, "[generating %s]\n", endEffectorSpriteFile.c_str());
        makeEndEffectorSpriteFile(endEffectorSpriteFile, robot);
    } else if (options[JACOBIAN] != nullptr) {
        fprintf(stderr, "[testing Jacobian]\n");
        uint16_t count = robot.getControlCount();
        std::vector<float> robotPose(count, 0);
        robotPose[0] = 0.0f;
        // Next two chosen to give a reasonable starting pose.
        robotPose[1] = -0.2;
        robotPose[2] = 0.01f;
        robotPose[3] = 0.1f;
        robotPose[4] = 0.2f;
        robotPose[5] = 0.3f;
        Eigen::Vector3f tcp(0.5f, 0.0f, 0.0f);
        const auto& derivatives
            = robot.makeJointDerivatives(robotPose, tcp);
        auto transforms0 = robot.makeLinkTransforms(robotPose);
        Eigen::Vector3f before = transforms0.back() * tcp;
        for (uint16_t i = 0; i < count; i++) {
            robotPose[i] += 0.01;
            auto transforms1 = robot.makeLinkTransforms(robotPose);
            Eigen::Vector3f after = transforms1.back() * tcp;
            Eigen::Vector3f test = (after - before) / 0.01;
            fprintf(stderr, "%.4f %s %s\n",
                    distance(derivatives[i], test),
                    vecToString(derivatives[i]),
                    vecToString(test));
            robotPose[i] -= 0.01;
        }
    }
}
