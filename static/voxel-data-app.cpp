// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "voxel-data.hpp"
#include "voxel-data-file.hpp"
#include "cruft.hpp"
#include "raster.hpp"
#include "robot.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cfloat>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

int main(int argc, char **argv)
{
    std::string configFileName = argv[1];
    std::string configDirectory = 
            configFileName.substr(0, configFileName.find_last_of("/"));
    nlohmann::json config = readConfig(configFileName);

    fprintf(stderr, "[%ld cameras]\n", config["cameras"].size());

    std::string columnDataFile = config["column_data_file"].get<std::string>();

    setCellDimensions(config);
    initDepthCorrection();
    makeVoxels();
    std::vector<uint8_t>voxelStatus(voxelCount, voxelIsUnseen);

    Robot robot(configDirectory
            + "/" 
            + config["robot"]["urdf"].get<std::string>());

    Mesh floor;
    floor.readStlFile(configDirectory
                      + "/"
                      + config["layout"].get<std::string>());
    
    const uint16_t cameraCount = config["cameras"].size();

    std::vector<CameraRays> cameraRays;
    for (const auto& cameraConfig : config["cameras"]) {
        // Read the background image from a file.
        std::string backgroundFile =
            configDirectory
            + "/"
            + cameraConfig["background"].get<std::string>();
        std::ifstream fin(backgroundFile, std::ios::in | std::ios::binary);
        if (!fin.is_open()) {
            fprintf(stderr, "Can't open background file '%s'\n", backgroundFile.c_str());
            exit(0);
        }
        std::vector<float> image(imagePixelCount);
        fin.read((char *) image.data(), imagePixelCount * sizeof(float));
        fin.close();
        cameraRays.emplace_back(cameraRays.size(), cameraConfig, std::move(image), voxelStatus);
    }

    // Project the voxels back on to the cameras.  This also identifies
    // voxels that are occupied by the background and marks them in 'voxelStatus'.
    for (auto& camera : cameraRays) {
        camera.makeCameraRays();
    }

    // Now that the background voxels have been identified, generate
    // intersection records for the unoccupied voxels that can be seen
    // by the runtime cameras.
    for (auto& camera : cameraRays) {
        if (!camera.mBackgroundOnly) {
            camera.makeVoxelData();
        }
    }

    // fprintf(stderr, "[voxel %d reals %d empties %d blocked %d intersects %d]\n", voxelStatus[voxelIndex], reals, empties, blocked, intersects);

    std::ofstream fout(columnDataFile, std::ios::out | std::ios::binary);
    // could add camera locations, etc. for safety
    char header[80] = "layout 200x200x40";
    fout.write(header, 80);
    uint32_t temp = voxelCount;
    fout.write((char *) &temp, 4);
    for (uint32_t i = 0; i < voxelCount; i++) {
        uint8_t status = voxelStatus[i];
        for (auto& camera : cameraRays) {
            if (!camera.mBackgroundOnly) {
                uint8_t count = isBackgroundValue(status)
                  ? status
                  // If more than half of the rays through a voxel
                  // are unobscured the voxel is considered clear.
                  // The exact details of this matter, hence the
                  // rounding up.
                  : (camera.mVoxelHitCounts[i] + 1) / 2;
                fout.write((char *) &count, 1);
            }
        }
    }

    int total = 0; // total number of intersections

    for (auto& camera : cameraRays) {
        if (!camera.mBackgroundOnly) {
            std::vector<PixelData> pixelData;
            uint32_t offset = 0;
            for (const auto& v : camera.mHitVoxels) {
                pixelData.emplace_back(offset, v.size());
                offset += v.size();
            }
            total += offset;
            temp = pixelData.size();
            fout.write((char *) &temp, 4);
            fout.write((char *) pixelData.data(), pixelData.size() * sizeof(PixelData));
            fout.write((char *) &offset, 4);
            for (const auto& v : camera.mHitVoxels) {
                fout.write((char *) v.data(), v.size() * 4);
            }
            fout.write((char *) camera.mVoxelDistances.data(),
                       camera.mVoxelDistances.size() * sizeof(uint16_t));
        }
    }

    fprintf(stderr, "[total intersections %d]\n", total);

    fout.close();
    return 0;
}
