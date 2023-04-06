// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "util.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include <chrono>
#include <cfloat>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

static pcl::PointCloud<pcl::PointXYZ> readCloud(const std::string& file)
{
    std::ifstream fin(file, std::ios::in);

    if (!fin.is_open()) {
        fprintf(stderr, "Can't open cloud file '%s'\n", file.c_str());
        exit(0);
    }

    char header[80];
    fin.read(header, 80);
    int count;
    fin.read((char*)&count, 4);
    // 240408
    //  10000;
    pcl::PointCloud<pcl::PointXYZ> points(count, 1);
    for (int i = 0; i < count; i++) {
        Eigen::Vector3f point;
        fin.read((char*)point.data(), 12);
        points(i) = pcl::PointXYZRGB(point(0), point(1), point(2));
    }
    return points;
}

int main(int argc, char **argv)
{
  std::string cloudFile0 = argv[1];
  pcl::PointCloud<pcl::PointXYZ> points0 = readCloud(cloudFile0);
  return 0;
}
