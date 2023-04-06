// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <string>

// Blue is hard to see
uint32_t colors[] = {0, 0xFFFFFF, 0xFF0000, 0x00FF00, 0x2020DF};

int main (int argc, char** argv)
{
  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  for (int i = 1; i < argc; i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
    char *file = argv[i];
    size_t len = strlen(file);
    bool isXyz = (4 < len && strcmp(file + len - 4, ".xyz") == 0);
    bool isPly = (4 < len && strcmp(file + len - 4, ".ply") == 0);
    bool isPcd = (4 < len && strcmp(file + len - 4, ".pcd") == 0);
    if (isPcd) {
      pcl::io::loadPCDFile(file, *cloud0);
    } else if (isXyz || isPly) {
      FILE* in = fopen(file, "r");
      if (in == NULL) {
        fprintf(stderr, "cannot open file '%s'\n", file);
        return 1;
      }
      if (isPly) {
        while (!feof(in)) {
          int count;
          char *line;
          size_t length = 0;
          if (getline(&line, &length, in) == -1) {
            fprintf(stderr, "Error reading from '%s'.\n", file);
            return(1);
          }
          if (strlen("end_header") <= length
              && memcmp("end_header", line, strlen("end_header")) == 0) {
            free(line);
            break;
          }
          free(line);
        }
      }
      while (!feof(in)) {
        float x, y, z;
        if (fscanf(in, "%f %f %f\n", &x, &y, &z) < 3) {
          fprintf(stderr, "fscanf error in '%s'\n", file);
          return 1;
        }
        cloud0->points.push_back(pcl::PointXYZ(x, y, z));
      }
    } else {
      fprintf(stderr, "unknown file type '%s'\n", file);
      return 1;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);

    // PCL appears to have no way to convert a cloud from one type to another.  Wacky.
    cloud1->points.resize(cloud0->size());
    for (size_t j = 0; j < cloud0->points.size(); j++) {
      cloud1->points[j].x = cloud0->points[j].x;
      cloud1->points[j].y = cloud0->points[j].y;
      cloud1->points[j].z = cloud0->points[j].z;
      cloud1->points[j].rgb = *reinterpret_cast<float*>(&colors[i % 4]);
    }
    fprintf(stderr, "using color %06X\n", colors[i]);
    viewer.showCloud(cloud1, argv[i]);
  }

  while (!viewer.wasStopped ()) {
  }
  return 0;
}
