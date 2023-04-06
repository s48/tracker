// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "robot.hpp"
#include "mesh.hpp"
#include "cruft.hpp"

int main(int argc, char **argv)
{
  Robot robot("../resources/person/person.urdf");
  fprintf(stderr, "[person has %d controls]\n", robot.getControlCount());
  std::vector<float> avatarPose{0.0f, 0.5f};
  auto transforms = robot.makeLinkTransforms(avatarPose);
  printTransform("person 0", transforms[0]);
  printTransform("person 1", transforms[1]);
  Robot robot2("../../model/resources/kr16/kr16.urdf");
  Robot robot3("../../model/resources/r2000/r2000.urdf");
  Mesh mesh;
  mesh.readStlFile("../resources/person/person.stl");
  mesh.writeStlFile("test.stl");
  return 0;
}
