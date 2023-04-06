// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// For producing STL files with various objects.

#include "cruft.hpp"
#include "mesh.hpp"

#include <iostream>
#include <fstream>

#include <Eigen/Core>

// The renderer screws up on triangles that extend too far outside
// the camera's view, so we draw the floor as a set of tiles instead
// of using one big square.

static void addFloor(Mesh& mesh, float tileSize, uint32_t tiles)
{
    float start = -(tiles/2 * tileSize);
    for (int i = 0; i < tiles; i++) {
        float x = start + i * tileSize;
        for (int j = 0; j < tiles; j++) {
            float y = start + j * tileSize;
            mesh.addSquare(Eigen::Vector3f(x, y, 0.0f),
                    Eigen::Vector3f(x + tileSize, y, 0.0f),
                    Eigen::Vector3f(x + tileSize, y + tileSize, 0.0f),
                    Eigen::Vector3f(x, y + tileSize, 0.0f));
        }
    }
}

const float legHeight = 0.66f;

// x,y is the location of the lower left corner of the table
static void addTable(Mesh& mesh, float x, float y, float xSize, float ySize)
{
    // top
    mesh.addBox(Eigen::Vector3f(x, y, legHeight),
                Eigen::Vector3f(xSize, ySize, 0.04f));

    // legs are 5cm square and located 4cm inside each corner
    auto legSize = Eigen::Vector3f(0.05f, 0.05f, legHeight);
    mesh.addBox(Eigen::Vector3f(x + 0.04,         y + 0.04,         0.0f), legSize);
    mesh.addBox(Eigen::Vector3f(x + xSize - 0.09, y + 0.04,         0.0f), legSize);
    mesh.addBox(Eigen::Vector3f(x + 0.04,         y + ySize - 0.09, 0.0f), legSize);
    mesh.addBox(Eigen::Vector3f(x + xSize - 0.09, y + ySize - 0.09, 0.0f), legSize);
}

// Same as tables, but with three levels.
const float shelfHeight0 = 0.75f;
const float shelfHeight1 = 1.25f;
const float shelfHeight2 = 1.75f;

static void addShelf(Mesh& mesh,
                     float x, float y, float z,
                     float xSize, float ySize, float zSize)
{
    mesh.addBox(Eigen::Vector3f(x, y, z + zSize), Eigen::Vector3f(xSize, ySize, 0.04f));
    auto legSize = Eigen::Vector3f(0.05f, 0.05f, zSize);
    mesh.addBox(Eigen::Vector3f(x,                y,                z), legSize);
    mesh.addBox(Eigen::Vector3f(x + xSize - 0.05, y,                z), legSize);
    mesh.addBox(Eigen::Vector3f(x,                y + ySize - 0.05, z), legSize);
    mesh.addBox(Eigen::Vector3f(x + xSize - 0.05, y + ySize - 0.05, z), legSize);
}


static void layout(char *filename)
{
    Mesh mesh{};

    addFloor(mesh, 0.5f, 30);

    // avatar's table
    addTable(mesh, -1.5f, -2.5f, 3.0f, 1.0f);

    // parts bins
    addShelf(mesh, 3.0f, -1.0f, 0.0f,  0.5f, 2.0f, 0.75f);
    addShelf(mesh, 3.0f, -1.0f, 0.75f, 0.5f, 2.0f, 0.50f);
    addShelf(mesh, 3.0f, -1.0f, 1.25f, 0.5f, 2.0f, 0.50f);

    // conveyor belt or whatever
    addTable(mesh, -2.0f, 2.0f, 2.5f, 1.0f);
    addTable(mesh, 0.5f, 2.0f, 2.5f, 1.0f);

    mesh.writeStlFile(filename);
}

int main(int argc, char** argv)
{
    layout(argv[1]);
    return 1;
}
