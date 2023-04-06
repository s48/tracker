// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "mesh.hpp"

#include <cstdio>
#include <fstream>
#include <random>

#include <Eigen/Core>

Mesh::Mesh() = default;
Mesh::Mesh(Mesh&&) = default;
Mesh::~Mesh() = default;
Mesh& Mesh::operator=(Mesh&&) = default;

Mesh::Mesh(const Mesh& other) :
  mIsCloud(other.mIsCloud),
  mPoints(other.mPoints),
  mNormals(other.mNormals)
{
}

Mesh& Mesh::operator=(Mesh const& other) {
  mIsCloud = other.mIsCloud;
  mPoints = other.mPoints;
  mNormals = other.mNormals;
  return *this;
}

void Mesh::readStlFile(const std::string& file)
{
    mIsCloud = false;
    std::ifstream fin(file, std::ios::in);

    if (!fin.is_open()) {
        fprintf(stderr, "Can't open STL file '%s'\n", file.c_str());
        exit(0);
    }

    char header[80];
    fin.read(header, 80);
    int count;
    fin.read((char*)&count, 4);
    //printf("%d triangles\n", count);
    for (int i = 0; i < count; i++) {
        Eigen::Vector3f normal;
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Eigen::Vector3f p3;
        uint16_t att;

        fin.read((char*)normal.data(), 12);
        fin.read((char*)p1.data(), 12);
        fin.read((char*)p2.data(), 12);
        fin.read((char*)p3.data(), 12);
        fin.read((char*)&att, 2);

        mPoints.push_back(p1);
        mPoints.push_back(p2);
        mPoints.push_back(p3);

        if (normal.x() == 0.0f && normal.y() == 0.0f && normal.z() == 0.0f) {
            normal = (p2 - p1).cross(p3 - p1);
            normal.normalize();
        }
        mNormals.push_back(normal);
    }
}

void Mesh::addPoint(const Eigen::Vector3f point)
{
    assert(mIsCloud || mPoints.size() == 0);
    mIsCloud = true;
    mPoints.push_back(point);
}

void Mesh::addTriangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3)
{
  assert(!mIsCloud || mPoints.size() == 0);
  mIsCloud = false;
  mPoints.push_back(p1);
  mPoints.push_back(p2);
  mPoints.push_back(p3);

  Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1);
  normal.normalize();
  mNormals.push_back(normal);
}

void Mesh::addSquare(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4)
{
    addTriangle(p1, p2, p3);
    addTriangle(p3, p4, p1);
}

// Makes boxes aligned with the coordinate axes.
void Mesh::addBox(Eigen::Vector3f corner, Eigen::Vector3f dimensions)
{
    Eigen::Vector3f a = corner;
    Eigen::Vector3f b = a + Eigen::Vector3f(dimensions.x(), 0.0f, 0.0f);
    Eigen::Vector3f c = b + Eigen::Vector3f(0.0f, dimensions.y(), 0.0f);
    Eigen::Vector3f d = a + Eigen::Vector3f(0.0f, dimensions.y(), 0.0f);
    
    Eigen::Vector3f e = a + Eigen::Vector3f(0.0f, 0.0f, dimensions.z());
    Eigen::Vector3f f = b + Eigen::Vector3f(0.0f, 0.0f, dimensions.z());
    Eigen::Vector3f g = c + Eigen::Vector3f(0.0f, 0.0f, dimensions.z());
    Eigen::Vector3f h = d + Eigen::Vector3f(0.0f, 0.0f, dimensions.z());
    
    addSquare(a, d, c, b); // bottom
    addSquare(e, f, g, h); // top
    
    addSquare(a, b, f, e);
    addSquare(b, c, g, f);
    addSquare(c, d, h ,g);
    addSquare(d, a, e, h);
}

// Icosahedron

const float X = 0.525731112119133606;
const float Z = 0.850650808352039932;

// Just to make these a little easier to read.
using vec3f = Eigen::Vector3f;
using vec3i = Eigen::Vector3i;

const std::array<vec3f, 12> icosahedronVertices{
    vec3f{-X, 0.0, Z}, vec3f{X, 0.0, Z}, vec3f{-X, 0.0, -Z}, vec3f{X, 0.0, -Z},    
    vec3f{0.0, Z, X}, vec3f{0.0, Z, -X}, vec3f{0.0, -Z, X}, vec3f{0.0, -Z, -X},    
    vec3f{Z, X, 0.0}, vec3f{-Z, X, 0.0}, vec3f{Z, -X, 0.0}, vec3f{-Z, -X, 0.0} 
};

const std::array<vec3i, 20> icosahedronFaces{
    vec3i{0, 4, 1}, vec3i{0, 9, 4}, vec3i{9, 5, 4}, vec3i{4, 5, 8}, vec3i{4, 8, 1},    
    vec3i{8, 10, 1}, vec3i{8, 3, 10}, vec3i{5, 3, 8}, vec3i{5, 2, 3}, vec3i{2, 7, 3},    
    vec3i{7, 10, 3}, vec3i{7, 6, 10}, vec3i{7, 11, 6}, vec3i{11, 0, 6}, vec3i{0, 1, 6}, 
    vec3i{6, 1 ,10}, vec3i{9, 0, 11}, vec3i{9, 11, 2}, vec3i{9, 2, 5}, vec3i{7, 2, 11} 
};

void Mesh::addIcosahedron(float radius, const Eigen::Vector3f& center)
{
    for (auto& face : icosahedronFaces) {
        addTriangle(icosahedronVertices[face.x()] * radius + center,
                icosahedronVertices[face.z()] * radius + center,
                icosahedronVertices[face.y()] * radius + center);
    }
}

void Mesh::addMesh(const Mesh& mesh)
{
  assert(mIsCloud == mesh.mIsCloud);
    for (auto& p : mesh.mPoints) {
        mPoints.push_back(p);
    }
    for (auto& p : mesh.mNormals) {
        mNormals.push_back(p);
    }
}

Eigen::Vector3f Mesh::boundingBoxSize(void)
{
  float xMin = std::numeric_limits<float>::max();
  float yMin = std::numeric_limits<float>::max();
  float zMin = std::numeric_limits<float>::max();
  float xMax = std::numeric_limits<float>::min();
  float yMax = std::numeric_limits<float>::min();
  float zMax = std::numeric_limits<float>::min();
  for (auto& vertex : mPoints) {
    xMin = std::min(vertex.x(), xMin);
    yMin = std::min(vertex.y(), yMin);
    zMin = std::min(vertex.z(), zMin);
    xMax = std::max(vertex.x(), xMax);
    yMax = std::max(vertex.y(), yMax);
    zMax = std::max(vertex.z(), zMax);
  }
  return Eigen::Vector3f(xMax - xMin, yMax - yMin, zMax - zMin);
}

void Mesh::writeStlFile(const std::string& file)
{
  assert(!mIsCloud);
    std::ofstream fout(file, std::ios::out | std::ios::binary);
    char header[80] = "stl";
    fout.write(header, 80);
    size_t count = mPoints.size() / 3;
    fout.write((char*)&count, 4);
    for (size_t i = 0; i < count; i++) {
        fout.write((char*)mNormals[i].data(), 12);
        fout.write((char*)mPoints[3 * i].data(), 12);
        fout.write((char*)mPoints[3 * i + 1].data(), 12);
        fout.write((char*)mPoints[3 * i + 2].data(), 12);
        uint16_t att = 0;
        fout.write((char*)&att, 2);
    }
    fout.close();
}

void Mesh::writeCloudFile(const std::string& file)
{
    assert(mIsCloud);
    std::ofstream fout(file, std::ios::out | std::ios::binary);
    char header[80] = "point cloud";
    fout.write(header, 80);
    size_t count = mPoints.size();
    fout.write((char*)&count, 4);
    for (auto& point : mPoints) {
        fout.write((char*)point.data(), 12);
    }
    fout.close();
}

void Mesh::writePlyFile(const std::string& file)
{
    assert(mIsCloud);
    std::ofstream fout(file);
    fout << "ply" << std::endl;
    fout << "format ascii 1.0" << std::endl;
    fout << "element vertex " << mPoints.size() << std::endl;
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
    fout << "end_header" << std::endl;
    for (auto& point : mPoints) {
      fout << point.x() << " " << point.y() << " "  << point.z() << std::endl;
    }
    fout.close();
}

class RandomFloat {
public:
    RandomFloat(std::mt19937::result_type seed = std::mt19937::default_seed) :
        mSeed(seed),
        mGenerator(std::mt19937(mSeed)),
        mUniform(std::bind(std::uniform_real_distribution<float>(0, 1), mGenerator))
    {
    }

    float operator()(float min, float max)
    {
        float r = mUniform();
        return min + r * (max - min);
    };
private:
    int64_t mSeed;
    std::mt19937 mGenerator;
    std::function<float()> mUniform;
};

// http://www.cs.princeton.edu/~funk/tog02.pdf
Mesh Mesh::makeSampledCloud(float pointDensity) const
{
  assert(!mIsCloud);

  Mesh cloud;
  cloud.mIsCloud = true;

  if (mPoints.empty()) {
    return cloud;
  }

  std::vector<float> areas;
  std::vector<float> cumulativeAreas;

  auto i = mPoints.begin();

  float totalArea = 0.0f;
  while (i < mPoints.end()) {
    Eigen::Vector3f p0 = *i++;
    Eigen::Vector3f p1 = *i++;
    Eigen::Vector3f p2 = *i++;

    float area = std::fabs((p1 - p0).cross(p2 - p0).norm() / 2.0f);
    areas.push_back(area);
    totalArea += area;
    cumulativeAreas.push_back(totalArea);
  }

  RandomFloat randomFloat;
  if (totalArea > 5000) {
    throw std::runtime_error("Input mesh area is too big, likely wrong units");
  }
  int totalPoints = (int)std::ceil(pointDensity * totalArea);
  // fprintf(stderr, "[area %f points %d]\n", totalArea, totalPoints);

  // The randomly
  // It would probably work better to select out those triangles that are large
  // enough to justify more than a few points and place their points in a grid.
  // You could do this by randomly assigning points to triangles and afterwards
  // add the points themselves.
  for (int j = 0; j < totalPoints; j++) {
    float r0 = randomFloat(0.0f, totalArea);
    auto it = std::upper_bound(cumulativeAreas.begin(), cumulativeAreas.end(), r0);
    int index = (int)(it - cumulativeAreas.begin());
    Eigen::Vector3f p0 = mPoints[3 * index];
    Eigen::Vector3f p1 = mPoints[3 * index + 1];
    Eigen::Vector3f p2 = mPoints[3 * index + 2];
    float r1 = std::sqrt(randomFloat(0.0f, 1.0f));
    float r2 = randomFloat(0.0f, 1.0f);
    Eigen::Vector3f p = p0 * (1.0f - r1) + p1 * r1 * (1.0f - r2) + p2 * r1 * r2;
    cloud.mPoints.push_back(p);
    cloud.mNormals.push_back(mNormals[index]);
  }
  return cloud;
}

/*
int main(int argc, char** argv)
{
    // Read in mesh file and transform per urdf link info
    Mesh mesh{};
    mesh.readSTL(argv.y());
    BBox3D bb = mesh.getBBox();
    printf("min %f %f %f\n", bb.min().x(), bb.min().y(), bb.min().z());
    printf("max %f %f %f\n", bb.max().x(), bb.max().y(), bb.max().z());

    Mesh bbMesh{};
    Eigen::Vector3f xyz{bb.min().x(), bb.min().y(), bb.min().z()};
    Eigen::Vector3f xyZ{bb.min().x(), bb.min().y(), bb.max().z()};
    Eigen::Vector3f xYz{bb.min().x(), bb.max().y(), bb.min().z()};
    Eigen::Vector3f xYZ{bb.min().x(), bb.max().y(), bb.max().z()};
    Eigen::Vector3f Xyz{bb.max().x(), bb.min().y(), bb.min().z()};
    Eigen::Vector3f XyZ{bb.max().x(), bb.min().y(), bb.max().z()};
    Eigen::Vector3f XYz{bb.max().x(), bb.max().y(), bb.min().z()};
    Eigen::Vector3f XYZ{bb.max().x(), bb.max().y(), bb.max().z()};

    bbMesh.addSquare(xyz, xYz, XYz, Xyz); // bottom
    bbMesh.addSquare(xyZ, XyZ, XYZ, xYZ); // top
    bbMesh.addSquare(xyz, Xyz, XyZ, xyZ); // front
    bbMesh.addSquare(xYz, xYZ, XYZ, XYz); // back
    bbMesh.addSquare(xyz, xyZ, xYZ, xYz); // left
    bbMesh.addSquare(Xyz, XYz, XYZ, XyZ); // right

    bbMesh.writeSTL(argv.z());

    return 1;
}
*/
/*
int main(int argc, char** argv)
{
    Mesh mesh{};

    // left leg
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, 0.15f, 0.18f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, 0.15f, 0.38f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, 0.15f, 0.58f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, 0.15f, 0.78f});

    // right leg
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.15f, 0.18f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.15f, 0.38f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.15f, 0.58f});
    mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.15f, 0.78f});

    // body
    mesh.addIcosahedron(0.35, Eigen::Vector3f{0.0f, 0.0f, 1.00f});
    mesh.addIcosahedron(0.35, Eigen::Vector3f{0.0f, 0.0f, 1.44f});

    // head
    mesh.addIcosahedron(0.20, Eigen::Vector3f{0.0f, 0.0f, 1.83f});

    mesh.writeSTL(argv.y());

    // left arm
    mesh.clear();
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, 0.09f, 0.0f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, 0.09f, -0.18f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, 0.09f, -0.36f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, 0.09f, -0.52f});
    mesh.writeSTL(argv.z());

    mesh.clear();
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, -0.09f, 0.0f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, -0.09f, -0.18f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, -0.09f, -0.36f});
    mesh.addIcosahedron(0.12, Eigen::Vector3f{0.0f, -0.09f, -0.52f});
    mesh.writeSTL(argv[3]);

    // right arm - version 1
    // mesh.clear();
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.12f, 0.0f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.12f, -0.20f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.12f, -0.40f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.12f, -0.60f});
    // mesh.writeSTL(argv[3]);
      // complete right arm in rest position
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.35f, 1.45f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.35f, 1.25f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.35f, 1.05f});
    // mesh.addIcosahedron(0.16, Eigen::Vector3f{0.0f, -0.35f, 0.85f});
    return 1;
}

*/
