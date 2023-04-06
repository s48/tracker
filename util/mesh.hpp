// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef MESH_HPP
#define MESH_HPP

//#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

extern char *vecToString(Eigen::Vector3f v);
extern void printTransform(const char *name, Eigen::Affine3f transform);

class Mesh {

public:
  Mesh();
  ~Mesh();
  Mesh(Mesh&&);
  Mesh(const Mesh&);
  Mesh& operator=(const Mesh&);
  Mesh& operator=(Mesh&&);

  void readStlFile(const std::string& file);
  void writeStlFile(const std::string& file);
  void writePlyFile(const std::string& file);
  void writeCloudFile(const std::string& file);

  // Add various basic chapes.
  void addPoint(Eigen::Vector3f p1);
  void addTriangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);
  void addSquare(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4);
  void addBox(Eigen::Vector3f corner, Eigen::Vector3f dimensions);
  void addIcosahedron(float radius, const Eigen::Vector3f& center);
  void addMesh(const Mesh& mesh);

  void clear() {
    mPoints.clear();
    mNormals.clear();
  }

  void transform(const Eigen::Affine3f& transform) {
    int count = 0;
    for (auto& vertex : mPoints) {
      vertex = transform * vertex;
    }
    Eigen::Affine3f normalTransform(transform.linear().inverse().transpose());
    for (auto& normal : mNormals) {
      normal = normalTransform * normal;
    }
  }

  void scale(float scaleFactor) {
    int count = 0;
    for (auto& vertex : mPoints) {
      vertex *= scaleFactor;
    }
  }

  Eigen::Vector3f boundingBoxSize(void);

  Mesh getTransformed(const Eigen::Affine3f& transform) const {
    Mesh copy(*this);
    copy.transform(transform);
    return copy;
  }

  const std::vector<Eigen::Vector3f>& getPoints() const {
    assert(mIsCloud);
    return mPoints;
  }

  std::vector<Eigen::Vector3f>& getPoints() {
    assert(mIsCloud);
    return mPoints;
  }

  const std::vector<Eigen::Vector3f>& getTriangles() const {
    assert(!mIsCloud);
    return mPoints;
  }

  std::vector<Eigen::Vector3f>& getTriangles() {
    assert(!mIsCloud);
    return mPoints;
  }

  const std::vector<Eigen::Vector3f>& getNormals() const {
    return mNormals;
  }

  std::vector<Eigen::Vector3f>& getNormals() {
    return mNormals;
  }

  Mesh makeSampledCloud(float pointDensity) const;

private:
  bool mIsCloud;
  std::vector<Eigen::Vector3f> mPoints;
  std::vector<Eigen::Vector3f> mNormals;
};

#endif // MESH_HPP
