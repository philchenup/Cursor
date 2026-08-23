#pragma once

#include <Eigen/Core>
#include <string>

// Minimal PCD reader (ascii / binary / binary_compressed). No PCL.
bool LoadPcdPoints(const std::string& path, Eigen::MatrixX3f& points);
bool LoadPcdPointsAndNormals(const std::string& path,
                             Eigen::MatrixX3f& points,
                             Eigen::MatrixX3f& normals);
bool SavePlyPoints(const std::string& path, const Eigen::MatrixX3f& points);
