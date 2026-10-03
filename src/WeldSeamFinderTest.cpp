#include "WeldSeamFinder.h"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

// 合成一块 T 型板点云，检查 FindWeldSeam 的焊缝方向。
//
// 底板在 z = 0，中间留一条沿 X 的缝隙；立板立在缝隙上方。
// 点云里混入零点、NaN、远离主体的小团块，以及一块更小的干扰平面，
// 再整体旋转和平移。期望主方向沿缝隙，地面法向垂直于底板。
//
// 在仓库根目录编译运行：
// g++ -std=c++17 -O2 -Iinclude $(pkg-config --cflags pcl_common) src/WeldSeamFinderTest.cpp src/WeldSeamFinder.cpp $(pkg-config --libs pcl_common pcl_filters pcl_segmentation pcl_surface pcl_sample_consensus pcl_search pcl_kdtree) -o weld_seam_test && ./weld_seam_test

namespace {

constexpr float kPi = 3.14159265358979323846f;

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeTPlate()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto add = [&](float x, float y, float z) {
        cloud->push_back(pcl::PointXYZ(x, y, z));
    };

    // 底板：400 mm x 300 mm，|y| < 8 mm 的缝隙不生成点
    for (float x = -200.0f; x <= 200.0f; x += 3.0f) {
        for (float y = -150.0f; y <= 150.0f; y += 3.0f) {
            if (std::fabs(y) < 8.0f) {
                continue;
            }
            add(x, y, 0.0f);
        }
    }

    // 立板：高度 25 mm 以上，厚度方向落在 y = ±4 mm
    for (float x = -180.0f; x <= 180.0f; x += 3.0f) {
        for (float z = 25.0f; z <= 120.0f; z += 3.0f) {
            add(x, -4.0f, z);
            add(x, 4.0f, z);
        }
    }

    // 高度 10 mm，应被 20 mm 阈值滤掉，不能把缝隙填上
    for (float x = -100.0f; x <= 100.0f; x += 2.0f) {
        add(x, 0.0f, 10.0f);
    }

    // 更小的高台，不应被当成地面
    for (float x = -20.0f; x <= 20.0f; x += 4.0f) {
        for (float y = 60.0f; y <= 100.0f; y += 4.0f) {
            add(x, y, 100.0f);
        }
    }

    // 小块点云团和稀疏杂点
    for (int i = 0; i < 20; ++i) {
        add(2000.0f + static_cast<float>(i) * 0.4f, 2000.0f, 2000.0f);
    }
    for (int i = 0; i < 30; ++i) {
        add(3000.0f + static_cast<float>(i) * 50.0f, -2500.0f, 10.0f);
    }

    add(0.0f, 0.0f, 0.0f);
    add(1e-8f, -1e-8f, 0.0f);
    cloud->push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 1.0f, 2.0f));
    cloud->push_back(pcl::PointXYZ(1.0f, std::numeric_limits<float>::infinity(), 2.0f));

    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

void Transform(pcl::PointCloud<pcl::PointXYZ>& cloud,
               const Eigen::Matrix3f& rotation,
               const Eigen::Vector3f& translation)
{
    for (pcl::PointXYZ& point : cloud.points) {
        const Eigen::Vector3f transformed = rotation * point.getVector3fMap() + translation;
        point.x = transformed.x();
        point.y = transformed.y();
        point.z = transformed.z();
    }
}

bool Expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "失败: " << message << '\n';
    }
    return condition;
}

} // namespace

int main()
{
    int failures = 0;

    const WeldSeamResult nullResult = FindWeldSeam(pcl::PointCloud<pcl::PointXYZ>::ConstPtr());
    failures += !Expect(!nullResult.success, "空指针应失败");

    pcl::PointCloud<pcl::PointXYZ>::Ptr blank(new pcl::PointCloud<pcl::PointXYZ>);
    failures += !Expect(!FindWeldSeam(blank).success, "空点云应失败");

    const Eigen::AngleAxisf tilt(15.0f * kPi / 180.0f, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw(28.0f * kPi / 180.0f, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix3f rotation = (yaw * tilt).toRotationMatrix();
    const Eigen::Vector3f translation(50.0f, -30.0f, 80.0f);
    const Eigen::Vector3f expectedDirection = (rotation * Eigen::Vector3f::UnitX()).normalized();
    const Eigen::Vector3f expectedNormal = (rotation * Eigen::Vector3f::UnitZ()).normalized();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = MakeTPlate();
    Transform(*cloud, rotation, translation);

    const WeldSeamResult result = FindWeldSeam(cloud);
    failures += !Expect(result.success, "应找到焊缝");
    if (result.success) {
        const float alignment = std::fabs(result.direction.normalized().dot(expectedDirection));
        const float normalAlign = std::fabs(result.plane.head<3>().normalized().dot(expectedNormal));
        const float onPlane = std::fabs(result.plane.head<3>().dot(result.centroid) + result.plane[3]);
        const float inPlane = std::fabs(result.direction.dot(result.plane.head<3>()));
        const float centroidError = (result.centroid - translation).norm();

        std::cout << "方向 " << result.direction.transpose()
                  << "  对齐 " << alignment << '\n'
                  << "法向 " << result.plane.head<3>().transpose()
                  << "  对齐 " << normalAlign << '\n'
                  << "质心 " << result.centroid.transpose()
                  << "  误差 " << centroidError << " mm\n"
                  << "空洞点数 " << result.hollow->size()
                  << "  平面残差 " << onPlane
                  << "  方向离面 " << inPlane << '\n';

        failures += !Expect(alignment > 0.98f, "主方向应沿焊缝");
        failures += !Expect(normalAlign > 0.98f, "最大平面应为底板");
        failures += !Expect(onPlane < 1.0f, "质心应在地面上");
        failures += !Expect(inPlane < 1e-3f, "方向应位于地面内");
        failures += !Expect(centroidError < 30.0f, "质心应落在焊缝附近");
        failures += !Expect(result.hollow && result.hollow->size() > 50, "空洞点数过少");
    }

    if (failures != 0) {
        std::cerr << failures << " 项检查失败\n";
        return EXIT_FAILURE;
    }
    std::cout << "通过\n";
    return EXIT_SUCCESS;
}
