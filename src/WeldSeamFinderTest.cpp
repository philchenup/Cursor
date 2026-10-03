#include "WeldSeamFinder.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

// 合成一块 T 型板点云，检查 FindWeldSeam 的焊缝方向，并打开窗口显示。
// 灰色是原始点云，红色是空洞区域，绿色箭头是空洞 PCA 主方向。
//
// 底板在 z = 0，中间留一条沿 X 的缝隙；立板立在缝隙上方。
// 点云里混入零点、NaN、远离主体的小团块，以及一块更小的干扰平面，
// 再整体旋转和平移。期望主方向沿缝隙，地面法向垂直于底板。
//
// 在仓库根目录编译运行（Ubuntu 上需能找到 VTK 头文件和库）：
// vtk_libs=$(ldd /usr/lib/x86_64-linux-gnu/libpcl_visualization.so | awk '/vtk/ {so=$1; sub(/\.so.*/, "", so); sub(/^lib/, "-l", so); printf "%s ", so}')
// g++ -std=c++17 -O2 -Iinclude -I/usr/include/vtk-9.1 $(pkg-config --cflags pcl_visualization) src/WeldSeamFinderTest.cpp src/WeldSeamFinder.cpp -Wl,--no-as-needed $(pkg-config --libs pcl_visualization pcl_filters pcl_segmentation pcl_surface pcl_sample_consensus pcl_search) $vtk_libs -o weld_seam_test && ./weld_seam_test

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

pcl::PointCloud<pcl::PointXYZ>::Ptr FinitePoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr finite(new pcl::PointCloud<pcl::PointXYZ>);
    finite->reserve(cloud->size());
    for (const pcl::PointXYZ& point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            finite->push_back(point);
        }
    }
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1;
    finite->is_dense = true;
    return finite;
}

void AddDirectionArrow(pcl::visualization::PCLVisualizer& viewer, const WeldSeamResult& result)
{
    const Eigen::Vector3f direction = result.direction.normalized();
    const Eigen::Vector3f normal = result.plane.head<3>().normalized();
    const Eigen::Vector3f side = normal.cross(direction).normalized();
    const Eigen::Vector3f lift = normal * 6.0f;

    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    for (const pcl::PointXYZ& point : result.hollow->points) {
        const float offset = (point.getVector3fMap() - result.centroid).dot(direction);
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
    }

    const float span = std::max(maxOffset - minOffset, 1.0f);
    const float padding = 0.06f * span;
    const float headLength = std::max(18.0f, 0.07f * span);
    const Eigen::Vector3f tail = result.centroid + (minOffset - padding) * direction + lift;
    const Eigen::Vector3f tip = result.centroid + (maxOffset + padding) * direction + lift;
    const Eigen::Vector3f headBase = tip - direction * headLength;
    const float headWidth = headLength * 0.38f;

    auto toPoint = [](const Eigen::Vector3f& position) {
        return pcl::PointXYZ(position.x(), position.y(), position.z());
    };

    const pcl::PointXYZ tailPoint = toPoint(tail);
    const pcl::PointXYZ tipPoint = toPoint(tip);
    const pcl::PointXYZ leftPoint = toPoint(headBase + side * headWidth);
    const pcl::PointXYZ rightPoint = toPoint(headBase - side * headWidth);

    viewer.addLine(tailPoint, tipPoint, 0.20, 0.95, 0.35, "seam-shaft");
    viewer.addLine(leftPoint, tipPoint, 0.20, 0.95, 0.35, "seam-head-left");
    viewer.addLine(rightPoint, tipPoint, 0.20, 0.95, 0.35, "seam-head-right");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "seam-shaft");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "seam-head-left");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "seam-head-right");
}

void ShowClouds(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& original, const WeldSeamResult& result)
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr drawable = FinitePoints(original);

    pcl::visualization::PCLVisualizer viewer("T-plate weld");
    viewer.setSize(1280, 800);
    viewer.setBackgroundColor(0.07, 0.08, 0.10);
    viewer.addText("gray: original cloud", 16, 64, 18, 0.82, 0.84, 0.86, "label-original");
    viewer.addText("red: hollow region", 16, 38, 18, 1.0, 0.35, 0.28, "label-hollow");
    viewer.addText("green: principal direction", 16, 12, 18, 0.25, 0.95, 0.35, "label-direction");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> originalColor(drawable, 186, 194, 204);
    viewer.addPointCloud(drawable, originalColor, "original");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> hollowColor(result.hollow, 255, 72, 56);
    viewer.addPointCloud(result.hollow, hollowColor, "hollow");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "hollow");

    AddDirectionArrow(viewer, result);

    const Eigen::Vector3f normal = result.plane.head<3>().normalized();
    const Eigen::Vector3f direction = result.direction.normalized();
    const Eigen::Vector3f side = normal.cross(direction).normalized();
    const Eigen::Vector3f eye = result.centroid + normal * 760.0f + side * 460.0f - direction * 40.0f;
    viewer.setCameraPosition(eye.x(), eye.y(), eye.z(),
                             result.centroid.x(), result.centroid.y(), result.centroid.z(),
                             normal.x(), normal.y(), normal.z());
    viewer.setCameraFieldOfView(0.62);
    viewer.setCameraClipDistances(10.0, 5000.0);

    std::cout << "关闭窗口后退出\n";
    viewer.spin();
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
    ShowClouds(cloud, result);
    return EXIT_SUCCESS;
}
