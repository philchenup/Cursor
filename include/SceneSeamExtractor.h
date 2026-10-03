#ifndef SCENE_SEAM_EXTRACTOR_H
#define SCENE_SEAM_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

/**
 * @brief 整场点云初始焊缝提取参数。所有长度单位为毫米，角度单位为度。
 *
 * 适用场景：3D 相机光轴垂直地面向下拍摄，组立工件底板平放在地面或垫块上，
 * 立板垂直于底板。整场点云已拼接到同一坐标系，Z 轴与地面法向大致平行；
 * 默认按相机坐标系处理（Z 轴指向地面，地面 Z 最大），内部翻转为 Z 向上计算，
 * 所有输出（平面、焊缝、点云）再翻转回输入坐标系。
 */
struct SceneSeamParams {
    // 坐标系
    bool zAxisDown = true;              ///< 输入 Z 轴指向地面（相机深度方向）；点云 Z 轴向上时设为 false

    // 预处理
    float voxelSize = 4.0f;             ///< 体素下采样边长，<= 0 关闭
    float sceneResolution = 5.0f;       ///< 整场高度图栅格边长，用于分割工件
    float workpieceResolution = 2.0f;   ///< 单个工件高度图栅格边长，用于找底板和立板

    // 地面
    float groundHeight = std::numeric_limits<float>::quiet_NaN(); ///< 已标定地面高度（输入坐标系的 Z 值）；非有限值时自动估计
    float groundThreshold = 3.0f;       ///< 高出地面超过该值的栅格视为物体，应小于最薄底板厚度
    float groundFitTolerance = 4.0f;    ///< 地面平面拟合内点容差
    float maxGroundTiltDeg = 5.0f;      ///< 地面法向与 Z 轴最大夹角，超出则退回水平面

    // 工件分割
    float closeRadius = 15.0f;          ///< 物体掩码闭运算半径，用于合并阴影缺失
    float minWorkpieceArea = 50000.0f;  ///< 工件最小投影面积 mm^2，用于过滤杂物
    float minWorkpieceSize = 200.0f;    ///< 工件外接框短边最小长度

    // 底板
    float basePlaneTolerance = 4.0f;    ///< 底板平面内点容差

    // 立板
    float ribMinHeight = 20.0f;         ///< 高出底板超过该值的点参与立板提取
    float ribMinLength = 60.0f;         ///< 立板落地轮廓最短长度
    float ribMinThickness = 4.0f;       ///< 板厚下限
    float ribMaxThickness = 30.0f;      ///< 板厚上限，同时决定直线提取带宽
    float ribGapTolerance = 45.0f;      ///< 同一直线上两段落地轮廓间隙小于该值时合并，应大于两倍带宽
    float ribSnapAngleDeg = 3.0f;       ///< 与主方向或其垂直方向夹角小于该值时吸附
    float junctionTolerance = 25.0f;    ///< 立板端部到另一块立板的距离小于该值时视为 T 形搭接
    float houghAngleStepDeg = 0.5f;     ///< Hough 直线角度步长

    // 焊缝
    float seamClearance = 5.0f;         ///< 平角焊缝在相交立板处留出的间隙
    float minSeamLength = 30.0f;        ///< 平角焊缝最短长度
    float minConfidence = 0.3f;         ///< 低于该置信度的焊缝不进入汇总列表
};

/// 立板在底板平面上的落地轮廓中心线。
struct RibSegment {
    int id = -1;
    Eigen::Vector2f start = Eigen::Vector2f::Zero(); ///< 场景 XY 坐标
    Eigen::Vector2f end = Eigen::Vector2f::Zero();
    float thickness = 0.0f;
    float height = 0.0f;      ///< 立板顶面高出底板的高度
    float confidence = 0.0f;  ///< 0~1

    Eigen::Vector2f direction() const;
    float length() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class SeamType {
    FlatFillet,     ///< 立板与底板之间的平角焊缝
    VerticalFillet  ///< 两块立板之间的立角焊缝
};

/// 初始焊缝，用于引导后续激光或接触寻位。
struct InitialSeam {
    int workpieceId = -1;
    SeamType type = SeamType::FlatFillet;
    Eigen::Vector3f start = Eigen::Vector3f::Zero(); ///< 场景坐标。立焊缝从底板指向顶部
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    Eigen::Vector3f approachSide = Eigen::Vector3f::Zero(); ///< 焊枪接近方向的单位向量，位于底板平面内
    float ribHeight = 0.0f;   ///< 相邻立板高度，用于焊枪避让
    float confidence = 0.0f;  ///< 0~1
    int ribA = -1;            ///< 所属立板
    int ribB = -1;            ///< 立焊缝的另一块立板，平焊缝为 -1

    float length() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 一个组立工件。
struct Workpiece {
    int id = -1;
    Eigen::Vector4f basePlane = Eigen::Vector4f::Zero(); ///< 底板上表面 ax + by + cz + d = 0，法向朝上
    float baseHeight = 0.0f;  ///< 底板上表面在工件中心处高出地面的值
    Eigen::Vector2f center = Eigen::Vector2f::Zero();
    Eigen::Vector2f minXY = Eigen::Vector2f::Zero();
    Eigen::Vector2f maxXY = Eigen::Vector2f::Zero();
    float yawRad = 0.0f;      ///< 最长立板的方向角，范围 [0, pi)
    std::vector<RibSegment, Eigen::aligned_allocator<RibSegment>> ribs;
    std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>> seams; ///< 含低置信度焊缝
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;                              ///< 属于该工件的点

    Workpiece();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SceneSeamResult {
    bool success = false;
    std::string message;
    Eigen::Vector4f groundPlane = Eigen::Vector4f::Zero();
    std::vector<Workpiece, Eigen::aligned_allocator<Workpiece>> workpieces;
    std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>> seams; ///< 置信度达标的全部焊缝
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;                              ///< 预处理后的整场点云

    SceneSeamResult();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 平面上 (x, y) 处的 z 值。
float PlaneZ(const Eigen::Vector4f& plane, float x, float y);

/**
 * @brief 从整场点云中提取每个组立工件的初始焊缝。
 *
 * 1. 去无效点、体素下采样；
 * 2. 建整场俯视高度图，估计地面平面；
 * 3. 高出地面的栅格做闭运算和连通域，得到单个工件；
 * 4. 每个工件内按高度直方图找底板平面；
 * 5. 高出底板的点投影到底板，Hough 提取立板落地轮廓中心线，吸附主方向并处理 T 形搭接；
 * 6. 立板两侧偏移半板厚得到平角焊缝，立板交点处得到立角焊缝，并给出置信度。
 *
 * 全流程确定性，同一输入得到同一输出。失败时 success 为 false 并在 message 中说明。
 * 输出与输入点云处于同一坐标系（见 SceneSeamParams::zAxisDown）。
 */
SceneSeamResult ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                  const SceneSeamParams& params = SceneSeamParams());

/// 把结果中的点云、平面和焊缝整体做 z -> -z 变换（相机坐标系与 Z 向上坐标系互换，用于显示或对接机器人坐标系）。
void FlipResultZ(SceneSeamResult& result);

#endif // SCENE_SEAM_EXTRACTOR_H
