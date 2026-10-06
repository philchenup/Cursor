#ifndef SCENE_SEAM_EXTRACTOR_H
#define SCENE_SEAM_EXTRACTOR_H

// Qt 在 Windows 上会把 main 宏替换成 qMain。PCL 头文件会间接包含 windows.h，
// 若在此过程中改掉该宏，链接器就找不到 Qt 的程序入口。先卸掉宏，包含完再恢复。

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

struct SceneSeamParams {
    // 坐标系
    bool zAxisDown = true;              ///< 输入 Z 轴指向地面（相机深度方向）；点云 Z 轴向上时设为 false

    // 预处理与分割
    float voxelSize = 2.0f;             ///< 体素下采样边长，<= 0 关闭
    float sceneResolution = 4.0f;       ///< 整场高度图栅格边长，用于分割工件
    float workpieceResolution = 2.0f;   ///< 单个工件高度图栅格边长，用于底板和顶面轮廓
    float groundHeight = std::numeric_limits<float>::quiet_NaN(); ///< 已标定地面高度（输入坐标系的 Z）；非有限值时自动估计
    float groundThreshold = 3.0f;       ///< 高出地面超过该值的栅格视为物体，应小于最薄底板厚度
    float minWorkpieceArea = 10000.0f;  ///< 工件最小投影面积 mm^2
    float minWorkpieceSize = 100.0f;    ///< 工件外接框短边最小长度

    // 顶面与焊缝
    float ribMinHeight = 30.0f;         ///< 高出底板超过该值才可能是立板顶面，应大于底板厚度、小于最矮立板
    float minSeamLength = 30.0f;        ///< 平角焊缝最短长度，短于该值的轮廓边（板厚端面）丢弃
};

/// 顶面轮廓上的一条直线段，投影在底板平面上。
struct RibSegment {
    int id = -1;
    Eigen::Vector2f start = Eigen::Vector2f::Zero(); ///< 场景 XY 坐标
    Eigen::Vector2f end = Eigen::Vector2f::Zero();
    float thickness = 0.0f;   ///< 轮廓定义下不再估计板厚，保留字段为 0
    float height = 0.0f;      ///< 该段相邻立板顶面高出底板的高度
    float confidence = 0.0f;  ///< 0~1

    Eigen::Vector2f direction() const;
    float length() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class SeamType {
    FlatFillet,     ///< 立板与底板之间的平角焊缝
    VerticalFillet  ///< 两块立板凹拐角处的立角焊缝
};

/// 初始焊缝，用于引导后续激光或接触寻位。
struct InitialSeam {
    int workpieceId = -1;
    SeamType type = SeamType::FlatFillet;
    Eigen::Vector3f start = Eigen::Vector3f::Zero(); ///< 场景坐标。立焊缝从底板指向顶部
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    Eigen::Vector3f approachSide = Eigen::Vector3f::Zero(); ///< 焊枪接近方向的单位向量，位于底板平面内，指向板外
    float ribHeight = 0.0f;   ///< 相邻立板高度，用于焊枪避让
    float confidence = 0.0f;  ///< 0~1
    int ribA = -1;            ///< 所属轮廓边
    int ribB = -1;            ///< 立焊缝的另一条轮廓边，平焊缝为 -1

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
    float yawRad = 0.0f;      ///< 最长轮廓边的方向角，范围 [0, pi)
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

/**
 * @brief 焊缝提取实现。算法都在类内，Qt 工程直接链接本 cpp 时调用这里。
 *
 * 本文件不提供 main。启动项目必须仍是已有的 Qt exe。
 */
class SceneSeamExtractor {
public:
    SceneSeamExtractor();
    explicit SceneSeamExtractor(SceneSeamParams params);

    void setParams(const SceneSeamParams& params);
    const SceneSeamParams& params() const;

    /// 平面上 (x, y) 处的 z 值。
    static float PlaneZ(const Eigen::Vector4f& plane, float x, float y);

    /// 用对象中保存的参数，从整场点云提取每个组立工件的初始焊缝。
    SceneSeamResult ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) const;

    /**
     * @brief 从整场点云中提取每个组立工件的初始焊缝。
     *
     * 1. 去无效点、体素下采样；
     * 2. 建整场俯视高度图，估计地面平面；
     * 3. 高出地面的栅格做闭运算和连通域，得到单个工件；只保留高出地面的点，外接框按这些点重算；
     * 4. 每个工件内按高度直方图找底板平面；
     * 5. 高出底板且为局部最高的栅格构成立板顶面，取其投影轮廓（外轮廓和孔洞）；
     * 6. 轮廓上的直线段投到底板，得到平角焊缝；凹拐角得到立角焊缝。
     *
     * 全流程确定性，同一输入得到同一输出。失败时 success 为 false 并在 message 中说明。
     * 输出与输入点云处于同一坐标系（见 SceneSeamParams::zAxisDown）。
     * 不修改对象中保存的参数。
     */
    static SceneSeamResult ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        const SceneSeamParams& params);

    /// 把结果中的点云、平面和焊缝整体做 z -> -z 变换（相机坐标系与 Z 向上坐标系互换，用于显示或对接机器人坐标系）。
    static void FlipResultZ(SceneSeamResult& result);

private:
    // 内部类型和步骤只在类内调用，不再放在匿名命名空间中。

    struct HeightGrid {
        int cols = 0;
        int rows = 0;
        float res = 1.0f;
        Eigen::Vector2f origin = Eigen::Vector2f::Zero();
        std::vector<float> z; ///< 每格最高点，无数据为 NaN

        std::size_t index(int c, int r) const
        {
            return static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c);
        }

        bool inside(int c, int r) const
        {
            return c >= 0 && r >= 0 && c < cols && r < rows;
        }

        bool cellOf(float x, float y, int& c, int& r) const
        {
            c = static_cast<int>(std::floor((x - origin.x()) / res));
            r = static_cast<int>(std::floor((y - origin.y()) / res));
            return inside(c, r);
        }

        Eigen::Vector2f center(int c, int r) const
        {
            return origin + Eigen::Vector2f((static_cast<float>(c) + 0.5f) * res, (static_cast<float>(r) + 0.5f) * res);
        }

        Eigen::Vector2f corner(int c, int r) const
        {
            return origin + Eigen::Vector2f(static_cast<float>(c) * res, static_cast<float>(r) * res);
        }
    };

    struct Component {
        int label = 0;
        int count = 0;
        int minC = 0;
        int minR = 0;
        int maxC = 0;
        int maxR = 0;
    };

    struct Rgb {
        std::uint8_t r = 0;
        std::uint8_t g = 0;
        std::uint8_t b = 0;
    };

    using Cloud = pcl::PointCloud<pcl::PointXYZ>;
    using CloudPtr = Cloud::Ptr;
    using Mask = std::vector<std::uint8_t>;

    static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    static constexpr float kPi = 3.14159265358979323846f;

    static constexpr float kGroundFitTolerance = 4.0f;
    static constexpr float kMaxGroundTiltDeg = 10.0f;
    static constexpr float kWorkpieceCloseRadius = 15.0f; ///< 底板掩码闭运算，补阴影缺口
    static constexpr float kBasePlaneTolerance = 4.0f;
    static constexpr float kTopNeighborhood = 4.0f;       ///< 判断飞点的邻域半径
    static constexpr float kTopTolerance = 4.0f;          ///< 同一顶面允许的高差
    static constexpr float kPlateStep = 20.0f;            ///< 超过该高差视为另一块板的台阶，不能把矮板削掉
    static constexpr float kTopSupportRadius = 4.0f;      ///< 孤立高点过滤半径
    static constexpr float kTopCloseRadius = 4.0f;        ///< 补顶面栅格缺口，把贴在一起的立板连成一块

    static constexpr float kRasterEpsilon = 2.0f;
    static constexpr float kSameDirectionJog = 28.0f;
    static constexpr float kCornerLookahead = 24.0f;
    static constexpr float kMinCornerTurnDeg = 30.0f;     ///< 行走方向右转超过该角度才是凹拐角
    static constexpr float kMinConfidence = 0.3f;

    static constexpr int kDirX[4] = { 1, 0, -1, 0 };
    static constexpr int kDirY[4] = { 0, 1, 0, -1 };

    static float Percentile(std::vector<float> values, float fraction);
    static bool BuildHeightGrid(const Cloud& cloud, const std::vector<int>* indices, float res, HeightGrid& grid);
    static std::vector<std::pair<int, int>> DiskOffsets(int radius);
    static Mask Dilate(const Mask& src, int cols, int rows, int radius);
    static Mask Erode(const Mask& src, int cols, int rows, int radius);
    static Mask Close(const Mask& src, int cols, int rows, int radius);
    static std::vector<Component> LabelComponents(const Mask& mask, int cols, int rows, std::vector<int>& labels);
    static int CellsOf(float length, float res);
    static Eigen::Vector4f HorizontalPlane(float z);
    static bool FitPlaneLeastSquares(const std::vector<Eigen::Vector3f>& points, Eigen::Vector4f& plane);
    static float PlaneTiltDeg(const Eigen::Vector4f& plane);
    static bool HistogramBaseLevel(const std::vector<float>& values, float bin, bool lowestSignificant, float& level);
    static bool FitDominantPlane(const HeightGrid& grid, const Mask* candidate, float tolerance, float maxTiltDeg,
        bool lowestSignificant, bool fallbackToPeak, Eigen::Vector4f& plane);
    static Eigen::Vector3f Lift(const Eigen::Vector4f& plane, const Eigen::Vector2f& xy, float above = 0.0f);
    static bool MaskAt(const Mask& mask, int cols, int rows, int c, int r);
    static float PointSegmentDistance(const Eigen::Vector2f& p, const Eigen::Vector2f& a, const Eigen::Vector2f& b);
    static void DouglasPeucker(const std::vector<Eigen::Vector2f>& pts, int i, int j, float epsilon, std::vector<char>& keep);
    static std::vector<Eigen::Vector2f> Chain(const std::vector<Eigen::Vector2f>& pts, int from, int to);
    static std::vector<Eigen::Vector2f> SimplifyChain(const std::vector<Eigen::Vector2f>& chain, float epsilon);
    static std::vector<Eigen::Vector2f> SimplifyLoop(std::vector<Eigen::Vector2f> pts, float epsilon);
    static std::vector<std::vector<Eigen::Vector2f>> TraceContours(const Mask& mask, const HeightGrid& grid);
    static float SampleEdgeHeight(const HeightGrid& grid, const Mask& crest, const std::vector<float>& above,
        const Eigen::Vector2f& anchor, const Eigen::Vector2f& toward, const Eigen::Vector2f& inward);
    static bool ProcessWorkpiece(const Cloud& scene, const std::vector<int>& indices,
        const Eigen::Vector4f& groundPlane, const SceneSeamParams& params, Workpiece& piece);
    static CloudPtr Preprocess(const Cloud::ConstPtr& cloud, const SceneSeamParams& params);
    static void NegateZ(Cloud& cloud);
    static Eigen::Vector4f NegateZ(const Eigen::Vector4f& plane);
    static Eigen::Vector3f NegateZ(const Eigen::Vector3f& v);
    static void NegateZ(InitialSeam& seam);
    static Rgb FromHue(float hue, float sat, float val);
    static Rgb ColorOfLabel(int label, bool kept);
    static SceneSeamResult ExtractImpl(const Cloud::ConstPtr& input, const SceneSeamParams& params);

    SceneSeamParams params_;
};

#endif // SCENE_SEAM_EXTRACTOR_H
