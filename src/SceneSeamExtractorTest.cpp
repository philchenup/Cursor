#include "SceneSeamExtractor.h"

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

// 合成 3 m x 18 m 场地，乱序摆放 19 个组立工件，检查 ExtractSceneSeams 提取的初始焊缝。
//
// 工件模板：
//   grid    长筋 + 三块贯穿横筋 + 四块弧形肘板
//   tee     长筋 + 一块 T 形搭接短筋
//   ladder  两根长筋 + 三块两端搭接的横筋
//   single  单筋 + 两块对称弧形肘板
//   hbeam   工字形：长筋 + 两端端板，端板两侧带 35–60 mm 宽阴影
//   comb    梳形：横筋 + 三块 T 形搭接竖筋，横筋外侧带 40 mm 宽飞点带
// 相机垂直向下拍摄，点云按相机坐标系生成（Z 轴指向地面，地面 Z 最大），
// 只采样底板上表面和立板顶边；立板根部一侧留阴影缺失，
// 并混入地面倾斜、噪声、杂点、小块杂物、NaN 和零点。
//
// 检查通过后打开窗口：
//   上半部分  整场点云按高度着色，工件外接框和编号，焊缝叠加显示
//   下半部分  单个工件细节，按 n / p 切换工件
//   红色 平角焊缝    橙色 立角焊缝    青色 立板中心线
//
// 在仓库根目录编译运行（Ubuntu，PCL 1.14，VTK 9.1）：
// vtk_libs=$(ldd /usr/lib/x86_64-linux-gnu/libpcl_visualization.so | awk '/vtk/ {so=$1; sub(/\.so.*/, "", so); sub(/^lib/, "-l", so); printf "%s ", so}')
// g++ -std=c++17 -O2 -Iinclude -I/usr/include/vtk-9.1 $(pkg-config --cflags pcl_visualization) src/SceneSeamExtractorTest.cpp src/SceneSeamExtractor.cpp -Wl,--no-as-needed $(pkg-config --libs pcl_visualization pcl_filters pcl_io) $vtk_libs -o scene_seam_test && ./scene_seam_test
//
// 不带窗口只跑检查：./scene_seam_test --no-viewer
// 把合成场景按相机坐标系（Z 指向地面）另存为 PLY：./scene_seam_test --no-viewer --save-ply scene.ply
//
// 直接处理拼接后的整场 PLY（跳过合成场景和真值检查）：
//   ./scene_seam_test --ply scene.ply                       相机坐标系，Z 指向地面，单位 mm
//   ./scene_seam_test --ply scene.ply --z-up                点云 Z 轴向上
//   ./scene_seam_test --ply scene.ply --scale 1000          点云单位 m，内部换算成 mm
//   ./scene_seam_test --ply scene.ply --export seams.csv    焊缝写入 CSV（输入坐标系）
// 可按相机分辨率和工件尺寸覆盖参数：
//   --voxel 4  --rib-min-height 20  --rib-min-thickness 4  --rib-max-thickness 30  --min-area 50000

namespace {

constexpr float kPi = 3.14159265358979323846f;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// ---------------------------------------------------------------------------
// 工件模板与真值
// ---------------------------------------------------------------------------

struct RibSpec {
    Eigen::Vector2f a;
    Eigen::Vector2f b;
    float thickness;
    float height;
    bool curved;                ///< 肘板：高度从 a 端的 height 线性降到 b 端的 0.3 height
    float shadowLeft = -1.0f;   ///< 法向正侧阴影宽度 mm，< 0 时随机 0–12
    float shadowRight = 0.0f;   ///< 法向负侧阴影宽度 mm
    float faceBand = 0.0f;      ///< 法向负侧飞点带宽度 mm：相机离轴时沿视线从顶边拖到底板的混合像素
};

struct Template {
    std::string name;
    float baseLength;
    float baseWidth;
    float baseThickness;
    std::vector<RibSpec> ribs;
};

std::vector<Template> MakeTemplates()
{
    std::vector<Template> templates;

    Template grid{"grid", 1300.0f, 800.0f, 12.0f, {}};
    grid.ribs.push_back({{-600.0f, 0.0f}, {600.0f, 0.0f}, 10.0f, 180.0f, false});
    for (float x : {-350.0f, 0.0f, 350.0f}) {
        grid.ribs.push_back({{x, -330.0f}, {x, 330.0f}, 10.0f, 140.0f, false});
    }
    for (float x : {-560.0f, 560.0f}) {
        grid.ribs.push_back({{x, 5.0f}, {x, 330.0f}, 8.0f, 120.0f, true});
        grid.ribs.push_back({{x, -5.0f}, {x, -330.0f}, 8.0f, 120.0f, true});
    }
    templates.push_back(grid);

    Template tee{"tee", 1100.0f, 600.0f, 10.0f, {}};
    tee.ribs.push_back({{-500.0f, 0.0f}, {500.0f, 0.0f}, 10.0f, 150.0f, false});
    tee.ribs.push_back({{150.0f, 5.0f}, {150.0f, 260.0f}, 8.0f, 120.0f, false});
    templates.push_back(tee);

    Template ladder{"ladder", 1300.0f, 800.0f, 12.0f, {}};
    for (float y : {-250.0f, 250.0f}) {
        ladder.ribs.push_back({{-600.0f, y}, {600.0f, y}, 10.0f, 160.0f, false});
    }
    for (float x : {-400.0f, 0.0f, 400.0f}) {
        ladder.ribs.push_back({{x, -245.0f}, {x, 245.0f}, 8.0f, 120.0f, false});
    }
    templates.push_back(ladder);

    Template single{"single", 900.0f, 500.0f, 10.0f, {}};
    single.ribs.push_back({{-400.0f, 0.0f}, {400.0f, 0.0f}, 12.0f, 160.0f, false});
    single.ribs.push_back({{0.0f, 6.0f}, {0.0f, 220.0f}, 8.0f, 100.0f, true});
    single.ribs.push_back({{0.0f, -6.0f}, {0.0f, -220.0f}, 8.0f, 100.0f, true});
    templates.push_back(single);

    // 工字形：长筋两端各一块端板，端板两侧阴影远宽于底板掩膜补洞半径，
    // 只有端板两端角落能看到底板，用来检查平焊缝不被阴影切碎
    Template hbeam{"hbeam", 700.0f, 320.0f, 10.0f, {}};
    hbeam.ribs.push_back({{-200.0f, 0.0f}, {200.0f, 0.0f}, 8.0f, 180.0f, false, 30.0f, 20.0f});
    hbeam.ribs.push_back({{-200.0f, -120.0f}, {-200.0f, 120.0f}, 8.0f, 180.0f, false, 60.0f, 35.0f});
    hbeam.ribs.push_back({{200.0f, -120.0f}, {200.0f, 120.0f}, 8.0f, 180.0f, false, 35.0f, 60.0f});
    templates.push_back(hbeam);

    // 梳形：底部一根横筋，三块竖筋以 T 形搭在横筋上；横筋外侧带 40 mm 宽飞点带，
    // 竖筋的 Hough 带会把这些飞点收进去而“越过”横筋，用来检查 T 形不被误判成十字、横筋外侧焊缝保持整条
    Template comb{"comb", 900.0f, 500.0f, 10.0f, {}};
    comb.ribs.push_back({{-400.0f, -180.0f}, {400.0f, -180.0f}, 10.0f, 150.0f, false, -1.0f, 0.0f, 40.0f});
    for (float x : {-250.0f, 0.0f, 250.0f}) {
        comb.ribs.push_back({{x, -175.0f}, {x, 180.0f}, 8.0f, 120.0f, false});
    }
    templates.push_back(comb);

    return templates;
}

struct Pose {
    Eigen::Vector2f position;
    float yaw;

    Eigen::Vector2f apply(const Eigen::Vector2f& local) const
    {
        const Eigen::Rotation2Df rotation(yaw);
        return rotation * local + position;
    }
};

struct GroundTruthSeam {
    SeamType type;
    Eigen::Vector3f start;
    Eigen::Vector3f end;
};

struct PlacedWorkpiece {
    const Template* shape;
    Pose pose;
    std::vector<GroundTruthSeam> seams;
};

// 场景按相机坐标系生成：相机位于原点上方俯视，Z 轴指向地面，地面 Z 最大，物体越高 Z 越小。
// 下面的几何量先按“高出水平基准面”的世界高度描述，再由 CameraZ 换算到相机坐标系。
constexpr float kCameraHeight = 1500.0f; ///< 相机到地面基准的距离

float GroundHeight(float x, float y)
{
    return 0.002f * x - 0.001f * y; ///< 地面略有倾斜（世界高度）
}

float CameraZ(float worldHeight)
{
    return kCameraHeight - worldHeight;
}

float GroundZ(float x, float y)
{
    return CameraZ(GroundHeight(x, y)); ///< 地面在相机坐标系中的 Z
}

// 按与算法无关的几何规则生成真值焊缝：
// 平焊缝 = 立板中心线两侧偏移半板厚，在相交立板处留出间隙；
// 立焊缝 = 两块立板交点处每个有板的象限一条，高度取两板较矮者。
std::vector<GroundTruthSeam> TemplateSeams(const Template& shape, const Pose& pose)
{
    constexpr float kClearance = 5.0f;
    constexpr float kMinArm = 15.0f;
    std::vector<GroundTruthSeam> seams;

    auto lift = [&](const Eigen::Vector2f& local, float above) {
        const Eigen::Vector2f world = pose.apply(local);
        return Eigen::Vector3f(world.x(), world.y(), CameraZ(GroundHeight(world.x(), world.y()) + shape.baseThickness + above));
    };

    struct Hit {
        float tSelf;
        float tOther;
        const RibSpec* other;
    };

    auto hits = [&](const RibSpec& rib) {
        std::vector<Hit> result;
        const Eigen::Vector2f d = (rib.b - rib.a).normalized();
        const float length = (rib.b - rib.a).norm();
        for (const RibSpec& other : shape.ribs) {
            if (&other == &rib) {
                continue;
            }
            const Eigen::Vector2f e = (other.b - other.a).normalized();
            const float cross = d.x() * e.y() - d.y() * e.x();
            if (std::fabs(cross) < 0.2f) {
                continue;
            }
            const Eigen::Vector2f delta = other.a - rib.a;
            const float t = (delta.x() * e.y() - delta.y() * e.x()) / cross;
            const float u = (delta.x() * d.y() - delta.y() * d.x()) / cross;
            const float otherLength = (other.b - other.a).norm();
            if (t < -other.thickness * 0.5f - 2.0f || t > length + other.thickness * 0.5f + 2.0f) {
                continue;
            }
            if (u < -rib.thickness * 0.5f - 2.0f || u > otherLength + rib.thickness * 0.5f + 2.0f) {
                continue;
            }
            result.push_back({t, u, &other});
        }
        std::sort(result.begin(), result.end(), [](const Hit& l, const Hit& r) { return l.tSelf < r.tSelf; });
        return result;
    };

    for (const RibSpec& rib : shape.ribs) {
        const Eigen::Vector2f d = (rib.b - rib.a).normalized();
        const Eigen::Vector2f n(-d.y(), d.x());
        const float length = (rib.b - rib.a).norm();
        const std::vector<Hit> ribHits = hits(rib);

        for (float side : {1.0f, -1.0f}) {
            const Eigen::Vector2f offset = n * side * rib.thickness * 0.5f;
            float cursor = 0.0f;
            auto emit = [&](float t0, float t1) {
                if (t1 - t0 < 30.0f) {
                    return;
                }
                seams.push_back({SeamType::FlatFillet, lift(rib.a + d * t0 + offset, 0.0f), lift(rib.a + d * t1 + offset, 0.0f)});
            };
            for (const Hit& hit : ribHits) {
                // 只有对方立板真的延伸到这一侧时才断开
                const Eigen::Vector2f e = (hit.other->b - hit.other->a).normalized();
                const float otherLength = (hit.other->b - hit.other->a).norm();
                const float arm = rib.thickness * 0.5f + kMinArm;
                const bool forwardIsThisSide = (n.dot(e) >= 0.0f) == (side > 0.0f);
                const bool occupies = forwardIsThisSide ? otherLength >= hit.tOther + arm : 0.0f <= hit.tOther - arm;
                if (!occupies) {
                    continue;
                }
                const float half = hit.other->thickness * 0.5f + kClearance;
                if (hit.tSelf - half > cursor) {
                    emit(cursor, std::min(hit.tSelf - half, length));
                }
                cursor = std::max(cursor, hit.tSelf + half);
            }
            if (cursor < length) {
                emit(cursor, length);
            }
        }

        for (const Hit& hit : ribHits) {
            if (hit.other < &rib) {
                continue;
            }
            const RibSpec& other = *hit.other;
            const Eigen::Vector2f e = (other.b - other.a).normalized();
            const float otherLength = (other.b - other.a).norm();
            const Eigen::Vector2f cross = rib.a + d * hit.tSelf;
            const float height = std::min(rib.height, other.height);
            for (float sa : {1.0f, -1.0f}) {
                const bool armA = sa > 0.0f ? length >= hit.tSelf + other.thickness * 0.5f + kMinArm
                                            : 0.0f <= hit.tSelf - other.thickness * 0.5f - kMinArm;
                if (!armA) {
                    continue;
                }
                for (float sb : {1.0f, -1.0f}) {
                    const bool armB = sb > 0.0f ? otherLength >= hit.tOther + rib.thickness * 0.5f + kMinArm
                                                : 0.0f <= hit.tOther - rib.thickness * 0.5f - kMinArm;
                    if (!armB) {
                        continue;
                    }
                    const Eigen::Vector2f corner = cross + d * (sa * other.thickness * 0.5f) + e * (sb * rib.thickness * 0.5f);
                    seams.push_back({SeamType::VerticalFillet, lift(corner, 0.0f), lift(corner, height)});
                }
            }
        }
    }
    return seams;
}

// ---------------------------------------------------------------------------
// 场景采样
// ---------------------------------------------------------------------------

struct Scene {
    Cloud::Ptr cloud;
    std::vector<PlacedWorkpiece> workpieces;
};

/// 按世界高度加点，写入相机坐标系
void AddPoint(Cloud& cloud, float x, float y, float worldHeight)
{
    cloud.push_back(pcl::PointXYZ(x, y, CameraZ(worldHeight)));
}

/// 直接写入原始坐标（零点、NaN 等无效点）
void AddRawPoint(Cloud& cloud, float x, float y, float z)
{
    cloud.push_back(pcl::PointXYZ(x, y, z));
}

void SampleWorkpiece(const Template& shape, const Pose& pose, std::mt19937& rng, Cloud& cloud)
{
    std::normal_distribution<float> noise(0.0f, 0.5f);
    std::uniform_real_distribution<float> shadowWidth(0.0f, 12.0f);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    std::vector<float> shadows;
    for (const RibSpec& rib : shape.ribs) {
        shadows.push_back(rib.shadowLeft >= 0.0f ? rib.shadowLeft : shadowWidth(rng));
    }

    auto emit = [&](const Eigen::Vector2f& local, float above) {
        const Eigen::Vector2f world = pose.apply(local);
        AddPoint(cloud, world.x(), world.y(), GroundHeight(world.x(), world.y()) + shape.baseThickness + above + noise(rng));
    };

    // 底板上表面：避开立板占位和阴影
    constexpr float kStep = 4.0f;
    for (float x = -shape.baseLength * 0.5f; x <= shape.baseLength * 0.5f; x += kStep) {
        for (float y = -shape.baseWidth * 0.5f; y <= shape.baseWidth * 0.5f; y += kStep) {
            const Eigen::Vector2f p(x, y);
            bool blocked = false;
            for (std::size_t i = 0; i < shape.ribs.size() && !blocked; ++i) {
                const RibSpec& rib = shape.ribs[i];
                const Eigen::Vector2f d = (rib.b - rib.a).normalized();
                const Eigen::Vector2f n(-d.y(), d.x());
                const float t = d.dot(p - rib.a);
                const float lateral = n.dot(p - rib.a);
                if (t < 0.0f || t > (rib.b - rib.a).norm()) {
                    continue;
                }
                if (std::fabs(lateral) <= rib.thickness * 0.5f) {
                    blocked = true;
                } else if (lateral > 0.0f && lateral <= rib.thickness * 0.5f + shadows[i]) {
                    blocked = true;
                } else if (lateral < 0.0f && -lateral <= rib.thickness * 0.5f + rib.shadowRight) {
                    blocked = true;
                }
            }
            if (!blocked) {
                emit(p, 0.0f);
            }
        }
    }

    // 立板顶边：相机只看到厚度方向的一条窄带
    for (const RibSpec& rib : shape.ribs) {
        const Eigen::Vector2f d = (rib.b - rib.a).normalized();
        const Eigen::Vector2f n(-d.y(), d.x());
        const float length = (rib.b - rib.a).norm();
        for (float t = 0.0f; t <= length; t += kStep) {
            const float ratio = t / length;
            const float height = rib.curved ? rib.height * (1.0f - 0.7f * ratio) : rib.height;
            for (float lateral = -rib.thickness * 0.5f + 1.0f; lateral <= rib.thickness * 0.5f; lateral += 3.0f) {
                emit(rib.a + d * t + n * lateral, height);
            }
            // 内角多次反射产生的悬空假点
            if (unit(rng) < 0.03f) {
                emit(rib.a + d * t + n * (rib.thickness * 0.5f + 3.0f), unit(rng) * 40.0f);
            }
            // 飞点带：高度从顶边线性降到底板，横向离板越远越低
            for (float w = 2.0f; w <= rib.faceBand; w += 2.0f) {
                emit(rib.a + d * t - n * (rib.thickness * 0.5f + w), height * (1.0f - w / rib.faceBand));
            }
        }
    }
}

Scene BuildScene()
{
    Scene scene;
    scene.cloud.reset(new Cloud);
    std::mt19937 rng(20261003u);
    std::normal_distribution<float> noise(0.0f, 0.5f);
    std::uniform_real_distribution<float> jitter(-100.0f, 100.0f);
    std::uniform_real_distribution<float> yaw(0.0f, 2.0f * kPi);

    static const std::vector<Template> templates = MakeTemplates();

    // 地面：3.2 m x 18 m，略有倾斜
    constexpr float kGroundStep = 8.0f;
    for (float x = 0.0f; x <= 18000.0f; x += kGroundStep) {
        for (float y = 0.0f; y <= 3200.0f; y += kGroundStep) {
            AddPoint(*scene.cloud, x, y, GroundHeight(x, y) + noise(rng));
        }
    }

    // 20 个槽位放 19 个工件
    std::vector<int> slots(20);
    for (int i = 0; i < 20; ++i) {
        slots[static_cast<std::size_t>(i)] = i;
    }
    std::shuffle(slots.begin(), slots.end(), rng);
    slots.resize(19);
    std::sort(slots.begin(), slots.end());

    for (std::size_t i = 0; i < slots.size(); ++i) {
        const int slot = slots[i];
        const int column = slot % 10;
        const int row = slot / 10;
        Pose pose;
        pose.position = Eigen::Vector2f(900.0f + 1800.0f * static_cast<float>(column) + jitter(rng),
                                        800.0f + 1600.0f * static_cast<float>(row) + jitter(rng));
        pose.yaw = yaw(rng);
        const Template& shape = templates[i % templates.size()];
        SampleWorkpiece(shape, pose, rng, *scene.cloud);
        scene.workpieces.push_back({&shape, pose, TemplateSeams(shape, pose)});
    }

    // 杂物：小块方料，面积低于工件下限
    for (int i = 0; i < 4; ++i) {
        const float cx = 1800.0f * static_cast<float>(2 * i + 1) + 400.0f;
        const float cy = 1600.0f;
        for (float x = -60.0f; x <= 60.0f; x += 4.0f) {
            for (float y = -60.0f; y <= 60.0f; y += 4.0f) {
                AddPoint(*scene.cloud, cx + x, cy + y, GroundHeight(cx + x, cy + y) + 60.0f + noise(rng));
            }
        }
    }

    // 离群点、零点、NaN
    std::uniform_real_distribution<float> ux(0.0f, 18000.0f);
    std::uniform_real_distribution<float> uy(0.0f, 3200.0f);
    std::uniform_real_distribution<float> uz(50.0f, 600.0f);
    for (int i = 0; i < 3000; ++i) {
        AddPoint(*scene.cloud, ux(rng), uy(rng), uz(rng));
    }
    for (int i = 0; i < 500; ++i) {
        AddRawPoint(*scene.cloud, 0.0f, 0.0f, 0.0f);
        AddRawPoint(*scene.cloud, std::numeric_limits<float>::quiet_NaN(), 1.0f, 1.0f);
    }

    scene.cloud->width = static_cast<std::uint32_t>(scene.cloud->size());
    scene.cloud->height = 1;
    scene.cloud->is_dense = false;
    return scene;
}

// ---------------------------------------------------------------------------
// 比对
// ---------------------------------------------------------------------------

bool Expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "失败: " << message << '\n';
    }
    return condition;
}

bool SameSeam(const InitialSeam& detected, const GroundTruthSeam& truth, float tolerance)
{
    if (detected.type != truth.type) {
        return false;
    }
    const bool direct = (detected.start - truth.start).norm() <= tolerance && (detected.end - truth.end).norm() <= tolerance;
    const bool swapped = (detected.start - truth.end).norm() <= tolerance && (detected.end - truth.start).norm() <= tolerance;
    return direct || swapped;
}

struct MatchReport {
    int truthTotal = 0;
    int truthMatched = 0;
    int detectedTotal = 0;
    int detectedMatched = 0;
    int truthFlat = 0;
    int truthVertical = 0;
    int detectedFlat = 0;
    int detectedVertical = 0;
};

// 真值与检出焊缝之间的端点误差，取两种端点配对中较小者。
float SeamError(const InitialSeam& detected, const GroundTruthSeam& truth)
{
    const float direct = std::max((detected.start - truth.start).norm(), (detected.end - truth.end).norm());
    const float swapped = std::max((detected.start - truth.end).norm(), (detected.end - truth.start).norm());
    return std::min(direct, swapped);
}

MatchReport CompareSeams(const Scene& scene, const SceneSeamResult& result, float tolerance)
{
    MatchReport report;
    std::vector<std::uint8_t> detectedHit(result.seams.size(), 0);
    int reported = 0;
    for (std::size_t pieceIndex = 0; pieceIndex < scene.workpieces.size(); ++pieceIndex) {
        const PlacedWorkpiece& piece = scene.workpieces[pieceIndex];
        for (const GroundTruthSeam& truth : piece.seams) {
            ++report.truthTotal;
            (truth.type == SeamType::FlatFillet ? report.truthFlat : report.truthVertical) += 1;
            bool matched = false;
            float nearest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < result.seams.size(); ++i) {
                if (result.seams[i].type == truth.type) {
                    nearest = std::min(nearest, SeamError(result.seams[i], truth));
                }
                if (SameSeam(result.seams[i], truth, tolerance)) {
                    detectedHit[i] = 1;
                    matched = true;
                }
            }
            report.truthMatched += matched ? 1 : 0;
            if (!matched && reported < 12) {
                ++reported;
                std::cout << "未匹配 " << piece.shape->name << " #" << pieceIndex
                          << (truth.type == SeamType::FlatFillet ? " 平焊缝 " : " 立焊缝 ")
                          << "长度 " << (truth.end - truth.start).norm() << " mm，最近检出误差 " << nearest << " mm\n";
                if (std::getenv("SEAM_DEBUG")) {
                    const InitialSeam* best = nullptr;
                    for (const InitialSeam& seam : result.seams) {
                        if (seam.type == truth.type && (!best || SeamError(seam, truth) < SeamError(*best, truth))) {
                            best = &seam;
                        }
                    }
                    std::cout << "  真值 " << truth.start.transpose() << " -> " << truth.end.transpose() << '\n';
                    if (best) {
                        std::cout << "  检出 " << best->start.transpose() << " -> " << best->end.transpose()
                                  << "  长度 " << best->length() << '\n';
                    }
                }
            }
        }
    }
    report.detectedTotal = static_cast<int>(result.seams.size());
    int extraReported = 0;
    for (std::size_t i = 0; i < result.seams.size(); ++i) {
        (result.seams[i].type == SeamType::FlatFillet ? report.detectedFlat : report.detectedVertical) += 1;
        report.detectedMatched += detectedHit[i] ? 1 : 0;
        if (!detectedHit[i] && std::getenv("SEAM_DEBUG") && extraReported < 24) {
            ++extraReported;
            const InitialSeam& seam = result.seams[i];
            std::cout << "多余 " << (seam.type == SeamType::FlatFillet ? "平" : "立") << " 工件 " << seam.workpieceId
                      << " 长 " << seam.length() << " 置信 " << seam.confidence << "  " << seam.start.head<2>().transpose()
                      << " -> " << seam.end.head<2>().transpose() << '\n';
        }
    }
    return report;
}

// ---------------------------------------------------------------------------
// 可视化
// ---------------------------------------------------------------------------

// 按高出地面的高度着色：地面深蓝灰，底板青绿，立板顶边橙黄。
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorByHeight(const Cloud& cloud, const Eigen::Vector4f& groundPlane)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored->reserve(cloud.size());
    constexpr float kMaxHeight = 220.0f;
    for (const pcl::PointXYZ& p : cloud.points) {
        const float above = p.z - PlaneZ(groundPlane, p.x, p.y);
        const float u = std::clamp(above / kMaxHeight, 0.0f, 1.0f);
        pcl::PointXYZRGB q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        if (u < 0.5f) {
            const float v = u * 2.0f;
            q.r = static_cast<std::uint8_t>(55.0f + (40.0f - 55.0f) * v);
            q.g = static_cast<std::uint8_t>(70.0f + (190.0f - 70.0f) * v);
            q.b = static_cast<std::uint8_t>(100.0f + (170.0f - 100.0f) * v);
        } else {
            const float v = (u - 0.5f) * 2.0f;
            q.r = static_cast<std::uint8_t>(40.0f + (255.0f - 40.0f) * v);
            q.g = static_cast<std::uint8_t>(190.0f + (200.0f - 190.0f) * v);
            q.b = static_cast<std::uint8_t>(170.0f + (60.0f - 170.0f) * v);
        }
        colored->push_back(q);
    }
    colored->width = static_cast<std::uint32_t>(colored->size());
    colored->height = 1;
    return colored;
}

pcl::PointXYZ ToPoint(const Eigen::Vector3f& p)
{
    return pcl::PointXYZ(p.x(), p.y(), p.z());
}

void AddSeamShapes(pcl::visualization::PCLVisualizer& viewer,
                   const std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>>& seams,
                   const std::string& prefix,
                   double width,
                   int viewport)
{
    for (std::size_t i = 0; i < seams.size(); ++i) {
        const InitialSeam& seam = seams[i];
        const std::string id = prefix + std::to_string(i);
        if (seam.type == SeamType::FlatFillet) {
            viewer.addLine(ToPoint(seam.start), ToPoint(seam.end), 1.0, 0.25, 0.2, id, viewport);
        } else {
            viewer.addLine(ToPoint(seam.start), ToPoint(seam.end), 1.0, 0.65, 0.1, id, viewport);
        }
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, width, id, viewport);
    }
}

struct DetailView {
    pcl::visualization::PCLVisualizer* viewer = nullptr;
    const SceneSeamResult* result = nullptr;
    int viewport = 0;
    int current = 0;

    void show()
    {
        viewer->removeAllPointClouds(viewport);
        viewer->removeAllShapes(viewport);
        if (result->workpieces.empty()) {
            return;
        }
        const Workpiece& piece = result->workpieces[static_cast<std::size_t>(current)];

        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored = ColorByHeight(*piece.cloud, result->groundPlane);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(colored);
        viewer->addPointCloud(colored, color, "detail-cloud", viewport);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "detail-cloud", viewport);

        for (std::size_t i = 0; i < piece.ribs.size(); ++i) {
            const RibSegment& rib = piece.ribs[i];
            const std::string id = "detail-rib" + std::to_string(i);
            const float lift = 2.0f;
            viewer->addLine(ToPoint(Eigen::Vector3f(rib.start.x(), rib.start.y(), PlaneZ(piece.basePlane, rib.start.x(), rib.start.y()) + lift)),
                            ToPoint(Eigen::Vector3f(rib.end.x(), rib.end.y(), PlaneZ(piece.basePlane, rib.end.x(), rib.end.y()) + lift)),
                            0.2, 0.9, 1.0, id, viewport);
        }
        AddSeamShapes(*viewer, piece.seams, "detail-seam", 4.0, viewport);

        int flat = 0;
        int vertical = 0;
        for (const InitialSeam& seam : piece.seams) {
            (seam.type == SeamType::FlatFillet ? flat : vertical) += 1;
        }
        const std::string text = "workpiece " + std::to_string(piece.id) + " / " + std::to_string(result->workpieces.size() - 1)
            + "   ribs " + std::to_string(piece.ribs.size()) + "   flat " + std::to_string(flat) + "   vertical "
            + std::to_string(vertical) + "   base " + std::to_string(static_cast<int>(std::round(piece.baseHeight)))
            + " mm   [n] next  [p] previous";
        viewer->addText(text, 16, 14, 16, 0.9, 0.9, 0.9, "detail-text", viewport);

        const Eigen::Vector3f center(piece.center.x(), piece.center.y(), PlaneZ(piece.basePlane, piece.center.x(), piece.center.y()));
        const float radius = std::max(400.0f, 0.5f * (piece.maxXY - piece.minXY).norm());
        // 视线斜向穿过最长立板，避免和立板平行
        const Eigen::Vector2f lateral = Eigen::Rotation2Df(piece.yawRad) * Eigen::Vector2f(-1.3f, -1.7f);
        const Eigen::Vector3f eye = center + Eigen::Vector3f(lateral.x(), lateral.y(), 1.2f) * radius;
        viewer->setCameraPosition(eye.x(), eye.y(), eye.z(), center.x(), center.y(), center.z(), 0.0, 0.0, 1.0, viewport);
        viewer->setCameraFieldOfView(0.5, viewport);
        viewer->setCameraClipDistances(10.0, 30000.0, viewport);
    }
};

vtkRenderer* RendererAt(pcl::visualization::PCLVisualizer& viewer, int viewport)
{
    vtkRendererCollection* renderers = viewer.getRenderWindow()->GetRenderers();
    renderers->InitTraversal();
    int index = 0;
    while (vtkRenderer* renderer = renderers->GetNextItem()) {
        if (index == viewport) {
            return renderer;
        }
        ++index;
    }
    return nullptr;
}

void ShowScene(const SceneSeamResult& result)
{
    pcl::visualization::PCLVisualizer viewer("Scene seams");
    viewer.setSize(1600, 900);
    viewer.setBackgroundColor(0.07, 0.08, 0.10);

    int overview = 0;
    int detail = 0;
    viewer.createViewPort(0.0, 0.5, 1.0, 1.0, overview);
    viewer.createViewPort(0.0, 0.0, 1.0, 0.5, detail);
    viewer.createViewPortCamera(overview);
    viewer.createViewPortCamera(detail);

    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored = ColorByHeight(*result.cloud, result.groundPlane);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored);
    viewer.addPointCloud(colored, rgb, "scene", overview);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene", overview);

    for (const Workpiece& piece : result.workpieces) {
        const float z0 = PlaneZ(result.groundPlane, piece.center.x(), piece.center.y());
        const std::string id = "box" + std::to_string(piece.id);
        viewer.addCube(piece.minXY.x(), piece.maxXY.x(), piece.minXY.y(), piece.maxXY.y(), z0, z0 + 220.0f, 0.3, 0.8, 0.4, id, overview);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                           pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id, overview);
        viewer.addText3D(std::to_string(piece.id), pcl::PointXYZ(piece.minXY.x(), piece.maxXY.y() + 40.0f, z0 + 220.0f), 120.0,
                         0.3, 0.8, 0.4, "label" + std::to_string(piece.id), overview);
    }
    AddSeamShapes(viewer, result.seams, "seam", 2.0, overview);

    viewer.addText("scene: height colored cloud, green boxes = workpieces, red = flat fillet, orange = vertical fillet",
                   16, 14, 16, 0.9, 0.9, 0.9, "overview-text", overview);

    Eigen::Vector2f minXY(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f maxXY = -minXY;
    for (const pcl::PointXYZ& p : result.cloud->points) {
        minXY = minXY.cwiseMin(Eigen::Vector2f(p.x, p.y));
        maxXY = maxXY.cwiseMax(Eigen::Vector2f(p.x, p.y));
    }
    const Eigen::Vector2f center = 0.5f * (minXY + maxXY);
    const Eigen::Vector2f span = maxXY - minXY;
    viewer.setCameraPosition(center.x(), center.y(), 20000.0, center.x(), center.y(), 0.0, 0.0, 1.0, 0.0, overview);
    viewer.setCameraClipDistances(100.0, 60000.0, overview);
    if (vtkRenderer* renderer = RendererAt(viewer, overview)) {
        constexpr float kAspect = 1600.0f / 450.0f;
        renderer->GetActiveCamera()->SetParallelProjection(1);
        renderer->GetActiveCamera()->SetParallelScale(std::max(span.y() * 0.5f, span.x() * 0.5f / kAspect) * 1.08f);
    }

    DetailView detailView;
    detailView.viewer = &viewer;
    detailView.result = &result;
    detailView.viewport = detail;
    if (const char* initial = std::getenv("SEAM_DETAIL")) {
        const int index = std::atoi(initial);
        if (index >= 0 && index < static_cast<int>(result.workpieces.size())) {
            detailView.current = index;
        }
    }
    detailView.show();

    viewer.registerKeyboardCallback([&detailView](const pcl::visualization::KeyboardEvent& event) {
        if (!event.keyDown() || detailView.result->workpieces.empty()) {
            return;
        }
        const int count = static_cast<int>(detailView.result->workpieces.size());
        if (event.getKeySym() == "n") {
            detailView.current = (detailView.current + 1) % count;
            detailView.show();
        } else if (event.getKeySym() == "p") {
            detailView.current = (detailView.current + count - 1) % count;
            detailView.show();
        }
    });

    if (const char* snapshot = std::getenv("SEAM_SNAPSHOT")) {
        for (int i = 0; i < 5; ++i) {
            viewer.spinOnce(100, true);
        }
        viewer.saveScreenshot(snapshot);
        std::cout << "截图已保存到 " << snapshot << '\n';
        return;
    }

    std::cout << "按 n / p 切换工件，关闭窗口后退出\n";
    viewer.spin();
}

// ---------------------------------------------------------------------------
// 处理拼接后的整场 PLY
// ---------------------------------------------------------------------------

const char* SeamTypeName(SeamType type)
{
    return type == SeamType::FlatFillet ? "flat" : "vertical";
}

void PrintResult(const SceneSeamResult& result)
{
    const Eigen::Vector3f normal = result.groundPlane.head<3>().normalized();
    std::cout << result.message << "\n地面法向 " << normal.transpose() << "  工件 " << result.workpieces.size() << " 个，焊缝 "
              << result.seams.size() << " 条\n";
    for (const Workpiece& piece : result.workpieces) {
        int flat = 0;
        int vertical = 0;
        for (const InitialSeam& seam : piece.seams) {
            (seam.type == SeamType::FlatFillet ? flat : vertical) += 1;
        }
        std::cout << "工件 " << piece.id << "  中心 (" << piece.center.x() << ", " << piece.center.y() << ")  尺寸 "
                  << (piece.maxXY - piece.minXY).x() << " x " << (piece.maxXY - piece.minXY).y() << "  底板高 " << piece.baseHeight
                  << "  立板 " << piece.ribs.size() << "  平角焊缝 " << flat << "  立角焊缝 " << vertical << '\n';
    }
}

bool ExportSeamsCsv(const SceneSeamResult& result, const std::string& path)
{
    std::ofstream out(path);
    if (!out) {
        std::cerr << "无法写入 " << path << '\n';
        return false;
    }
    out << "workpiece,type,start_x,start_y,start_z,end_x,end_y,end_z,approach_x,approach_y,approach_z,rib_height,confidence\n";
    for (const InitialSeam& seam : result.seams) {
        out << seam.workpieceId << ',' << SeamTypeName(seam.type) << ',' << seam.start.x() << ',' << seam.start.y() << ','
            << seam.start.z() << ',' << seam.end.x() << ',' << seam.end.y() << ',' << seam.end.z() << ',' << seam.approachSide.x()
            << ',' << seam.approachSide.y() << ',' << seam.approachSide.z() << ',' << seam.ribHeight << ',' << seam.confidence << '\n';
    }
    std::cout << "焊缝已写入 " << path << '\n';
    return true;
}

struct Options {
    bool showViewer = true;
    std::string plyPath;      ///< 为空时运行合成场景检查
    float scale = 1.0f;       ///< 读入坐标乘以该系数，PLY 为 m 时填 1000
    std::string exportPath;   ///< 焊缝 CSV 输出路径
    std::string savePlyPath;  ///< 合成场景另存为 PLY
    SceneSeamParams params;
};

bool ParseOptions(int argc, char** argv, Options& options)
{
    auto value = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            std::cerr << argv[i] << " 缺少参数\n";
            return nullptr;
        }
        return argv[++i];
    };
    auto number = [&](int& i, float& target) {
        const char* text = value(i);
        if (!text) {
            return false;
        }
        target = std::strtof(text, nullptr);
        return true;
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        bool ok = true;
        if (arg == "--no-viewer") {
            options.showViewer = false;
        } else if (arg == "--z-up") {
            options.params.zAxisDown = false;
        } else if (arg == "--ply") {
            const char* text = value(i);
            ok = text != nullptr;
            if (ok) options.plyPath = text;
        } else if (arg == "--export") {
            const char* text = value(i);
            ok = text != nullptr;
            if (ok) options.exportPath = text;
        } else if (arg == "--save-ply") {
            const char* text = value(i);
            ok = text != nullptr;
            if (ok) options.savePlyPath = text;
        } else if (arg == "--scale") {
            ok = number(i, options.scale);
        } else if (arg == "--voxel") {
            ok = number(i, options.params.voxelSize);
        } else if (arg == "--rib-min-height") {
            ok = number(i, options.params.ribMinHeight);
        } else if (arg == "--rib-min-thickness") {
            ok = number(i, options.params.ribMinThickness);
        } else if (arg == "--rib-max-thickness") {
            ok = number(i, options.params.ribMaxThickness);
        } else if (arg == "--min-area") {
            ok = number(i, options.params.minWorkpieceArea);
        } else {
            std::cerr << "未知参数 " << arg << '\n';
            ok = false;
        }
        if (!ok) {
            return false;
        }
    }
    return true;
}

int RunOnPly(const Options& options)
{
    Cloud::Ptr cloud(new Cloud);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(options.plyPath, *cloud) < 0) {
        std::cerr << "读取 PLY 失败: " << options.plyPath << '\n';
        return EXIT_FAILURE;
    }
    if (options.scale != 1.0f) {
        for (pcl::PointXYZ& p : cloud->points) {
            p.x *= options.scale;
            p.y *= options.scale;
            p.z *= options.scale;
        }
    }
    std::cout << "读入 " << cloud->size() << " 点: " << options.plyPath << (options.params.zAxisDown ? "（Z 指向地面）" : "（Z 向上）")
              << '\n';

    const auto begin = std::chrono::steady_clock::now();
    SceneSeamResult result = ExtractSceneSeams(cloud, options.params);
    const auto end = std::chrono::steady_clock::now();
    std::cout << "耗时 " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms\n";
    PrintResult(result);
    if (!result.success) {
        return EXIT_FAILURE;
    }

    // CSV 保持输入坐标系；可视化按 Z 向上绘制
    if (!options.exportPath.empty() && !ExportSeamsCsv(result, options.exportPath)) {
        return EXIT_FAILURE;
    }
    if (options.showViewer) {
        if (options.params.zAxisDown) {
            FlipResultZ(result);
        }
        ShowScene(result);
    }
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        return EXIT_FAILURE;
    }
    if (!options.plyPath.empty()) {
        return RunOnPly(options);
    }
    const bool showViewer = options.showViewer;

    int failures = 0;

    const SceneSeamResult nullResult = ExtractSceneSeams(Cloud::ConstPtr());
    failures += !Expect(!nullResult.success, "空指针应失败");

    const Scene scene = BuildScene();
    std::cout << "场景点数 " << scene.cloud->size() << "，工件 " << scene.workpieces.size() << " 个\n";
    // 合成场景为相机坐标系（Z 指向地面），与默认参数 zAxisDown = true 一致
    if (!options.savePlyPath.empty()) {
        if (pcl::io::savePLYFileBinary(options.savePlyPath, *scene.cloud) < 0) {
            std::cerr << "写入 PLY 失败: " << options.savePlyPath << '\n';
            return EXIT_FAILURE;
        }
        std::cout << "合成场景已按相机坐标系写入 " << options.savePlyPath << '\n';
    }

    const auto begin = std::chrono::steady_clock::now();
    SceneSeamResult result = ExtractSceneSeams(scene.cloud);
    const auto end = std::chrono::steady_clock::now();
    std::cout << "耗时 " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms，"
              << result.message << '\n';

    failures += !Expect(result.success, "提取应成功");

    // 同一场景转成 Z 向上后用 zAxisDown = false 处理，翻回相机坐标系应得到一致结果
    {
        Cloud::Ptr uprightCloud(new Cloud(*scene.cloud));
        for (pcl::PointXYZ& p : uprightCloud->points) {
            p.z = -p.z;
        }
        SceneSeamParams upright;
        upright.zAxisDown = false;
        SceneSeamResult uprightResult = ExtractSceneSeams(uprightCloud, upright);
        failures += !Expect(uprightResult.success, "Z 向上输入提取应成功");
        failures += !Expect(uprightResult.seams.size() == result.seams.size(), "Z 向上输入焊缝数量应一致");
        FlipResultZ(uprightResult);
        float maxDiff = 0.0f;
        for (std::size_t i = 0; i < std::min(uprightResult.seams.size(), result.seams.size()); ++i) {
            maxDiff = std::max(maxDiff, (uprightResult.seams[i].start - result.seams[i].start).norm());
            maxDiff = std::max(maxDiff, (uprightResult.seams[i].end - result.seams[i].end).norm());
        }
        std::cout << "Z 向上输入与相机坐标系结果最大偏差 " << maxDiff << " mm\n";
        // 体素栅格边界上的点在翻转后可能落入相邻体素，端点收缩按 2 mm 步进，允许几个栅格的差异
        failures += !Expect(maxDiff < 5.0f, "两种坐标系输入的结果应一致");
    }
    failures += !Expect(result.workpieces.size() == scene.workpieces.size(), "工件数量应为 19");

    // 相机坐标系中地面 z = kCameraHeight - 0.002 x + 0.001 y，朝向相机的法向为 (-0.002, 0.001, -1)
    const Eigen::Vector3f groundNormal = result.groundPlane.head<3>().normalized();
    const Eigen::Vector3f expectedNormal = Eigen::Vector3f(-0.002f, 0.001f, -1.0f).normalized();
    std::cout << "地面法向 " << groundNormal.transpose() << "  对齐 " << groundNormal.dot(expectedNormal) << '\n';
    failures += !Expect(groundNormal.dot(expectedNormal) > 0.9999f, "地面倾斜应被估计出来");

    int centerMatched = 0;
    for (const PlacedWorkpiece& truth : scene.workpieces) {
        for (const Workpiece& piece : result.workpieces) {
            if ((piece.center - truth.pose.position).norm() < 150.0f) {
                ++centerMatched;
                const float expectedBase = truth.shape->baseThickness;
                if (std::fabs(piece.baseHeight - expectedBase) > 3.0f) {
                    std::cerr << "工件 " << piece.id << " 底板高度 " << piece.baseHeight << " 期望 " << expectedBase << '\n';
                    ++failures;
                }
                break;
            }
        }
    }
    failures += !Expect(centerMatched == static_cast<int>(scene.workpieces.size()), "每个工件都应被定位");

    if (std::getenv("SEAM_DEBUG")) {
        for (const Workpiece& piece : result.workpieces) {
            const PlacedWorkpiece* truth = nullptr;
            for (const PlacedWorkpiece& candidate : scene.workpieces) {
                if ((piece.center - candidate.pose.position).norm() < 150.0f) {
                    truth = &candidate;
                }
            }
            std::cout << "工件 " << piece.id << (truth ? " " + truth->shape->name : std::string(" ?")) << "  yaw "
                      << piece.yawRad * 180.0f / kPi << "  立板 " << piece.ribs.size() << '\n';
            if (truth) {
                for (const RibSpec& rib : truth->shape->ribs) {
                    const Eigen::Vector2f a = truth->pose.apply(rib.a);
                    const Eigen::Vector2f b = truth->pose.apply(rib.b);
                    std::cout << "  真值 " << a.transpose() << " -> " << b.transpose() << "  t " << rib.thickness << "  h "
                              << rib.height << '\n';
                }
            }
            for (const RibSegment& rib : piece.ribs) {
                std::cout << "  检出 " << rib.start.transpose() << " -> " << rib.end.transpose() << "  t " << rib.thickness
                          << "  h " << rib.height << "  conf " << rib.confidence << '\n';
            }
        }
    }

    const MatchReport report = CompareSeams(scene, result, 20.0f);
    const float recall = report.truthTotal > 0 ? static_cast<float>(report.truthMatched) / report.truthTotal : 0.0f;
    const float precision = report.detectedTotal > 0 ? static_cast<float>(report.detectedMatched) / report.detectedTotal : 0.0f;
    std::cout << "真值焊缝 " << report.truthTotal << "（平 " << report.truthFlat << "，立 " << report.truthVertical << "）"
              << "  检出 " << report.detectedTotal << "（平 " << report.detectedFlat << "，立 " << report.detectedVertical << "）\n"
              << "召回 " << recall << "  精度 " << precision << '\n';
    failures += !Expect(recall >= 0.95f, "焊缝召回应不低于 0.95");

    // 间隙阈值小于立板提取带宽时，横穿立板会把一根筋切成两段，两侧焊缝一起断开。
    // 共线合并应把它们接回去，召回不下降。
    {
        SceneSeamParams tight;
        tight.ribGapTolerance = 20.0f;
        const SceneSeamResult tightResult = ExtractSceneSeams(scene.cloud, tight);
        const MatchReport tightReport = CompareSeams(scene, tightResult, 20.0f);
        const float tightRecall = tightReport.truthTotal > 0
            ? static_cast<float>(tightReport.truthMatched) / tightReport.truthTotal : 0.0f;
        std::cout << "ribGapTolerance 20 mm 召回 " << tightRecall << "  检出 " << tightReport.detectedTotal << '\n';
        failures += !Expect(tightRecall >= 0.95f, "间隙阈值 20 mm 时共线立板应合并");
    }
    failures += !Expect(precision >= 0.95f, "焊缝精度应不低于 0.95");

    // 相机坐标系中离相机越近 z 越小，焊缝 z 应小于地面 z
    for (const InitialSeam& seam : result.seams) {
        const float groundClearance = GroundZ(seam.start.x(), seam.start.y()) - std::max(seam.start.z(), seam.end.z());
        if (groundClearance < 5.0f) {
            std::cerr << "焊缝落到地面以下: 高出地面 " << groundClearance << '\n';
            ++failures;
            break;
        }
    }

    if (failures != 0) {
        std::cerr << failures << " 项检查失败\n";
    } else {
        std::cout << "通过\n";
    }
    if (showViewer) {
        FlipResultZ(result); // 可视化按 Z 向上绘制
        ShowScene(result);
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
