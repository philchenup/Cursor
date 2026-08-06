/**
 * @file pcl_vtk_ortho_views.cpp
 * @brief PCL + VTK：点云正视图 / 俯视图 / 左视图 / 右视图 / 等轴测
 *
 * 思路：
 *   1. PCL 读点云，PCLVisualizer 管窗口与多视口（底层即 VTK）
 *   2. setCameraPosition 摆工程视角；VTK Camera::ParallelProjectionOn 开正交投影
 *
 * 坐标系约定（Z 向上，右手系）：
 *   +X 右，+Y 前，+Z 上
 *   正视 = 从 -Y 看向原点前方
 *   俯视 = 从 +Z 往下看
 *   左视 = 从 -X 往右看
 *
 * 用法：
 *   ./pcl_vtk_ortho_views [cloud.pcd|ply] [--single]
 *
 * 快捷键：
 *   1 正视  2 俯视  3 左视  4 右视  5 后视  6 等轴测
 *   o 正交/透视切换   q 退出
 */

#include <cmath>
#include <iostream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

namespace {

enum class ViewType { Front, Top, Left, Right, Back, Iso };

struct CloudBounds {
  Eigen::Vector3f center;
  float radius;
};

CloudBounds computeBounds(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  CloudBounds b;
  b.center = 0.5f * (Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z) +
                     Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z));
  const Eigen::Vector3f diag(max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
  b.radius = std::max(0.5f * diag.norm(), 1e-3f);
  return b;
}

const char* viewName(ViewType v) {
  switch (v) {
    case ViewType::Front: return "Front (正视)";
    case ViewType::Top:   return "Top (俯视)";
    case ViewType::Left:  return "Left (左视)";
    case ViewType::Right: return "Right (右视)";
    case ViewType::Back:  return "Back (后视)";
    case ViewType::Iso:   return "Iso (等轴测)";
  }
  return "?";
}

/** 工程视图 → 相机位置 / 焦点 / 上方向 */
void viewCameraPose(ViewType view,
                    const CloudBounds& b,
                    Eigen::Vector3f& pos,
                    Eigen::Vector3f& focal,
                    Eigen::Vector3f& up) {
  focal = b.center;
  const float d = b.radius * 3.0f;

  switch (view) {
    case ViewType::Front:  // 从 -Y 看 +Y，Z 向上
      pos = b.center + Eigen::Vector3f(0.f, -d, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
    case ViewType::Top:  // 俯视，画面上方为 +Y
      pos = b.center + Eigen::Vector3f(0.f, 0.f, d);
      up = Eigen::Vector3f(0.f, 1.f, 0.f);
      break;
    case ViewType::Left:  // 从 -X 看 +X
      pos = b.center + Eigen::Vector3f(-d, 0.f, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
    case ViewType::Right:
      pos = b.center + Eigen::Vector3f(d, 0.f, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
    case ViewType::Back:
      pos = b.center + Eigen::Vector3f(0.f, d, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
    case ViewType::Iso:
    default:
      pos = b.center + Eigen::Vector3f(d, d, d) / std::sqrt(3.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
  }
}

/**
 * 核心：PCL 设位姿 + VTK 开/关正交投影。
 * viewport 与 PCLVisualizer 内部 renderer 下标一致（createViewPort 返回值）。
 * viewport==0 表示作用于全部 renderer（单视口常用）。
 */
void applyView(pcl::visualization::PCLVisualizer& viewer,
               int viewport,
               ViewType view,
               const CloudBounds& bounds,
               bool parallel) {
  Eigen::Vector3f pos, focal, up;
  viewCameraPose(view, bounds, pos, focal, up);

  viewer.setCameraPosition(pos.x(), pos.y(), pos.z(),
                           focal.x(), focal.y(), focal.z(),
                           up.x(), up.y(), up.z(),
                           viewport);

  vtkRendererCollection* renderers = viewer.getRenderWindow()->GetRenderers();
  renderers->InitTraversal();
  int i = 0;
  for (vtkRenderer* ren = renderers->GetNextItem(); ren != nullptr;
       ren = renderers->GetNextItem(), ++i) {
    if (!(viewport == 0 || viewport == i)) {
      continue;
    }
    vtkCamera* cam = ren->GetActiveCamera();
    if (!cam) {
      continue;
    }
    cam->SetPosition(pos.x(), pos.y(), pos.z());
    cam->SetFocalPoint(focal.x(), focal.y(), focal.z());
    cam->SetViewUp(up.x(), up.y(), up.z());
    if (parallel) {
      cam->ParallelProjectionOn();
      cam->SetParallelScale(static_cast<double>(bounds.radius * 1.1f));
    } else {
      cam->ParallelProjectionOff();
    }
    ren->ResetCameraClippingRange();
  }

  viewer.getRenderWindow()->Render();
}

/** 四视口一次刷新正交开关时，四个视角相机位置不同，需按视口分别 apply */
void applyAllQuadViews(pcl::visualization::PCLVisualizer& viewer,
                       int vp_front, int vp_top, int vp_left, int vp_iso,
                       const CloudBounds& bounds, bool parallel) {
  applyView(viewer, vp_front, ViewType::Front, bounds, parallel);
  applyView(viewer, vp_top, ViewType::Top, bounds, parallel);
  applyView(viewer, vp_left, ViewType::Left, bounds, parallel);
  applyView(viewer, vp_iso, ViewType::Iso, bounds, parallel);
}

bool loadCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    std::cerr << "无法识别后缀: " << path << std::endl;
    return false;
  }
  const std::string ext = path.substr(dot);
  int ret = -1;
  if (ext == ".ply" || ext == ".PLY") {
    ret = pcl::io::loadPLYFile(path, *cloud);
  } else if (ext == ".pcd" || ext == ".PCD") {
    ret = pcl::io::loadPCDFile(path, *cloud);
  } else {
    std::cerr << "仅支持 .ply / .pcd\n";
    return false;
  }
  if (ret < 0 || cloud->empty()) {
    std::cerr << "读取失败或为空: " << path << std::endl;
    return false;
  }
  return true;
}

/** 演示点云：扁长方体 + XYZ 色轴，三视图外形可区分 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeDemoCloud() {
  auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  auto addPoint = [&](float x, float y, float z, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    cloud->push_back(p);
  };

  const float sx = 2.f, sy = 1.f, sz = 0.5f;
  const float step = 0.05f;
  for (float x = -sx; x <= sx; x += step) {
    for (float y = -sy; y <= sy; y += step) {
      addPoint(x, y, -sz, 180, 180, 200);
      addPoint(x, y, sz, 200, 200, 220);
    }
  }
  for (float x = -sx; x <= sx; x += step) {
    for (float z = -sz; z <= sz; z += step) {
      addPoint(x, -sy, z, 160, 170, 190);
      addPoint(x, sy, z, 170, 180, 200);
    }
  }
  for (float y = -sy; y <= sy; y += step) {
    for (float z = -sz; z <= sz; z += step) {
      addPoint(-sx, y, z, 150, 160, 180);
      addPoint(sx, y, z, 190, 200, 210);
    }
  }
  for (float t = 0.f; t <= 3.f; t += 0.02f) {
    addPoint(t, 0.f, 0.f, 255, 40, 40);   // X 红
    addPoint(0.f, t, 0.f, 40, 220, 40);   // Y 绿
    addPoint(0.f, 0.f, t, 40, 100, 255);  // Z 蓝
  }

  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

struct UiState {
  ViewType view = ViewType::Iso;
  bool parallel = true;
  bool single_mode = false;
  int main_viewport = 0;
  int vp_front = 0, vp_top = 0, vp_left = 0, vp_iso = 0;
  CloudBounds bounds{};
  pcl::visualization::PCLVisualizer* viewer = nullptr;
};

void keyboardEvent(const pcl::visualization::KeyboardEvent& event, void* cookie) {
  if (!event.keyDown()) {
    return;
  }
  auto* st = static_cast<UiState*>(cookie);
  const std::string key = event.getKeySym();

  auto setView = [&](ViewType v) {
    st->view = v;
    applyView(*st->viewer, st->main_viewport, v, st->bounds, st->parallel);
    std::cout << "视角 -> " << viewName(v)
              << (st->parallel ? " [正交]" : " [透视]") << std::endl;
  };

  if (key == "1") {
    setView(ViewType::Front);
  } else if (key == "2") {
    setView(ViewType::Top);
  } else if (key == "3") {
    setView(ViewType::Left);
  } else if (key == "4") {
    setView(ViewType::Right);
  } else if (key == "5") {
    setView(ViewType::Back);
  } else if (key == "6") {
    setView(ViewType::Iso);
  } else if (key == "o" || key == "O") {
    st->parallel = !st->parallel;
    if (st->single_mode) {
      applyView(*st->viewer, st->main_viewport, st->view, st->bounds, st->parallel);
    } else {
      applyAllQuadViews(*st->viewer, st->vp_front, st->vp_top, st->vp_left, st->vp_iso,
                        st->bounds, st->parallel);
    }
    std::cout << (st->parallel ? "正交投影 ON" : "透视投影 ON") << std::endl;
  }
}

void runQuadView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, UiState& ui) {
  pcl::visualization::PCLVisualizer viewer("PCL+VTK Ortho Views (四视口)");
  ui.viewer = &viewer;
  ui.single_mode = false;

  // 左上俯视 | 右上等轴测
  // 左下正视 | 右下左视
  // 注意：createViewPort 后必须 createViewPortCamera，否则多视口共用一台相机，
  // 正视/俯视/左视无法同时成立。
  viewer.createViewPort(0.0, 0.5, 0.5, 1.0, ui.vp_top);
  viewer.createViewPortCamera(ui.vp_top);
  viewer.createViewPort(0.5, 0.5, 1.0, 1.0, ui.vp_iso);
  viewer.createViewPortCamera(ui.vp_iso);
  viewer.createViewPort(0.0, 0.0, 0.5, 0.5, ui.vp_front);
  viewer.createViewPortCamera(ui.vp_front);
  viewer.createViewPort(0.5, 0.0, 1.0, 0.5, ui.vp_left);
  viewer.createViewPortCamera(ui.vp_left);

  // 构造时自带的 renderer[0] 仍是全窗，缩到空避免盖住四视口
  if (vtkRenderer* base = viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()) {
    base->SetViewport(0.0, 0.0, 0.0, 0.0);
  }

  viewer.setBackgroundColor(0.12, 0.12, 0.14, ui.vp_front);
  viewer.setBackgroundColor(0.12, 0.12, 0.14, ui.vp_top);
  viewer.setBackgroundColor(0.12, 0.12, 0.14, ui.vp_left);
  viewer.setBackgroundColor(0.10, 0.10, 0.12, ui.vp_iso);

  viewer.addText("Front 正视", 10, 10, 14, 1, 1, 1, "txt_front", ui.vp_front);
  viewer.addText("Top 俯视", 10, 10, 14, 1, 1, 1, "txt_top", ui.vp_top);
  viewer.addText("Left 左视", 10, 10, 14, 1, 1, 1, "txt_left", ui.vp_left);
  viewer.addText("Iso 等轴测 (1-6切换)", 10, 10, 14, 1, 1, 1, "txt_iso", ui.vp_iso);

  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud_f", ui.vp_front);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud_t", ui.vp_top);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud_l", ui.vp_left);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud_i", ui.vp_iso);

  for (const char* id : {"cloud_f", "cloud_t", "cloud_l", "cloud_i"}) {
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
  }

  viewer.addCoordinateSystem(0.5, "axes_f", ui.vp_front);
  viewer.addCoordinateSystem(0.5, "axes_t", ui.vp_top);
  viewer.addCoordinateSystem(0.5, "axes_l", ui.vp_left);
  viewer.addCoordinateSystem(0.5, "axes_i", ui.vp_iso);

  applyAllQuadViews(viewer, ui.vp_front, ui.vp_top, ui.vp_left, ui.vp_iso,
                    ui.bounds, ui.parallel);

  // 键盘 1~6 改右上等轴测视口，便于对照
  ui.main_viewport = ui.vp_iso;
  ui.view = ViewType::Iso;
  viewer.registerKeyboardCallback(keyboardEvent, &ui);

  std::cout << "四视口：左下正视 / 左上俯视 / 右下左视 / 右上等轴测\n"
            << "1~6 切换右上视口；o 正交/透视；q 退出\n";

  while (!viewer.wasStopped()) {
    viewer.spinOnce(50);
  }
}

void runSingleView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, UiState& ui) {
  pcl::visualization::PCLVisualizer viewer("PCL+VTK Ortho Views (单视口)");
  ui.viewer = &viewer;
  ui.single_mode = true;
  ui.main_viewport = 0;

  viewer.setBackgroundColor(0.12, 0.12, 0.14);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer.addCoordinateSystem(0.5);
  viewer.addText("1 Front  2 Top  3 Left  4 Right  5 Back  6 Iso  |  o ortho",
                 10, 20, 14, 1, 1, 1, "help");

  ui.view = ViewType::Front;
  applyView(viewer, 0, ui.view, ui.bounds, ui.parallel);
  viewer.registerKeyboardCallback(keyboardEvent, &ui);

  std::cout << "单视口：1正视 2俯视 3左视 4右视 5后视 6等轴测；o 正交/透视；q 退出\n";

  while (!viewer.wasStopped()) {
    viewer.spinOnce(50);
  }
}

}  // namespace

int main(int argc, char** argv) {
  bool single = false;
  std::string path;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--single" || a == "-s") {
      single = true;
    } else if (a == "--help" || a == "-h") {
      std::cout << "用法: " << argv[0] << " [cloud.pcd|ply] [--single]\n";
      return 0;
    } else {
      path = a;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (path.empty()) {
    std::cout << "未指定点云，使用演示数据（长方体 + XYZ 色轴）\n";
    cloud = makeDemoCloud();
  } else if (!loadCloud(path, cloud)) {
    return 1;
  }

  std::cout << "点数: " << cloud->size() << std::endl;

  UiState ui;
  ui.bounds = computeBounds(cloud);
  ui.parallel = true;

  if (single) {
    runSingleView(cloud, ui);
  } else {
    runQuadView(cloud, ui);
  }
  return 0;
}
