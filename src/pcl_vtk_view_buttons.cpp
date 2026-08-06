/**
 * @file pcl_vtk_view_buttons.cpp
 * @brief PCL + VTK + Qt：三个按钮切换正视 / 俯视 / 左视
 *
 * 布局：上方三个 QPushButton，下方 QVTK 窗口显示点云。
 * 点击按钮 → 设置 VTK Camera 位姿 + 正交投影。
 *
 * 坐标系：Z 向上，+X 右，+Y 前
 *   正视 = 从 -Y 看   俯视 = 从 +Z 看   左视 = 从 -X 看
 *
 * 用法：
 *   ./pcl_vtk_view_buttons [cloud.pcd|ply]
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWidget>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkAutoInit.h>

#include <QVTKOpenGLNativeWidget.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

namespace {

enum class ViewType { Front, Top, Left };

struct CloudBounds {
  Eigen::Vector3f center{0.f, 0.f, 0.f};
  float radius{1.f};
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

void viewCameraPose(ViewType view,
                    const CloudBounds& b,
                    Eigen::Vector3f& pos,
                    Eigen::Vector3f& focal,
                    Eigen::Vector3f& up) {
  focal = b.center;
  const float d = b.radius * 3.0f;
  switch (view) {
    case ViewType::Front:
      pos = b.center + Eigen::Vector3f(0.f, -d, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
    case ViewType::Top:
      pos = b.center + Eigen::Vector3f(0.f, 0.f, d);
      up = Eigen::Vector3f(0.f, 1.f, 0.f);
      break;
    case ViewType::Left:
      pos = b.center + Eigen::Vector3f(-d, 0.f, 0.f);
      up = Eigen::Vector3f(0.f, 0.f, 1.f);
      break;
  }
}

vtkSmartPointer<vtkPolyData> cloudToVtk(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  auto verts = vtkSmartPointer<vtkCellArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  points->SetNumberOfPoints(static_cast<vtkIdType>(cloud->size()));

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    points->SetPoint(static_cast<vtkIdType>(i), p.x, p.y, p.z);
    colors->InsertNextTuple3(p.r, p.g, p.b);
    const vtkIdType id = static_cast<vtkIdType>(i);
    verts->InsertNextCell(1, &id);
  }

  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(verts);
  poly->GetPointData()->SetScalars(colors);
  return poly;
}

bool loadCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    return false;
  }
  const std::string ext = path.substr(dot);
  int ret = -1;
  if (ext == ".ply" || ext == ".PLY") {
    ret = pcl::io::loadPLYFile(path, *cloud);
  } else if (ext == ".pcd" || ext == ".PCD") {
    ret = pcl::io::loadPCDFile(path, *cloud);
  }
  return ret >= 0 && !cloud->empty();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeDemoCloud() {
  auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  auto add = [&](float x, float y, float z, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    cloud->push_back(p);
  };

  const float sx = 2.f, sy = 1.f, sz = 0.5f, step = 0.05f;
  for (float x = -sx; x <= sx; x += step) {
    for (float y = -sy; y <= sy; y += step) {
      add(x, y, -sz, 180, 180, 200);
      add(x, y, sz, 200, 200, 220);
    }
  }
  for (float x = -sx; x <= sx; x += step) {
    for (float z = -sz; z <= sz; z += step) {
      add(x, -sy, z, 160, 170, 190);
      add(x, sy, z, 170, 180, 200);
    }
  }
  for (float y = -sy; y <= sy; y += step) {
    for (float z = -sz; z <= sz; z += step) {
      add(-sx, y, z, 150, 160, 180);
      add(sx, y, z, 190, 200, 210);
    }
  }
  for (float t = 0.f; t <= 3.f; t += 0.02f) {
    add(t, 0.f, 0.f, 255, 40, 40);
    add(0.f, t, 0.f, 40, 220, 40);
    add(0.f, 0.f, t, 40, 100, 255);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

/** 主窗口：三按钮 + VTK 点云视图 */
class OrthoViewWindow : public QWidget {
 public:
  OrthoViewWindow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QWidget* parent = nullptr)
      : QWidget(parent), bounds_(computeBounds(cloud)) {
    setWindowTitle(QString::fromUtf8("点云工程视图 — 正视 / 俯视 / 左视"));
    resize(1100, 750);

    // —— 三个按钮 ——
    auto* btn_front = new QPushButton(QString::fromUtf8("正视图"), this);
    auto* btn_top = new QPushButton(QString::fromUtf8("俯视图"), this);
    auto* btn_left = new QPushButton(QString::fromUtf8("左视图"), this);
    btn_front->setMinimumHeight(40);
    btn_top->setMinimumHeight(40);
    btn_left->setMinimumHeight(40);
    btn_front->setStyleSheet("font-size: 16px; font-weight: bold;");
    btn_top->setStyleSheet("font-size: 16px; font-weight: bold;");
    btn_left->setStyleSheet("font-size: 16px; font-weight: bold;");

    status_ = new QLabel(QString::fromUtf8("当前：正视图（正交）"), this);
    status_->setAlignment(Qt::AlignCenter);

    auto* btn_row = new QHBoxLayout;
    btn_row->addWidget(btn_front);
    btn_row->addWidget(btn_top);
    btn_row->addWidget(btn_left);

    // —— VTK 嵌入 Qt ——
    vtk_widget_ = new QVTKOpenGLNativeWidget(this);
    auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtk_widget_->setRenderWindow(render_window);

    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetBackground(0.12, 0.12, 0.14);
    render_window->AddRenderer(renderer_);

    auto poly = cloudToVtk(cloud);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(poly);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2);
    renderer_->AddActor(actor);

    auto* root = new QVBoxLayout(this);
    root->addLayout(btn_row);
    root->addWidget(status_);
    root->addWidget(vtk_widget_, /*stretch=*/1);

    // 按钮信号 → 切视角
    connect(btn_front, &QPushButton::clicked, this, [this]() { setView(ViewType::Front); });
    connect(btn_top, &QPushButton::clicked, this, [this]() { setView(ViewType::Top); });
    connect(btn_left, &QPushButton::clicked, this, [this]() { setView(ViewType::Left); });

    setView(ViewType::Front);
  }

 private:
  void setView(ViewType view) {
    Eigen::Vector3f pos, focal, up;
    viewCameraPose(view, bounds_, pos, focal, up);

    vtkCamera* cam = renderer_->GetActiveCamera();
    cam->SetPosition(pos.x(), pos.y(), pos.z());
    cam->SetFocalPoint(focal.x(), focal.y(), focal.z());
    cam->SetViewUp(up.x(), up.y(), up.z());
    cam->ParallelProjectionOn();
    cam->SetParallelScale(static_cast<double>(bounds_.radius * 1.1f));
    renderer_->ResetCameraClippingRange();
    vtk_widget_->renderWindow()->Render();

    const char* name = (view == ViewType::Front) ? "正视图"
                      : (view == ViewType::Top)  ? "俯视图"
                                                 : "左视图";
    status_->setText(QString::fromUtf8("当前：") + QString::fromUtf8(name) +
                     QString::fromUtf8("（正交）"));
  }

  CloudBounds bounds_;
  QLabel* status_ = nullptr;
  QVTKOpenGLNativeWidget* vtk_widget_ = nullptr;
  vtkSmartPointer<vtkRenderer> renderer_;
};

}  // namespace

int main(int argc, char** argv) {
  // Qt + VTK OpenGL 需在创建 QApplication 前设置
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  QApplication app(argc, argv);

  std::string path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      std::cout << "用法: " << argv[0] << " [cloud.pcd|ply]\n";
      return 0;
    }
    path = a;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (path.empty()) {
    std::cout << "未指定点云，使用演示数据\n";
    cloud = makeDemoCloud();
  } else if (!loadCloud(path, cloud)) {
    std::cerr << "读取失败: " << path << std::endl;
    return 1;
  }
  std::cout << "点数: " << cloud->size() << std::endl;

  OrthoViewWindow win(cloud);
  win.show();
  return app.exec();
}
