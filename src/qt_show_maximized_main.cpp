#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>
#include <vtkOutputWindow.h>
#include <QMetaType>

MainWindow* MainWindow::singleton = nullptr;

int main(int argc, char* argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0); //不弹出vtkOutputWindow窗口

    qRegisterMetaType<ct::Cloud::Ptr>("ct::Cloud::Ptr");
    qRegisterMetaType<RobotPara>("RobotPara");
    qRegisterMetaType<std::vector<Eigen::Affine3f>>("std::vector<Eigen::Affine3f>");
    qRegisterMetaType<Eigen::Affine3f>("Eigen::Affine3f");
    qRegisterMetaType<std::vector<std::pair<std::vector<float>, std::vector<float>>>>("std::vector<std::pair<std::vector<float>, std::vector<float>>>");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<bool>("bool");
    qRegisterMetaType<CamData>("CamData");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<rl::plan::VectorList>(" rl::plan::VectorList");
    qRegisterMetaType<rl::math::Real>("rl::math::Real");
    qRegisterMetaType<rl::math::Transform>("rl::math::Transform");
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>>("std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>>");

    QApplication a(argc, argv);
    MainWindow w;
    w.showMaximized();
    return a.exec();
}
