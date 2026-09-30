// 贴进 MainWindow：用 this->mdl 取法兰位姿（mm + ABC 度）。
// 与 OperationalModel 表格同一套欧拉角：eulerAngles(2,1,0).reverse()。

#include "GetFlangePoseMmAbc.h"
#include "MainWindow.h"

#include <rl/mdl/Kinematic.h>

bool MainWindow::getFlangePoseMmAbc(FlangePoseMmAbc& out, std::size_t ee)
{
    if (!this->mdl)
        return false;

    auto* kin = dynamic_cast<rl::mdl::Kinematic*>(this->mdl.get());
    return ::getFlangePoseMmAbc(kin, out, ee);
}

// 只取数组 [X,Y,Z,A,B,C]
bool MainWindow::getFlangePoseMmAbc(double xyzabc[6], std::size_t ee)
{
    FlangePoseMmAbc p;
    if (!getFlangePoseMmAbc(p, ee))
        return false;
    xyzabc[0] = p.x_mm;
    xyzabc[1] = p.y_mm;
    xyzabc[2] = p.z_mm;
    xyzabc[3] = p.a_deg;
    xyzabc[4] = p.b_deg;
    xyzabc[5] = p.c_deg;
    return true;
}

// 用法：
//   FlangePoseMmAbc pose;
//   if (getFlangePoseMmAbc(pose)) {
//       qDebug() << pose.x_mm << pose.y_mm << pose.z_mm
//                << pose.a_deg << pose.b_deg << pose.c_deg;
//   }
