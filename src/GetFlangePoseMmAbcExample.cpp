#include "GetFlangePoseMmAbc.h"

#include <iostream>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: GetFlangePoseMmAbcExample <robot.rlmdl.xml>\n";
        return 1;
    }

    rl::mdl::XmlFactory factory;
    std::shared_ptr<rl::mdl::Model> model(factory.create(argv[1]));
    auto* kin = dynamic_cast<rl::mdl::Kinematic*>(model.get());
    if (!kin) {
        std::cerr << "model is not Kinematic\n";
        return 1;
    }

    kin->setPosition(kin->getHomePosition());

    FlangePoseMmAbc pose;
    if (!getFlangePoseMmAbc(kin, pose, 0)) {
        std::cerr << "getFlangePoseMmAbc failed\n";
        return 1;
    }

    std::cout << "X Y Z [mm]  = " << pose.x_mm << " " << pose.y_mm << " " << pose.z_mm << "\n"
              << "A B C [deg] = " << pose.a_deg << " " << pose.b_deg << " " << pose.c_deg << "\n";
    return 0;
}
