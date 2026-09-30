// 贴进 MainWindow，直接打印法兰 X Y Z [mm]  A B C [deg]
{
    mdl->forwardPosition();
    const auto& T = mdl->getOperationalPosition(0);
    const auto p = T.translation() * 1000.0;
    const auto abc = T.rotation().eulerAngles(2, 1, 0).reverse() * rl::math::RAD2DEG;
    std::cout << p.x() << " " << p.y() << " " << p.z() << " "
              << abc.x() << " " << abc.y() << " " << abc.z() << "\n";
}
