#include "PlaceConfigJson.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool NearlyEqual(double a, double b)
{
    return std::fabs(a - b) < 1e-9;
}

bool JointsEqual(const Joint& a, const Joint& b)
{
    return NearlyEqual(a.j1, b.j1)
        && NearlyEqual(a.j2, b.j2)
        && NearlyEqual(a.j3, b.j3)
        && NearlyEqual(a.j4, b.j4)
        && NearlyEqual(a.j5, b.j5)
        && NearlyEqual(a.j6, b.j6);
}

bool ConfigsEqual(const PlaceConfig& a, const PlaceConfig& b)
{
    return a.ArrayConfig.layerX == b.ArrayConfig.layerX
        && a.ArrayConfig.layerY == b.ArrayConfig.layerY
        && NearlyEqual(a.ArrayConfig.obj_length, b.ArrayConfig.obj_length)
        && NearlyEqual(a.ArrayConfig.obj_width, b.ArrayConfig.obj_width)
        && NearlyEqual(a.ArrayConfig.obj_height, b.ArrayConfig.obj_height)
        && JointsEqual(a.PassJoint, b.PassJoint)
        && JointsEqual(a.PlaceJoint, b.PlaceJoint)
        && JointsEqual(a.WaitJoint, b.WaitJoint);
}

} // namespace

int main(int argc, char** argv)
{
    const std::string filePath = (argc > 1) ? argv[1] : "place_config.json";

    PlaceConfig cfg;
    cfg.ArrayConfig.layerX = 2;
    cfg.ArrayConfig.layerY = 3;
    cfg.ArrayConfig.obj_length = 120.5;
    cfg.ArrayConfig.obj_width = 80.25;
    cfg.ArrayConfig.obj_height = 15.0;

    cfg.PassJoint = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    cfg.PlaceJoint = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
    cfg.WaitJoint = {-10.0, 20.0, -30.0, 40.0, -50.0, 60.0};

    SavePlaceConfig(filePath, cfg);
    std::cout << "Saved: " << filePath << '\n';

    const PlaceConfig loaded = LoadPlaceConfig(filePath);
    if (!ConfigsEqual(cfg, loaded)) {
        std::cerr << "Round-trip mismatch\n";
        return 1;
    }

    std::cout << "Loaded ArrayConfig.layerX=" << loaded.ArrayConfig.layerX
              << " layerY=" << loaded.ArrayConfig.layerY
              << " obj_length=" << loaded.ArrayConfig.obj_length
              << " obj_width=" << loaded.ArrayConfig.obj_width
              << " obj_height=" << loaded.ArrayConfig.obj_height << '\n';
    std::cout << "PlaceJoint.j1=" << loaded.PlaceJoint.j1
              << " j2=" << loaded.PlaceJoint.j2
              << " j3=" << loaded.PlaceJoint.j3 << '\n';
    std::cout << "Round-trip OK\n";
    return 0;
}
