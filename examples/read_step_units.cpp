/**
 * 读取 STEP，打印长度单位，并在 mm 时绕包围盒圆心缩放到米。
 *
 * 用法:
 *   read_step_units model.step
 */

#include "readmodel.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <iostream>

namespace {

const char* unitName(StepLengthUnit unit)
{
    switch (unit) {
    case StepLengthUnit::Metre:      return "m";
    case StepLengthUnit::Millimetre: return "mm";
    case StepLengthUnit::Other:      return "other";
    }
    return "unknown";
}

void printBBox(const char* title, const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        std::cout << title << " bbox: (void)\n";
        return;
    }
    Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double cx = 0.5 * (xmin + xmax);
    const double cy = 0.5 * (ymin + ymax);
    const double cz = 0.5 * (zmin + zmax);
    std::cout << title
              << " bbox min=(" << xmin << ", " << ymin << ", " << zmin << ")"
              << " max=(" << xmax << ", " << ymax << ", " << zmax << ")"
              << " center=(" << cx << ", " << cy << ", " << cz << ")\n";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: read_step_units <file.step>\n";
        return 1;
    }

    const char* path = argv[1];

    STEPControl_Reader probe;
    if (probe.ReadFile(path) != IFSelect_RetDone) {
        std::cerr << "failed to read " << path << "\n";
        return 2;
    }

    const StepLengthUnit unit = ReadModel::detectStepLengthUnit(probe);
    std::cout << "detected unit: " << unitName(unit) << "\n";

    if (unit == StepLengthUnit::Metre) {
        std::cout << "unit is metre: read as-is, no conversion\n";
    } else if (unit == StepLengthUnit::Millimetre) {
        std::cout << "unit is millimetre: scale about bbox center (x,y,z) by 0.001 to metre\n";
    } else {
        std::cout << "unit is other: scale about bbox center to metre\n";
    }

    const TopoDS_Shape shape = ReadModel::readStepModel(path);
    if (shape.IsNull()) {
        std::cerr << "transfer produced empty shape\n";
        return 3;
    }
    printBBox("result (metres)", shape);
    return 0;
}
