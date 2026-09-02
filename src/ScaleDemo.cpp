#include "ScaleAISShape.h"

#include <AIS_Shape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

struct Dimensions {
    double dx;
    double dy;
    double dz;
};

Dimensions BoundingBoxSize(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return {xmax - xmin, ymax - ymin, zmax - zmin};
}

bool NearlyEqual(double a, double b, double tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

} // namespace

int main()
{
    const double sizeX = 1000.0;
    const double sizeY = 2000.0;
    const double sizeZ = 3000.0;

    TopoDS_Shape boxShape = BRepPrimAPI_MakeBox(sizeX, sizeY, sizeZ).Shape();
    Handle(AIS_Shape) original = new AIS_Shape(boxShape);

    // Give the source realistic display attributes so the demo also exercises
    // the attribute-copy path inside ScaleAISShapeBy1000.
    const Quantity_Color sourceColor(Quantity_NOC_RED);
    original->SetColor(sourceColor);
    original->SetWidth(2.0);

    Handle(AIS_Shape) scaled = ScaleAISShapeBy1000(original);
    if (scaled.IsNull()) {
        std::fprintf(stderr, "FAIL: ScaleAISShapeBy1000 returned null\n");
        return EXIT_FAILURE;
    }

    const Dimensions before = BoundingBoxSize(original->Shape());
    const Dimensions after = BoundingBoxSize(scaled->Shape());

    std::printf("Original bounding box: %.3f x %.3f x %.3f\n", before.dx, before.dy, before.dz);
    std::printf("Scaled   bounding box: %.6f x %.6f x %.6f\n", after.dx, after.dy, after.dz);

    const double expectedX = sizeX / 1000.0;
    const double expectedY = sizeY / 1000.0;
    const double expectedZ = sizeZ / 1000.0;
    const double tolerance = 1e-6;

    const bool ok =
        NearlyEqual(after.dx, expectedX, tolerance) &&
        NearlyEqual(after.dy, expectedY, tolerance) &&
        NearlyEqual(after.dz, expectedZ, tolerance);

    if (!ok) {
        std::fprintf(stderr,
                     "FAIL: expected %.6f x %.6f x %.6f after scaling\n",
                     expectedX, expectedY, expectedZ);
        return EXIT_FAILURE;
    }

    bool attributesCopied = true;
    if (scaled->HasColor()) {
        Quantity_Color scaledColor;
        scaled->Color(scaledColor);
        std::printf("Copied color matches source: %s\n",
                    scaledColor.IsEqual(sourceColor) ? "yes" : "no");
        attributesCopied = scaledColor.IsEqual(sourceColor);
    } else {
        attributesCopied = false;
    }
    std::printf("Copied edge width: %.3f (source 2.000)\n", scaled->Width());

    if (!attributesCopied || !NearlyEqual(scaled->Width(), 2.0, tolerance)) {
        std::fprintf(stderr, "FAIL: display attributes were not copied\n");
        return EXIT_FAILURE;
    }

    std::printf("PASS: shape scaled down by exactly 1000x with attributes preserved\n");
    return EXIT_SUCCESS;
}
