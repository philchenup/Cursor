/**
 * @file demo_common.cpp
 * @brief common.h 中每个公开函数的示例。
 */
#include "base/common.h"

#include <Eigen/Geometry>
#include <iostream>

namespace
{
    void demo_HSVtoRGB()
    {
        float r = 0.f, g = 0.f, b = 0.f;
        ct::HSVtoRGB(0.0f, 1.0f, 1.0f, r, g, b);
        std::cout << "[ HSVtoRGB ] h=0 s=1 v=1 -> rgb=(" << r << "," << g << "," << b << ")\n";
    }

    Eigen::Affine3f makeSampleTransform()
    {
        return ct::getTransformation(1.0f, 2.0f, 3.0f, 10.0f, 20.0f, 30.0f);
    }

    void demo_getTransformation_xyzrpy_return()
    {
        Eigen::Affine3f t = ct::getTransformation(1.f, 2.f, 3.f, 10.f, 20.f, 30.f);
        std::cout << "[ getTransformation(x,y,z,r,p,y) ] t(0,3)=" << t.translation().x()
                  << " t(1,3)=" << t.translation().y()
                  << " t(2,3)=" << t.translation().z() << "\n";
    }

    void demo_getTransformation_xyzrpy_outparam()
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        ct::getTransformation(0.5f, -1.0f, 2.0f, 5.f, -8.f, 12.f, t);
        std::cout << "[ getTransformation(..., t) ] translation=("
                  << t.translation().transpose() << ")\n";
    }

    void demo_getTransformation_from_string()
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        const bool ok_euler = ct::getTransformation("1 2 3 10 20 30", t);
        std::cout << "[ getTransformation(string euler) ] ok=" << ok_euler
                  << " xyz=(" << t.translation().transpose() << ")\n";

        Eigen::Affine3f t2 = Eigen::Affine3f::Identity();
        const bool ok_mat = ct::getTransformation(
            "1 0 0 1 "
            "0 1 0 2 "
            "0 0 1 3 "
            "0 0 0 1",
            t2);
        std::cout << "[ getTransformation(string matrix) ] ok=" << ok_mat
                  << " xyz=(" << t2.translation().transpose() << ")\n";
    }

    void demo_getTransformation_axis_angle()
    {
        Eigen::Affine3f t = ct::getTransformation(45.f, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f);
        std::cout << "[ getTransformation(angle,axis,xyz) ] translation=("
                  << t.translation().transpose() << ")\n";
    }

    void demo_getEulerAngles()
    {
        const Eigen::Affine3f t = makeSampleTransform();
        float roll = 0.f, pitch = 0.f, yaw = 0.f;
        ct::getEulerAngles(t, roll, pitch, yaw);
        std::cout << "[ getEulerAngles ] roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << "\n";
    }

    void demo_getAngleAxis()
    {
        const Eigen::Affine3f t = makeSampleTransform();
        float angle = 0.f, ax = 0.f, ay = 0.f, az = 0.f;
        ct::getAngleAxis(t, angle, ax, ay, az);
        std::cout << "[ getAngleAxis ] angle=" << angle << " axis=(" << ax << "," << ay << "," << az << ")\n";
    }

    void demo_getTranslationAndEulerAngles()
    {
        const Eigen::Affine3f t = makeSampleTransform();
        float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        ct::getTranslationAndEulerAngles(t, x, y, z, roll, pitch, yaw);
        std::cout << "[ getTranslationAndEulerAngles ] xyz=(" << x << "," << y << "," << z
                  << ") rpy=(" << roll << "," << pitch << "," << yaw << ")\n";
    }

    void demo_getTransformationString()
    {
        const Eigen::Affine3f t = makeSampleTransform();
        const std::string s = ct::getTransformationString(t.matrix(), 3);
        std::cout << "[ getTransformationString ]\n" << s << "\n";
    }

    void demo_getRotationMatrix()
    {
        const Eigen::Matrix3f R = ct::getRotationMatrix(10.f, 20.f, 30.f);
        std::cout << "[ getRotationMatrix ] R=\n" << R << "\n";
    }
}  // namespace

int main()
{
    std::cout << "======== demo_common (common.h) ========\n";
    demo_HSVtoRGB();
    demo_getTransformation_xyzrpy_return();
    demo_getTransformation_xyzrpy_outparam();
    demo_getTransformation_from_string();
    demo_getTransformation_axis_angle();
    demo_getEulerAngles();
    demo_getAngleAxis();
    demo_getTranslationAndEulerAngles();
    demo_getTransformationString();
    demo_getRotationMatrix();
    std::cout << "done.\n";
    return 0;
}
