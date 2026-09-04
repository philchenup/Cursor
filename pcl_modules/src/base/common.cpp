/**
 * @file common.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/common.h"

#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

#define MATRIX_SIZE 16
#define EULER_SIZE 6

namespace ct
{
    void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b)
    {
        if (s == 0.0f)
        {
            r = g = b = v;
            return;
        }

        h = std::fmod(h, 1.0f) / (60.0f / 360.0f);
        const int i = static_cast<int>(h);
        const float f = h - static_cast<float>(i);
        const float p = v * (1.0f - s);
        const float q = v * (1.0f - s * f);
        const float t = v * (1.0f - s * (1.0f - f));

        switch (i)
        {
        case 0:
            r = v, g = t, b = p;
            break;
        case 1:
            r = q, g = v, b = p;
            break;
        case 2:
            r = p, g = v, b = t;
            break;
        case 3:
            r = p, g = q, b = v;
            break;
        case 4:
            r = t, g = p, b = v;
            break;
        case 5:
        default:
            r = v, g = p, b = q;
            break;
        }
    }

    void getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw)
    {
        pcl::getEulerAngles(t, roll, pitch, yaw);
        roll = pcl::rad2deg(roll);
        pitch = pcl::rad2deg(pitch);
        yaw = pcl::rad2deg(yaw);
    }

    void getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ)
    {
        Eigen::Matrix3f rotation_matrix = t.matrix().topLeftCorner(3, 3);
        Eigen::AngleAxisf angleAxis;
        angleAxis.fromRotationMatrix(rotation_matrix);
        Eigen::Vector3f axis(angleAxis.axis());
        angle = pcl::rad2deg(angleAxis.angle());
        axisX = axis[0];
        axisY = axis[1];
        axisZ = axis[2];
    }

    void getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z,
                                      float& roll, float& pitch, float& yaw)
    {
        pcl::getTranslationAndEulerAngles(t, x, y, z, roll, pitch, yaw);
        roll = pcl::rad2deg(roll);
        pitch = pcl::rad2deg(pitch);
        yaw = pcl::rad2deg(yaw);
    }

    void getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t)
    {
        pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw), t);
    }

    bool getTransformation(const std::string& text, Eigen::Affine3f& t)
    {
        std::vector<float> values;
        std::string token;
        std::istringstream iss(text);
        while (iss >> token)
        {
            for (char& ch : token)
            {
                if (ch == ',')
                    ch = ' ';
            }
            std::istringstream value_stream(token);
            float value = 0.f;
            while (value_stream >> value)
                values.push_back(value);
        }

        if (static_cast<int>(values.size()) == MATRIX_SIZE)
        {
            for (int r = 0, idx = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                    t.matrix()(r, c) = values[idx++];
            }
            if (t.matrix()(3, 3) != 1.f && t.matrix()(3, 3) != 0.f)
            {
                const float scale = 1.0f / t.matrix()(3, 3);
                for (int r = 0; r < 4; ++r)
                {
                    for (int c = 0; c < 4; ++c)
                        t.matrix()(r, c) *= scale;
                }
                t.matrix()(3, 3) = 1.f;
            }
            return true;
        }

        if (static_cast<int>(values.size()) == EULER_SIZE)
        {
            t = getTransformation(values[0], values[1], values[2], values[3], values[4], values[5]);
            return true;
        }
        return false;
    }

    std::string getTransformationString(const Eigen::MatrixXf& mat, int decimals)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(decimals);
        for (int i = 0; i < mat.rows(); ++i)
        {
            for (int j = 0; j < mat.cols(); ++j)
            {
                oss << mat(i, j);
                if (j + 1 < mat.cols())
                    oss << ' ';
            }
            if (i + 1 < mat.rows())
                oss << '\n';
        }
        return oss.str();
    }

    Eigen::Affine3f getTransformation(float x, float y, float z, float roll, float pitch, float yaw)
    {
        return pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw));
    }

    Eigen::Affine3f getTransformation(float angle, float axisX, float axisY, float axisZ, float x, float y, float z)
    {
        Eigen::AngleAxisf rotation_vector(pcl::deg2rad(angle), Eigen::Vector3f(axisX, axisY, axisZ));
        Eigen::Vector3f eulerAngle = rotation_vector.matrix().eulerAngles(0, 1, 2);
        return pcl::getTransformation(x, y, z, eulerAngle[0], eulerAngle[1], eulerAngle[2]);
    }

    Eigen::Matrix3f getRotationMatrix(float roll, float pitch, float yaw)
    {
        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(pcl::deg2rad(roll), Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(pcl::deg2rad(pitch), Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(pcl::deg2rad(yaw), Eigen::Vector3f::UnitZ()));
        return Eigen::Matrix3f(yawAngle * pitchAngle * rollAngle);
    }
}  // namespace ct
