/**
 * @file cloud.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CLOUD_H
#define CT_BASE_CLOUD_H

#include "base/exports.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#define CLOUD_TYPE_XYZ "XYZ"
#define CLOUD_TYPE_XYZRGB "XYZRGB"
#define CLOUD_TYPE_XYZN "XYZNormal"
#define CLOUD_TYPE_XYZRGBN "XYZRGBNormal"

#define BOX_PRE_FLAG "-box"
#define NORMALS_PRE_FLAG "-normals"

namespace ct
{
    typedef pcl::Indices Indices;
    typedef pcl::console::TicToc TicToc;
    typedef pcl::PointXYZRGBNormal PointXYZRGBN;

    struct Box
    {
        double width;
        double height;
        double depth;
        Eigen::Affine3f pose;
        Eigen::Vector3f translation;
        Eigen::Quaternionf rotation;
    };

    struct RGB
    {
        RGB() : r(0), g(0), b(0) {}
        RGB(int r_, int g_, int b_) : r(r_), g(g_), b(b_) {}
        double rf() const { return static_cast<double>(r) / 255.0; }
        double gf() const { return static_cast<double>(g) / 255.0; }
        double bf() const { return static_cast<double>(b) / 255.0; }
        int r;
        int g;
        int b;
    };

    namespace Color
    {
        const RGB White = {255, 255, 255};
        const RGB Black = {0, 0, 0};
        const RGB Red = {255, 0, 0};
        const RGB Green = {0, 255, 0};
        const RGB Blue = {0, 0, 255};
        const RGB Yellow = {255, 255, 0};
        const RGB Cyan = {0, 255, 255};
        const RGB Purple = {255, 0, 255};
    }

    class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN>
    {
    public:
        Cloud()
            : m_id("cloud"),
              m_box_rgb(Color::White),
              m_normals_rgb(Color::White),
              m_type(CLOUD_TYPE_XYZ),
              m_path(),
              m_file_size(0),
              m_point_size(1),
              m_opacity(1.0f),
              m_resolution(0.0f)
        {
            m_box.width = 0.0;
            m_box.height = 0.0;
            m_box.depth = 0.0;
            m_box.pose = Eigen::Affine3f::Identity();
            m_box.translation = Eigen::Vector3f::Zero();
            m_box.rotation = Eigen::Quaternionf::Identity();
        }

        Cloud(const Cloud& cloud, const Indices& indices)
            : pcl::PointCloud<PointXYZRGBN>(cloud, indices),
              m_id(cloud.m_id),
              m_box(cloud.m_box),
              m_box_rgb(cloud.m_box_rgb),
              m_normals_rgb(cloud.m_normals_rgb),
              m_type(cloud.m_type),
              m_path(cloud.m_path),
              m_file_size(cloud.m_file_size),
              m_point_size(cloud.m_point_size),
              m_opacity(cloud.m_opacity),
              m_resolution(cloud.m_resolution),
              m_min(cloud.m_min),
              m_max(cloud.m_max)
        {
        }

        Cloud& operator+=(const Cloud& rhs)
        {
            concatenate((*this), rhs);
            return (*this);
        }

        Cloud operator+(const Cloud& rhs) { return (Cloud(*this) += rhs); }

        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        Ptr makeShared() const { return Ptr(new Cloud(*this)); }

        Box box() const { return m_box; }
        const std::string& id() const { return m_id; }
        std::string normalId() const { return m_id + NORMALS_PRE_FLAG; }
        std::string boxId() const { return m_id + BOX_PRE_FLAG; }
        RGB boxColor() const { return m_box_rgb; }
        RGB normalColor() const { return m_normals_rgb; }
        Eigen::Vector3f center() const { return m_box.translation; }
        const std::string& type() const { return m_type; }
        const std::string& path() const { return m_path; }
        PointXYZRGBN min() const { return m_min; }
        PointXYZRGBN max() const { return m_max; }
        int fileSize() const { return m_file_size; }
        int pointSize() const { return m_point_size; }
        float opacity() const { return m_opacity; }
        float resolution() const { return m_resolution; }
        float volume() const { return static_cast<float>(m_box.depth * m_box.height * m_box.width); }

        bool hasNormals() const
        {
            if (empty())
                return false;
            const PointXYZRGBN& p = points[rand() % size()];
            return p.normal_x != 0.0f || p.normal_y != 0.0f || p.normal_z != 0.0f;
        }

        void setId(const std::string& id) { m_id = id; }
        void setBox(const Box& box) { m_box = box; }
        void setInfo(const std::string& path, int file_size_kb = 0)
        {
            m_path = path;
            m_file_size = file_size_kb;
        }
        void setPointSize(int point_size) { m_point_size = point_size; }
        void setCloudColor(const RGB& rgb);
        void setCloudColor(const std::string& axis);
        void setBoxColor(const RGB& rgb) { m_box_rgb = rgb; }
        void setNormalColor(const RGB& rgb) { m_normals_rgb = rgb; }
        void setOpacity(float opacity) { m_opacity = opacity; }

        /**
         * @brief 按照维度(x,y,z)缩放点云尺寸
         * @param origin 是否以坐标原点为缩放中心
         */
        void scale(double x, double y, double z, bool origin = false);

        /**
         * @brief 更新点云
         * @param type_flag 是否更新点云类型
         * @param box_flag 是否更新包围盒
         * @param resolution_flag 是否更新分辨率
         */
        void update(bool box_flag = true, bool type_flag = true, bool resolution_flag = true);

    private:
        Box m_box;
        std::string m_id;
        RGB m_box_rgb;
        RGB m_normals_rgb;
        std::string m_type;
        std::string m_path;
        int m_file_size;
        int m_point_size;
        float m_opacity;
        float m_resolution;
        PointXYZRGBN m_min;
        PointXYZRGBN m_max;
    };
}  // namespace ct

#endif  // CT_BASE_CLOUD_H
