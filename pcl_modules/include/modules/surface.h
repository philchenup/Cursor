/**
 * @file surface.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_SURFACE_H
#define CT_MODULES_SURFACE_H

#include "base/cloud.h"
#include "base/exports.h"

#include <pcl/PolygonMesh.h>

namespace ct
{
    typedef pcl::PolygonMesh PolygonMesh;

    class CT_EXPORT Surface
    {
    public:
        Surface() : cloud_(nullptr) {}

        /**
         * @brief 设置输入点云
         */
        void setInputCloud(const Cloud::Ptr& cloud) { cloud_ = cloud; }

        /**
         * @brief 基于局部 2D 投影的 3D 点的贪心三角剖分算法的实现
         * @param mesh 重建得到的网格
         * @param time 耗时（毫秒）
         */
        void GreedyProjectionTriangulation(double mu, int nnn, double radius, double min,
                                           double max, double ep, bool consistent,
                                           bool consistent_ordering,
                                           PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 网格投影面重建方法
         */
        void GridProjection(double resolution, int padding_size, int k, int max_binary_search_level,
                            PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 泊松曲面重建算法
         */
        void Poisson(int depth, int min_depth, float point_weight, float scale,
                     int solver_divide, int iso_divide, float samples_per_node,
                     bool confidence, bool output_polygons, bool manifold,
                     PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 行进立方体表面重建算法，使用基于径向基函数的有符号距离函数
         */
        void MarchingCubesRBF(float iso_level, int res_x, int res_y, int res_z,
                              float percentage, float epsilon,
                              PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 行进立方体表面重建算法，使用基于切平面距离的有符号距离函数
         */
        void MarchingCubesHoppe(float iso_level, int res_x, int res_y, int res_z,
                                float percentage, float dist_ignore,
                                PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief ConvexHull 使用 libqhull 库
         * @param value 如果设置为 true，则调用 qhull 库来计算凸包的总面积和体积
         * @param dimensio 设置输入数据的维度，2D 或 3D
         */
        void ConvexHull(bool value, int dimensio, PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 使用 libqhull 库的 ConcaveHull（alpha 形状）
         */
        void ConcaveHull(double alpha, bool value, int dimensio, PolygonMesh::Ptr& mesh, float& time);

        /**
         * @brief 剪耳三角测量算法
         */
        void EarClipping(PolygonMesh::Ptr& surface, PolygonMesh::Ptr& mesh, float& time);

    private:
        Cloud::Ptr cloud_;
    };
}  // namespace ct

#endif  // CT_MODULES_SURFACE_H
