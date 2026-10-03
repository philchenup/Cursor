#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 同一平面上被缺口切开的两块连通域。
// 只在两块正对的重叠段里补点，新点落在拟合平面上，不向外扩。
// spacing 是点距；cluster_tolerance 要大于点距、小于缺口宽度。
// 返回补上的点，不含原来的点。分不出两块连通域时返回空云。
pcl::PointCloud<pcl::PointXYZ>::Ptr fillPlanarGap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    float spacing,
    float cluster_tolerance);
