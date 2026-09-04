/**
 * @file demo_cloud.cpp
 * @brief cloud.h 中每个公开接口的示例。
 */
#include "demo_utils.h"

#include <iostream>
#include <string>

namespace
{
    void demo_makeShared_and_ids()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(10, 0.05f);
        cloud->setId("source");
        ct::Cloud::Ptr copy = cloud->makeShared();
        demo::log("makeShared/id",
                  "id=" + copy->id() +
                      " normalId=" + copy->normalId() +
                      " boxId=" + copy->boxId());
    }

    void demo_setInfo_and_accessors()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(8, 0.05f);
        cloud->setInfo("C:/data/demo.pcd", 128);
        cloud->setPointSize(3);
        cloud->setOpacity(0.8f);
        cloud->setBoxColor(ct::Color::Yellow);
        cloud->setNormalColor(ct::Color::Cyan);
        std::cout << "[ setInfo/accessors ] path=" << cloud->path()
                  << " fileSize=" << cloud->fileSize()
                  << " pointSize=" << cloud->pointSize()
                  << " opacity=" << cloud->opacity()
                  << " type=" << cloud->type()
                  << " hasNormals=" << cloud->hasNormals()
                  << " volume=" << cloud->volume() << std::endl;
    }

    void demo_setCloudColor_rgb()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(8, 0.05f);
        cloud->setCloudColor(ct::Color::Red);
        const auto& p = cloud->points.front();
        demo::log("setCloudColor(RGB)",
                  "r=" + std::to_string(p.r) + " g=" + std::to_string(p.g) + " b=" + std::to_string(p.b));
    }

    void demo_setCloudColor_axis()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(8, 0.05f);
        cloud->update();
        cloud->setCloudColor("z");
        demo::log("setCloudColor(axis)", "colored by z axis");
    }

    void demo_update_box_center()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(12, 0.04f);
        cloud->update(true, true, true);
        const ct::Box box = cloud->box();
        const Eigen::Vector3f c = cloud->center();
        const ct::PointXYZRGBN mn = cloud->min();
        const ct::PointXYZRGBN mx = cloud->max();
        std::cout << "[ update/box ] whd=" << box.width << "x" << box.height << "x" << box.depth
                  << " center=(" << c.x() << "," << c.y() << "," << c.z() << ")"
                  << " min=(" << mn.x << "," << mn.y << "," << mn.z << ")"
                  << " max=(" << mx.x << "," << mx.y << "," << mx.z << ")"
                  << " resolution=" << cloud->resolution() << std::endl;
    }

    void demo_setBox()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(6, 0.05f);
        ct::Box box = cloud->box();
        box.width *= 1.1;
        cloud->setBox(box);
        demo::log("setBox", "width=" + std::to_string(cloud->box().width));
    }

    void demo_scale()
    {
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(8, 0.05f);
        const float x0 = cloud->points.front().x;
        cloud->scale(2.0, 1.0, 1.0, false);
        demo::log("scale",
                  "first.x before~0, after=" + std::to_string(cloud->points.front().x) +
                      " (origin scale, x0=" + std::to_string(x0) + ")");
    }

    void demo_plus_operators()
    {
        ct::Cloud a = *demo::makePlaneCloud(6, 0.05f);
        ct::Cloud b = *demo::makeSphereCloud(8, 8, 0.2f);
        ct::Cloud sum = a + b;
        a += b;
        demo::log("operator+/+=",
                  "sum.size=" + std::to_string(sum.size()) +
                      " a.size=" + std::to_string(a.size()));
    }

    void demo_index_constructor()
    {
        ct::Cloud::Ptr src = demo::makePlaneCloud(10, 0.05f);
        ct::Indices ids;
        for (int i = 0; i < 20 && i < static_cast<int>(src->size()); ++i)
            ids.push_back(i);
        ct::Cloud subset(*src, ids);
        demo::log("Cloud(cloud, indices)", "subset.size=" + std::to_string(subset.size()));
    }
}  // namespace

int main()
{
    std::cout << "======== demo_cloud (cloud.h) ========\n";
    demo_makeShared_and_ids();
    demo_setInfo_and_accessors();
    demo_setCloudColor_rgb();
    demo_setCloudColor_axis();
    demo_update_box_center();
    demo_setBox();
    demo_scale();
    demo_plus_operators();
    demo_index_constructor();
    std::cout << "done.\n";
    return 0;
}
