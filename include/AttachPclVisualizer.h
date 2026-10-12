#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <string>

// VTK 9 的 QVTKOpenGLNativeWidget 不再提供 SetRenderWindow / GetInteractor /
// GetRenderWindow。窗口必须是 vtkGenericOpenGLRenderWindow，不能用
// PCLVisualizer 默认创建的 vtkWin32OpenGLRenderWindow。
inline pcl::visualization::PCLVisualizer::Ptr
attachPclVisualizer(QVTKOpenGLNativeWidget& widget, const std::string& name = "viewer")
{
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> window =
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(renderer, window, name, false));

    widget.setRenderWindow(window);
    viewer->setupInteractor(widget.interactor(), widget.renderWindow());
    return viewer;
}
