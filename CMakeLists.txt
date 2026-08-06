cmake_minimum_required(VERSION 3.10)
project(pcl_vtk_ortho_views LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)
find_package(VTK REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ---------- 键盘 / 多视口版 ----------
add_executable(pcl_vtk_ortho_views src/pcl_vtk_ortho_views.cpp)
target_include_directories(pcl_vtk_ortho_views PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_vtk_ortho_views PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(pcl_vtk_ortho_views PRIVATE ${PCL_DEFINITIONS})
if(PCL_COMPILE_OPTIONS)
  target_compile_options(pcl_vtk_ortho_views PRIVATE ${PCL_COMPILE_OPTIONS})
endif()

# ---------- 三按钮 Qt 版（正视 / 俯视 / 左视）----------
add_executable(pcl_vtk_view_buttons src/pcl_vtk_view_buttons.cpp)
target_include_directories(pcl_vtk_view_buttons PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
target_link_libraries(pcl_vtk_view_buttons PRIVATE
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  Qt5::Widgets
)
target_compile_definitions(pcl_vtk_view_buttons PRIVATE ${PCL_DEFINITIONS})
if(PCL_COMPILE_OPTIONS)
  target_compile_options(pcl_vtk_view_buttons PRIVATE ${PCL_COMPILE_OPTIONS})
endif()

# VTK 9 模块自动初始化（若可用）
if(COMMAND vtk_module_autoinit)
  vtk_module_autoinit(
    TARGETS pcl_vtk_view_buttons
    MODULES ${VTK_LIBRARIES}
  )
endif()
