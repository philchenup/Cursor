cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(filterreg_rigid LANGUAGES CXX)

# Prefer g++ when the default c++ is clang without libstdc++ quirks
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(WARNING "Clang detected; if link fails, reconfigure with -DCMAKE_CXX_COMPILER=g++")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Dependencies (no PCL / OpenCV / CUDA toolkit) ----
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})
find_package(Glog REQUIRED)

set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/eigen3)
set(CUDA_STUB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/cuda_stub)

include_directories(
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/external
	${EIGEN_INCLUDE_DIR}
	${CUDA_STUB_INCLUDE_DIR}
	${GLOG_INCLUDE_DIRS}
)

# ---- Libraries ----
add_library(common_lib
	common/data_transfer.cpp
	common/feature_channel_type.cpp
	common/feature_map.cpp
	common/geometric_target_interface.cpp
	common/tensor_blob.cpp
	common/tensor_utils.cpp
)
target_link_libraries(common_lib PUBLIC ${GLOG_LIBRARIES})

add_library(geometry_utils_lib
	geometry_utils/device_mat.cpp
	geometry_utils/device2eigen.cpp
)
target_link_libraries(geometry_utils_lib PUBLIC common_lib)

add_library(kinematic_base_lib
	kinematic/kinematic_model_base.cpp
)
target_link_libraries(kinematic_base_lib PUBLIC common_lib geometry_utils_lib)

add_library(rigid_kinematic_lib
	kinematic/rigid/rigid_kinematic_model.cpp
	kinematic/rigid/rigid_geometric_update.cpp
	kinematic/rigid/rigid_point2point_cpu.cpp
	kinematic/rigid/rigid_point2plane_cpu.cpp
	kinematic/rigid/rigid_point2point_kabsch.cpp
)
target_link_libraries(rigid_kinematic_lib PUBLIC kinematic_base_lib)

add_library(gmm_corr_lib
	corr_search/target_computer_base.cpp
	corr_search/gmm/gmm_permutohedral_updatedvar_pt2pt.cpp
	corr_search/gmm/gmm_permutohedral_updatedvar_pt2pl.cpp
)
target_link_libraries(gmm_corr_lib PUBLIC common_lib geometry_utils_lib)

add_library(io_lib
	io/pcd_io.cpp
)
target_link_libraries(io_lib PUBLIC common_lib)

add_library(dbg_visualizer_lib
	visualizer/debug_visualizer.cpp
)
target_link_libraries(dbg_visualizer_lib PUBLIC common_lib)

# ---- Apps ----
add_executable(rigid_pt2pt apps/rigid_pt2pt/main.cpp)
target_link_libraries(rigid_pt2pt
	common_lib
	geometry_utils_lib
	rigid_kinematic_lib
	gmm_corr_lib
	io_lib
	dbg_visualizer_lib
)

add_executable(rigid_pt2pl apps/rigid_pt2pl/main.cpp)
target_link_libraries(rigid_pt2pl
	common_lib
	geometry_utils_lib
	rigid_kinematic_lib
	gmm_corr_lib
	io_lib
	dbg_visualizer_lib
)

# Convenience: copy sample clouds next to binaries
add_custom_command(TARGET rigid_pt2pt POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_SOURCE_DIR}/apps/rigid_pt2pt/bunny.pcd
		$<TARGET_FILE_DIR:rigid_pt2pt>/bunny.pcd)
add_custom_command(TARGET rigid_pt2pl POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_SOURCE_DIR}/apps/rigid_pt2pl/cloud_0.pcd
		$<TARGET_FILE_DIR:rigid_pt2pl>/cloud_0.pcd
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_SOURCE_DIR}/apps/rigid_pt2pl/cloud_1.pcd
		$<TARGET_FILE_DIR:rigid_pt2pl>/cloud_1.pcd)
