# Source File List

本包实现 `rigid_pt2pt` / `rigid_pt2pl` 所需的全部源文件清单。
`external/eigen3/` 为完整 Eigen 头文件树（体积较大，此处不逐一列出）。

## Apps

- `apps/rigid_pt2pl/cloud_0.pcd`
- `apps/rigid_pt2pl/cloud_1.pcd`
- `apps/rigid_pt2pl/main.cpp`
- `apps/rigid_pt2pt/bunny.pcd`
- `apps/rigid_pt2pt/main.cpp`

## common

- `common/blob_access.h`
- `common/common_type.h`
- `common/data_transfer.cpp`
- `common/data_transfer.h`
- `common/feature_channel_type.cpp`
- `common/feature_channel_type.h`
- `common/feature_map.cpp`
- `common/feature_map.h`
- `common/geometric_target_interface.cpp`
- `common/geometric_target_interface.h`
- `common/macro_copyable.h`
- `common/safe_call_utils.h`
- `common/tensor_access.h`
- `common/tensor_blob.cpp`
- `common/tensor_blob.h`
- `common/tensor_utils.cpp`
- `common/tensor_utils.h`

## geometry_utils

- `geometry_utils/device2eigen.cpp`
- `geometry_utils/device2eigen.h`
- `geometry_utils/device_mat.cpp`
- `geometry_utils/device_mat.h`
- `geometry_utils/permutohedral_common.h`
- `geometry_utils/permutohedral_common.hpp`
- `geometry_utils/vector_operations.hpp`

## kinematic

- `kinematic/kinematic_model_base.cpp`
- `kinematic/kinematic_model_base.h`
- `kinematic/rigid/rigid.h`
- `kinematic/rigid/rigid_geometric_update.cpp`
- `kinematic/rigid/rigid_geometric_update.h`
- `kinematic/rigid/rigid_kinematic_model.cpp`
- `kinematic/rigid/rigid_kinematic_model.h`
- `kinematic/rigid/rigid_point2plane_cpu.cpp`
- `kinematic/rigid/rigid_point2plane_cpu.h`
- `kinematic/rigid/rigid_point2point_cpu.cpp`
- `kinematic/rigid/rigid_point2point_cpu.h`
- `kinematic/rigid/rigid_point2point_kabsch.cpp`
- `kinematic/rigid/rigid_point2point_kabsch.h`

## corr_search

- `corr_search/gmm/gmm.h`
- `corr_search/gmm/gmm_permutohedral_base.h`
- `corr_search/gmm/gmm_permutohedral_base.hpp`
- `corr_search/gmm/gmm_permutohedral_fixedvar_pt2pl.h`
- `corr_search/gmm/gmm_permutohedral_fixedvar_pt2pl.hpp`
- `corr_search/gmm/gmm_permutohedral_fixedvar_pt2pt.h`
- `corr_search/gmm/gmm_permutohedral_fixedvar_pt2pt.hpp`
- `corr_search/gmm/gmm_permutohedral_updatedvar_pt2pl.cpp`
- `corr_search/gmm/gmm_permutohedral_updatedvar_pt2pl.h`
- `corr_search/gmm/gmm_permutohedral_updatedvar_pt2pt.cpp`
- `corr_search/gmm/gmm_permutohedral_updatedvar_pt2pt.h`
- `corr_search/gmm/gmm_permutohedral_updatedvar_withblur.h`
- `corr_search/target_computer_base.cpp`
- `corr_search/target_computer_base.h`

## io

- `io/pcd_io.cpp`
- `io/pcd_io.h`

## stack_filter

- `stack_filter/example.cpp`
- `stack_filter/stacked_object_filter.cpp`
- `stack_filter/stacked_object_filter.h`

## visualizer

- `visualizer/debug_visualizer.cpp`
- `visualizer/debug_visualizer.h`

## external (non-Eigen)

- `external/cuda_stub/cublas_v2.h`
- `external/cuda_stub/cuda.h`
- `external/cuda_stub/cuda_runtime_api.h`
- `external/cuda_stub/vector_functions.h`
- `external/cuda_stub/vector_types.h`
- `external/nlohmann/json.hpp`

## cmake / build

- `CMakeLists.txt`
- `cmake/UtilFunctions.cmake`
- `cmake/modules/FindGlog.cmake`

## docs

- `.gitignore`
- `FILE_LIST.md`
- `README.md`
