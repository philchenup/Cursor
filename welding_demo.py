#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Weld groove detection and trajectory planning (ROS-free).

This script keeps the original robotic-welding-demo logic:

* convert / load an Open3D point cloud
* find the corresponding groove
* compute the trajectory
* multilayer planning

ROS topics, messages, and rospy have been removed. Point clouds are loaded
from files (PCD/PLY/XYZ). Universal Robots motion via ``urx`` is optional.
"""

from __future__ import print_function

import argparse
import copy
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
from scipy import interpolate
from scipy import spatial
from scipy.spatial.transform import Rotation as R

try:
    import urx
except ImportError:
    urx = None


# == COMPATIBILITY HELPERS ============================================================================================

def _angle(v1, v2):
    """Angle in degrees between two vectors (replacement for vg.angle)."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 == 0 or n2 == 0:
        return 0.0
    cosang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cosang)))


def _select_by_index(pcd, indices):
    """Open3D select_down_sample / select_by_index compatibility."""
    indices = np.asarray(indices, dtype=np.int64)
    if hasattr(pcd, "select_by_index"):
        return pcd.select_by_index(indices)
    return pcd.select_down_sample(indices)


def _rotation_from_matrix(matrix):
    matrix = np.asarray(matrix)
    if hasattr(R, "from_matrix"):
        return R.from_matrix(matrix)
    return R.from_dcm(matrix)


def _as_matrix(rotation):
    if hasattr(rotation, "as_matrix"):
        return rotation.as_matrix()
    return rotation.as_dcm()


# Module-level parameters (same defaults as the original ROS node)
voxel_size = 0.005
neighbor = 5 * voxel_size
delete_percentage = 0.95
max_dis = 0.7
total_time = []
capture_number = 0
start = 0.0
is_first_pose = True
is_sec_pose = False
first_pose = None
last_pose = None
vel_vector = None


# == FUNCTIONS ========================================================================================================

# Takes points in [[x1, y1, z1], [x2, y2, z2]...] Numpy Array format
def thin_line(points, point_cloud_thickness=0.5, iterations=1, sample_points=0):
    if sample_points != 0:
        points = points[:sample_points]

    # Sort points into KDTree for nearest neighbors computation later
    point_tree = spatial.cKDTree(points)

    # Empty array for transformed points
    new_points = []
    # Empty array for regression lines corresponding ^^ points
    regression_lines = []
    for point in point_tree.data:
        # Get list of points within specified radius {point_cloud_thickness}
        points_in_radius = point_tree.data[point_tree.query_ball_point(point, point_cloud_thickness)]

        # Get mean of points within radius
        data_mean = points_in_radius.mean(axis=0)

        # Calulate 3D regression line/principal component in point form with 2 coordinates
        uu, dd, vv = np.linalg.svd(points_in_radius - data_mean)
        linepts = vv[0] * np.mgrid[-1:1:2j][:, np.newaxis]
        linepts += data_mean
        regression_lines.append(list(linepts))

        # Project original point onto 3D regression line
        ap = point - linepts[0]
        ab = linepts[1] - linepts[0]
        point_moved = linepts[0] + np.dot(ap, ab) / np.dot(ab, ab) * ab

        new_points.append(list(point_moved))
    return np.array(new_points), regression_lines


# Sorts points outputed from thin_points()s
def sort_points(points, regression_lines, sorted_point_distance=0.01):
    sort_points_time = time.time()
    # Index of point to be sorted
    index = 0

    # sorted points array for left and right of intial point to be sorted
    sort_points_left = [points[index]]
    sort_points_right = []

    # Regression line of previously sorted point
    regression_line_prev = regression_lines[index][1] - regression_lines[index][0]

    # Sort points into KDTree for nearest neighbors computation later
    point_tree = spatial.cKDTree(points)
    visited_left = {0}

    # Iterative add points sequentially to the sort_points_left array
    while 1:
        # Calulate regression line vector; makes sure line vector is similar direction as previous regression line
        v = regression_lines[index][1] - regression_lines[index][0]
        if np.dot(regression_line_prev, v) / (np.linalg.norm(regression_line_prev) * np.linalg.norm(v)) < 0:
            v = regression_lines[index][0] - regression_lines[index][1]
        regression_line_prev = v

        # Find point {distR_point} on regression line distance {sorted_point_distance} from original point
        distR_point = points[index] + ((v / np.linalg.norm(v)) * sorted_point_distance)

        # Search nearest neighbors of distR_point within radius {sorted_point_distance / 3}
        points_in_radius = point_tree.data[point_tree.query_ball_point(distR_point, sorted_point_distance / 1.5)]
        if len(points_in_radius) < 1:
            break

        # Neighbor of distR_point with smallest angle to regression line vector is selected as next point in order
        nearest_point = points_in_radius[0]
        distR_point_vector = distR_point - points[index]
        nearest_point_vector = nearest_point - points[index]
        for x in points_in_radius:
            x_vector = x - points[index]
            if _angle(distR_point_vector, x_vector) < _angle(distR_point_vector, nearest_point_vector):
                nearest_point_vector = nearest_point - points[index]
                nearest_point = x
        matches = np.where(np.all(np.isclose(points, nearest_point), axis=1))[0]
        if len(matches) == 0:
            break
        new_index = int(matches[0])
        if new_index == index or new_index in visited_left:
            break
        visited_left.add(new_index)
        index = new_index

        # Add nearest point to 'sort_points_left' array
        sort_points_left.append(nearest_point)
        if len(sort_points_left) >= len(points):
            break

    # Do it again but in the other direction of initial starting point
    index = 0
    visited_right = {0}
    regression_line_prev = regression_lines[index][1] - regression_lines[index][0]
    while 1:
        # Calulate regression line vector; makes sure line vector is similar direction as previous regression line
        v = regression_lines[index][1] - regression_lines[index][0]
        if np.dot(regression_line_prev, v) / (np.linalg.norm(regression_line_prev) * np.linalg.norm(v)) < 0:
            v = regression_lines[index][0] - regression_lines[index][1]
        regression_line_prev = v

        # Find point {distR_point} on regression line distance {sorted_point_distance} from original point
        # Now vector is substracted from the point to go in other direction
        distR_point = points[index] - ((v / np.linalg.norm(v)) * sorted_point_distance)

        # Search nearest neighbors of distR_point within radius {sorted_point_distance / 3}
        points_in_radius = point_tree.data[point_tree.query_ball_point(distR_point, sorted_point_distance / 3)]
        if len(points_in_radius) < 1:
            break

        # Neighbor of distR_point with smallest angle to regression line vector is selected as next point in order
        nearest_point = points_in_radius[0]
        distR_point_vector = distR_point - points[index]
        nearest_point_vector = nearest_point - points[index]
        for x in points_in_radius:
            x_vector = x - points[index]
            if _angle(distR_point_vector, x_vector) < _angle(distR_point_vector, nearest_point_vector):
                nearest_point_vector = nearest_point - points[index]
                nearest_point = x
        matches = np.where(np.all(np.isclose(points, nearest_point), axis=1))[0]
        if len(matches) == 0:
            break
        new_index = int(matches[0])
        if new_index == index or new_index in visited_right:
            break
        visited_right.add(new_index)
        index = new_index

        # Add next point to 'sort_points_right' array
        sort_points_right.append(nearest_point)
        if len(sort_points_right) >= len(points):
            break

    # Combine 'sort_points_right' and 'sort_points_left'
    sort_points_right = sort_points_right[::-1]
    sort_points_right.extend(sort_points_left)
    sort_points_right = np.flip(sort_points_right, 0)
    print("--- %s seconds to sort points ---" % (time.time() - sort_points_time))
    return np.array(sort_points_right)


def generate_new_trajectory(pcd, groove, normal):

    points = np.asarray(groove.points)

    # Thin & sort points
    thinned_points, regression_lines = thin_line(points)
    sorted_points = sort_points(thinned_points, regression_lines)

    draw = False

    if draw is True:

        # Run thinning and sorting algorithms
        # Plotting
        fig2 = plt.figure(2)
        ax3d = fig2.add_subplot(111, projection="3d")

        # Plot unordedered point cloud
        ax3d.plot(points.T[0], points.T[1], points.T[2], "m*")

        # Plot sorted points
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], "bo")

        # Plot line going through sorted points
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], "-b")

        fig2.show()
        plt.show()

    x = sorted_points[:, 0]
    y = sorted_points[:, 1]
    z = sorted_points[:, 2]
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], s=float("inf"), full_output=1)
    # Generate 5x points from approximated B-spline for drawing curve later
    u_fine = np.linspace(0, 1, x.size * 2)

    # Evaluate points on B-spline
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    sorted_points = np.vstack((x_fine, y_fine, z_fine)).T
    point_size_line = sorted_points.shape[0]
    start_point = sorted_points[0]
    end_point = sorted_points[point_size_line - 1]
    middle_point = (start_point + end_point) / 2
    vec = end_point - middle_point
    proj = np.dot(vec, normal) * normal
    vec = vec - proj
    print(np.dot(vec, normal))
    displacemnt = []

    for i in np.linspace(-1, 1, x.size):
        displacemnt.append(i * vec)

    sorted_points = np.array(np.add(displacemnt, middle_point))

    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(sorted_points)

    return trajectory_pcd


def generate_trajectory(pcd, groove):

    points = np.asarray(groove.points)

    # Thin & sort points
    thinned_points, regression_lines = thin_line(points)
    sorted_points = sort_points(thinned_points, regression_lines)

    draw = False

    if draw is True:

        fig2 = plt.figure(2)
        ax3d = fig2.add_subplot(111, projection="3d")

        ax3d.plot(points.T[0], points.T[1], points.T[2], "m*")
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], "bo")
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], "-b")

        fig2.show()
        plt.show()

    x = sorted_points[:, 0]
    y = sorted_points[:, 1]
    z = sorted_points[:, 2]
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], s=float("inf"), full_output=1)
    u_fine = np.linspace(0, 1, x.size * 2)

    # Evaluate points on B-spline
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    sorted_points = np.vstack((x_fine, y_fine, z_fine)).T

    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(sorted_points)

    return trajectory_pcd


def load_point_cloud(path):
    """Load a point cloud from disk (replaces convertCloudFromRosToOpen3d)."""
    open3d_cloud = o3d.io.read_point_cloud(path)
    if len(open3d_cloud.points) == 0:
        print("Converting an empty cloud")
        return None
    return open3d_cloud


def numpy_to_point_cloud(points, colors=None):
    """Build an Open3D cloud from an Nx3 array (replaces ROS PointCloud2 conversion)."""
    open3d_cloud = o3d.geometry.PointCloud()
    points = np.asarray(points)
    if points.size == 0:
        print("Converting an empty cloud")
        return None
    open3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
    if colors is not None:
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.asarray(colors))
    return open3d_cloud


def transform_cam_wrt_base(pcd, T_end_effector_wrt_base):

    T_cam_wrt_end_effector = np.array([[-0.02160632, -0.97207334, -0.23368052, 0.122350972619],
                                       [0.98357004, 0.02123446, -0.17927374, -0.08164344],
                                       [0.17922931, -0.2337146, 0.95564342, 0.1156235],
                                       [0., 0., 0., 1.]])

    pcd_copy1 = copy.deepcopy(pcd).transform(T_cam_wrt_end_effector)
    pcd_copy1.paint_uniform_color([0.5, 0.5, 1])

    pcd_copy2 = copy.deepcopy(pcd_copy1).transform(T_end_effector_wrt_base)
    pcd_copy2.paint_uniform_color([1, 0, 0])
    return pcd_copy2


def normalise_feautre(feautre_value_list):

    normalised_feautre_value_list = (feautre_value_list - feautre_value_list.min()) / (
        feautre_value_list.max() - feautre_value_list.min())
    return np.array(normalised_feautre_value_list)


# find feature value list
def find_feature_value(feature, pcd, voxel_size):

    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    pc_number = np.asarray(pcd.points).shape[0]
    feautre_value_list = []

    n_list = np.asarray(pcd.normals)

    if feature == "asymmetry":
        neighbor_count = min(pc_number // 100, 30)
        for index in range(pc_number):
            [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[index], neighbor_count)
            vector = np.mean(n_list[idx, :], axis=0)
            feature_value = np.linalg.norm(
                vector - n_list[index, :] * np.dot(vector, n_list[index, :]) / np.linalg.norm(n_list[index, :]))
            feautre_value_list.append(feature_value)

    return np.array(feautre_value_list)


def cluster_groove_from_point_cloud(pcd_selected, voxel_size, verbose=False):

    global neighbor

    labels = np.array(pcd_selected.cluster_dbscan(eps=neighbor, min_points=20, print_progress=verbose))
    max_label = labels.max()  # noqa: F841  (kept from original)

    label, label_counts = np.unique(labels, return_counts=True)
    label_number = label[np.argsort(label_counts)[-1]]

    if label_number == -1:
        if label.shape[0] > 1:
            label_number = label[np.argsort(label_counts)[-2]]
        elif label.shape[0] == 1:
            print("can not find a valid groove cluster")

    groove_index = np.where(labels == label_number)
    groove = _select_by_index(pcd_selected, groove_index[0])

    return groove


def save_pcd(pcd):
    now = datetime.now()
    python_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__))) + "/"
    output_filename = python_file_path + "pc" + str(capture_number) + str(now) + ".pcd"
    o3d.io.write_point_cloud(output_filename, pcd)
    print("Saved point cloud to {}".format(output_filename))
    return output_filename


def find_normal(trajectory, pcd):

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.003, ransac_n=20, num_iterations=100)
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c])
    print("\n\n\n============")
    print(plane_model)

    trajectory_points = np.asarray(trajectory.points)
    pcd_points = np.asarray(pcd.points)
    trajectory_number = np.array(trajectory_points).shape[0]
    total = np.concatenate((trajectory_points, pcd_points), axis=0)
    total_pcd = o3d.geometry.PointCloud()
    total_pcd.points = o3d.utility.Vector3dVector(total)
    total_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=300))
    total_pcd.normalize_normals()
    total_pcd.orient_normals_to_align_with_direction(orientation_reference=-plane_normal)

    selected_pcd = _select_by_index(total_pcd, range(trajectory_number))
    points = np.asarray(selected_pcd.points)  # noqa: F841  (kept from original)
    normals = np.asarray(selected_pcd.normals)
    normal = np.mean(normals, axis=0)
    return normal


def points_in_cylinder(pt1, pt2, r, query_points):
    """
    to check if a query point is in the cylinder definded as
    @param pt1:
    @param pt2:
    @param q: query points
    @return: points inside the cylinder
    """
    vec = pt2 - pt1
    const = r * np.linalg.norm(vec)
    points_lst = []
    for q in query_points:
        flag = np.where(
            np.dot(q - pt1, vec) >= 0 and np.dot(q - pt2, vec) <= 0 and np.linalg.norm(np.cross(q - pt1, vec)) <= const,
            True,
            False)
        if flag:
            points_lst.append(q)
        else:
            pass
    return points_lst


def find_orientation(trajectory, pcd, groove, normal):
    """Compute torch poses along the trajectory.

    ROS publishers (first_pose, last_pose, vel_vector, PoseArray, MarkerArray)
    are replaced by printed values and returned numpy arrays.
    """
    global is_first_pose, is_sec_pose, first_pose, last_pose, vel_vector

    points = np.asarray(trajectory.points)

    rotvecs = []
    pose_list = []

    z_dir = normal
    pos_diff = points[1] - points[0]
    proj = np.dot(pos_diff, z_dir) * z_dir
    x_dir = pos_diff - proj
    x_dir = x_dir / np.linalg.norm(x_dir, axis=0)
    y_dir = np.cross(z_dir, x_dir)
    y_dir = y_dir / np.linalg.norm(y_dir, axis=0)
    r = _rotation_from_matrix(np.vstack((-y_dir, x_dir, z_dir)).T)
    orientation = r.as_quat()  # x, y, z, w
    rotvec = r.as_rotvec()

    for i in range(np.array(points).shape[0]):

        if is_first_pose:
            first_pose = {
                "position": points[i][0:3].copy(),
                "orientation": orientation.copy(),
            }
            print("first pose:")
            print(first_pose)
            is_first_pose = False
            is_sec_pose = True

        if is_first_pose is False and is_sec_pose is True:
            first_point = points[i - 1][0:3]
            sec_point = points[i][0:3]
            velocity_vec = sec_point - first_point
            print("velocity vector:")
            print(velocity_vec)
            vel_vector = -velocity_vec
            is_sec_pose = False

        if i == np.array(points).shape[0] - 1:
            last_pose = {
                "position": points[i][0:3].copy(),
                "orientation": orientation.copy(),
            }
            print("last pose:")
            print(last_pose)

        pose_list.append({
            "position": points[i][0:3].copy(),
            "orientation": orientation.copy(),
            "index": i,
        })
        rotvecs.append(rotvec)

    ur_poses = np.hstack((points, np.array(rotvecs)))
    return ur_poses


def trajectory_execution(robot, pose_list):
    print("\nPress `Enter` to execute or q to quit: ")
    if not sys.stdin.readline().strip() == "q":
        tcp_torch = [-0.0002, -0.09216, 0.32202, 0, 0, 0]
        robot.set_tcp(tcp_torch)
        time.sleep(0.2)  # pause is essentail for tcp to take effect, min time is 0.1s

        robot.movel(pose_list[0], acc=0.1, vel=0.1, wait=True)

        print("start process")
        robot.set_digital_out(0, True)
        time.sleep(0)
        robot.movels(pose_list, acc=0.015, vel=0.03, wait=True)
        print("stop process")

        robot.set_digital_out(0, False)
        robot.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)


def mutilayer(poses):

    z_height = -0.004  # m
    y_height = -0.006
    r_origin = R.from_rotvec(poses[0][3:])
    Rot_matrix = _as_matrix(r_origin)
    new_y = Rot_matrix[:, 1]
    new_z = Rot_matrix[:, 2]
    z_offset = new_z * z_height
    y_offset = new_y * y_height

    angle = math.atan(y_height / z_height)
    left_angle = angle / 2 - np.pi / 4
    poses_copy_left = copy.deepcopy(poses)
    poses_copy_right = copy.deepcopy(poses)

    left_poses = []
    for left_ur_pose in poses_copy_left:
        left_ur_pose[0] = left_ur_pose[0] + z_offset[0] + y_offset[0]
        left_ur_pose[1] = left_ur_pose[1] + z_offset[1] + y_offset[1]
        left_ur_pose[2] = left_ur_pose[2] + z_offset[2] + y_offset[2]
        r_orien_left = R.from_euler("x", left_angle, degrees=False)
        r_left = r_orien_left * r_origin
        left_orientation = r_left.as_rotvec()
        left_ur_pose[3:] = left_orientation
        left_poses.append(left_ur_pose)

    right_angle = -left_angle

    right_poses = []
    for right_ur_pose in poses_copy_right:
        right_ur_pose[0] = right_ur_pose[0] + z_offset[0] - y_offset[0]
        right_ur_pose[1] = right_ur_pose[1] + z_offset[1] - y_offset[1]
        right_ur_pose[2] = right_ur_pose[2] + z_offset[2] - y_offset[2]
        r_orien_right = R.from_euler("x", right_angle, degrees=False)
        r_right = r_orien_right * r_origin
        right_orientation = r_right.as_rotvec()
        right_ur_pose[3:] = right_orientation
        right_poses.append(right_ur_pose)

    return left_poses, right_poses


def uplift_z(ur_poses):

    r = R.from_rotvec(ur_poses[0][3:])
    Rot_matrix = _as_matrix(r)
    new_z = Rot_matrix[:, 2]
    new_y = Rot_matrix[:, 1]
    offset_z = -0.005
    offset_y = 0
    displacement_z = offset_z * new_z
    displacement_y = offset_y * new_y
    new_ur_poses = []
    for urpose in ur_poses:
        urpose[0] = urpose[0] + displacement_z[0] + displacement_y[0]
        urpose[1] = urpose[1] + displacement_z[1] + displacement_y[1]
        urpose[2] = urpose[2] + displacement_z[2] + displacement_y[2]
        new_ur_poses.append(urpose)
    return new_ur_poses


def detect_groove_workflow(pcd, transfromation_end_to_base, detect_feature="asymmetry",
                           show_groove=False, publish=True, save_data=True, apply_cam_to_base=True):

    # 2.downsample of point cloud

    global max_dis, total_time, voxel_size, delete_percentage

    original_pcd = pcd

    voxel_size = 0.005
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    pcd_points = np.asarray(pcd.points)
    pcd.clear()
    pcd_points = pcd_points[pcd_points[:, 2] < max_dis]
    pcd.points = o3d.utility.Vector3dVector(pcd_points)

    pcd.remove_non_finite_points()
    pc_number = np.asarray(pcd.points).shape[0]
    print("Total number of pc {}".format(pc_number))

    # 3.estimate normal toward cam location and normalise it
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.01, max_nn=30))
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location(camera_location=[0., 0., 0.])

    # 4.use different geometry features to find groove
    feautre_value_list = find_feature_value(detect_feature, pcd, voxel_size)
    normalised_feautre_value_list = normalise_feautre(feautre_value_list)

    # 5.delete low value points and cluster
    delete_points = int(pc_number * delete_percentage)
    pcd_selected = _select_by_index(pcd, np.argsort(normalised_feautre_value_list)[delete_points:])
    groove = cluster_groove_from_point_cloud(pcd_selected, voxel_size)

    groove_t = time.time()
    print("Runtime of groove detection is {}".format(groove_t - start))

    if apply_cam_to_base:
        pcd = transform_cam_wrt_base(pcd, transfromation_end_to_base)
        groove = transform_cam_wrt_base(groove, transfromation_end_to_base)

    trajectory = generate_trajectory(pcd, groove)
    normal = find_normal(trajectory, pcd)
    points = np.asarray(trajectory.points)
    point_size_line = points.shape[0]
    start_point = points[0]
    end_point = points[point_size_line - 1]

    refined_groove = points_in_cylinder(start_point, end_point, voxel_size * 5, np.asarray(groove.points))
    refined_groove_pcd = o3d.geometry.PointCloud()
    refined_groove_pcd.points = o3d.utility.Vector3dVector(refined_groove)
    refined_groove_pcd, _ = refined_groove_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    new_trajectory = generate_new_trajectory(pcd, refined_groove_pcd, normal)
    ur_poses = find_orientation(new_trajectory, pcd, refined_groove_pcd, normal)
    groove = refined_groove_pcd

    traj_t = time.time()
    print("Runtime of trajectory is {}".format(traj_t - groove_t))

    if publish:
        # Original code published PointCloud2 on ROS topics. Without ROS, paint
        # colors for optional Open3D visualization / file export instead.
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
        groove.paint_uniform_color([1, 0, 0])
        trajectory.paint_uniform_color([0, 1, 0])
        new_trajectory.paint_uniform_color([0, 0, 1])
        print("Conversion success (ROS publish skipped) ...\n")

    if show_groove:
        o3d.visualization.draw_geometries([pcd, groove, trajectory, new_trajectory])

    if save_data:
        save_pcd(original_pcd)

    end = time.time()
    print("Runtime of process is {}".format(end - start))
    total_time.append(end - start)
    print("Total process {}".format(np.array(total_time).shape[0]))
    print("Average Runtime of process is {}".format(np.mean(total_time)))

    return ur_poses


def _end_effector_matrix(robot):
    if robot is None:
        return np.eye(4)
    pose = robot.get_pose()
    if hasattr(pose, "array"):
        return np.asarray(pose.array)
    return np.asarray(pose)


def _connect_robot(robot_ip):
    if not robot_ip:
        return None
    if urx is None:
        print("urx is not installed; skip robot connection")
        return None
    print("Connecting to UR robot at {}".format(robot_ip))
    return urx.Robot(robot_ip)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Weld groove detection and trajectory planning without ROS.")
    parser.add_argument("cloud", help="Input point cloud file (.pcd / .ply / .xyz)")
    parser.add_argument("--robot-ip", default=None, help="UR controller IP (enables cam-to-base transform and motion)")
    parser.add_argument("--execute", action="store_true", help="Execute the planned trajectory on the robot")
    parser.add_argument("--multilayer", action="store_true", help="Also execute left/right multilayer passes")
    parser.add_argument("--show", action="store_true", help="Visualize groove and trajectory with Open3D")
    parser.add_argument("--no-save", action="store_true", help="Do not write a copy of the input cloud")
    parser.add_argument("--max-dis", type=float, default=0.7, help="Keep points with z < this (camera frame, meters)")
    parser.add_argument("--no-transform", action="store_true",
                        help="Keep the cloud in its original frame (skip camera-to-base)")
    parser.add_argument("--out-csv", default=None, help="Optional path to write x,y,z,rx,ry,rz poses")
    return parser.parse_args(argv)


def main(argv=None):
    global received_open3d_cloud, delete_percentage, max_dis, total_time
    global voxel_size, neighbor, is_first_pose, is_sec_pose, start, capture_number

    args = parse_args(argv)

    is_first_pose = True
    is_sec_pose = False
    voxel_size = 0.005
    neighbor = 5 * voxel_size
    delete_percentage = 0.95
    max_dis = args.max_dis
    total_time = []
    capture_number = 1

    robot = _connect_robot(args.robot_ip)

    horizontal_plane = [-1.067366902028219, -0.8888581434832972, -2.384127680455343,
                        0.12086498737335205, 1.0657631158828735, -1.4749344030963343]  # noqa: F841
    normal_plane = [-1.5366695562945765, -1.6485264937030237, -1.1686652342425745,
                    -1.7946108023272913, 1.4708768129348755, -1.4940932432757776]
    startj = normal_plane

    if robot is not None:
        robot.movej(startj, acc=0.8, vel=0.4, wait=True)
        time.sleep(0.5)
        robot.set_tcp((0, 0, 0, 0, 0, 0))
        time.sleep(0.3)

    print("starting, please don't move the workpiece")
    received_open3d_cloud = load_point_cloud(args.cloud)
    if received_open3d_cloud is None:
        print("Failed to load point cloud: {}".format(args.cloud))
        if robot is not None:
            robot.stop()
            robot.close()
        return 1

    print("\n==start seam detection===")
    start = time.time()
    T_end_effector_wrt_base = _end_effector_matrix(robot)
    apply_cam_to_base = (robot is not None) and (not args.no_transform)

    ur_poses = detect_groove_workflow(
        received_open3d_cloud,
        T_end_effector_wrt_base,
        detect_feature="asymmetry",
        show_groove=args.show,
        publish=True,
        save_data=not args.no_save,
        apply_cam_to_base=apply_cam_to_base,
    )

    if args.out_csv:
        np.savetxt(args.out_csv, ur_poses, delimiter=",", header="x,y,z,rx,ry,rz", comments="")
        print("Wrote poses to {}".format(args.out_csv))

    if args.execute:
        if robot is None:
            print("Cannot execute: no robot connected (pass --robot-ip)")
        else:
            print("enter to execute or q to quit is handled inside trajectory_execution")
            # ur_poses = uplift_z(ur_poses)
            trajectory_execution(robot, ur_poses)
            if args.multilayer:
                left_poses, right_poses = mutilayer(ur_poses)
                trajectory_execution(robot, left_poses)
                trajectory_execution(robot, right_poses)

    print("Finish display\n")

    if robot is not None:
        robot.stop()
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
