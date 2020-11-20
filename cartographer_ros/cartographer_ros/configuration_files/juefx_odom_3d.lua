-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true, -- false
  use_nav_sat = false, -- true
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 180 -- 160
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1 -- 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 0.5, -- 2
  min_num_points = 500, -- 150
  max_range = 25., -- 15
}
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 2, -- 4
  min_num_points = 250, -- 200
  max_range = 60.,
}
TRAJECTORY_BUILDER_3D.ceres_scan_matcher = {
  occupied_space_weight_0 = 1.,
  occupied_space_weight_1 = 6.,
  translation_weight = 50, -- 5
  translation_weight_z = 50, 
  rotation_weight = 1e2, -- 4e2,
  rotation_weight_yaw = 1e2, 
  odom_translation_weight = 10, 
  odom_translation_weight_z = 5, 
  odom_rotation_weight = 10, 
  odom_rotation_weight_yaw = 7.5, 
  only_optimize_yaw = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 12,
    num_threads = 1,
  },
}
TRAJECTORY_BUILDER_3D.submaps = {
  high_resolution = 0.10,
  high_resolution_max_range = 25., -- 20
  low_resolution = 0.30, -- 0.45
  num_range_data = 100, -- 160
  range_data_inserter = {
    hit_probability = 0.55,
    miss_probability = 0.49,
    num_free_space_voxels = 2,
  },
}

POSE_GRAPH.optimize_every_n_nodes = 200 -- 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- 0.3
POSE_GRAPH.constraint_builder.min_score = 0.44 -- 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66 -- 0.6
POSE_GRAPH.constraint_builder.max_constraint_distance = 10. -- 15
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e6 --1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e6 --1e5

POSE_GRAPH.optimization_problem = {
  huber_scale = 5e2, 
  acceleration_weight = 1, -- 1e3,
  rotation_weight = 1e2, -- 3e5,
  local_slam_pose_translation_weight = 1e5,
  local_slam_pose_rotation_weight = 1e5,
  odometry_translation_weight = 1, --1e5,
  odometry_rotation_weight = 1, --1e5,
  fixed_frame_pose_translation_weight = 1e1,
  fixed_frame_pose_rotation_weight = 1e2,
  log_solver_summary = true, -- false,
  use_online_imu_extrinsics_in_3d = true,
  fix_z_in_3d = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 10, -- 50,
    num_threads = 7,
  },
}

return options
