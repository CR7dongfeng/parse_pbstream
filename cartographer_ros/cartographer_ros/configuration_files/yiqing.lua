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
  use_odometry = false,
  use_nav_sat = true,
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

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1 -- 0.15
-- 16 is proper for gravity integrate
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 16
-- wheel_odometry means have only movement, but no orientation
TRAJECTORY_BUILDER_3D.use_wheel_odometry = true
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
  translation_weight = 5, 
  translation_weight_z = 5, 
  rotation_weight = 2e2, -- 2e2 + 2e2 = 4e2 
  rotation_weight_yaw = 1e2, -- 4e2
  -- configuration for full odometry
  -- or only rotation_weight for gravity from imu
  odom_translation_weight = 0, --10, 
  odom_translation_weight_z = 0, --5, 
  odom_rotation_weight = 2e2, 
  odom_rotation_weight_yaw = 0, --7.5, 
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
  low_resolution = 0.4, -- 0.45
  -- need to consider moving speed, the size of submap will influence constraint
  num_range_data = 50, -- 160
  range_data_inserter = {
	-- need to consider node_size, lidar number and indoor/outdoor
    hit_probability = 0.75, --0.55,
    miss_probability = 0.49,
    num_free_space_voxels = 0, --2,
  },
}

-- recommand be double of submap_node_size
POSE_GRAPH.optimize_every_n_nodes = 100 -- 320
-- sampling_ratio 0.05 is enough
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 -- 0.3
-- min_score can't be less than 0.35, otherwise it will be lots of local mis-match
POSE_GRAPH.constraint_builder.min_score = 0.4 -- 0.55
POSE_GRAPH.constraint_builder.submap_node_size = 50
POSE_GRAPH.constraint_builder.long_distance_submap_count = 20
POSE_GRAPH.constraint_builder.long_distance_min_score = 0.16667
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4 -- 0.6
-- the distance limit for whether count constraint or not
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e5 -- 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d = {
  branch_and_bound_depth = 8,
  full_resolution_depth = 3,
  min_rotational_score = 0.77,
  -- 0.5 is proper, no need be less than 0.45
  min_low_resolution_score = 0.5, --0.55,
  -- the search limit for correct location error
  linear_xy_search_window = 7, --5 
  linear_z_search_window = 1,
  angular_search_window = math.rad(15.),
}

POSE_GRAPH.optimization_problem = {
  huber_scale = 5e2, 
  -- can be used to keep z-height stable
  acceleration_weight = 1e3, -- 1 
  rotation_weight = 1e3, 
  local_slam_pose_translation_weight = 1e5,
  local_slam_pose_rotation_weight = 1e5,
  odometry_translation_weight = 1, --1e5,
  odometry_rotation_weight = 1, --1e5,
  fixed_frame_pose_translation_weight = 10, --1,
  fixed_frame_pose_rotation_weight = 0, --1,
  log_solver_summary = false,
  use_online_imu_extrinsics_in_3d = false, -- true,
  fix_z_in_3d = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 10, -- 50,
    num_threads = 7,
  },
}

return options
