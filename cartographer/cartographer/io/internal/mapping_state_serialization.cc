/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/internal/mapping_state_serialization.h"

#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"

#include <fstream>

namespace cartographer {
namespace io {
namespace {
using mapping::MapById;
using mapping::NodeId;
using mapping::PoseGraphInterface;
using mapping::SubmapId;
using mapping::TrajectoryNode;
using mapping::proto::SerializedData;

mapping::proto::AllTrajectoryBuilderOptions
CreateAllTrajectoryBuilderOptionsProto(
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        all_options_with_sensor_ids,
    const std::vector<int>& trajectory_ids_to_serialize) {
  mapping::proto::AllTrajectoryBuilderOptions all_options_proto;
  for (auto id : trajectory_ids_to_serialize) {
    *all_options_proto.add_options_with_sensor_ids() =
        all_options_with_sensor_ids[id];
  }
  return all_options_proto;
}

// Will return all trajectory ids, that have `state != DELETED`.
std::vector<int> GetValidTrajectoryIds(
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectory_states) {
  std::vector<int> valid_trajectories;
  for (const auto& t : trajectory_states) {
    if (t.second != PoseGraphInterface::TrajectoryState::DELETED) {
      valid_trajectories.push_back(t.first);
    }
  }
  return valid_trajectories;
}

mapping::proto::SerializationHeader CreateHeader() {
  mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

SerializedData SerializePoseGraph(const mapping::PoseGraph& pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.ToProto(include_unfinished_submaps);
  return proto;
}

SerializedData SerializeTrajectoryBuilderOptions(
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    const std::vector<int>& trajectory_ids_to_serialize) {
  SerializedData proto;
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_options,
                                             trajectory_ids_to_serialize);
  return proto;
}

void SerializeSubmaps(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    bool include_unfinished_submaps, ProtoStreamWriterInterface* const writer) {
  // Next serialize all submaps.
  for (const auto& submap_id_data : submap_data) {
    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto* const submap_proto = proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(
        /*include_probability_grid_data=*/true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(proto);
  }
}

void SerializeTrajectoryNodes(
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : trajectory_nodes) {
    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
}

void SerializeTrajectoryData(
    const std::map<int, PoseGraphInterface::TrajectoryData>&
        all_trajectory_data,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& trajectory_data : all_trajectory_data) {
    SerializedData proto;
    auto* const trajectory_data_proto = proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(proto);
  }
}

void SerializeImuData(const sensor::MapByTime<sensor::ImuData>& all_imu_data,
                      ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_imu_data.trajectory_ids()) {
    for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const imu_data_proto = proto.mutable_imu_data();
      imu_data_proto->set_trajectory_id(trajectory_id);
      *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeOdometryData(
    const sensor::MapByTime<sensor::OdometryData>& all_odometry_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
    for (const auto& odometry_data :
         all_odometry_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const odometry_data_proto = proto.mutable_odometry_data();
      odometry_data_proto->set_trajectory_id(trajectory_id);
      *odometry_data_proto->mutable_odometry_data() =
          sensor::ToProto(odometry_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeFixedFramePoseData(
    const sensor::MapByTime<sensor::FixedFramePoseData>&
        all_fixed_frame_pose_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
    for (const auto& fixed_frame_pose_data :
         all_fixed_frame_pose_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const fixed_frame_pose_data_proto =
          proto.mutable_fixed_frame_pose_data();
      fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
      *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
          sensor::ToProto(fixed_frame_pose_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeLandmarkNodes(
    const std::map<std::string, PoseGraphInterface::LandmarkNode>&
        all_landmark_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node : all_landmark_nodes) {
    for (const auto& observation : node.second.landmark_observations) {
      SerializedData proto;
      auto* landmark_data_proto = proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));
      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);
      writer->WriteProto(proto);
    }
  }
}

}  // namespace

void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps) {
    /*
  writer->WriteProto(CreateHeader());
  writer->WriteProto(
      SerializePoseGraph(pose_graph, include_unfinished_submaps));
  writer->WriteProto(SerializeTrajectoryBuilderOptions(
      trajectory_builder_options,
      GetValidTrajectoryIds(pose_graph.GetTrajectoryStates())));

  SerializeSubmaps(pose_graph.GetAllSubmapData(), include_unfinished_submaps,
                   writer);
  SerializeTrajectoryNodes(pose_graph.GetTrajectoryNodes(), writer);
  SerializeTrajectoryData(pose_graph.GetTrajectoryData(), writer);
  SerializeImuData(pose_graph.GetImuData(), writer);
  SerializeOdometryData(pose_graph.GetOdometryData(), writer);
  SerializeFixedFramePoseData(pose_graph.GetFixedFramePoseData(), writer);
  SerializeLandmarkNodes(pose_graph.GetLandmarkNodes(), writer);
  */
    // 输出TrajectoryNodes
    
  std::ofstream ofs("/home/juefx/pbstream_output/trajectory_nodes.txt", std::ios::app);
  ofs.precision(20);
  ofs << "traj_id , node_id, time, x, y, z, qw, qx, qy, qz" << std::endl;
  auto trajectory_nodes = pose_graph.GetTrajectoryNodes();
  for(const auto& elem : trajectory_nodes) {
      int trajectory_id = elem.id.trajectory_id;
      int node_id = elem.id.node_index;
      const auto& trajectory_node = elem.data;

      double x = trajectory_node.global_pose.translation().x();
      double y = trajectory_node.global_pose.translation().y();
      double z = trajectory_node.global_pose.translation().z();

      double qw = trajectory_node.global_pose.rotation().w();
      double qx = trajectory_node.global_pose.rotation().x();
      double qy = trajectory_node.global_pose.rotation().y();
      double qz = trajectory_node.global_pose.rotation().z();

      int64_t uts_timestamp = ::cartographer::common::ToUniversal(trajectory_node.time());
      int64_t ns_since_unix_epoch =
              (uts_timestamp -
               ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
               10000000ll) *
              100ll;
      double utc_sec = (double)ns_since_unix_epoch / 1e9;

      ofs << trajectory_id << " " << node_id << " " << utc_sec << " "
          << x << " " << y << " " << z << " " << qw << " " << qx << " " << qy << " " << qz << std::endl;
  }
  ofs.close();

  // 输出TrajectoryData
  ofs.open("/home/juefx/pbstream_output/trajectory_data.txt", std::ios::app);
  ofs.precision(15);
  ofs << "id , gravity_constant, imu_calibration(0,1,2,3), fixed_frame_origin(t + q(wxyz))" << std::endl;
  auto trajectory_data = pose_graph.GetTrajectoryData();
  for(const auto& elem : trajectory_data) {
      int id = elem.first;
      double gravity_constant = elem.second.gravity_constant;
      auto arr = elem.second.imu_calibration;
      double x = elem.second.fixed_frame_origin_in_map->translation().x();
      double y = elem.second.fixed_frame_origin_in_map->translation().y();
      double z = elem.second.fixed_frame_origin_in_map->translation().z();

      double qw = elem.second.fixed_frame_origin_in_map->rotation().w();
      double qx = elem.second.fixed_frame_origin_in_map->rotation().x();
      double qy = elem.second.fixed_frame_origin_in_map->rotation().y();
      double qz = elem.second.fixed_frame_origin_in_map->rotation().z();

      ofs << id << " " << gravity_constant << " " << arr[0] << " " << arr[1] << " " << arr[2] << " " << arr[3] << " "
          << x << " " << y << " " << z << " " << qw << " " << qx << " " << qy << " " << qz << std::endl;
  }
  ofs.close();

}

}  // namespace io
}  // namespace cartographer
