#include <boost/thread/thread.hpp>
#include <cfloat>
#include <jog_controller/jog_frame_node_abs.h>
#include <math.h>
#include <thread>
#include <typeinfo>

namespace jog_frame {

JogFrameNodeAbs::JogFrameNodeAbs() {
  ros::NodeHandle gnh, pnh("~");

  ROS_WARN("Absolute mode enabled!");

  pnh.param<double>("time_from_start", time_from_start_, 0.5);
  pnh.param<bool>("use_action", use_action_, false);
  pnh.param<bool>("intermittent", intermittent_, false);
  pnh.param<bool>("publish_tf", publish_tf_, true);

  std::vector<std::string> group_names;
  gnh.getParam("/jog_frame_node/group_names", group_names);
  group_name_ = group_names.size() > 0 ? group_names[0] : "";

  std::vector<std::string> link_names;
  gnh.getParam("/jog_frame_node/link_names", link_names);
  target_link_ = link_names.size() > 0 ? link_names[0] : "";

  avoid_collisions_ = true;
  damping_fac_ = 0.5;

  if (not use_action_ && intermittent_) {
    ROS_WARN("'intermittent' param should be true with 'use_action'. Assuming "
             "to use action'");
    use_action_ = true;
  }
  // Exclude joint list
  gnh.getParam("exclude_joint_names", exclude_joints_);

  if (get_controller_list() < 0) {
    ROS_ERROR("get_controller_list failed. Aborted.");
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("controller_list:");
  for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
    auto cinfo = it->second;
    ROS_INFO_STREAM("- " << it->first);
    for (int i = 0; i < cinfo.joints.size(); i++) {
      ROS_INFO_STREAM("  - " << cinfo.joints[i]);
    }
  }

  // Create subscribers
  joint_state_sub_ =
      gnh.subscribe("joint_states", 1, &JogFrameNodeAbs::joint_state_cb, this);
  jog_frame_sub_ =
      gnh.subscribe("jog_frame_abs", 1, &JogFrameNodeAbs::jog_frame_cb, this);
  fk_client_ = gnh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
  ik_client_ = gnh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

  if (use_action_) {
    // Create action client for each controller
    for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;

      traj_clients_[controller_name] = new TrajClient(action_name, true);
    }
    for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;

      for (;;) {
        if (traj_clients_[controller_name]->waitForServer(ros::Duration(1))) {
          ROS_INFO_STREAM(action_name << " is ready.");
          break;
        }
        ROS_WARN_STREAM("Waiting for " << action_name << " server...");
      }
    }
  } else {
    // Create publisher for each controller
    for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
      auto controller_name = it->first;
      traj_pubs_[controller_name] =
          gnh.advertise<trajectory_msgs::JointTrajectory>(
              controller_name + "/command", 10);
    }
  }
}

/**
 * @brief Callback function for the topic jog_frame
 *
 */
void JogFrameNodeAbs::jog_frame_cb(jog_msgs::JogFrameAbsConstPtr msg) {
  joint_state_.header.stamp = ros::Time::now();

  // In intermittent mode, confirm to all of the action is completed
  if (intermittent_) {
    for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
      auto controller_name = it->first;
      actionlib::SimpleClientGoalState state =
          traj_clients_[controller_name]->getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        return;
      }
    }
  }
  // Update reference frame only if the stamp is older than last_stamp_ +
  // time_from_start_
  if (msg->header.stamp > last_stamp_) {
    // update our reference message for the secondary thread
    frame_id_ = msg->header.frame_id;
    group_name_ = msg->group_name;
    target_link_ = msg->link_name;
    avoid_collisions_ = msg->avoid_collisions;
    if (msg->damping_factor != 0) {
      damping_fac_ = std::min(1.0, std::max(0.1, msg->damping_factor));
    }
    ref_msg_ = msg;

    // Update timestamp of the last jog command
    last_stamp_ = msg->header.stamp;
  }

  if (publish_tf_) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::poseMsgToTF(msg->pose, transform);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, msg->header.frame_id, "jog_goal"));
  }
}

/**
 * Worker thread for calulation
 */
void JogFrameNodeAbs::update() {
  // TODO necessary?
  if (ref_msg_ != nullptr) {
    if (ref_msg_->header.stamp.sec > 0 && ref_msg_->header.stamp.nsec > 0) {
      jogStep();
    }
  }
}

/**
 * @brief get the current pose of the endeffector. Accessible via pose_stamped_
 */
void JogFrameNodeAbs::getFkPose() {
  joint_state_.name.clear();
  joint_state_.position.clear();
  joint_state_.velocity.clear();
  joint_state_.effort.clear();

  for (auto it = joint_map_.begin(); it != joint_map_.end(); it++) {
    // Exclude joint in exclude_joints_
    if (std::find(exclude_joints_.begin(), exclude_joints_.end(), it->first) !=
        exclude_joints_.end()) {
      ROS_INFO_STREAM("joint " << it->first << "is excluded from FK");
      continue;
    }
    // Update reference joint_state
    joint_state_.name.push_back(it->first);
    joint_state_.position.push_back(it->second);
    joint_state_.velocity.push_back(0.0);
    joint_state_.effort.push_back(0.0);
  }

  // Update forward kinematics
  moveit_msgs::GetPositionFK fk;

  fk.request.header.frame_id = frame_id_;
  fk.request.header.stamp = ros::Time::now();
  fk.request.fk_link_names.clear();
  fk.request.fk_link_names.push_back(target_link_);
  fk.request.robot_state.joint_state = joint_state_;

  if (fk_client_.call(fk)) {
    if (fk.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_INFO_STREAM("fk: " << fk.request);
      ROS_WARN("****FK error %d", fk.response.error_code.val);
      return;
    }
    if (fk.response.pose_stamped.size() != 1) {
      for (int i = 0; i < fk.response.pose_stamped.size(); i++) {
        ROS_ERROR_STREAM("fk[" << i << "]:\n" << fk.response.pose_stamped[0]);
      }
    }
    pose_stamped_ = fk.response.pose_stamped[0];
  } else {
    ROS_ERROR("Failed to call service /compute_fk");
    return;
  }
}

void JogFrameNodeAbs::jogStep() {
  // get the current pose, accessiable via pose_stamped_
  getFkPose();

  // Solve inverse kinematics
  moveit_msgs::GetPositionIK ik;

  ik.request.ik_request.group_name = group_name_;
  ik.request.ik_request.ik_link_name = target_link_;
  ik.request.ik_request.robot_state.joint_state = joint_state_;
  ik.request.ik_request.avoid_collisions = avoid_collisions_;

  geometry_msgs::Pose act_pose = pose_stamped_.pose;
  geometry_msgs::PoseStamped ref_pose;

  ref_pose.header.frame_id = ref_msg_->header.frame_id;
  ref_pose.header.stamp = ros::Time::now();

  double dirX = ref_msg_->pose.position.x - act_pose.position.x;
  double dirY = ref_msg_->pose.position.y - act_pose.position.y;
  double dirZ = ref_msg_->pose.position.z - act_pose.position.z;
  double position_dist = sqrt(((double)dirX * dirX) + ((double)dirY * dirY) +
                              ((double)dirZ * dirZ));

  double nDirX = (dirX * damping_fac_);
  double nDirY = (dirY * damping_fac_);
  double nDirZ = (dirZ * damping_fac_);
  ref_pose.pose.position.x = pose_stamped_.pose.position.x + nDirX;
  ref_pose.pose.position.y = pose_stamped_.pose.position.y + nDirY;
  ref_pose.pose.position.z = pose_stamped_.pose.position.z + nDirZ;

  // Apply orientation jog
  tf::Quaternion q_ref, q_act, q_jog, q_target;
  tf::quaternionMsgToTF(act_pose.orientation, q_act);
  tf::quaternionMsgToTF(ref_msg_->pose.orientation, q_target);

  double orientation_dist = tf::angleShortestPath(q_act, q_target);
  q_ref = tf::slerp(q_act, q_target, damping_fac_);

  try {
    tf::assertQuaternionValid(q_act);
    tf::assertQuaternionValid(q_target);
    tf::assertQuaternionValid(q_ref);
  } catch (const std::exception &e) {
    ROS_ERROR("Quaternion is invalid!");
    return;
  }

  // position and orientation are close enough
  if (position_dist < 0.0005 && orientation_dist < 0.001) {
    return;
  }

  quaternionTFToMsg(q_ref, ref_pose.pose.orientation);
  ik.request.ik_request.pose_stamped = ref_pose;

  if (!ik_client_.call(ik)) {
    ROS_ERROR("Failed to call service /compute_ik");
    return;
  }
  if (ik.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_WARN("****IK error %d", ik.response.error_code.val);
    return;
  }

  auto state = ik.response.solution.joint_state;
  geometry_msgs::PoseStamped pose_check;

  // Make sure the jump in joint space is not to large
  bool has_errors = false;

  for (int i = 0; i < state.name.size(); i++) {
    for (int j = 0; j < joint_state_.name.size(); j++) {
      if (state.name[i] == joint_state_.name[j]) {
        // if the joint state range is outside -Pi to Pi, we map it to that
        // range
        double ref_state = fmod(joint_state_.position[j] + M_PI, 2 * M_PI);
        if (ref_state < 0)
          ref_state += 2 * M_PI;
        ref_state -= M_PI;
        double e = fabs(state.position[i] - ref_state);
        if (e > M_PI / 2 and e < M_PI * 1.5) {
          has_errors = true;
        }
        break;
      }
    }
  }
  if (has_errors) {
    ROS_ERROR_STREAM("**** Abort due to errors!");
    return;
  }

  publishPose(state);
}

void JogFrameNodeAbs::publishPose(sensor_msgs::JointState state) {
  // Publish trajectory message for each controller
  for (auto it = cinfo_map_.begin(); it != cinfo_map_.end(); it++) {
    auto controller_name = it->first;
    auto joint_names = it->second.joints;

    std::vector<double> positions, velocities, accelerations;

    positions.resize(joint_names.size());
    velocities.resize(joint_names.size());
    accelerations.resize(joint_names.size());

    for (int i = 0; i < joint_names.size(); i++) {
      size_t index = std::distance(
          state.name.begin(),
          std::find(state.name.begin(), state.name.end(), joint_names[i]));
      if (index == state.name.size()) {
        ROS_WARN_STREAM("Cannot find joint " << joint_names[i]
                                             << " in IK solution");
      }
      positions[i] = state.position[index];
      velocities[i] = 0;
      accelerations[i] = 0;
    }
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = accelerations;
    point.time_from_start = ros::Duration(time_from_start_);

    if (use_action_) {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.header.frame_id = "base_link";
      goal.trajectory.joint_names = joint_names;
      goal.trajectory.points.push_back(point);

      traj_clients_[controller_name]->sendGoal(goal);
    } else {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now();
      traj.header.frame_id = "base_link";
      traj.joint_names = joint_names;
      traj.points.push_back(point);
      traj_pubs_[controller_name].publish(traj);
    }
  }
}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogFrameNodeAbs::joint_state_cb(sensor_msgs::JointStateConstPtr msg) {
  // Check that the msg contains joints
  if (msg->name.empty() || msg->name.size() != msg->position.size()) {
    ROS_WARN("Invalid JointState message");
    return;
  }
  // Update joint information

  for (int i = 0; i < msg->name.size(); i++) {
    joint_map_[msg->name[i]] = msg->position[i];
  }
}

int JogFrameNodeAbs::get_controller_list() {
  ros::NodeHandle gnh;

  // Get controller information from move_group/controller_list
  if (!gnh.hasParam("move_group/controller_list")) {
    ROS_ERROR_STREAM("move_group/controller_list is not specified.");
    return -1;
  }
  XmlRpc::XmlRpcValue controller_list;
  gnh.getParam("move_group/controller_list", controller_list);
  for (int i = 0; i < controller_list.size(); i++) {
    if (!controller_list[i].hasMember("name")) {
      ROS_ERROR("name must be specifed for each controller.");
      return -1;
    }
    if (!controller_list[i].hasMember("joints")) {
      ROS_ERROR("joints must be specifed for each controller.");
      return -1;
    }
    try {
      // get name member
      std::string name = std::string(controller_list[i]["name"]);
      // get action_ns member if exists
      std::string action_ns = std::string("");
      if (controller_list[i].hasMember("action_ns")) {
        action_ns = std::string(controller_list[i]["action_ns"]);
      }
      // get joints member
      if (controller_list[i]["joints"].getType() !=
          XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("joints for controller "
                         << name << " is not specified as an array");
        return -1;
      }
      auto joints = controller_list[i]["joints"];
      // Get type member
      std::string type = std::string("FollowJointTrajectory");
      if (!controller_list[i].hasMember("type")) {
        ROS_WARN_STREAM("type is not specifed for controller "
                        << name << ", using default FollowJointTrajectory");
      }
      type = std::string(controller_list[i]["type"]);
      if (type != "FollowJointTrajectory") {
        ROS_ERROR_STREAM("controller type " << type << " is not supported");
        return -1;
      }
      // Create controller map
      cinfo_map_[name].action_ns = action_ns;
      cinfo_map_[name].joints.resize(joints.size());
      for (int j = 0; j < cinfo_map_[name].joints.size(); ++j) {
        cinfo_map_[name].joints[j] = std::string(joints[j]);
      }
    } catch (...) {
      ROS_ERROR_STREAM(
          "Caught unknown exception while parsing controller information");
      return -1;
    }
  }
  return 0;
}

} // namespace jog_frame

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "jog_frame_node_abs");
  jog_frame::JogFrameNodeAbs node;
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    node.update();
    loop_rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
