#ifndef JOG_FRAME_ABS_H
#define JOG_FRAME_ABS_H

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jog_msgs/JogFrameAbs.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

namespace jog_frame {

typedef struct {
public:
  std::string action_ns;
  std::string type;
  std::vector<std::string> joints;

} Controller;

/**
 * Class JogFrameNodeAbs - Provides the jog_frame for absolute poses
 */
class JogFrameNodeAbs {
public:
  /**
   * @breif: Default constructor for JogFrameNodeAbs Class.
   */
  JogFrameNodeAbs();
  void jog_frame_cb(jog_msgs::JogFrameAbsConstPtr msg);
  void joint_state_cb(sensor_msgs::JointStateConstPtr msg);
  int get_controller_list();
  void update();
  void getFkPose();
  void jogStep();
  void publishPose(sensor_msgs::JointState state);

protected:
  ros::Subscriber joint_state_sub_, jog_frame_sub_;
  ros::ServiceClient fk_client_, ik_client_;

  std::map<std::string, Controller> cinfo_map_;
  std::map<std::string, TrajClient *> traj_clients_;
  std::map<std::string, ros::Publisher> traj_pubs_;

  std::map<std::string, double> joint_map_;
  geometry_msgs::PoseStamped pose_stamped_;
  std::mutex pose_stamped_mutex_;

  double time_from_start_;
  bool use_action_;
  bool intermittent_;
  bool publish_tf_;
  double cart_position_limit_;
  double cart_orientation_limit_;

  bool motion_completed_; 

  robot_model_loader::RobotModelLoader *robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::JointModelGroup *joint_model_group_;

  jog_msgs::JogFrameAbsConstPtr ref_msg_;
  std::mutex ref_msg_mutex_;

  std::string target_link_;
  std::string group_name_;
  std::string frame_id_;
  bool avoid_collisions_;
  double damping_fac_;

  std::vector<std::string> exclude_joints_;
  sensor_msgs::JointState joint_state_;
  ros::Time last_stamp_;

  static tf::TransformBroadcaster br;
};

} // namespace jog_frame

#endif // JOG_FRAME_NODE_H
