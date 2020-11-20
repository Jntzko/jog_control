#include "ros/ros.h"
#include <jog_msgs/JogFrameAbs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wrist_rotation_test");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<jog_msgs::JogFrameAbs>("/jog_frame_abs", 1);

  jog_msgs::JogFrameAbs msg;
  msg.header.frame_id = "base_link";
  msg.group_name = "left_arm";
  msg.link_name = "l_gripper_tool_frame";
  msg.avoid_collisions = true;
  msg.damping_factor = 0.95;
  msg.pose.position.x = 0.68;
  msg.pose.position.y = 0.16;
  msg.pose.position.z = 1.04;

  tf2::Quaternion q;

  ros::Rate loop_rate(100);
  float x = 0;
  while(ros::ok()) {
    msg.header.stamp = ros::Time::now();
    x += 3.1415/300;
    x = fmod(x,2*3.1415);
    ROS_INFO_STREAM(x);
    q.setRPY(x,0,0);
    tf2::convert(q, msg.pose.orientation);
    pub.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}

