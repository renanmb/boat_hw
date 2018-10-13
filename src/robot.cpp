#include "boat_hw.h"
#include <iostream>
#include "controller_manager/controller_manager.h" // delete
#include <hardware_interface/robot_hw.h> //delete

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boat_robot");
  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string right_thruster_port;
  std::string left_thruster_port;

  ros::Rate r(100); // 100Hz
  if (!private_nh.getParam("right_thruster_port", right_thruster_port)) {
    ROS_FATAL("VESC communication right_thruster_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("left_thruster_port", left_thruster_port)) {
    ROS_FATAL("VESC communication left_thruster_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("max_rpm", max_rpm)) {
      ROS_FATAL("VESC communication max_rpm parameter required.");
      ros::shutdown();
      return -1;
  }

  if (!private_nh.getParam("min_rpm", min_rpm)) {
      ROS_FATAL("VESC communication min_rpm parameter required.");
      ros::shutdown();
      return -1;
  }

  if (!private_nh.getParam("throttle_smoother_rate", throttle_smoother_rate)) {
      ROS_FATAL("VESC communication throttle_smoother_rate parameter required.");
      ros::shutdown();
      return -1;
  }

  if (!private_nh.getParam("rpm_to_erpm_gain", rpm_to_erpm_gain)) {
      ROS_FATAL("VESC communication rpm_to_erpm_gain parameter required.");
      ros::shutdown();
      return -1;
  }


  boat::BOAT_HW robot(right_thruster_port, left_thruster_port, max_rpm_acceleration, max_rpm, min_rpm, throttle_smoother_rate, rpm_to_erpm_gain, nh);

  while (ros::ok())
  {
     robot.read(ros::Time::now(),r.cycleTime());
     cm.update(ros::Time::now(), r.cycleTime());
     robot.write(ros::Time::now(), r.cycleTime());
     r.sleep();
  }

  return 0;
}
