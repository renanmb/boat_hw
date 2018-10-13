#ifndef BOAT_HW_H
#define BOAT_HW_H
#include "boat_hw.h"
#include "boat_driver.h"

namespace boat
{

class BOAT_HW : public hardware_interface::RobotHW
{
public:
  BOAT_HW(std::string right_thruster_port, std::string left_thruster_port, double max_rpm_acceleration, double max_rpm, double min_rpm, double throttle_smoother_rate, double rpm_to_erpm_gain, ros::NodeHandle nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  wheel_driver _right_thruster_driver;
  wheel_driver _left_thruster_driver;

  double max_rpm_acceleration;
  double max_rpm; //using this
  double min_rpm;
  double throttle_smoother_rate; // throttle_smoother_rate = Hz (messages per second)
  double rpm_to_erpm_gain; // using this rpm_to_erpm_gain = (number of magnetic poles/2)


  // ROS services

  ros::Subscriber speed_sub_;

  // ROS callbacks

  void speedCallback(const boat_controller::Drive::ConstPtr& speed);


};

}

#endif // BOAT_HW_H
