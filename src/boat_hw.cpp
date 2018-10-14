#include "boat_hw.h"
#include "boat_driver.h"

#include <math.h>
#include <boat_controller/Drive.msg>

float cmd_vel_left = 0;
float cmd_vel_right = 0;
float desired_rpm_left = 0;
float desired_rpm_right = 0;

namespace boat {

BOAT_HW::BOAT_HW(std::string right_thruster_port, std::string left_thruster_port, double max_rpm_acceleration, double max_rpm, double min_rpm, double throttle_smoother_rate, double rpm_to_erpm_gain, ros::NodeHandle nh) :
  _right_thruster_driver(right_thruster_port, nh, "right_thruster"),
  _left_thruster_driver(left_thruster_port, nh, "left_thruster")
 {

     max_accel = max_rpm_acceleration;
     _max_rpm = max_rpm; //using this
     _min_rpm = min_rpm;
     _throttle_smoother_rate = throttle_smoother_rate;
     _rpm_to_erpm_gain = rpm_to_erpm_gain; //using this

     speed_sub_ = nh.subscribe("commands/motor/speed", 10, &BOAT_HW::speedCallback, this);

    }


  void BOAT_HW::speedCallback(const boat_controller::Drive::ConstPtr& speed){

	  cmd_vel_left = speed.left;
	  cmd_vel_right = speed.right;

	  desired_rpm_left = _max_rpm*cmd_vel_left;
	  desired_rpm_right = _max_rpm*cmd_vel_right;
  }

  void BOAT_HW::read(const ros::Time& time, const ros::Duration& period)
  {
	  // modify  ---- create subscribers for the pid_controller


  }

  void BOAT_HW::write(const ros::Time& time, const ros::Duration& period)
  {
	  // modify below  --------------- code responsible for thrusters
    ROS_DEBUG("Writing to hardware...");


    // modify to receive the subscriber from the pid_controller _cmd[] need to change to a subscriber

    if (desired_rpm_left != 0.0)
    {
      double requestedERPM = _rpm_to_erpm_gain * desired_rpm_left; // convert from PID_controller command to ERPM
      ROS_DEBUG("Requested ERPM left: %f", requestedERPM);

       _left_thruster_driver.setSpeed(requestedERPM);
    }
    else
    {
      _left_thruster_driver.releaseMotor();
    }


    if (desired_rpm_right != 0.0)
    {
      double requestedERPM = _rpm_to_erpm_gain * desired_rpm_right; // convert from PID_controller command to ERPM
      ROS_DEBUG("Requested ERPM right: %f", requestedERPM);

      _right_thruster_driver.setSpeed(requestedERPM);
    }
    else
    {
      _right_thruster_driver.releaseMotor();
    }
  }
  // modify above ---------------------------------------------------------------
}
