#include <math.h>
#include "boat_driver.h"
#include "vesc_driver/vesc_driver.h"
#include <vesc_msgs/VescStateStamped.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

boat_driver::boat_driver(std::string port, ros::NodeHandle nh, std::string name ) :
      vesc_(std::string(),boost::bind(&boat_driver::vescPacketCallback, this, _1), boost::bind(&boat_driver::vescErrorCallback, this, _1)),
      driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
    name_ = name;

    // attempt to connect to the serial port
    try {
      vesc_.connect(port);
    }
    catch (SerialException e) {
      ROS_FATAL("Failed to connect to the VESC on port %s, %s.", port.c_str(), e.what());
      ros::shutdown();
      return;
    }

    // create vesc state (telemetry) publisher
    state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>(name + "/sensors/core", 10);

    // create a 50Hz timer, used for state machine & polling VESC telemetry
    timer_ = nh.createTimer(ros::Duration(1.0/50.0), &boat_driver::timerCallback, this);

}

void boat_driver::setSpeed(double speed){
	if (driver_mode_ = MODE_OPERATING){
	      vesc_.setSpeed(speed);
	}
}

void boat_driver::releaseMotor()
{
  ROS_DEBUG("Releasing motor %s", name_.c_str());
  vesc_.setCurrent(0.0);
}

void boat_driver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    // timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void boat_driver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
    boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    {
      boost::mutex::scoped_lock lock(mutex_);
      speed = values->rpm();
      voltageIn = values->v_in();
    }

    vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();
    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }

}

double  boat_driver::getSpeed()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return speed;
  }
}

double  boat_driver::getVoltageIn()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return voltageIn;
  }
}

void boat_driver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}
