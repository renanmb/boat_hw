#ifndef BOAT_DRIVER_H
#define BOAT_DRIVER_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
using namespace vesc_driver;

class boat_driver
{
public:
    boat_driver(std::string port, ros::NodeHandle nh, std::string name, disp_pos_mode rotor_position_mode);

    double getSpeed();
    double getVoltageIn();

    void setDutyCycle(double dutyCycle); // change to setSpeed
    void setSpeed(double speed);
    void releaseMotor();

private:
    std::string name_;
    VescInterface vesc_;
    void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
    void vescErrorCallback(const std::string& error);
    void timerCallback(const ros::TimerEvent& event);
    ros::Publisher state_pub_;
    ros::Publisher rotor_position_pub_; // delete
    ros::Timer timer_;
    boost::mutex mutex_;
    double speed;
    double voltageIn;


    // driver modes (possible states)
    typedef enum {
      MODE_INITIALIZING,
      MODE_OPERATING
    } driver_mode_t;

    // other variables
    driver_mode_t driver_mode_;           ///< driver state machine mode (state)
    int fw_version_major_;                ///< firmware major version reported by vesc
    int fw_version_minor_;                ///< firmware minor version reported by vesc
};

#endif // WHEEL_DRIVER_H
