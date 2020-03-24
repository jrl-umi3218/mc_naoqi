#pragma once

#include "ContactForcePublisher.h"
#include <condition_variable>

#include <qi/session.hpp>

#include <mc_rtc/ros.h>
#include "nav_msgs/Odometry.h"

namespace mc_control
{
class MCGlobalController;
} /* mc_control */


namespace mc_rtc_naoqi
{
/**
 * @brief Control interface for NAO and PEPPER robots running NAOqi operating system
 */
class MCControlNAOqi
{
 public:
   /**
    * @brief Interface constructor and destructor
    */
  MCControlNAOqi(mc_control::MCGlobalController& controller, std::unique_ptr<ContactForcePublisher> &cfp_ptr,
                  const std::string& host, const unsigned int port);
  virtual ~MCControlNAOqi();

  /*! Publish contact forces from mc_rtc to ROS */
  bool publish_contact_forces = true;

  /**
   * @brief Is the interface running
   */
  bool running();

  /**
   * @brief Start the experiment
   */
  void start();

  /**
   * @brief Stop the experimnet
   */
  void stop();

  /**
   * @brief Set the joints stiffness to max value or switch off
   * Connect a preproccess callback to DCM loop
   *
   * @param state
   *  true: Turn on the actuators
   *  false: Turn off the actuators
   */
  void servo(const bool state);

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController& controller();

 private:
  /**
   * @brief Thread that sends mc_rtc controller commands to the robot.
   */
  void control_thread();

  /**
   * @brief Thread that retrieves robot sensor values and sends them to mc_rtc
   */
  void sensor_thread();

 private:
  /*! Global mc_rtc controller */
  mc_control::MCGlobalController& globalController;

  /*! Controller timestep expressed in ms */
  unsigned int timestep;

  /*! Running the mc_rtc_naoqi interface */
  bool interfaceRunning = true;

  /*! Servo on/off (joint stiffness 0 if off) */
  bool servoState = false;

  /* Sensor information */
  /*! Encoder values */
  std::vector<double> qIn;
  /*! ElectricCurrent values */
  std::vector<double> tauIn;
  /*! Orientation sensor */
  Eigen::Vector3d rpyIn;
  /*! Accelerometer */
  Eigen::Vector3d accIn;
  /*! Angular velocity */
  Eigen::Vector3d rateIn;

  /*! Contact force publisher */
  std::unique_ptr<ContactForcePublisher> &cfp_ptr;

  /* Connection information */
  /*! Connection host */
  std::string host;
  /*! Connection port */
  unsigned int portControl;

  /* Handles communication with NAOqi */
  qi::SessionPtr al_broker;
  /*! Custom DCM module for fast access to NAOqi memory and actuators */
  qi::AnyObject al_fastdcm;
  /*! Tables service (Pepper only) */
  qi::AnyObject al_tabletservice;
  /*! ALLauncher module (check if needed modules are present on robot) */
  qi::AnyObject al_launcher;

  /*! Control and sensor threads */
  std::thread control_th;
  // Wait for sensor input before starting control
  std::condition_variable control_cv;
  std::mutex control_mut;
  std::thread sensor_th;

  /*!  Maps sensor name to sensor index */
  std::map<std::string, size_t> sensorOrderMap;
  /*! Sensor values read from the robot memory */
  std::vector<float> sensors;
  /*! Total number of sensors */
  int numSensors;
  /*! Use robot IMU sensor */
  bool useRobotIMU = false;

  /*! Eye blinking ability */
  // note: enabling ALAutonomousBlinking works in interactive mode only
  bool enableBlinking = true;
  int msTillBlink;

  /*! Enable talking */
  bool enableTalking = false;

  /*! Mobile base control (Pepper only) */
  bool moveMobileBase = true;
  unsigned int numWheels = 3;
  double wheel_radius = 0.07; // meters
  std::vector<std::string> wheelNames = {"WheelFL_link", "WheelFR_link", "WheelB_link"};
  Eigen::Matrix3d wheelsJacobian;
  Eigen::Vector3d mobileBaseSpeedCommand;
  Eigen::Vector3d wheelsSpeedCommand;

public:
  /* ROS topic monitoring thread */
  bool useROS = false;
  std::thread spin_th;
  void monitorROSTopic();

  /* T265 tracking camera */
  void updateBodySensor(const nav_msgs::Odometry::ConstPtr& msg);
  Eigen::Vector3d t265_position;
  Eigen::Quaterniond t265_orientation;
  Eigen::Vector3d t265_linvel;
  Eigen::Vector3d t265_angvel;
  Eigen::Vector3d init_t265_pos_from_kin;
  Eigen::Quaterniond init_t265_rot_from_kin;

};

} /* mc_nao */
