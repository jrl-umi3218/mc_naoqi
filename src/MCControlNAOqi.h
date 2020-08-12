#pragma once

#include "ContactForcePublisher.h"
#include "nav_msgs/Odometry.h"
#include <condition_variable>
#include <qi/session.hpp>
#include <mc_rtc/ros.h>


namespace mc_naoqi
{
/**
 * @brief mc_rtc control interface for NAO and PEPPER robots running NAOqi operating system
 */
struct MCControlNAOqi
{
  /**
  * @brief Interface constructor and destructor
  */
  MCControlNAOqi(mc_control::MCGlobalController& controller, std::unique_ptr<ContactForcePublisher> &cfp_ptr,
                  const std::string& host, const unsigned int port);

  virtual ~MCControlNAOqi();

  bool publishContactForces() { return publishContactForces_; };

  /**
   * @brief Is the interface running
   */
  bool running() { return interfaceRunning_; }

  /**
   * @brief Start or stop the experiment
   *
   * @param state
   *  true: Start controller
   *  false: Stop controller
   */
  void startOrStop(const bool state);
  bool controllerStartedState() { return controllerStartedState_; };

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
   * @brief Set the wheels stiffness to max value or switch off
   * Connect a preproccess callback to DCM loop
   *
   * @param state
   *  true: Turn on the wheels actuators
   *  false: Turn off the wheels actuators
   */
  void wheelsServo(const bool state);

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController& controller() { return globalController_; }

 private:
  /*! Publish contact forces from mc_rtc to ROS */
  bool publishContactForces_ = true;

  /*! Controller state (started or stopped) */
  bool controllerStartedState_ = false;
  std::string controllerButtonText_ = "Start/Stop controller";

  /**
   * @brief Thread that sends mc_rtc controller commands to the robot.
   */
  void control_thread();

  /**
   * @brief Thread that retrieves robot sensor values and sends them to mc_rtc
   */
  void sensor_thread();

  /*! Global mc_rtc controller */
  mc_control::MCGlobalController& globalController_;
  std::string controllerToRun_;

  /*! Controller timestep expressed in ms */
  unsigned int timestep_;

  /*! Running the mc_naoqi interface */
  bool interfaceRunning_;

  /*! Servo on/off (joint stiffness 0 if off) */
  bool servoState_ = false;
  std::string servoButtonText_ = "Motors ON/OFF";

  /*! Wheels servo on/off */
  bool wheelsServoState_ = false;
  std::string wheelsServoButtonText_ = "Wheels ON/OFF";

  /* Sensor information */
  /*! Encoder values */
  std::vector<double> qIn_;
  /*! ElectricCurrent values */
  std::vector<double> tauIn_;
  /*! Orientation sensor */
  Eigen::Vector3d rpyIn_;
  /*! Accelerometer */
  Eigen::Vector3d accIn_;
  /*! Angular velocity */
  Eigen::Vector3d rateIn_;

  /*! Contact force publisher */
  std::unique_ptr<ContactForcePublisher> &cfp_ptr;

  /* Connection information */
  /*! Connection host */
  std::string host_;
  /*! Connection port */
  unsigned int port_;
  /*! Connection state */
  std::string connectionState_ = "none";

  /* Handles communication with NAOqi */
  qi::SessionPtr ALBroker_;
  /*! Custom DCM module for fast access to NAOqi memory and actuators */
  qi::AnyObject MCNAOqiDCM_;
  /*! Tables service (Pepper only) */
  qi::AnyObject ALTabletservice_;
  /*! ALLauncher module (check if needed modules are present on robot) */
  qi::AnyObject ALlauncher_;

  /*! Control and sensor threads */
  std::thread controlThread_;
  // Wait for sensor input before starting control
  std::condition_variable controlCV_;
  std::mutex controlMut_;
  std::thread sensorThread_;

  /*!  Maps sensor name to sensor index */
  std::map<std::string, size_t> sensorOrderMap_;
  /*! Sensor values read from the robot memory */
  std::vector<float> sensors_;
  /*! Total number of sensors */
  int numSensors_;
  /*! Use robot IMU sensor */
  bool useRobotIMU_ = false;

  /*! Eye blinking ability */
  // note: enabling ALAutonomousBlinking works in interactive mode only
  bool blinking_ = true;
  int msTillBlink_;

  /*! Enable talking */
  bool talking_ = true;

  /*! Mobile base control (Pepper only) */
  bool moveMobileBase_ = false;
  double wheelRadius_ = 0.07; // meters
  std::vector<std::string> wheelNames_ = {"WheelFL_link", "WheelFR_link", "WheelB_link"};
  Eigen::Matrix3d wheelsJacobian_;
  Eigen::Vector3d mobileBaseSpeedCommand_;
  Eigen::Vector3d wheelsSpeedCommand_;

  /* Bumper names */
  std::vector<std::string> bumpers_;

  /* Enable or disable custom safety reflex */
  bool wheelsOffOnBumperPressed_ = true;
  bool wheelsOffOnBumperPressedState_ = false;

  /* Name of the speakers device in mc_rtc RobotModule */
  std::string speakerDeviceName_ = "Speakers";

  /* Name of the visual display device in mc_rtc RobotModule */
  std::string displayDeviceName_ = "Tablet";
  bool enableVisualDisplay_ = true;
};

} /* mc_naoqi */
