#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <Eigen/Core>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

namespace AL
{
class ALMotionProxy;
class ALMemoryProxy;
class ALPreferenceManagerProxy;
} /* AL */

namespace mc_control
{
class MCGlobalController;
} /* mc_control */

namespace mc_rtc_naoqi
{
/**
 * @brief Control interface for NAO and PEPPER robots
 */
class MCControlNAOqi
{
 public:
  MCControlNAOqi(mc_control::MCGlobalController& controller, const std::string& host, const unsigned int port);
  virtual ~MCControlNAOqi();

  bool running();

  /**
   * @brief Start the control loop
   */
  void start();

  /**
   * @brief Stop the control loop
   */
  void stop();

  /**
   * @brief Gradually increase the stiffness to max value
   *
   * @param state
   *  true: Turn on the actuators
   *  false: Turn off the actuators
   */
  void servo(const bool state);

  mc_control::MCGlobalController& controller();

 private:
  /**
   * @brief Sends mc_rtc controller commands to the robot.
   */
  void control_thread();
  /**
   * @brief Retrieves robot sensor values, and provide them to
   * MCGlobalController
   */
  void handleSensors();

 private:
  mc_control::MCGlobalController& m_controller;

  /*! Timestep expressed in ms */
  unsigned int m_timeStep;
  bool m_running = true;
  /*! Servo on/off (joint stiffness 0 if off) */
  bool m_servo = false;
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
  /*! Controller's iteration count*/
  unsigned int iter_since_start;

  /* Connection information */
  /*! Connection host */
  std::string host;
  /*! Remote port for control connection */
  unsigned int portControl;

  /* Handles communication with NAO */
  qi::SessionPtr al_broker;

  /*! Custom DCM module for fast access to NAO memory */
  // std::unique_ptr<AL::ALProxy> al_fastdcm;
  qi::AnyObject al_fastdcm;
  qi::AnyObject al_tabletservice;

  std::thread control_th;
  // Wait for sensor input before starting control
  std::condition_variable control_cv;
  std::mutex control_mut;
  std::thread sensor_th;

  // Maps sensor name to sensor index
  std::map<std::string, size_t> sensorOrderMap;
  // Sensor values read from the memory
  std::vector<float> sensors;
  // total number of sensors
  int numSensors;
  // blinking timer
  bool enableBlinking;
  int msTillBlink;
};

} /* mc_nao */
