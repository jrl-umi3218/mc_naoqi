#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <Eigen/Core>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

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
  MCControlNAOqi(mc_control::MCGlobalController& controller, const std::string& host, const unsigned int port);
  virtual ~MCControlNAOqi();

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
   * @brief Sends mc_rtc controller commands to the robot.
   */
  void control_thread();
  /**
   * @brief Retrieves robot sensor values, and provide them to
   * MCGlobalController
   */
  void handleSensors();

 private:
  /*! Global mc_rtc controller */
  mc_control::MCGlobalController& m_controller;

  /*! Timestep expressed in ms */
  unsigned int m_timeStep;
  /*! Running the mc_rtc_naoqi interface */
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

  /* Handles communication with NAOqi */
  qi::SessionPtr al_broker;

  /*! Custom DCM module for fast access to NAOqi memory and actuators */
  qi::AnyObject al_fastdcm;
  /*! Tables service (Pepper only) */
  qi::AnyObject al_tabletservice;
  /*! ALLauncher module */
  qi::AnyObject al_launcher;

  /*! Control and sensor threads */
  std::thread control_th;
  // Wait for sensor input before starting control
  std::condition_variable control_cv;
  std::mutex control_mut;
  std::thread sensor_th;

  /*!  Maps sensor name to sensor index */
  std::map<std::string, size_t> sensorOrderMap;
  /*! Sensor values read from the memory */
  std::vector<float> sensors;
  /*! Total number of sensors */
  int numSensors;
  /*! blinking ability */
  // note: enabling ALAutonomousBlinking seems to work in interactive mode only
  bool enableBlinking;
  int msTillBlink;

  /*! Mobile base control (Pepper only) */
  bool moveMobileBase = true;
};

} /* mc_nao */
