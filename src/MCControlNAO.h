#pragma once

// mc_rtc
#include <mc_control/mc_global_controller.h>

// boost
#include <boost/thread.hpp>
#include <condition_variable>
#include <mutex>

// std
#include <fstream>

#include <Eigen/Core>

#include <qi/session.hpp>
#include <qi/anyobject.hpp>

namespace AL
{
class ALMotionProxy;
class ALMemoryProxy;
class ALPreferenceManagerProxy;
}

namespace mc_nao
{
/**
 * @brief Control interface for NAO and PEPPER robots
 */
class MCControlNAO
{
 public:
  MCControlNAO(const std::string& host, mc_control::MCGlobalController& controller);
  virtual ~MCControlNAO();

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
  void control_thread();
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
  /*! Orientation sensor */
  Eigen::Vector3d rpyIn;
  /*! Accelerometer */
  Eigen::Vector3d accIn;
  /*! Angular velocity */
  Eigen::Vector3d rateIn;
  /*! Log file */
  std::ofstream m_log;
  /*! Controller's iteration count*/
  unsigned int iter_since_start;

  /* Connection information */
  /*! Connection host */
  std::string host;
  /*! Remote port for control connection */
  unsigned int portControl;

  /* Handles communication with NAO */
  //boost::shared_ptr<AL::ALBroker> al_broker;
  qi::SessionPtr al_broker;
  //std::unique_ptr<AL::ALPreferenceManagerProxy> al_preference;

  /*! Custom DCM module for fast access to NAO memory */
  //std::unique_ptr<AL::ALProxy> al_fastdcm;
  qi::AnyObject al_fastdcm;

  std::thread control_th;
  // Wait for sensor input before starting control
  std::condition_variable control_cv;
  std::mutex control_mut;

  std::thread sensor_th;

  // Maps sensor name to sensor index
  std::map<std::string, size_t> sensorOrderMap;
};

} /* mc_nao */
