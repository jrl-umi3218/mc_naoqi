#pragma once

// mc_rtc
#include <mc_control/mc_global_controller.h>

// boost
#include <boost/thread.hpp>

// std
#include <fstream>

#include <Eigen/Core>

#include "NAOModule.h"
#include "MCControlNAOServices.h"

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>

namespace AL
{
class ALMotionProxy;
class ALMemoryProxy;
}


namespace mc_nao
{

class MCControlNAO
{
 public:
  MCControlNAO(const std::string& host, mc_control::MCGlobalController& controller, const mc_control::Configuration&);
  virtual ~MCControlNAO();

  bool running();
  void stop();
  void servo(const bool state);

  mc_control::MCGlobalController& controller();

 private:
  void control_thread();
  void handleSensors();

 private:
  mc_control::MCGlobalController& m_controller;
  MCControlNAOService m_service;
  std::vector<std::string> deactivatedJoints;
  std::vector<std::string> activeJoints;

  // MCControlNAOService m_service;
  /*! Timestep expressed in ms */
  unsigned int m_timeStep;
  bool m_running = true;
  /*! Servo on/off (joint stiffness 0 if off) */
  bool m_servo = false;
  bool init = false;
  /* Sensor information */
  /*! Encoder values */
  std::vector<double> qIn;
  /*! Names of force sensors */
  std::vector<std::string> m_wrenchesNames;
  /*! Value of force sensors */
  std::map<std::string, sva::ForceVecd> m_wrenches;
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
  /*! Remote port for sensor connection */
  unsigned int portSensor;
  /*! Remote port for control connection */
  unsigned int portControl;

  /* Handles communication with NAO */
  boost::shared_ptr<AL::ALBroker> al_broker;
  boost::shared_ptr<NAOModule> nao_module;
  /*! Gives high level access to actuators */
  std::unique_ptr<AL::ALMotionProxy> al_motion;
  /*! Gives access to nao memory (read force sensors...) */
  std::unique_ptr<AL::ALMemoryProxy> al_memory;
  std::thread control_th;
  std::thread sensor_th;
};
} /* mc_nao */
