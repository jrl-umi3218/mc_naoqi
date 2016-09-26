#pragma once

// mc_rtc
#include <mc_control/mc_global_controller.h>

// boost
#include <boost/thread.hpp>

// std
#include <fstream>

#include <Eigen/Core>

namespace AL
{
class ALMotionProxy;
}

namespace mc_nao
{

class MCControlNAO
{
public:
  MCControlNAO(const std::string & host, mc_control::MCGlobalController & controller);
  virtual ~MCControlNAO();

  int initialize();

  // template<class Tsensor, class Tcontrol>
  // void start();

  // template<class Tsensor>
  // void sensorCallback(const Tsensor & data);

  // template<class Tcontrol>
  // void controlCallback(WriteAndAck<Tcontrol> & proto, Tcontrol & data);

  bool running();

  void stop();

  mc_control::MCGlobalController & controller();

private:
  void control_thread();

private:
  mc_control::MCGlobalController & m_controller;
  // MCControlNAOService m_service;
  /*! Timestep expressed in ms */
  unsigned int m_timeStep;
  bool m_running = true;
  bool init;
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
  std::string strPortSensor;
  /*! Remote port for control connection */
  std::string strPortControl;

  std::unique_ptr<AL::ALMotionProxy> al_motion;
  std::thread control_th;
};
} /* mc_nao */

