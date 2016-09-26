#include "MCControlNAO.h"

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

// std
#include <chrono>
#include <memory>
#include <thread>

#include <algorithm>

namespace mc_nao
{
MCControlNAO::MCControlNAO(const std::string& host, mc_control::MCGlobalController& controller,
                           const mc_control::Configuration& config)
    : m_controller(controller),

      m_service(this->m_controller),
      // m_service(this->m_controller),
      m_timeStep(1000 * ceil(static_cast<unsigned int>(controller.timestep()))),
      m_running(true),
      init(false),
      m_wrenchesNames(controller.robot().forceSensorsByName()),
      iter_since_start(0),
      host(host),
      strPortSensor("4002"),
      strPortControl("9559")
{
  if (config.isMember("Deactivated"))
  {
    std::string robot_name = m_controller.robot().name();
    deactivatedJoints = config("Deactivated");
  }

  LOG_INFO("Deactivated joints: ");
  for (const auto& j : deactivatedJoints)
  {
    LOG_INFO(j);
  }

  const auto& ref_joint_order = m_controller.ref_joint_order();
  for (const auto& j : ref_joint_order)
  {
    if (std::find(deactivatedJoints.begin(), deactivatedJoints.end(), j) == std::end(deactivatedJoints))
    {
      activeJoints.push_back(j);
    }
  }

  LOG_INFO("MCControlNAO: Connecting to " << host << ":" << 9559);
  al_motion = std::unique_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(host, 9559));
  control_th = std::thread(std::bind(&MCControlNAO::control_thread, this));
}

MCControlNAO::~MCControlNAO() { control_th.join(); }
void MCControlNAO::control_thread()
{
  while (m_running)
  {
    // LOG_INFO("Running controller");
    auto start = std::chrono::high_resolution_clock::now();

    /**
     * CONTROL stuff goes here
     **/

    if (m_controller.run())
    {
      // LOG_INFO("Controller running");
      // FIXME Fill t
      double t = 0.;  // in nano second
      const mc_control::QPResultMsg& res = m_controller.send(t);

      AL::ALValue names(activeJoints);
      //"HeadYaw", "HeadPitch");
      std::vector<float> joint_angles(activeJoints.size(), 0.);
      for (unsigned int i = 0; i < activeJoints.size(); ++i)
      {
        joint_angles[i] = res.robots_state[0].q.at(activeJoints[i])[0];
        // std::cout << "Would go to " << activeJoints[i] << ": " << joint_angles[i]  << std::endl;
      }

      AL::ALValue angles(joint_angles);
      AL::ALValue joint_stiffness(std::vector<float>(activeJoints.size(), 1));
      AL::ALValue joint_zero_stiffness(std::vector<float>(activeJoints.size(), 0.));
      // LOG_INFO("Joint Names: " << names);
      // LOG_INFO("Joint angles: " << angles);
      // LOG_INFO("Joint stiffness: " << joint_stiffness);

      float fractionMaxSpeed = 0.1f;
      try
      {
        // Uncomment for doom
        al_motion->setStiffnesses(names, joint_stiffness);
        al_motion->setAngles(names, angles, fractionMaxSpeed);

        // al_motion->setStiffnesses(names, joint_zero_stiffness);
      }
      catch (const AL::ALError& e)
      {
        qiLogError("module.MCControlNAO") << "Error with command: " << e.what() << std::endl;
      }
      catch (...)
      {
        LOG_ERROR("Error communicating command to the robot");
      }
    }
    iter_since_start++;

    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start)
            .count();
    std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - elapsed));
  }
}

bool MCControlNAO::running() { return m_running; }
void MCControlNAO::stop() { m_running = false; }
mc_control::MCGlobalController& MCControlNAO::controller() { return m_controller; }
} /* mc_nao */
