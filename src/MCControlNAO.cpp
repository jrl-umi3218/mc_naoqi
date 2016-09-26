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
MCControlNAO::MCControlNAO(const std::string& host, mc_control::MCGlobalController& controller, const mc_control::Configuration& config) : m_controller(controller),
                                                                                                  // m_service(this->m_controller),
                                                                                                  m_timeStep(1000 * ceil(static_cast<unsigned int>(controller.timestep()))),
                                                                                                  m_running(true),
                                                                                                  init(false),
                                                                                                  m_wrenchesNames(controller.robot().forceSensorsByName()),
                                                                                                  iter_since_start(0),
                                                                                                  host(host),
                                                                                                  strPortSensor("4002"),
                                                                                                  strPortControl("4003")
{


  if(config.isMember("Deactivated"))
  {
    std::string robot_name = m_controller.robot().name();
    deactivatedJoints = config("Deactivated");
  }

  LOG_INFO ("Deactivated joints: ");
  for(const auto& j : deactivatedJoints)
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

MCControlNAO::~MCControlNAO()
{
  control_th.join();
}

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
      //FIXME Fill t
      double t = 0.;  //in nano second
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
        // al_motion->setStiffnesses(names, 1.);
        // Uncomment for doom
        // al_motion->setAngles(names, angles, fractionMaxSpeed);
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

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
    std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - elapsed));
  }
}

bool MCControlNAO::running()
{
  return m_running;
}

void MCControlNAO::stop()
{
  m_running = false;
}

// template<class Tcontrol>
// void MCControlNAO::controlCallback(WriteAndAck<Tcontrol>& control_proto, Tcontrol & control_data)
// {
//   control_data.hasZmp = false;
//   control_data.hasBaseFF = false;

//   if(m_controller.running && init)
//   {
//     if(m_controller.run())
//     {
//       //FIXME Fill t
//       double t = 0.; //in nano second
//       const mc_control::QPResultMsg & res = m_controller.send(t);
//       const auto & ref_joint_order = m_controller.ref_joint_order();
//       //FIXME The index correspondance should be computed only one time
//       auto gripperQs = m_controller.gripperQ();

//       //FIXME The controller's robot name should be the same as the module's name
//       for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
//       {
//         if(deactivatedJoints.find(ref_joint_order[i]) != deactivatedJoints.end())
//         {
//           control_data.control[i] = deactivatedJoints.at(ref_joint_order[i]);
//         }
//         else
//         {
//           control_data.control[i] = res.robots_state[0].q.at(ref_joint_order[i])[0];
//         }
//       }
//       /* Update gripper state */
//       for(const auto & cG : gripper_out_index)
//       {
//         const auto & qs = gripperQs[cG.first];
//         for(const auto & idx_p : cG.second)
//         {
//           control_data.control[idx_p.first] = qs[idx_p.second];
//         }
//       }
//     }
//     iter_since_start++;
//   }
//   control_proto.data(control_data);
//   control_proto.send();
// }

// template<class Tsensor>
// void MCControlNAO::sensorCallback(const Tsensor& data)
// {
//   // Joint states and gripper states
//   if(m_controller.running)
//   {
//     qIn.resize(m_controller.robot().mb().nrDof());
//     const auto & ref_joint_order = m_controller.ref_joint_order();
//     for(unsigned i = 0; i < sensors_traits<Tsensor>::dof; ++i)
//     {
//       if(deactivatedJoints.find(ref_joint_order[i]) != deactivatedJoints.end())
//       {
//         qIn[i] = deactivatedJoints.at(ref_joint_order[i]);
//       }
//       else
//       {
//         qIn[i] = data.position[i];
//       }
//     }
//     auto gripperQs = m_controller.gripperQ();
//     for(auto & rG : realGripperQs)
//     {
//       const auto & idx = gripper_in_index[rG.first];
//       auto & qs = rG.second;
//       for(size_t i = 0; i < idx.size(); ++i)
//       {
//         qs[i] = data.position[idx[i]];
//       }
//     }
//     m_controller.setActualGripperQ(realGripperQs);

//     // Wrench
//     m_wrenches[m_wrenchesNames[0]].force()=Eigen::Vector3d(data.forceRF[0],data.forceRF[1],data.forceRF[2]);
//     m_wrenches[m_wrenchesNames[0]].couple()=Eigen::Vector3d(data.forceRF[3],data.forceRF[4],data.forceRF[5]);
//     m_wrenches[m_wrenchesNames[1]].force()=Eigen::Vector3d(data.forceLF[0],data.forceLF[1],data.forceLF[2]);
//     m_wrenches[m_wrenchesNames[1]].couple()=Eigen::Vector3d(data.forceLF[3],data.forceLF[4],data.forceLF[5]);
//     m_wrenches[m_wrenchesNames[2]].force()=Eigen::Vector3d(data.forceRH[0],data.forceRH[1],data.forceRH[2]);
//     m_wrenches[m_wrenchesNames[2]].couple()=Eigen::Vector3d(data.forceRH[3],data.forceRH[4],data.forceRH[5]);
//     m_wrenches[m_wrenchesNames[3]].force()=Eigen::Vector3d(data.forceLH[0],data.forceLH[1],data.forceLH[2]);
//     m_wrenches[m_wrenchesNames[3]].couple()=Eigen::Vector3d(data.forceLH[3],data.forceLH[4],data.forceLH[5]);

//     // rpy
//     rpyIn(0) = data.rpy[0];
//     rpyIn(1) = data.rpy[1];
//     rpyIn(2) = data.rpy[2];

//     // acceleration
//     accIn(0) = data.accelerometer[0];
//     accIn(1) = data.accelerometer[1];
//     accIn(2) = data.accelerometer[2];

//     // angular velocity
//     rateIn(0) = data.gyrometer[0];
//     rateIn(1) = data.gyrometer[1];
//     rateIn(2) = data.gyrometer[2];

//     m_controller.setSensorOrientation(rpyIn);
//     m_controller.setSensorAcceleration(accIn);
//     m_controller.setSensorAngularVelocity(rateIn);
//     m_controller.setEncoderValues(qIn);
//     m_controller.setWrenches(m_wrenches);
//     if(!init)
//     {
//       LOG_INFO("Init controller");
//       m_controller.init(qIn);
//       init = true;
//     }
//   }
// }

mc_control::MCGlobalController& MCControlNAO::controller()
{
  return m_controller;
}

} /* mc_nao */
