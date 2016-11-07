#include "MCControlNAO.h"

#include <alerror/alerror.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alpreferencemanagerproxy.h>

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
      m_timeStep(1000 * controller.timestep()),
      m_running(true),
      init(false),
      m_wrenchesNames(controller.robot().forceSensorsByName()),
      iter_since_start(0),
      host(host),
      portSensor(9559),
      portControl(9559)
{
  LOG_INFO("timestep : " << controller.timestep());
  LOG_INFO("m_timeStep : " << m_timeStep);
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
      // HACK for mimic joint RHipYawPitch
      if(j != "RHipYawPitch")
      {
        activeJoints.push_back(j);
      }
    }
  }

  LOG_INFO("MCControlNAO: Connecting to " << host << ":" << portControl);

  al_broker = AL::ALBroker::createBroker("MCControlNAOBroker", "0.0.0.0", 54000, host, portControl);
  try
  {
    al_broker->createBroker("MCControlNAOBroker", "0.0.0.0", 0, host, portControl);
  }
  catch (...)
  {
    LOG_ERROR("Failed to create broker");
  }

  nao_module = AL::ALModule::createModule<NAOModule>(al_broker, "MCNAOModule");
  nao_module->setController(this);

  al_motion = std::unique_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(al_broker));
  al_preference = std::unique_ptr<AL::ALPreferenceManagerProxy>(new AL::ALPreferenceManagerProxy(al_broker));

  al_memory = std::unique_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(al_broker));
  al_memory->subscribeToEvent("robotIsFalling", "MCNAOModule", "onRobotFalling");
  al_memory->subscribeToEvent("robotHasFallen", "MCNAOModule", "onRobotHasFallen");
  al_memory->subscribeToEvent("RightBumperPressed", "MCNAOModule", "onRightBumperPressed");

  // Example showing how to get the robot config
  AL::ALValue robotConfig = al_motion->getRobotConfig();
  for (unsigned int i = 0; i < robotConfig[0].getSize(); i++)
  {
    std::cout << robotConfig[0][i] << ": " << robotConfig[1][i] << std::endl;
  }

  al_fastdcm = std::unique_ptr<AL::ALProxy>(new AL::ALProxy(al_broker, "FastGetSetDCM"));

  std::vector<std::string> sensorsOrder;
  al_fastdcm->call<AL::ALValue>("getSensorsOrder").ToStringArray(sensorsOrder);
  for (size_t i = 0; i < sensorsOrder.size(); i++)
  {
    // LOG_INFO("Sensor[" << i << "]: " << sensorsOrder[i]);
    sensorOrderMap[sensorsOrder[i]] = i;
  }

  //////////////
  // Disable some of the interferring embeded nao safeties (collision avoidance...)
  //////////////

  // Disable whole body balancer
  al_motion->wbEnable(false);

  // Disable fall manager
  al_preference->setValue("MCNAOModule", "ENABLE_DEACTIVATION_OF_FALL_MANAGER", true);
  al_motion->setFallManagerEnabled(false);

  // Disable self-collision checks
  al_motion->setCollisionProtectionEnabled("Arms", false);

  control_th = std::thread(std::bind(&MCControlNAO::control_thread, this));
  sensor_th = std::thread(std::bind(&MCControlNAO::handleSensors, this));
}

MCControlNAO::~MCControlNAO() { control_th.join(); }
void MCControlNAO::control_thread()
{
  AL::ALValue names(activeJoints);
  AL::ALValue angles = AL::ALValue::array(0);
  angles.arraySetSize(activeJoints.size());

  while (m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();

    if (m_controller.running && init)
    {
      /**
       * CONTROL stuff goes here
       **/

      if (m_controller.run())
      {
        // LOG_INFO("Controller running");
        // FIXME Fill t
        double t = 0.;  // in nano second
        const mc_control::QPResultMsg& res = m_controller.send(t);

        for (unsigned int i = 0; i < activeJoints.size(); ++i)
        {
          angles[i] = res.robots_state[0].q.at(activeJoints[i])[0];
          // LOG_INFO(activeJoints[i] << " = " << angles[i]);
        }

        al_fastdcm->callVoid("setJointAngles", names, angles);

        // float fractionMaxSpeed = 1.f;
        // try
        // {
        //   al_motion->setAngles(names, angles, fractionMaxSpeed);

        //   // Move at max_speed percent of joint max velocity
        //   // al_motion->angleInterpolationWithSpeed(names, angles, 1);
        // }
        // catch (const AL::ALError& e)
        // {
        //   qiLogError("module.MCControlNAO") << "Error with command: " << e.what() << std::endl;
        // }
        // catch (...)
        // {
        //   LOG_ERROR("Error communicating command to the robot");
        // }
      }
    }

    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > m_timeStep)
    {
      LOG_WARNING("Loop time " << elapsed * 1000 << " exeeded timestep of " << m_timeStep << " ms");
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
  LOG_INFO("MCControlNAO running thread stopped");
}

void MCControlNAO::handleSensors()
{
  // XXX: Sensor thread currently runs at about 50-60ms
  // Should be made faster

  while (m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();
    AL::ALValue sensors = al_fastdcm->call<AL::ALValue>("getSensors");

    // Get encoder values
    qIn.resize(m_controller.robot().mb().nrDof());
    const auto& ref_joint_order = m_controller.ref_joint_order();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      if (std::find(std::begin(deactivatedJoints), std::end(deactivatedJoints), ref_joint_order[i]) !=
          std::end(deactivatedJoints))
      {
        // XXX default value
        qIn[i] = 0;
      }
      // HACK, RHipYawPitch is a joint mimic of LHipYawPitch
      else if(ref_joint_order[i] == "RHipYawPitch")
      {
        qIn[i] = sensors[sensorOrderMap["Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value"]];
      }
      else
      {
        qIn[i] = sensors[sensorOrderMap["Device/SubDeviceList/" + ref_joint_order[i] + "/Position/Sensor/Value"]];
        // LOG_INFO("qIn[i] = " << qIn[i]);
      }
    }
    accIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value"]];
    accIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value"]];
    accIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"]];
    // LOG_INFO("Accelerometer: " << accIn);

    rateIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value"]];
    rateIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value"]];
    rateIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value"]];
    // LOG_INFO("Gyrometer: " << rateIn);

    // Get The Left Foot Force Sensor Values
    double LFsrFL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"]];
    double LFsrFR = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value"]];
    double LFsrBL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value"]];
    double LFsrBR = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value"]];
    // LOG_INFO("Left FSR [Kg] " << LFsrFL << LFsrFR << LFsrBL << LFsrBR);

    // Get The Right Foot Force Sensor Values
    double RFsrFL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value"]];
    double RFsrFR = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value"]];
    double RFsrBL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value"]];
    double RFsrBR = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value"]];
    // LOG_INFO("Left FSR [Kg] " << RFsrFL << RFsrFR << RFsrBL << RFsrBR);

    // Get The Left Foot Force Sensor Values
    // double LFsrFL = al_memory->getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value");
    // double LFsrFR = al_memory->getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value");
    // double LFsrBL = al_memory->getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value");
    // double LFsrBR = al_memory->getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value");
    // // LOG_INFO("Left FSR [Kg] " << LFsrFL << LFsrFR << LFsrBL << LFsrBR);

    // // Get The Right Foot Force Sensor Values
    // double RFsrFL = al_memory->getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value");
    // double RFsrFR = al_memory->getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value");
    // double RFsrBL = al_memory->getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value");
    // double RFsrBR = al_memory->getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value");
    // // LOG_INFO("Left FSR [Kg] " << RFsrFL << RFsrFR << RFsrBL << RFsrBR);

    // accIn(0) = al_memory->getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
    // accIn(1) = al_memory->getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
    // accIn(2) = al_memory->getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
    // // LOG_INFO("Accelerometer: " << accIn);

    // rateIn(0) = al_memory->getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
    // rateIn(1) = al_memory->getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
    // rateIn(2) = al_memory->getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
    // // LOG_INFO("Gyrometer: " << rateIn);

    // // Get encoder values
    // // XXX consider using the fast version
    // // http://doc.aldebaran.com/1-14/dev/cpp/examples/sensors/fastgetsetdcm/fastgetsetexample.html
    // qIn.resize(m_controller.robot().mb().nrDof());
    // const auto& ref_joint_order = m_controller.ref_joint_order();
    // for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    // {
    //   if (std::find(std::begin(deactivatedJoints), std::end(deactivatedJoints), ref_joint_order[i]) !=
    //       std::end(deactivatedJoints))
    //   {
    //     // XXX default value
    //     qIn[i] = 0;
    //   }
    //   else
    //   {
    //     qIn[i] = al_memory->getData("Device/SubDeviceList/" + ref_joint_order[i] + "/Position/Sensor/Value");
    //   }
    // }

    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.setEncoderValues(qIn);
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > m_timeStep)
    {
      LOG_WARNING("[Sensors] Loop time " << elapsed * 1000 << " exeeded timestep " << m_timeStep << " ms");
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
};

void MCControlNAO::servo(const bool state)
{
  m_servo = state;

  AL::ALValue names(activeJoints);

  // XXX sets stiffness to max
  if (m_servo)
  {
    al_fastdcm->callVoid("setStiffness", 1.);
    // AL::ALValue joint_stiffness(std::vector<float>(activeJoints.size(), 1));
    // // Uncomment for doom
    // al_motion->setStiffnesses(names, joint_stiffness);
  }
  else
  {
    al_fastdcm->callVoid("setStiffness", 0.);
    // AL::ALValue joint_zero_stiffness(std::vector<float>(activeJoints.size(), 0.));
    // al_motion->setStiffnesses(names, joint_zero_stiffness);
  }
}

bool MCControlNAO::running() { return m_running; }
void MCControlNAO::start()
{
  if (!init)
  {
    LOG_INFO("Init controller");
    m_controller.init(qIn);
    init = true;
  }
  m_controller.running = true;
}
void MCControlNAO::stop()
{
  m_controller.running = false;
  LOG_INFO("Controller Stopped");
}
mc_control::MCGlobalController& MCControlNAO::controller() { return m_controller; }
} /* mc_nao */
