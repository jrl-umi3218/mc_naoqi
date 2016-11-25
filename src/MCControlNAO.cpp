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
      m_timeStep(1000 * controller.timestep()),
      m_running(true),
      init(false),
      m_wrenchesNames(controller.robot().forceSensorsByName()),
      iter_since_start(0),
      host(host),
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
      if (j != "RHipYawPitch")
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

  m_controller.setSensorOrientation(Eigen::Quaterniond::Identity());
  m_controller.setSensorPosition(Eigen::Vector3d::Zero());

  qIn.resize(m_controller.ref_joint_order().size());
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
        // LOG_INFO("NAO: RHipYawPitch: " << res.robots_state[0].q.at("RHipYawPitch")[0]);
        // LOG_INFO("NAO: LHipYawPitch: " << res.robots_state[0].q.at("LHipYawPitch")[0]);

        al_fastdcm->callVoid("setJointAngles", names, angles);
      }
    }

    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > m_timeStep)
    {
      LOG_WARNING("[Control] Loop time " << elapsed * 1000 << " exeeded timestep of " << m_timeStep << " ms");
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
  while (m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();
    AL::ALValue sensors = al_fastdcm->call<AL::ALValue>("getSensors");

    // Get encoder values
    const auto& ref_joint_order = m_controller.ref_joint_order();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      // HACK, RHipYawPitch is a joint mimic of LHipYawPitch
      if (ref_joint_order[i] == "RHipYawPitch")
      {
        qIn[i] = sensors[sensorOrderMap["Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value"]];
        // LOG_INFO("[Sensors] RHipYawPitch: " << qIn[i]);
      }
      else if (std::find(std::begin(deactivatedJoints), std::end(deactivatedJoints), ref_joint_order[i]) !=
          std::end(deactivatedJoints))
      {
        // XXX default value
        qIn[i] = 0;
      }
      else
      {
        qIn[i] = sensors[sensorOrderMap["Device/SubDeviceList/" + ref_joint_order[i] + "/Position/Sensor/Value"]];
        // if(ref_joint_order[i] == "LHipYawPitch")
        // {
        //   LOG_INFO("[Sensors] LHipYawPitch: " << qIn[i]);
        // }
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
    double LFsrTOTAL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value"]];
    double RFsrTOTAL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value"]];

    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches["LF_TOTAL"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
    wrenches["RF_TOTAL"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});

    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.setEncoderValues(qIn);
    m_controller.setWrenches(wrenches);
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
    // If controller is not running, set to current joint state
    // from encoder
    if (!m_controller.running)
    {
      AL::ALValue names(activeJoints);
      AL::ALValue angles = AL::ALValue::array(0);
      angles.arraySetSize(activeJoints.size());
      // XXX improve once GlobalController improvements are merged
      // If controller is not running, send actual joint values to robot
      // so that servo on starts at correct values
      for (unsigned int i = 0; i < activeJoints.size(); ++i)
      {
        const auto& jname = activeJoints[i];
        const auto& order = m_controller.ref_joint_order();
        size_t index = std::find(std::begin(order), std::end(order), jname) - std::begin(order);
        if (index < order.size())
        {
          angles[i] = qIn[index];
          LOG_INFO("setting " << jname << " = " << angles[i]);
        }
      }
      al_fastdcm->callVoid("setJointAngles", names, angles);
    }

    // Gradually increase stiffness over 1s to prevent initial jerky motion
    for (int i = 0; i < 100; ++i) {
      al_fastdcm->callVoid("setStiffness", i/100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  else
  { // Servo OFF
    al_fastdcm->callVoid("setStiffness", 0.);
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
