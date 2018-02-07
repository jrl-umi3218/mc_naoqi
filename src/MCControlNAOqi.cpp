#include "MCControlNAOqi.h"

// mc_rtc
#include <mc_control/mc_global_controller.h>

// std
#include <algorithm>
#include <chrono>
#include <memory>

namespace mc_rtc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController& controller, const std::string& host, const unsigned int port = 9559)
    : m_controller(controller),
      m_timeStep(static_cast<unsigned int>(1000 * controller.timestep())),
      m_running(true),
      iter_since_start(0),
      host(host),
      portControl(port)
{
  m_controller.running = false;
  LOG_INFO("timestep : " << controller.timestep());
  LOG_INFO("m_timeStep : " << m_timeStep);

  LOG_INFO("MCControlNAOqi: Connecting to " << m_controller.robot().name() << " robot on address " << host << ":"
                                          << portControl);

  // Create Naoqi session
  al_broker = qi::makeSession();
  // Try to connect with TCP to robot
  try
  {
    std::stringstream strstr;
    strstr << "tcp://" << host << ":" << portControl;
    std::cout << "Connecting to " << host << ":" << portControl << std::endl;
    al_broker->connect(strstr.str()).wait();
  }
  catch (const std::exception& e)
  {
    std::cout << "Cannot connect to session: " << e.what() << std::endl;
    al_broker->close();
  }
  std::cout << "Connected" << std::endl;

  al_fastdcm = al_broker->service("FastGetSetDCM");

  std::vector<std::string> sensorsOrder = al_fastdcm.call<std::vector<std::string>>("getSensorsOrder");
  for (size_t i = 0; i < sensorsOrder.size(); i++)
  {
    // LOG_INFO("Sensor[" << i << "]: " << sensorsOrder[i]);
    sensorOrderMap[sensorsOrder[i]] = i;
  }

  m_controller.setSensorOrientation(Eigen::Quaterniond::Identity());
  m_controller.setSensorPosition(Eigen::Vector3d::Zero());

  qIn.resize(m_controller.robot().refJointOrder().size());
  control_th = std::thread(std::bind(&MCControlNAOqi::control_thread, this));
  sensor_th = std::thread(std::bind(&MCControlNAOqi::handleSensors, this));
}

MCControlNAOqi::~MCControlNAOqi() { control_th.join(); }
void MCControlNAOqi::control_thread()
{
  LOG_INFO("[Control] Waiting for sensor data");
  std::unique_lock<std::mutex> lk(control_mut);
  control_cv.wait(lk);
  LOG_INFO("[Control] Got sensor data, starting control");

  LOG_INFO("[Control] Initializing controller");
  m_controller.init(qIn);
  LOG_INFO("[Control] Controller initialized");

  std::vector<float> angles;
  // number of actual joints to control
  angles.resize(m_controller.robot().refJointOrder().size());

  while (m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();

    if (m_controller.running)
    {
      /**
       * CONTROL loop
       **/

      // LOG_INFO("[Control] Running controller");
      if (m_controller.run())
      {
        // FIXME Fill t
        double t = 0.;  // in nano second
        const mc_solver::QPResultMsg& res = m_controller.send(t);

        for (size_t i = 0; i < m_controller.robot().refJointOrder().size(); ++i)
        {
          const auto& jname = m_controller.robot().refJointOrder()[i];
          // XXX can we use double precision on fastgetset_dcm side?
          angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
          // LOG_INFO("setting " << jname << " = " << angles[i]);
        }

        // WARNING
        // Sending actuator commands to the fastgetset_dcm module,
        // in the same order as the robot module's ref_joint_order
        // Make sure that fastgetsetdcm joint order is the same!
        al_fastdcm.call<void>("setJointAngles", angles);
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
  }
  LOG_INFO("MCControlNAOqi running thread stopped");
}

void MCControlNAOqi::handleSensors()
{
  while (m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<float> sensors = al_fastdcm.call<std::vector<float>>("getSensors");

    // RETRIEVE AND PARSE SENSORS COMMON TO BOTH ROBOTS FIRST

    // Get encoder values
    const auto& ref_joint_order = m_controller.robot().refJointOrder();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      qIn[i] = sensors[sensorOrderMap["Device/SubDeviceList/" + ref_joint_order[i] + "/Position/Sensor/Value"]];
      // LOG_INFO(ref_joint_order[i] << " = " << qIn[i]);
    }
    // SENSORS SPECIFIC TO NAO ROBOT
    if (m_controller.robot().name() == "nao")
    {
      accIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value"]];
      accIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value"]];
      accIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"]];
      // LOG_INFO("Accelerometer: " << accIn);

      rateIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value"]];
      rateIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value"]];
      rateIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value"]];

      double LFsrTOTAL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value"]];
      double RFsrTOTAL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value"]];

      // Other available force measurements. Not used for the moment {
      // // Get The Left Foot Force Sensor Values
      // double LFsrFL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"]];
      // double LFsrFR = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value"]];
      // double LFsrBL = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value"]];
      // double LFsrBR = sensors[sensorOrderMap["Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value"]];
      // // LOG_INFO("Left FSR [Kg] " << LFsrFL << LFsrFR << LFsrBL << LFsrBR);

      // // Get The Right Foot Force Sensor Values
      // double RFsrFL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value"]];
      // double RFsrFR = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value"]];
      // double RFsrBL = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value"]];
      // double RFsrBR = sensors[sensorOrderMap["Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value"]];
      // LOG_INFO("Left FSR [Kg] " << RFsrFL << RFsrFR << RFsrBL << RFsrBR);
      // }

      std::map<std::string, sva::ForceVecd> wrenches;
      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      m_controller.setWrenches(wrenches);
    }
    // SENSORS SPECIFIC TO PEPPER
    else if (m_controller.robot().name() == "pepper")
    {
      accIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/AccelerometerX/Sensor/Value"]];
      accIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/AccelerometerY/Sensor/Value"]];
      accIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/AccelerometerZ/Sensor/Value"]];
      // LOG_INFO("Accelerometer: " << accIn);

      rateIn(0) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/GyroscopeX/Sensor/Value"]];
      rateIn(1) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/GyroscopeY/Sensor/Value"]];
      rateIn(2) = sensors[sensorOrderMap["Device/SubDeviceList/InertialSensorBase/GyroscopeZ/Sensor/Value"]];
    }

    // LOG_INFO("Gyrometer: " << rateIn);
    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.setEncoderValues(qIn);

    // Start control only once the robot state has been read at least once
    control_cv.notify_one();
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
}

void MCControlNAOqi::servo(const bool state)
{
  m_servo = state;

  if (m_servo)
  {
    // If controller is not running, set to current joint state
    // from encoder
    if (!m_controller.running)
    {
      std::vector<float> angles;
      angles.resize(m_controller.robot().refJointOrder().size());

      for (size_t i = 0; i < m_controller.robot().encoderValues().size(); ++i)
      {
        // XXX can we use double precision on fastgetset_dcm side?
        angles[i] = static_cast<float>(m_controller.robot().encoderValues()[i]);
        LOG_INFO("setting " << m_controller.robot().refJointOrder()[i] << " = " << angles[i]);
      }
      al_fastdcm.call<void>("setJointAngles", angles);
    }

    // Gradually increase stiffness over 1s to prevent initial jerky motion
    for (int i = 0; i < 100; ++i)
    {
      al_fastdcm.call<void>("setStiffness", i / 100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  else
  {  // Servo OFF
    al_fastdcm.call<void>("setStiffness", 0.);
  }
}

bool MCControlNAOqi::running() { return m_running; }
void MCControlNAOqi::start()
{
  m_controller.init(qIn);
  m_controller.running = true;
}
void MCControlNAOqi::stop()
{
  m_controller.running = false;
  LOG_INFO("Controller Stopped");
}
mc_control::MCGlobalController& MCControlNAOqi::controller() { return m_controller; }
} /* mc_nao */
