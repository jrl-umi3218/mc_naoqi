#include "MCControlNAOqi.h"

// mc_rtc
#include <mc_control/mc_global_controller.h>

// std
#include <algorithm>
#include <chrono>
#include <memory>

namespace mc_rtc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController& controller, const std::string& host,
                               const unsigned int port = 9559)
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

  LOG_INFO("MCControlNAOqi: Connecting to " << m_controller.robot().name()
                                            << " robot on address " << host
                                            << ":" << portControl);
  // base speed transformation
  tf << 12.5667, -6.79404, -2.55931, -12.5667, -6.79404, -2.55931, 0.0, 14.2857, -2.42857;
  enableBlinking = true;
  msTillBlink = rand() % 6000 + 1000;

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
  al_tabletservice = al_broker->service("ALTabletService");

  al_tabletservice.call<void>("setBrightness", 1.0);
  // show the image and don't hide
  al_tabletservice.call<void>("showImage", "http://198.18.0.1/apps/boot-config/tablet_screen.png");

  std::vector<std::string> sensorsOrder = al_fastdcm.call<std::vector<std::string>>("getSensorsOrder");
  for (size_t i = 0; i < sensorsOrder.size(); i++)
  {
    LOG_INFO("Sensor[" << i << "]: " << sensorsOrder[i]);
    const auto& sensorName = sensorsOrder[i];
    sensorOrderMap[sensorName] = i;
  }

  m_controller.setSensorOrientation(Eigen::Quaterniond::Identity());
  m_controller.setSensorPosition(Eigen::Vector3d::Zero());
  // TODO: m_controller.robot().refJointOrder() == dcm.robot_module.getJointOrder() == dcm.robot_module.getSensorsOrder[1:numof joints] -> only then proceed

  qIn.resize(m_controller.robot().refJointOrder().size());
  // TODO: tau for now is just electric current sensor readings
  tauIn.resize(m_controller.robot().refJointOrder().size());
  // Allocate space for reading all sensors from the memory
  numSensors = al_fastdcm.call<int>("numSensors");
  sensors.resize(numSensors);
  // control thread will run QP (every m_timeStep ms or more often) and send result joint commands to robot
  control_th = std::thread(std::bind(&MCControlNAOqi::control_thread, this));
  // sensor thread reads the sensor values and update real robot representation in controller
  // real robot representation in controller is displayed in (RViz)
  sensor_th = std::thread(std::bind(&MCControlNAOqi::handleSensors, this));
}

// destructor (wait for control thread to finish)
MCControlNAOqi::~MCControlNAOqi() { control_th.join(); }

void MCControlNAOqi::control_thread()
{
  LOG_INFO("[Control] Waiting for sensor data");
  std::unique_lock<std::mutex> lk(control_mut);
  control_cv.wait(lk);
  LOG_INFO("[Control] Got sensor data, ready for control");

  // NB! controller is and should only be initialized when "start" is called
  //m_controller.init(qIn); // this was the segfault trouble maker TODO: double check

  std::vector<float> angles;
  // number of actual joints to control
  angles.resize(m_controller.robot().refJointOrder().size());

  // m_running seems to be always true (what is it's role? threads?)
  while (m_running)
  {
    // TODO: run it after if (m_controller.running)
    auto start = std::chrono::high_resolution_clock::now();

    if (m_controller.running)
    {
      /**
       * CONTROL loop
       **/

      // LOG_INFO("[Control] Running controller");
      if (m_controller.run())
      {
        double t = 0.;  // get latest QP result // can I remove this int?
        const mc_solver::QPResultMsg& res = m_controller.send(t);

        for (size_t i = 0; i < m_controller.robot().refJointOrder().size(); ++i)
        {
          const auto& jname = m_controller.robot().refJointOrder()[i];
          angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
          // LOG_INFO("setting " << jname << " = " << angles[i]);
        }

        // WARNING
        // Sending actuator commands to the fastgetset_dcm module,
        // in the same order as the robot module's ref_joint_order
        // Make sure that fastgetsetdcm joint order is the same!
        al_fastdcm.call<void>("setJointAngles", angles);

        // update realRobot().posW from control
        m_controller.realRobots().robot().posW(m_controller.robot().bodyPosW("base_link"));
        m_controller.realRobots().robot().velW(m_controller.robot().bodyVelW("base_link"));
      }

      double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
      if (elapsed * 1000 > m_timeStep){
        LOG_WARNING("[Control] Loop time " << elapsed * 1000 << " exeeded timestep of " << m_timeStep << " ms");
      }else{
        // TODO: is it really a good idea to sleep here?
        std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - static_cast<unsigned int>(elapsed * 1000)));
      }

      if(enableBlinking){
        msTillBlink -= int(m_timeStep + elapsed * 1000);
        if(msTillBlink<=0){
          al_fastdcm.call<void>("blink");
          msTillBlink = rand() % 6000 + 1000;
        }
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
    sensors = al_fastdcm.call<std::vector<float>>("getSensors");

    // RETRIEVE AND PARSE SENSORS COMMON TO BOTH ROBOTS FIRST

    // Get encoder values
    const auto& ref_joint_order = m_controller.robot().refJointOrder();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      qIn[i] = sensors[sensorOrderMap["Encoder" + jname]];
      // LOG_INFO("Encoder: " << jname << " = " << qIn[i]);
    }

    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      // TODO: multiply these values by current to torque constants of each motor
      tauIn[i] = sensors[sensorOrderMap["ElectricCurrent" + jname]];
      //LOG_INFO("ElectricCurrent:" << jname << " = " << tauIn[i]);
    }

    accIn(0) = sensors[sensorOrderMap["AccelerometerX"]];
    accIn(1) = sensors[sensorOrderMap["AccelerometerY"]];
    accIn(2) = sensors[sensorOrderMap["AccelerometerZ"]];
    // LOG_INFO("Accelerometer: " << accIn);

    rateIn(0) = sensors[sensorOrderMap["GyroscopeX"]];
    rateIn(1) = sensors[sensorOrderMap["GyroscopeY"]];
    rateIn(2) = sensors[sensorOrderMap["GyroscopeZ"]];

    // SENSORS SPECIFIC TO NAO ROBOT
    if (m_controller.robot().name() == "nao")
    {
      double LFsrTOTAL = sensors[sensorOrderMap["LF_FSR_TotalWeight"]];
      double RFsrTOTAL = sensors[sensorOrderMap["RF_FSR_TotalWeight"]];

      std::map<std::string, sva::ForceVecd> wrenches;
      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      m_controller.setWrenches(wrenches);
    }
    // SENSORS SPECIFIC TO PEPPER
    else if (m_controller.robot().name() == "pepper")
    {
      // TODO: remove this - not really specific to Pepper
      accIn(0) = sensors[sensorOrderMap["AccelerometerX"]];
      accIn(1) = sensors[sensorOrderMap["AccelerometerY"]];
      accIn(2) = sensors[sensorOrderMap["AccelerometerZ"]];
      // LOG_INFO("Accelerometer: " << accIn);

      rateIn(0) = sensors[sensorOrderMap["GyroscopeX"]];
      rateIn(1) = sensors[sensorOrderMap["GyroscopeY"]];
      rateIn(2) = sensors[sensorOrderMap["GyroscopeZ"]];
    }

    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.setEncoderValues(qIn);
    m_controller.setJointTorques(tauIn);

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
        angles[i] = static_cast<float>(m_controller.robot().encoderValues()[i]);
        // LOG_INFO("setting " << m_controller.robot().refJointOrder()[i] << " = " << angles[i]);
      }
      al_fastdcm.call<void>("setJointAngles", angles);
    }

    // Gradually increase stiffness over 1s to prevent initial jerky motion
    for (int i = 1; i <= 100; ++i)
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
  LOG_SUCCESS("Running start")

  // Initialize controller with values from the robot sensors
  LOG_INFO("[Control] Initializing controller");
  m_controller.init(qIn);
  LOG_INFO("[Control] Controller initialized with sensor data from the robot");

  m_controller.running = true;
  if (m_controller.robot().name() == "pepper")
  {
    al_fastdcm.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
    al_fastdcm.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
    al_fastdcm.call<void>("setLeds", "shoulderLeds", 1.0f, 0.5f, 1.0f);
    al_fastdcm.call<void>("isetLeds", "earsLeds", 1.0);

    al_fastdcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    for (int i = 1; i <= 100; ++i)
    {
      al_fastdcm.call<void>("setWheelsStiffness", i / 100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void MCControlNAOqi::stop()
{
  m_controller.running = false;
  if (m_controller.robot().name() == "pepper")
  {
    al_fastdcm.call<void>("setWheelsStiffness", 0.0);
    al_fastdcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);

    al_fastdcm.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
    al_fastdcm.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
    al_fastdcm.call<void>("setLeds", "shoulderLeds", 0.5f, 0.5f, 0.5f);
    al_fastdcm.call<void>("isetLeds", "earsLeds", 0.0);
  }
  LOG_INFO("Controller Stopped");
}
mc_control::MCGlobalController& MCControlNAOqi::controller() { return m_controller; }
} /* mc_nao */
