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

  // led animation option
  enableBlinking = true;
  msTillBlink = rand() % 6000 + 1000;

  // Create Naoqi session
  al_broker = qi::makeSession();
  // Try to connect with TCP to robot
  try{
    std::stringstream strstr;
    strstr << "tcp://" << host << ":" << portControl;
    std::cout << "Connecting to " << host << ":" << portControl << std::endl;
    al_broker->connect(strstr.str()).wait();
  }
  catch (const std::exception& e){
    std::cout << "Cannot connect to session: " << e.what() << std::endl;
    al_broker->close();
  }
  std::cout << "Connected" << std::endl;

  // Connect to local robot modules
  al_fastdcm = al_broker->service("FastGetSetDCM");
  al_launcher = al_broker->service("ALLauncher");
  if (m_controller.robot().name() == "pepper"){
    al_tabletservice = al_broker->service("ALTabletService");
  }

  // Order of sensors readings
  std::vector<std::string> sensorsOrder = al_fastdcm.call<std::vector<std::string>>("getSensorsOrder");
  for (size_t i = 0; i < sensorsOrder.size(); i++)
  {
    LOG_INFO("Sensor[" << i << "]: " << sensorsOrder[i]);
    const auto& sensorName = sensorsOrder[i];
    sensorOrderMap[sensorName] = i;
  }

  // Initialize main robot position/orentation sensor data
  m_controller.setSensorOrientation(Eigen::Quaterniond::Identity());
  m_controller.setSensorPosition(Eigen::Vector3d::Zero());

  // Check that actuator order is the same everywhere
  std::vector<std::string> jointsOrder = al_fastdcm.call<std::vector<std::string>>("getJointOrder");
  for(unsigned int i = 0; i<m_controller.robot().refJointOrder().size();i++){
    if(m_controller.robot().refJointOrder()[i] != jointsOrder[i] || jointsOrder[i] != sensorsOrder[i].substr(std::string("Encoder").length())){
      LOG_WARNING(m_controller.robot().refJointOrder()[i] << "!=" << jointsOrder[i] << "!=" << sensorsOrder[i].substr(std::string("Encoder").length()))
      LOG_ERROR_AND_THROW(std::runtime_error, "Joints reference order does not match! Check the definitions in the remote and local robot modules.")
    }
  }
  LOG_INFO("Joints reference order check: OK")

  // Joints position sensor readings
  qIn.resize(m_controller.robot().refJointOrder().size());
  // Tau for now is just electric current sensor readings
  tauIn.resize(m_controller.robot().refJointOrder().size());
  // Allocate space for reading all sensors from the memory
  numSensors = al_fastdcm.call<int>("numSensors");
  sensors.resize(numSensors);
  // Control thread will run QP (every m_timeStep ms) and send result joint commands to robot
  control_th = std::thread(std::bind(&MCControlNAOqi::control_thread, this));
  // Sensor thread reads the sensor values (which can be used by observer to update real robot representation in controller)
  sensor_th = std::thread(std::bind(&MCControlNAOqi::handleSensors, this));
}

// Destructor
MCControlNAOqi::~MCControlNAOqi() {
  // disconnect callback from DCM
  al_fastdcm.call<void>("stopLoop");
  // wait for control thread to finish
  control_th.join();
}


void MCControlNAOqi::control_thread()
{
  LOG_INFO("[Control] Waiting for sensor data");
  std::unique_lock<std::mutex> lk(control_mut);
  control_cv.wait(lk);
  LOG_INFO("[Control] Got sensor data, ready for control");

  std::vector<float> angles;
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
        double t = 0.;  // get latest QP result
        const mc_solver::QPResultMsg& res = m_controller.send(t);

        for (size_t i = 0; i < m_controller.robot().refJointOrder().size(); ++i)
        {
          const auto& jname = m_controller.robot().refJointOrder()[i];
          angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
          // LOG_INFO("setting " << jname << " = " << angles[i]);
        }

        // Sending actuator commands to the fastgetset_dcm local module (running on the robot)
        al_fastdcm.call<void>("setJointAngles", angles);
      }

      double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
      if (elapsed * 1000 > m_timeStep){
        LOG_WARNING("[Control] Loop time " << elapsed * 1000 << " exeeded timestep of " << m_timeStep << " ms");
      }else{
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
    std::map<std::string, sva::ForceVecd> wrenches;
    if (m_controller.robot().name() == "nao")
    {
      double LFsrTOTAL = sensors[sensorOrderMap["LF_FSR_TotalWeight"]];
      double RFsrTOTAL = sensors[sensorOrderMap["RF_FSR_TotalWeight"]];

      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      m_controller.setWrenches(wrenches);
    }
    // SENSORS SPECIFIC TO PEPPER
    else if (m_controller.robot().name() == "pepper")
    {
      // Bumpers
      wrenches["BumperFrontLeft"] = sva::ForceVecd({0., 0., 0.}, {sensors[sensorOrderMap["BumperFrontLeft"]], 0., 0.});
      wrenches["BumperFrontRight"] = sva::ForceVecd({0., 0., 0.}, {sensors[sensorOrderMap["BumperFrontRight"]], 0., 0.});
      wrenches["BumperBack"] = sva::ForceVecd({0., 0., 0.}, {sensors[sensorOrderMap["BumperBack"]], 0., 0.});
      for(const auto & w : wrenches){
        m_controller.controller().realRobot().forceSensor(w.first).wrench(w.second);
      }

      // Switch off the wheels when bumpers detect contact
      if(moveMobileBase){
        if(sensors[sensorOrderMap["BumperFrontLeft"]] == 1.0 ||
            sensors[sensorOrderMap["BumperFrontRight"]] == 1.0 ||
              sensors[sensorOrderMap["BumperBack"]] == 1.0){
                al_fastdcm.call<void>("setWheelsStiffness", 0.);
                LOG_WARNING("[Sensors] Bumpers: switched off wheels")
        }
      }
    }

    // Communicate sensor readings to controller
    m_controller.setEncoderValues(qIn);
    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.controller().realRobot().jointTorques(tauIn);

    // Start control only once the robot state has been read at least once
    control_cv.notify_one();
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > m_timeStep){
      LOG_WARNING("[Sensors] Loop time " << elapsed * 1000 << " exeeded timestep " << m_timeStep << " ms");
    }
    else if(m_timeStep - elapsed > m_timeStep/2.0 ){
      // blink if there is enough spare time after sensors reading until next DCM cicle
      auto startExtraAnimation = std::chrono::high_resolution_clock::now();
      if(enableBlinking){
        msTillBlink -= int(m_timeStep + elapsed * 1000);
        if(msTillBlink<=0){
          al_fastdcm.call<void>("blink");
          msTillBlink = rand() % 6000 + 1000;
        }
      }
      double elapsedExtraAnimation = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startExtraAnimation).count();
      std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - static_cast<unsigned int>(elapsed * 1000) - static_cast<unsigned int>(elapsedExtraAnimation * 1000)));
    }
    else{
      std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
}

void MCControlNAOqi::servo(const bool state)
{
  m_servo = state;
  bool isConnected2DCM = al_fastdcm.call<bool>("isPreProccessConnected");

  if (m_servo) // Servo ON
  {
    // Deactivate safety reflexes if ALMotion module is running
    if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
      LOG_INFO("ALMotion module is active on the robot. Disabling safety reflexes...")
      try{
        qi::AnyObject al_motion = al_broker->service("ALMotion");
        al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", false);
        al_motion.call<void>("setDiagnosisEffectEnabled", false);
        al_motion.call<void>("setSmartStiffnessEnabled", false);
        al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", false);
        al_motion.call<void>("setFallManagerEnabled", false);
        if(m_controller.robot().name() == "pepper"){
          al_motion.call<void>("setPushRecoveryEnabled", false);
        }
      }catch (const std::exception& e){
        std::cout << "Cannot deactivate safety reflexes " << e.what() << std::endl;
        std::cout << "Try going to http://your_robot_ip/advanced/#/settings to enable the deactivation first" << std::endl;
      }
    }

    // connect the mc_rtc joint update callback to robot's DCM loop
    if(!isConnected2DCM){
      al_fastdcm.call<void>("startLoop");
      LOG_INFO("Connected to DCM loop")
    }

    if(m_controller.robot().name() == "pepper"){
      // set tablet image
      al_tabletservice.call<bool>("setBrightness", 1.0);
      // Display a local image located in /opt/aldebaran/www/apps/media/html/
      // Custom image needs to be loaded to robot first
      // The ip of the robot from the tablet is 198.18.0.1
      al_tabletservice.call<bool>("showImage", "http://198.18.0.1/apps/media/tablet_screen.png");
    }

    // If controller is not running, set to current joint state from encoder
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

    // Make sure Pepper wheels actuators are not commanded to move before turning motors on
    if(m_controller.robot().name() == "pepper"){
      al_fastdcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    }

    // Gradually increase stiffness over 1s to prevent initial jerky motion
    for (int i = 1; i <= 100; ++i)
    {
      al_fastdcm.call<void>("setStiffness", i / 100.);
      if(m_controller.robot().name() == "pepper" && moveMobileBase){
        al_fastdcm.call<void>("setWheelsStiffness", i / 100.);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // end of servo on
  }
  else
  { // Servo OFF
    // Gradually decrease stiffness over 1s to prevent jerky motion
    for (int i = 1; i <= 100; ++i)
    {
      al_fastdcm.call<void>("setStiffness", 1.0 - i / 100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if(m_controller.robot().name() == "pepper"){
      if(moveMobileBase){
        al_fastdcm.call<void>("setWheelsStiffness", 0.);
      }
      // Hide tablet image
      al_tabletservice.call<void>("hideImage");
    }

    // disconnect the mc_rtc joint update callback from robot's DCM loop
    if(isConnected2DCM){
      al_fastdcm.call<void>("stopLoop");
      LOG_INFO("Disconnected from DCM loop")
    }

    // Re-activate safety reflexes
    if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
      LOG_INFO("ALMotion module is active on the robot. Re-activating safety reflexes...")
      qi::AnyObject al_motion = al_broker->service("ALMotion");
      al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", true);
      al_motion.call<void>("setDiagnosisEffectEnabled", true);
      al_motion.call<void>("setSmartStiffnessEnabled", true);
      al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", true);
      al_motion.call<void>("setFallManagerEnabled", true);
      if(m_controller.robot().name() == "pepper"){
        al_motion.call<void>("setPushRecoveryEnabled", true);
      }
      LOG_INFO("Safety reflexes reactivated")
    }
  }
}

bool MCControlNAOqi::running() { return m_running; }

void MCControlNAOqi::start()
{
  if(!m_controller.running){
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
    }
  }
}

void MCControlNAOqi::stop()
{
  m_controller.running = false;
  if (m_controller.robot().name() == "pepper")
  {
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
