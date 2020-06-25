// mc_naoqi
#include "MCControlNAOqi.h"
#include "ContactForcePublisher.h"

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>

// mc_pepper
#include <mc_pepper/devices/Speaker.h>
#include <mc_pepper/devices/TouchSensor.h>
#include <mc_pepper/devices/VisualDisplay.h>

namespace mc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController& controller, std::unique_ptr<ContactForcePublisher> &cfp_ptr,
                                const std::string& host,const unsigned int port = 9559)
    : globalController_(controller),
      timestep_(static_cast<unsigned int>(1000 * controller.timestep())),
      interfaceRunning_(true),
      cfp_ptr(cfp_ptr),
      host(host),
      port(port)
{
  /* Configure interface */
  if(!globalController_.configuration().config.has("PublishContactForces")){
    mc_rtc::log::warning("'PublishContactForces' config entry missing. Using default value: {}", publishContactForces_);
  }
  globalController_.configuration().config("PublishContactForces", publishContactForces_);


  if(!globalController_.configuration().config.has("UseRobotIMU")){
    mc_rtc::log::warning("'UseRobotIMU' config entry missing. Using default value: {}", useRobotIMU); 
  }
  globalController_.configuration().config("UseRobotIMU", useRobotIMU);

  if(!globalController_.configuration().config.has("Blinking")){
    mc_rtc::log::warning("'Blinking' config entry missing. Using default value: {}", blinking); 
  }
  globalController_.configuration().config("Blinking", blinking);

  if(!globalController_.configuration().config.has("Talking")){
    mc_rtc::log::warning("'Talking' config entry missing. Using default value: {}", talking);
  }
  globalController_.configuration().config("Talking", talking);

  if(!globalController_.configuration().config.has("MoveMobileBase")){
    mc_rtc::log::warning("'MoveMobileBase' config entry missing. Using default value: {}", moveMobileBase);
  }
  globalController_.configuration().config("MoveMobileBase", moveMobileBase);

  /* Set up interface GUI tab */
  controllerToRun_ = globalController_.current_controller();
  globalController_.controller().gui()->addElement({"NAOqi"}, // TODO make this element first tab in the gui panel if possible
    mc_rtc::gui::StringInput("Host", [this]() { return this->host; }, [this](const std::string & in){ this->host = in; }),
    mc_rtc::gui::NumberInput("Port", [this]() { return this->port; }, [this](unsigned int in){ this->port = in; }),
    mc_rtc::gui::Button("Connect", [this]() { return; }), // TODO implement connect/disconnect
    mc_rtc::gui::Label("Connection status", [this]() { return this->connectionState; }),
    mc_rtc::gui::StringInput("Controller", [this]()
                  { return this->controllerToRun_; },
                  [this](const std::string & in){ this->controllerToRun_ = in; }), // controller to start (e.g. Posture, FSM,...)
    mc_rtc::gui::Button(controllerButtonText_, [this]() { startOrStop(!controllerStartedState_); }), // TODO sart/stop the controllerToRun_
    mc_rtc::gui::Button(servoButtonText_, [this]() { servo(!servoState_); })
  );

  if(globalController_.robot().name() == "pepper"){
    globalController_.controller().gui()->addElement({"NAOqi"},
      mc_rtc::gui::Button(wheelsServoButtonText_, [this]() { wheelsServo(!wheelsServoState_); })
    );
  }

  /* Don't start running controller before commanded `start` */
  globalController_.running = false;

  /* Start a thread to monitor ROS topics if required */
  if(useROS){
    spin_th = std::thread(std::bind(&MCControlNAOqi::monitorROSTopic, this));
    mc_rtc::log::info("ROS thread started");
  }

  /* Eye led anomation option */
  if(blinking){
    msTillBlink = rand() % 6000 + 1000;
  }

  /* Connect to robot (real or simulation) */
  if(host != "simulation"){
    /* Create Naoqi session */
    al_broker = qi::makeSession();
    /* Try to connect via TCP to the robot */
    mc_rtc::log::info("MCControlNAOqi: Connecting to {} robot on address {}:{}", 
                                            globalController_.robot().name(), host, port);
    std::stringstream strstr;
    try{
      strstr << "tcp://" << host << ":" << port;
      std::cout << "Connecting to " << host << ":" << port << std::endl;
      al_broker->connect(strstr.str()).wait();
    }
    catch (const std::exception& e){
      std::cout << "Cannot connect to session: " << e.what() << std::endl;
      al_broker->close();
    }
    mc_rtc::log::success("Connected to {}", host);
    connectionState = strstr.str() + " OK";

    /* Connect to local robot modules */
    mc_naoqi_dcm = al_broker->service("MCNAOqiDCM");
    al_launcher = al_broker->service("ALLauncher");
    if (globalController_.robot().name() == "pepper"){

      /* Check  that mc_rc Robot object has visual display*/
      if(!globalController_.robot().hasDevice<mc_pepper::VisualDisplay>(displayDeviceName)){
        mc_rtc::log::warning("Pepper Robot object does not have a VisualDisplay named {}", displayDeviceName);
        mc_rtc::log::warning("Visual display functionality will not be available from controller");
        enableVisualDisplay = false;
      }
      /* Tablet service used by the mc_naoqi interface */
      al_tabletservice = al_broker->service("ALTabletService");

      /* Compute wheels jacobian for Pepper mobile base control */
      if(moveMobileBase){
        for(unsigned int i = 0; i < numWheels; i++){
          // TODO asses that the order of wheels is the same in mc_naoqi_dcm and in wheelNames
          sva::PTransformd base_X_wheel = globalController_.robot().X_b1_b2("base_link", wheelNames[i]);
          Eigen::Matrix4d hom_base_X_wheel = sva::conversions::toHomogeneous(base_X_wheel, sva::conversions::RightHanded);
          wheelsJacobian(i,0) = hom_base_X_wheel(0,1);
          wheelsJacobian(i,1) = hom_base_X_wheel(1,1);
          wheelsJacobian(i,2) = -hom_base_X_wheel(1,3)*hom_base_X_wheel(0,1) + hom_base_X_wheel(0,3)*hom_base_X_wheel(1,1);
        }
        wheelsJacobian /= -wheel_radius;
        mc_rtc::log::info("Pepper wheels jacobian computed");
      }
    }

    /* Map sensor name to sensor index in `sensors` vector */
    std::vector<std::string> sensorsOrder = mc_naoqi_dcm.call<std::vector<std::string>>("getSensorsOrder");
    for (size_t i = 0; i < sensorsOrder.size(); i++)
    {
      mc_rtc::log::info("Sensor[{}]: {}", i, sensorsOrder[i]);
      const auto& sensorName = sensorsOrder[i];
      sensorOrderMap[sensorName] = i;
    }

    /* Check that actuator order and names is the same everywhere */
    std::vector<std::string> jointsOrder = mc_naoqi_dcm.call<std::vector<std::string>>("getJointOrder");
    for(unsigned int i = 0; i<globalController_.robot().refJointOrder().size();i++){
      if(globalController_.robot().refJointOrder()[i] != jointsOrder[i] || jointsOrder[i] != sensorsOrder[i].substr(std::string("Encoder").length())){
        mc_rtc::log::error("{} != {} != {}", globalController_.robot().refJointOrder()[i], jointsOrder[i], sensorsOrder[i].substr(std::string("Encoder").length()));
        mc_rtc::log::error_and_throw<std::runtime_error>("Joints reference order does not match! Check the definitions in the remote and local robot modules");
      }
    }
    mc_rtc::log::info("Joints reference order check: OK");

    /* Check that bumpers exist in mc_rtc Robot object (thier name should be the same as in local DCM module) */
    std::vector<std::string> bumperNames = mc_naoqi_dcm.call<std::vector<std::string>>("bumperNames");
    for (auto bn : bumperNames) {
      if(!globalController_.robot().hasDevice<mc_pepper::TouchSensor>(bn)){
        mc_rtc::log::warning("Robot object does not have a TouchSensor named {}", bn);
        mc_rtc::log::warning("Bumper functionality will not be available for {}", bn);
      }else if(sensorOrderMap.find(bn) == sensorOrderMap.end()){
        mc_rtc::log::warning("Robot local DCM module does not have sensor entry for {}", bn);
        mc_rtc::log::warning("Bumper functionality will not be available for {}", bn);
      }else{
        bumpers.push_back(bn);
      }
    }

    /* Check that mc_rtc Robot object has speakers */
    if(!globalController_.robot().hasDevice<mc_pepper::Speaker>(speakerDeviceName)){
      mc_rtc::log::warning("Robot object does not have a Speaker named {}", speakerDeviceName);
      mc_rtc::log::warning("Speaker functionality will not be available");
      talking = false;
    }

  }else{
    connectionState = "virtual robot";
    mc_rtc::log::warning("Host is '{}'. Running simulation only. No connection to real robot.", host);
  }

  /* Control thread will run QP (every timestep ms) and send result joint commands to the robot */
  control_th = std::thread(std::bind(&MCControlNAOqi::control_thread, this));

  /* Sensor thread reads the sensor values and passes them along to `mc_rtc` (mc_observers etc) */
  if(host != "simulation"){
    /* Joints position sensor readings */
    qIn.resize(globalController_.robot().refJointOrder().size());
    /* Torque for now is just electric current sensor readings */
    tauIn.resize(globalController_.robot().refJointOrder().size());
    /* Allocate space for reading all sensors from the robot memory */
    numSensors = mc_naoqi_dcm.call<int>("numSensors");
    sensors.resize(numSensors);
    /* Start sensor thread */
    sensor_th = std::thread(std::bind(&MCControlNAOqi::sensor_thread, this));
  }else{
    /* Running simulation only */
    /* Setting realRobot encoder values same as control robot at the start of controller first run */
    auto & mbc = globalController_.robot().mbc();
    const auto & rjo = globalController_.ref_joint_order();
    for(const auto & jn : rjo){
      if(globalController_.robot().hasJoint(jn)){
        for(auto & qj : mbc.q[globalController_.robot().jointIndexByName(jn)]){
          qIn.push_back(qj);
        }
      }
    }
    globalController_.setEncoderValues(qIn);
    globalController_.realRobot().posW(globalController_.robot().posW());
    //globalController_.setSensorPosition(globalController_.realRobot().bodyPosW("t265_pose").translation());
    //globalController_.setSensorOrientation(Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation()));
  }

  /* First update of FB from controller */
  if(useROS){
    globalController_.realRobot().posW(globalController_.robot().posW());
    globalController_.setSensorPosition(globalController_.realRobot().bodyPosW("t265_pose").translation());
    globalController_.setSensorOrientation(Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation()));
    /* Initialize t265 position from robot kinematics */
    init_t265_pos_from_kin = globalController_.realRobot().bodyPosW("t265_pose").translation();
    init_t265_rot_from_kin = Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation());
  }

  mc_rtc::log::info("MCControlNAOqi interface initialized");
}

/* Destructor */
MCControlNAOqi::~MCControlNAOqi() {
  /* Disconnect callback from DCM */
  if(host != "simulation"){
    mc_naoqi_dcm.call<void>("stopLoop");
  }
  if(cfp_ptr){
    cfp_ptr->stop();
  }
  /* Wait for control thread to finish */
  control_th.join();
}


void MCControlNAOqi::control_thread()
{
  /* Wait for first sensor readings from sensor_thread */
  if(host != "simulation"){
    mc_rtc::log::info("[Control] Waiting for sensor data");
    std::unique_lock<std::mutex> lk(control_mut);
    control_cv.wait(lk);
    mc_rtc::log::info("[Control] Got sensor data, ready for control");
  }

  /* Vector of joint angles to be sent to robot actuators */
  std::vector<float> angles;
  angles.resize(globalController_.robot().refJointOrder().size());

  while (interfaceRunning_){
    auto start = std::chrono::high_resolution_clock::now();

    if (globalController_.running){

      /** CONTROL loop **/
      if (globalController_.run()){
        /* Get latest QP result */
        const mc_solver::QPResultMsg& res = globalController_.send(0);

        /* Prepare to send desired joint angles as actuator commands */
        for (size_t i = 0; i < globalController_.robot().refJointOrder().size(); ++i){
          const auto& jname = globalController_.robot().refJointOrder()[i];
          angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
        }

        /* Send actuator commands to the robot */
        if(host != "simulation"){
          mc_naoqi_dcm.call<void>("setJointAngles", angles);

          /* Prepera wheels speed command */
          if(globalController_.robot().name() == "pepper" && moveMobileBase){
            mobileBaseSpeedCommand(0) = globalController_.robot().mbc().alpha[0][3];
            mobileBaseSpeedCommand(1) = globalController_.robot().mbc().alpha[0][4];
            mobileBaseSpeedCommand(2) = globalController_.robot().mbc().alpha[0][2];
            wheelsSpeedCommand = wheelsJacobian * mobileBaseSpeedCommand;
            /* Send wheel speed commands */
            mc_naoqi_dcm.call<void>("setWheelSpeed", wheelsSpeedCommand(0), wheelsSpeedCommand(1), wheelsSpeedCommand(2));
          }
        }

        /* Update bodySensor in control thread if working in simulation mode */
        // TODO update in sensor_thread otherwise
        if(host == "simulation" and useROS){
          globalController_.setSensorPosition(t265_position);
          globalController_.setSensorOrientation(t265_orientation);
          globalController_.setSensorLinearVelocity(t265_linvel);
          globalController_.setSensorAngularVelocity(t265_angvel);
        }

        /* Publish contact forces computed by mc_rtc to ROS */
        if(cfp_ptr){
          cfp_ptr->update();
        }
      }
    }else{
      globalController_.run(); // keep running the gui and plugins
    }

    /* Wait until next controller run */
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > timestep_){
      mc_rtc::log::warning("[Control] Loop time {} exeeded timestep of {} ms", elapsed * 1000, timestep_);
    }else{
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
  mc_rtc::log::info("MCControlNAOqi running thread stopped");
}

void MCControlNAOqi::sensor_thread()
{
  while (interfaceRunning_)
  {
    auto start = std::chrono::high_resolution_clock::now();

    /* Get all sensor readings from the robot */
    sensors = mc_naoqi_dcm.call<std::vector<float>>("getSensors");

    /* Process the sensor readings common to bith robots */
    /* Encoder values */
    const auto& ref_joint_order = globalController_.robot().refJointOrder();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      qIn[i] = sensors[sensorOrderMap["Encoder" + jname]];
    }

    /* Electric current sensor values */
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      // TODO: multiply these values by current to torque constants of each motor
      tauIn[i] = sensors[sensorOrderMap["ElectricCurrent" + jname]];
    }

    if(useRobotIMU){
      accIn(0) = sensors[sensorOrderMap["AccelerometerX"]];
      accIn(1) = sensors[sensorOrderMap["AccelerometerY"]];
      accIn(2) = sensors[sensorOrderMap["AccelerometerZ"]];

      rateIn(0) = sensors[sensorOrderMap["GyroscopeX"]];
      rateIn(1) = sensors[sensorOrderMap["GyroscopeY"]];
      rateIn(2) = sensors[sensorOrderMap["GyroscopeZ"]];
    }

    /* Bumpers */
    for(auto& b : bumpers)
    {
      auto & bumper = globalController_.robot().device<mc_pepper::TouchSensor>(b);
      bumper.touch(sensors[sensorOrderMap[b]]);
      if(bumper.touch() && wheelsOffOnBumperPressed){
        wheelsServoState_ = false;
        wheelsServoButtonText_ = "Wheels ON";
      }
    }

    /* Speakers */
    if(talking)
    {
      auto & speaker = globalController_.robot().device<mc_pepper::Speaker>(speakerDeviceName);
      if(speaker.hasSomethingToSay())
      {
       // Non-blocking call to ALTextToSpeech
       mc_naoqi_dcm.post("sayText", speaker.say());
       mc_rtc::log::info("Saying sentence in this loop");
      }
    }

    /* Sensors specific to NAO robot */
    std::map<std::string, sva::ForceVecd> wrenches;
    if (globalController_.robot().name() == "nao")
    {
      /* Feet force sensors */
      double LFsrTOTAL = sensors[sensorOrderMap["LF_FSR_TotalWeight"]];
      double RFsrTOTAL = sensors[sensorOrderMap["RF_FSR_TotalWeight"]];

      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      globalController_.setWrenches(wrenches);
    }
    /* Devices specific to Pepper robot */
    else if (globalController_.robot().name() == "pepper")
    {
      /* VisualDisplay */
      if(enableVisualDisplay)
      {
        auto & tablet = globalController_.robot().device<mc_pepper::VisualDisplay>(displayDeviceName);
        if(tablet.newURL())
        {
          // Non-blocking call to ALTabletService
          al_tabletservice.post("showImage", tablet.display());
          mc_rtc::log::info("Showing image in this loop");
        }
        if(tablet.reset()){
          al_tabletservice.post("hideImage");
          tablet.reset(false);
        }
      }
    }

    /* Send sensor readings to mc_rtc controller */
    globalController_.setEncoderValues(qIn);
    if(useRobotIMU){
      globalController_.setSensorAcceleration(accIn);
      globalController_.setSensorAngularVelocity(rateIn);
    }
    //globalController_.controller().realRobot().jointTorques(tauIn);
    globalController_.setJointTorques(tauIn);

    /* Update bodySensor from tracking camera */
    if(useROS && !useRobotIMU){
      globalController_.setSensorPosition(t265_position);
      globalController_.setSensorOrientation(t265_orientation);
      globalController_.setSensorLinearVelocity(t265_linvel);
      globalController_.setSensorAngularVelocity(t265_angvel);
    }

    /* Start control only once the robot state has been read at least once */
    control_cv.notify_one();
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > timestep_){
      mc_rtc::log::warning("[Sensors] Loop time {} exeeded timestep {} ms", elapsed * 1000, timestep_);
    }
    else if(timestep_ - elapsed > timestep_/2.0 && blinking){
      /* Blink if there is enough time after sensors reading until next DCM cicle */
      auto startExtraAnimation = std::chrono::high_resolution_clock::now();
      msTillBlink -= int(timestep_ + elapsed * 1000);
      if(msTillBlink<=0){
        mc_naoqi_dcm.post("blink");
        msTillBlink = rand() % 6000 + 1000;
      }
      double elapsedExtraAnimation = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startExtraAnimation).count();
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000) - static_cast<unsigned int>(elapsedExtraAnimation * 1000)));
    }
    else{
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
}

void MCControlNAOqi::servo(const bool state)
{
  if(host != "simulation"){
    bool isConnected2DCM = mc_naoqi_dcm.call<bool>("isPreProccessConnected");

    if (state) // Servo ON
    {
      mc_rtc::log::warning("Turning ON the motors");
      /* Deactivate safety reflexes if ALMotion module is running */
      if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
        mc_rtc::log::info("ALMotion module is active on the robot. Disabling safety reflexes...");
        try{
          qi::AnyObject al_motion = al_broker->service("ALMotion");
          al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", false);
          al_motion.call<void>("setDiagnosisEffectEnabled", false);
          al_motion.call<void>("setSmartStiffnessEnabled", false);
          al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", false);
          al_motion.call<void>("setFallManagerEnabled", false);
          if(globalController_.robot().name() == "pepper"){
            al_motion.call<void>("setPushRecoveryEnabled", false);
          }
        }catch (const std::exception& e){
          std::cout << "Cannot deactivate safety reflexes " << e.what() << std::endl;
          std::cout << "Try going to http://your_robot_ip/advanced/#/settings to enable the deactivation first" << std::endl;
        }
      }

      /* Connect the mc_rtc joint update callback to robot's DCM loop */
      if(!isConnected2DCM){
        mc_naoqi_dcm.call<void>("startLoop");
        mc_rtc::log::info("Connected to DCM loop");
      }

      if(globalController_.robot().name() == "pepper"){
        /* Set tablet image */
        al_tabletservice.call<bool>("setBrightness", 1.0);
        /* Display a local image located in /opt/aldebaran/www/apps/media/html/
           Custom image needs to be loaded to the robot first
           The ip of the robot from the tablet is 198.18.0.1 */
        al_tabletservice.post("showImage", "http://198.18.0.1/apps/media/tablet_screen.png");
      }

      /* If controller is not running, set joint angle commands to current joint state from encoders */
      if (!globalController_.running)
      {
        std::vector<float> angles;
        angles.resize(globalController_.robot().refJointOrder().size());
        for (size_t i = 0; i < globalController_.robot().encoderValues().size(); ++i)
        {
          angles[i] = static_cast<float>(globalController_.robot().encoderValues()[i]);
        }
        mc_naoqi_dcm.call<void>("setJointAngles", angles);
      }

      if(globalController_.robot().name() == "pepper"){
        /* Enable mobile base safety reflex */
        if(wheelsOffOnBumperPressed && !wheelsOffOnBumperPressedState){
          mc_naoqi_dcm.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState);
          wheelsOffOnBumperPressedState = !wheelsOffOnBumperPressedState;
        }

        /* Make sure Pepper wheels actuators are not commanded to move before turning motors on */
        mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
      }

      /* Gradually increase stiffness over 1s to prevent initial jerky motion */
      for (int i = 1; i <= 100; ++i)
      {
        mc_naoqi_dcm.call<void>("setStiffness", i / 100.);
        if(globalController_.robot().name() == "pepper" && moveMobileBase){
          mc_naoqi_dcm.call<void>("setWheelsStiffness", i / 100.);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      servoState_ = true;
      wheelsServoState_ = true;
      servoButtonText_ = "Motors OFF";
      wheelsServoButtonText_ = "Wheels OFF";
      mc_rtc::log::warning("Motors ON");
      // end of servo ON
    }
    else
    { /* Servo OFF */
      mc_rtc::log::warning("Turning OFF the motors");
      /* Gradually decrease stiffness over 1s to prevent jerky motion */
      for (int i = 1; i <= 100; ++i)
      {
        mc_naoqi_dcm.call<void>("setStiffness", 1.0 - i / 100.);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(globalController_.robot().name() == "pepper"){
        /* Switch off wheels */
        if(moveMobileBase){
          mc_naoqi_dcm.call<void>("setWheelsStiffness", 0.);
        }

        /* Disable mobile base safety reflex */
        if(wheelsOffOnBumperPressed && wheelsOffOnBumperPressedState){
          mc_naoqi_dcm.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState);
          wheelsOffOnBumperPressedState = !wheelsOffOnBumperPressedState;
        }

        /* Hide tablet image */
        al_tabletservice.post("hideImage");
      }

      /* Disconnect the mc_rtc joint update callback from robot's DCM loop */
      if(isConnected2DCM){
        mc_naoqi_dcm.call<void>("stopLoop");
        mc_rtc::log::info("Disconnected from DCM loop");
      }

      /* Re-activate safety reflexes */
      if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
        mc_rtc::log::info("ALMotion module is active on the robot. Re-activating safety reflexes...");
        qi::AnyObject al_motion = al_broker->service("ALMotion");
        al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", true);
        al_motion.call<void>("setDiagnosisEffectEnabled", true);
        al_motion.call<void>("setSmartStiffnessEnabled", true);
        al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", true);
        al_motion.call<void>("setFallManagerEnabled", true);
        if(globalController_.robot().name() == "pepper"){
          al_motion.call<void>("setPushRecoveryEnabled", true);
        }
        mc_rtc::log::info("Safety reflexes reactivated");
      }
      servoState_ = false;
      wheelsServoState_ = false;
      servoButtonText_ = "Motors ON";
      wheelsServoButtonText_ = "Wheels ON";
      mc_rtc::log::warning("Motors OFF");
      // end of servo OFF
    }
  }else{
    mc_rtc::log::error("Host is virtual robot, cannot turn ON/OFF motors");
  }
}

void MCControlNAOqi::wheelsServo(bool state){
  if(state){
    // zero commanded velocity before turning wheels on for safety
    mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    // gradually turn on wheels actuators
    for (int i = 1; i <= 100; ++i)
    {
      mc_naoqi_dcm.call<void>("setWheelsStiffness", i / 100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wheelsServoState_ = true;
    wheelsServoButtonText_ = "Wheels OFF";
  }else{
    // switch off wheels
    mc_naoqi_dcm.call<void>("setWheelsStiffness", 0.);
    wheelsServoState_ = false;
    wheelsServoButtonText_ = "Wheels ON";
  }
  // Enable/disable bumpers safety reflex
  if(wheelsOffOnBumperPressed && !wheelsOffOnBumperPressedState){
    mc_naoqi_dcm.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState);
    wheelsOffOnBumperPressedState = !wheelsOffOnBumperPressedState;
  }
}

void MCControlNAOqi::startOrStop(const bool state)
{

  if(state){ // don't start if already started
    mc_rtc::log::info("Starting experiment");
    /* Initialize controller with values from the encoders */
    mc_rtc::log::info("[Control] Initializing controller");
    globalController_.init(qIn);
    mc_rtc::log::info("[Control] Controller initialized with sensor data from encoders");

    /* Start running controller */
    globalController_.running = true;

    /* Change led colors */
    if (globalController_.robot().name() == "pepper" && host != "simulation")
    {
      mc_naoqi_dcm.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "shoulderLeds", 1.0f, 0.5f, 1.0f);
      mc_naoqi_dcm.call<void>("isetLeds", "earsLeds", 1.0);

      mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    }
    controllerStartedState_ = true;
    controllerButtonText_ = "Stop";
    mc_rtc::log::info("Controller stated");
    mc_rtc::log::info("Experiment started");
  }else{
    mc_rtc::log::info("Stopping experiment");
    /* Stop running controller */
    globalController_.running = false;

    /* Change led colors */
    if (globalController_.robot().name() == "pepper" && host != "simulation")
    {
      mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);

      mc_naoqi_dcm.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "shoulderLeds", 0.5f, 0.5f, 0.5f);
      mc_naoqi_dcm.call<void>("isetLeds", "earsLeds", 0.0);
    }

    /* Stop contact force publishing */
    if(cfp_ptr){
      cfp_ptr->stop();
    }
    controllerStartedState_ = false;
    controllerButtonText_ = "Start";
    mc_rtc::log::info("Controller Stopped");
    mc_rtc::log::info("Experiment stopped");
  }
}

void MCControlNAOqi::monitorROSTopic(){
  // Subscribe to ROS topic
  std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
  ros::Subscriber sub = nh->subscribe("/camera_fisheye/odom/sample", 500, &MCControlNAOqi::updateBodySensor, this);
  // Monitor messages
  ros::Rate r(200);
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

void MCControlNAOqi::updateBodySensor(const nav_msgs::Odometry::ConstPtr& msg)
{
  /* Update position */
  t265_position[0] = init_t265_pos_from_kin[0] + msg->pose.pose.position.x;
  t265_position[1] = init_t265_pos_from_kin[1] + msg->pose.pose.position.y;
  t265_position[2] = init_t265_pos_from_kin[2];

  /* Update orientation */
  t265_orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                        msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z).inverse();

  /* Update linear velocity */
  t265_linvel[0] = msg->twist.twist.linear.x;
  t265_linvel[1] = msg->twist.twist.linear.y;
  t265_linvel[2] = msg->twist.twist.linear.z;

  /* Update angular velocity */
  t265_angvel[0] = msg->twist.twist.angular.x;
  t265_angvel[1] = msg->twist.twist.angular.y;
  t265_angvel[2] = msg->twist.twist.angular.z;
}

} /* mc_naoqi */
