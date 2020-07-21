// mc_naoqi
#include "MCControlNAOqi.h"
#include "ContactForcePublisher.h"

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>

// mc_pepper
#include <mc_pepper/devices/Speaker.h>
#include <mc_pepper/devices/TouchSensor.h>
#include <mc_pepper/devices/VisualDisplay.h>

#include <mc_rbdyn/rpy_utils.h>

namespace mc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController& controller, std::unique_ptr<ContactForcePublisher> &cfp_ptr,
                                const std::string& host, const unsigned int port = 9559)
    : globalController_(controller),
      timestep_(static_cast<unsigned int>(1000 * controller.timestep())),
      interfaceRunning_(true),
      cfp_ptr(cfp_ptr),
      host_(host),
      port_(port)
{
  /* Configure interface */
  if(!globalController_.configuration().config.has("PublishContactForces")){
    mc_rtc::log::warning("'PublishContactForces' config entry missing. Using default value: {}", publishContactForces_);
  }
  globalController_.configuration().config("PublishContactForces", publishContactForces_);


  if(!globalController_.configuration().config.has("UseRobotIMU")){
    mc_rtc::log::warning("'UseRobotIMU' config entry missing. Using default value: {}", useRobotIMU_);
  }
  globalController_.configuration().config("UseRobotIMU", useRobotIMU_);

  if(!globalController_.configuration().config.has("Blinking")){
    mc_rtc::log::warning("'Blinking' config entry missing. Using default value: {}", blinking_);
  }
  globalController_.configuration().config("Blinking", blinking_);

  if(!globalController_.configuration().config.has("Talking")){
    mc_rtc::log::warning("'Talking' config entry missing. Using default value: {}", talking_);
  }
  globalController_.configuration().config("Talking", talking_);

  if(!globalController_.configuration().config.has("MoveMobileBase")){
    mc_rtc::log::warning("'MoveMobileBase' config entry missing. Using default value: {}", moveMobileBase_);
  }
  globalController_.configuration().config("MoveMobileBase", moveMobileBase_);

  /* Set up interface GUI tab */
  controllerToRun_ = globalController_.current_controller();
  globalController_.controller().gui()->addElement({"NAOqi"}, // TODO make this element first tab in the gui panel if possible
    mc_rtc::gui::StringInput("Host", [this]() { return this->host_; }, [this](const std::string & in){ this->host_ = in; }),
    mc_rtc::gui::NumberInput("Port", [this]() { return this->port_; }, [this](unsigned int in){ this->port_ = in; }),
    mc_rtc::gui::Button("Connect", [this]() { return; }), // TODO implement connect/disconnect
    mc_rtc::gui::Label("Connection status", [this]() { return this->connectionState_; }),
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
  if(useROS_){
    spinThread_ = std::thread(std::bind(&MCControlNAOqi::monitorROSTopic, this));
    mc_rtc::log::info("ROS thread started");
  }

  /* Eye led anomation option */
  if(blinking_){
    msTillBlink_ = rand() % 6000 + 1000;
  }

  /* Connect to robot (real or simulation) */
  if(host_ != "simulation"){
    /* Create Naoqi session */
    ALBroker_ = qi::makeSession();
    /* Try to connect via TCP to the robot */
    mc_rtc::log::info("MCControlNAOqi: Connecting to {} robot on address {}:{}",
                                            globalController_.robot().name(), host_, port_);
    std::stringstream strstr;
    try{
      mc_rtc::log::info("Connecting to {}:{}", host_, port_);
      strstr << "tcp://" << host_ << ":" << port_;
      ALBroker_->connect(strstr.str()).wait();
    }
    catch (const std::exception& e){
      mc_rtc::log::error_and_throw<std::runtime_error>("Cannot connect to session: {}", e.what());
      ALBroker_->close();
    }
    mc_rtc::log::success("Connected to {}", host_);
    connectionState_ = strstr.str() + " OK";

    /* Connect to local robot modules */
    MCNAOqiDCM_ = ALBroker_->service("MCNAOqiDCM");
    /* Check that controller main robot and real robot are of the same type */
    std::string realRobotName = MCNAOqiDCM_.call<std::string>("getRobotName");
    if(realRobotName != globalController_.robot().name()){
      mc_rtc::log::error_and_throw<std::runtime_error>("Controller main robot '{}' and the real robot '{}' are not of the same type",
                                                        globalController_.robot().name(), realRobotName);
    }
    ALlauncher_ = ALBroker_->service("ALLauncher");
    if (globalController_.robot().name() == "pepper"){

      /* Check  that mc_rc Robot object has visual display*/
      if(!globalController_.robot().hasDevice<mc_pepper::VisualDisplay>(displayDeviceName_)){
        mc_rtc::log::warning("Pepper Robot object does not have a VisualDisplay named {}", displayDeviceName_);
        mc_rtc::log::warning("Visual display functionality will not be available from controller");
        enableVisualDisplay_ = false;
      }
      /* Tablet service used by the mc_naoqi interface */
      ALTabletservice_ = ALBroker_->service("ALTabletService");

      /* Compute wheels jacobian for Pepper mobile base control */
      if(moveMobileBase_){
        for(unsigned int i = 0; i < wheelNames_.size(); i++){
          // TODO asses that the order of wheels is the same in MCNAOqiDCM_ and in wheelNames
          sva::PTransformd base_X_wheel = globalController_.robot().X_b1_b2("base_link", wheelNames_[i]);
          Eigen::Matrix4d hom_base_X_wheel = sva::conversions::toHomogeneous(base_X_wheel, sva::conversions::RightHanded);
          wheelsJacobian_(i,0) = hom_base_X_wheel(0,1);
          wheelsJacobian_(i,1) = hom_base_X_wheel(1,1);
          wheelsJacobian_(i,2) = -hom_base_X_wheel(1,3)*hom_base_X_wheel(0,1) + hom_base_X_wheel(0,3)*hom_base_X_wheel(1,1);
        }
        wheelsJacobian_ /= -wheelRadius_;
        mc_rtc::log::info("Pepper wheels jacobian computed");
      }
    }

    /* Map sensor name to sensor index in `sensors` vector */
    std::vector<std::string> sensorsOrder = MCNAOqiDCM_.call<std::vector<std::string>>("getSensorsOrder");
    for (size_t i = 0; i < sensorsOrder.size(); i++)
    {
      mc_rtc::log::info("Sensor[{}]: {}", i, sensorsOrder[i]);
      const auto& sensorName = sensorsOrder[i];
      sensorOrderMap_[sensorName] = i;
    }

    /* Check that actuator order and names is the same everywhere */
    std::vector<std::string> jointsOrder = MCNAOqiDCM_.call<std::vector<std::string>>("getJointOrder");
    for(unsigned int i = 0; i<globalController_.robot().refJointOrder().size();i++){
      if(globalController_.robot().refJointOrder()[i] != jointsOrder[i] || jointsOrder[i] != sensorsOrder[i].substr(std::string("Encoder").length())){
        mc_rtc::log::error("{} != {} != {}", globalController_.robot().refJointOrder()[i], jointsOrder[i], sensorsOrder[i].substr(std::string("Encoder").length()));
        mc_rtc::log::error_and_throw<std::runtime_error>("Joints reference order does not match! Check the definitions in the remote and local robot modules");
      }
    }
    mc_rtc::log::info("Joints reference order check: OK");

    /* Check that bumpers exist in mc_rtc Robot object (thier name should be the same as in local DCM module) */
    std::vector<std::string> bumperNames = MCNAOqiDCM_.call<std::vector<std::string>>("bumperNames");
    for (auto bn : bumperNames) {
      if(!globalController_.robot().hasDevice<mc_pepper::TouchSensor>(bn)){
        mc_rtc::log::warning("Robot object does not have a TouchSensor named {}", bn);
        mc_rtc::log::warning("Bumper functionality will not be available for {}", bn);
      }else if(sensorOrderMap_.find(bn) == sensorOrderMap_.end()){
        mc_rtc::log::warning("Robot local DCM module does not have sensor entry for {}", bn);
        mc_rtc::log::warning("Bumper functionality will not be available for {}", bn);
      }else{
        bumpers_.push_back(bn);
      }
    }

    /* Check that mc_rtc Robot object has speakers */
    if(!globalController_.robot().hasDevice<mc_pepper::Speaker>(speakerDeviceName_)){
      mc_rtc::log::warning("Robot object does not have a Speaker named {}", speakerDeviceName_);
      mc_rtc::log::warning("Speaker functionality will not be available");
      talking_ = false;
    }

  }else{
    connectionState_ = "virtual robot";
    mc_rtc::log::warning("Host is '{}'. Running simulation only. No connection to real robot.", host_);
  }

  /* Control thread will run QP (every timestep ms) and send result joint commands to the robot */
  controlThread_ = std::thread(std::bind(&MCControlNAOqi::control_thread, this));

  /* Sensor thread reads the sensor values and passes them along to `mc_rtc` (mc_observers etc) */
  if(host_ != "simulation"){
    /* Joints position sensor readings */
    qIn_.resize(globalController_.robot().refJointOrder().size());
    /* Torque for now is just electric current sensor readings */
    tauIn_.resize(globalController_.robot().refJointOrder().size());
    /* IMU readings */
    if(useRobotIMU_){
      accIn_.resize(3);
      rpyIn_.resize(3);
      rateIn_.resize(3);
    }
    /* Allocate space for reading all sensors from the robot memory */
    numSensors_ = MCNAOqiDCM_.call<int>("numSensors");
    sensors_.resize(numSensors_);
    /* Start sensor thread */
    sensorThread_ = std::thread(std::bind(&MCControlNAOqi::sensor_thread, this));
  }else{
    /* Running simulation only */
    /* Setting realRobot encoder values same as control robot at the start of controller first run */
    auto & mbc = globalController_.robot().mbc();
    const auto & rjo = globalController_.ref_joint_order();
    for(const auto & jn : rjo){
      if(globalController_.robot().hasJoint(jn)){
        for(auto & qj : mbc.q[globalController_.robot().jointIndexByName(jn)]){
          qIn_.push_back(qj);
        }
      }
    }
    globalController_.setEncoderValues(qIn_);
    globalController_.realRobot().posW(globalController_.robot().posW());
    //globalController_.setSensorPosition(globalController_.realRobot().bodyPosW("t265_pose").translation());
    //globalController_.setSensorOrientation(Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation()));
  }

  /* First update of FB from controller */
  if(useROS_){
    globalController_.realRobot().posW(globalController_.robot().posW());
    globalController_.setSensorPosition(globalController_.realRobot().bodyPosW("t265_pose").translation());
    globalController_.setSensorOrientation(Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation()));
    /* Initialize t265 position from robot kinematics */
    initT265Pos_ = globalController_.realRobot().bodyPosW("t265_pose").translation();
    initT265Ori_ = Eigen::Quaterniond(globalController_.realRobot().bodyPosW("t265_pose").rotation());
  }

  mc_rtc::log::info("MCControlNAOqi interface initialized");
}

/* Destructor */
MCControlNAOqi::~MCControlNAOqi() {
  /* Disconnect callback from DCM */
  if(host_ != "simulation"){
    MCNAOqiDCM_.call<void>("stopLoop");
  }
  if(cfp_ptr){
    cfp_ptr->stop();
  }
  /* Wait for control thread to finish */
  controlThread_.join();
}


void MCControlNAOqi::control_thread()
{
  /* Wait for first sensor readings from sensor_thread */
  if(host_ != "simulation"){
    mc_rtc::log::info("[Control] Waiting for sensor data");
    std::unique_lock<std::mutex> lk(controlMut_);
    controlCV_.wait(lk);
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
        if(host_ != "simulation"){
          MCNAOqiDCM_.call<void>("setJointAngles", angles);

          /* Prepera wheels speed command */
          if(globalController_.robot().name() == "pepper" && moveMobileBase_){
            mobileBaseSpeedCommand_(0) = globalController_.robot().mbc().alpha[0][3];
            mobileBaseSpeedCommand_(1) = globalController_.robot().mbc().alpha[0][4];
            mobileBaseSpeedCommand_(2) = globalController_.robot().mbc().alpha[0][2];
            wheelsSpeedCommand_ = wheelsJacobian_ * mobileBaseSpeedCommand_;
            /* Send wheel speed commands */
            MCNAOqiDCM_.call<void>("setWheelSpeed", wheelsSpeedCommand_(0), wheelsSpeedCommand_(1), wheelsSpeedCommand_(2));
          }
        }

        /* Update bodySensor in control thread if working in simulation mode */
        // TODO update in sensor_thread otherwise
        if(host_ == "simulation" && useROS_){
          globalController_.setSensorPosition(t265Pos_);
          globalController_.setSensorOrientation(t265Ori_);
          globalController_.setSensorLinearVelocity(t265Linvel_);
          globalController_.setSensorAngularVelocity(t265Angvel_);
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
    sensors_ = MCNAOqiDCM_.call<std::vector<float>>("getSensors");

    /* Process the sensor readings common to bith robots */
    /* Encoder values */
    const auto& ref_joint_order = globalController_.robot().refJointOrder();
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      qIn_[i] = sensors_[sensorOrderMap_["Encoder" + jname]];
    }

    /* Electric current sensor values */
    for (unsigned i = 0; i < ref_joint_order.size(); ++i)
    {
      const auto& jname = ref_joint_order[i];
      // TODO: multiply these values by current to torque constants of each motor
      tauIn_[i] = sensors_[sensorOrderMap_["ElectricCurrent" + jname]];
    }

    if(useRobotIMU_){
      accIn_(0) = sensors_[sensorOrderMap_["AccelerometerX"]];
      accIn_(1) = sensors_[sensorOrderMap_["AccelerometerY"]];
      accIn_(2) = sensors_[sensorOrderMap_["AccelerometerZ"]];

      rpyIn_(0) = sensors_[sensorOrderMap_["AngleX"]];
      rpyIn_(1) = sensors_[sensorOrderMap_["AngleY"]];
      rpyIn_(2) = sensors_[sensorOrderMap_["AngleZ"]];

      rateIn_(0) = sensors_[sensorOrderMap_["GyroscopeX"]];
      rateIn_(1) = sensors_[sensorOrderMap_["GyroscopeY"]];
      rateIn_(2) = sensors_[sensorOrderMap_["GyroscopeZ"]];
    }

    /* Bumpers */
    for(auto& b : bumpers_)
    {
      auto & bumper = globalController_.robot().device<mc_pepper::TouchSensor>(b);
      bumper.touch(sensors_[sensorOrderMap_[b]]);
      if(bumper.touch() && wheelsOffOnBumperPressed_){
        wheelsServoState_ = false;
        wheelsServoButtonText_ = "Wheels ON";
      }
    }

    /* Speakers */
    if(talking_)
    {
      auto & speaker = globalController_.robot().device<mc_pepper::Speaker>(speakerDeviceName_);
      if(speaker.hasSomethingToSay())
      {
       // Non-blocking call to ALTextToSpeech
       MCNAOqiDCM_.post("sayText", speaker.say());
       mc_rtc::log::info("Saying sentence in this loop");
      }
    }

    /* Sensors specific to NAO robot */
    std::map<std::string, sva::ForceVecd> wrenches;
    if (globalController_.robot().name() == "nao")
    {
      /* Feet force sensors */
      double LFsrTOTAL = sensors_[sensorOrderMap_["LF_FSR_TotalWeight"]];
      double RFsrTOTAL = sensors_[sensorOrderMap_["RF_FSR_TotalWeight"]];

      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      globalController_.setWrenches(wrenches);
    }
    /* Devices specific to Pepper robot */
    else if (globalController_.robot().name() == "pepper")
    {
      /* VisualDisplay */
      if(enableVisualDisplay_)
      {
        auto & tablet = globalController_.robot().device<mc_pepper::VisualDisplay>(displayDeviceName_);
        if(tablet.newURL())
        {
          // Non-blocking call to ALTabletService
          ALTabletservice_.post("showImage", tablet.display());
          mc_rtc::log::info("Showing image in this loop");
        }
        if(tablet.reset()){
          ALTabletservice_.post("hideImage");
          tablet.reset(false);
        }
      }
    }

    /* Send sensor readings to mc_rtc controller */
    globalController_.setEncoderValues(qIn_);
    if(useRobotIMU_){
      globalController_.setSensorAcceleration(accIn_);
      globalController_.setSensorOrientation(Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpyIn_)));
      globalController_.setSensorAngularVelocity(rateIn_);
    }
    //globalController_.controller().realRobot().jointTorques(tauIn_);
    globalController_.setJointTorques(tauIn_);

    /* Update bodySensor from tracking camera */
    if(useROS_ && !useRobotIMU_){
      globalController_.setSensorPosition(t265Pos_);
      globalController_.setSensorOrientation(t265Ori_);
      globalController_.setSensorLinearVelocity(t265Linvel_);
      globalController_.setSensorAngularVelocity(t265Angvel_);
    }

    /* Start control only once the robot state has been read at least once */
    controlCV_.notify_one();
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > timestep_){
      mc_rtc::log::warning("[Sensors] Loop time {} exeeded timestep {} ms", elapsed * 1000, timestep_);
    }
    else if(timestep_ - elapsed > timestep_/2.0 && blinking_){
      /* Blink if there is enough time after sensors reading until next DCM cicle */
      auto startExtraAnimation = std::chrono::high_resolution_clock::now();
      msTillBlink_ -= int(timestep_ + elapsed * 1000);
      if(msTillBlink_<=0){
        MCNAOqiDCM_.post("blink");
        msTillBlink_ = rand() % 6000 + 1000;
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
  if(host_ != "simulation"){
    bool isConnected2DCM = MCNAOqiDCM_.call<bool>("isPreProccessConnected");

    if (state) // Servo ON
    {
      mc_rtc::log::warning("Turning ON the motors");
      /* Deactivate safety reflexes if ALMotion module is running */
      if(ALlauncher_.call<bool>("isModulePresent", "ALMotion")){
        mc_rtc::log::info("ALMotion module is active on the robot. Disabling safety reflexes...");
        try{
          qi::AnyObject al_motion = ALBroker_->service("ALMotion");
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
        MCNAOqiDCM_.call<void>("startLoop");
        mc_rtc::log::info("Connected to DCM loop");
      }

      if(globalController_.robot().name() == "pepper"){
        /* Set tablet image */
        ALTabletservice_.call<bool>("setBrightness", 1.0);
        /* Display a local image located in /opt/aldebaran/www/apps/media/html/
           Custom image needs to be loaded to the robot first
           The ip of the robot from the tablet is 198.18.0.1 */
        ALTabletservice_.post("showImage", "http://198.18.0.1/apps/media/tablet_screen.png");
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
        MCNAOqiDCM_.call<void>("setJointAngles", angles);
      }

      if(globalController_.robot().name() == "pepper"){
        /* Enable mobile base safety reflex */
        if(wheelsOffOnBumperPressed_ && !wheelsOffOnBumperPressedState_){
          MCNAOqiDCM_.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState_);
          wheelsOffOnBumperPressedState_ = !wheelsOffOnBumperPressedState_;
        }

        /* Make sure Pepper wheels actuators are not commanded to move before turning motors on */
        MCNAOqiDCM_.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
      }

      /* Gradually increase stiffness over 1s to prevent initial jerky motion */
      for (int i = 1; i <= 100; ++i)
      {
        MCNAOqiDCM_.call<void>("setStiffness", i / 100.);
        if(globalController_.robot().name() == "pepper" && moveMobileBase_){
          MCNAOqiDCM_.call<void>("setWheelsStiffness", i / 100.);
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
        MCNAOqiDCM_.call<void>("setStiffness", 1.0 - i / 100.);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(globalController_.robot().name() == "pepper"){
        /* Switch off wheels */
        if(moveMobileBase_){
          MCNAOqiDCM_.call<void>("setWheelsStiffness", 0.);
        }

        /* Disable mobile base safety reflex */
        if(wheelsOffOnBumperPressed_ && wheelsOffOnBumperPressedState_){
          MCNAOqiDCM_.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState_);
          wheelsOffOnBumperPressedState_ = !wheelsOffOnBumperPressedState_;
        }

        /* Hide tablet image */
        ALTabletservice_.post("hideImage");
      }

      /* Disconnect the mc_rtc joint update callback from robot's DCM loop */
      if(isConnected2DCM){
        MCNAOqiDCM_.call<void>("stopLoop");
        mc_rtc::log::info("Disconnected from DCM loop");
      }

      /* Re-activate safety reflexes */
      if(ALlauncher_.call<bool>("isModulePresent", "ALMotion")){
        mc_rtc::log::info("ALMotion module is active on the robot. Re-activating safety reflexes...");
        qi::AnyObject al_motion = ALBroker_->service("ALMotion");
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
    MCNAOqiDCM_.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    // gradually turn on wheels actuators
    for (int i = 1; i <= 100; ++i)
    {
      MCNAOqiDCM_.call<void>("setWheelsStiffness", i / 100.);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wheelsServoState_ = true;
    wheelsServoButtonText_ = "Wheels OFF";
  }else{
    // switch off wheels
    MCNAOqiDCM_.call<void>("setWheelsStiffness", 0.);
    wheelsServoState_ = false;
    wheelsServoButtonText_ = "Wheels ON";
  }
  // Enable/disable bumpers safety reflex
  if(wheelsOffOnBumperPressed_ && !wheelsOffOnBumperPressedState_){
    MCNAOqiDCM_.call<void>("bumperSafetyReflex", !wheelsOffOnBumperPressedState_);
    wheelsOffOnBumperPressedState_ = !wheelsOffOnBumperPressedState_;
  }
}

void MCControlNAOqi::startOrStop(const bool state)
{

  if(state){ // don't start if already started
    mc_rtc::log::info("Starting experiment");
    /* Initialize controller with values from the encoders */
    mc_rtc::log::info("[Control] Initializing controller");
    globalController_.init(qIn_);
    mc_rtc::log::info("[Control] Controller initialized with sensor data from encoders");

    /* Start running controller */
    globalController_.running = true;

    /* Change led colors */
    if (globalController_.robot().name() == "pepper" && host_ != "simulation")
    {
      MCNAOqiDCM_.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
      MCNAOqiDCM_.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
      MCNAOqiDCM_.call<void>("setLeds", "shoulderLeds", 1.0f, 0.5f, 1.0f);
      MCNAOqiDCM_.call<void>("isetLeds", "earsLeds", 1.0);

      MCNAOqiDCM_.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
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
    if (globalController_.robot().name() == "pepper" && host_ != "simulation")
    {
      MCNAOqiDCM_.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);

      MCNAOqiDCM_.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
      MCNAOqiDCM_.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
      MCNAOqiDCM_.call<void>("setLeds", "shoulderLeds", 0.5f, 0.5f, 0.5f);
      MCNAOqiDCM_.call<void>("isetLeds", "earsLeds", 0.0);
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
  t265Pos_[0] = initT265Pos_[0] + msg->pose.pose.position.x;
  t265Pos_[1] = initT265Pos_[1] + msg->pose.pose.position.y;
  t265Pos_[2] = initT265Pos_[2];

  /* Update orientation */
  t265Ori_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                        msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z).inverse();

  /* Update linear velocity */
  t265Linvel_[0] = msg->twist.twist.linear.x;
  t265Linvel_[1] = msg->twist.twist.linear.y;
  t265Linvel_[2] = msg->twist.twist.linear.z;

  /* Update angular velocity */
  t265Angvel_[0] = msg->twist.twist.angular.x;
  t265Angvel_[1] = msg->twist.twist.angular.y;
  t265Angvel_[2] = msg->twist.twist.angular.z;
}

} /* mc_naoqi */
