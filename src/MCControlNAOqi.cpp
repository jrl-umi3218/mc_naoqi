// mc_naoqi
#include "MCControlNAOqi.h"
#include "ContactForcePublisher.h"

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>


namespace mc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController& controller, std::unique_ptr<ContactForcePublisher> &cfp_ptr,
                                const std::string& host,const unsigned int port = 9559)
    : globalController(controller),
      timestep(static_cast<unsigned int>(1000 * controller.timestep())),
      interfaceRunning(true),
      cfp_ptr(cfp_ptr),
      host(host),
      port(port)
{
  /* Set up interface GUI tab */
  controllerToRun_ = globalController.current_controller();
  globalController.controller().gui()->addElement({"NAQqi"}, // Can make this element first tab in the gui
    mc_rtc::gui::StringInput("Host", [this]() { return this->host; }, [this](const std::string & in){ this->host = in; }),
    mc_rtc::gui::NumberInput("Port", [this]() { return this->port; }, [this](unsigned int in){ this->port = in; }),
    mc_rtc::gui::Button("Connect", [this]() { return; }), // implement connect/disconnect
    mc_rtc::gui::Label("Connection state", [this]() { return this->connectionState; }),
    mc_rtc::gui::StringInput("Controller", [this]()
                  { return this->controllerToRun_; },
                  [this](const std::string & in){ this->controllerToRun_ = in; }), // controller to start (e.g. Posture, FSM,...)
    mc_rtc::gui::Button(controllerButtonText_, [this]() { startOrStop(!controllerStartedState); }), // TODO sart/stop the controllerToRun_
    mc_rtc::gui::Button(servoButtonText_, [this]() { servo(!servoState); })
  );

  /* Don't start running controller before commanded `start` */
  globalController.running = false;

  /* Start ROS thread to monitor ROS topics if required */
  if(useROS){
    spin_th = std::thread(std::bind(&MCControlNAOqi::monitorROSTopic, this));
    LOG_INFO("ROS thread started")
  }

  /* Eye led anomation option */
  if(enableBlinking){
    msTillBlink = rand() % 6000 + 1000;
  }

  /* Connect to robot (real or simulation) */
  if(host != "simulation"){
    /* Create Naoqi session */
    al_broker = qi::makeSession();
    /* Try to connect via TCP to the robot */
    LOG_INFO("MCControlNAOqi: Connecting to " << globalController.robot().name()
                                              << " robot on address " << host
                                              << ":" << port);
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
    LOG_SUCCESS("Connected to " << host)
    connectionState = strstr.str() + " OK";

    /* Connect to local robot modules */
    mc_naoqi_dcm = al_broker->service("MCNAOqiDCM");
    al_launcher = al_broker->service("ALLauncher");
    if (globalController.robot().name() == "pepper"){
      al_tabletservice = al_broker->service("ALTabletService");
      /* Compute wheels jacobian for Pepper mobile base control */
      if(moveMobileBase){
        for(unsigned int i = 0; i < numWheels; i++){
          // TODO asses that the order of wheels is the same in nao_fastgetsetdcm and in wheelNames
          sva::PTransformd base_X_wheel = globalController.robot().X_b1_b2("base_link", wheelNames[i]);
          Eigen::Matrix4d hom_base_X_wheel = sva::conversions::toHomogeneous(base_X_wheel, sva::conversions::RightHanded);
          wheelsJacobian(i,0) = hom_base_X_wheel(0,1);
          wheelsJacobian(i,1) = hom_base_X_wheel(1,1);
          wheelsJacobian(i,2) = -hom_base_X_wheel(1,3)*hom_base_X_wheel(0,1) + hom_base_X_wheel(0,3)*hom_base_X_wheel(1,1);
        }
        wheelsJacobian /= -wheel_radius;
        LOG_INFO("Pepper wheels jacobian computed")
      }
    }

    /* Map sensor name to sensor index in `sensors` vector */
    std::vector<std::string> sensorsOrder = mc_naoqi_dcm.call<std::vector<std::string>>("getSensorsOrder");
    for (size_t i = 0; i < sensorsOrder.size(); i++)
    {
      LOG_INFO("Sensor[" << i << "]: " << sensorsOrder[i]);
      const auto& sensorName = sensorsOrder[i];
      sensorOrderMap[sensorName] = i;
    }

    /* Check that actuator order is the same everywhere */
    std::vector<std::string> jointsOrder = mc_naoqi_dcm.call<std::vector<std::string>>("getJointOrder");
    for(unsigned int i = 0; i<globalController.robot().refJointOrder().size();i++){
      if(globalController.robot().refJointOrder()[i] != jointsOrder[i] || jointsOrder[i] != sensorsOrder[i].substr(std::string("Encoder").length())){
        LOG_WARNING(globalController.robot().refJointOrder()[i] << "!=" << jointsOrder[i] << "!=" << sensorsOrder[i].substr(std::string("Encoder").length()))
        LOG_ERROR_AND_THROW(std::runtime_error, "Joints reference order does not match! Check the definitions in the remote and local robot modules.")
      }
    }
    LOG_INFO("Joints reference order check: OK")
  }else{
    connectionState = "virtual robot";
    LOG_WARNING("Host is '" << host << "'. Running simulation only. No connection to real robot.")
  }

  /* Control thread will run QP (every timestep ms) and send result joint commands to the robot */
  control_th = std::thread(std::bind(&MCControlNAOqi::control_thread, this));

  /* Sensor thread reads the sensor values and passes them along to `mc_rtc` (mc_observers etc) */
  if(host != "simulation"){
    /* Joints position sensor readings */
    qIn.resize(globalController.robot().refJointOrder().size());
    /* Torque for now is just electric current sensor readings */
    tauIn.resize(globalController.robot().refJointOrder().size());
    /* Allocate space for reading all sensors from the robot memory */
    numSensors = mc_naoqi_dcm.call<int>("numSensors");
    sensors.resize(numSensors);
    /* Start sensor thread */
    sensor_th = std::thread(std::bind(&MCControlNAOqi::sensor_thread, this));
  }else{
    /* Running simulation only */
    /* Setting realRobot encoder values same as control robot at the start of controller first run */
    auto & mbc = globalController.robot().mbc();
    const auto & rjo = globalController.ref_joint_order();
    for(const auto & jn : rjo){
      if(globalController.robot().hasJoint(jn)){
        for(auto & qj : mbc.q[globalController.robot().jointIndexByName(jn)]){
          qIn.push_back(qj);
        }
      }
    }
    globalController.setEncoderValues(qIn);
    globalController.realRobot().posW(globalController.robot().posW());
    //globalController.setSensorPosition(globalController.realRobot().bodyPosW("t265_pose").translation());
    //globalController.setSensorOrientation(Eigen::Quaterniond(globalController.realRobot().bodyPosW("t265_pose").rotation()));
  }

  /* First update of FB from controller */
  if(useROS){
    globalController.realRobot().posW(globalController.robot().posW());
    globalController.setSensorPosition(globalController.realRobot().bodyPosW("t265_pose").translation());
    globalController.setSensorOrientation(Eigen::Quaterniond(globalController.realRobot().bodyPosW("t265_pose").rotation()));
    /* Initialize t265 position from robot kinematics */
    init_t265_pos_from_kin = globalController.realRobot().bodyPosW("t265_pose").translation();
    init_t265_rot_from_kin = Eigen::Quaterniond(globalController.realRobot().bodyPosW("t265_pose").rotation());
  }

  LOG_INFO("MCControlNAOqi interface initialized")
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
    LOG_INFO("[Control] Waiting for sensor data");
    std::unique_lock<std::mutex> lk(control_mut);
    control_cv.wait(lk);
    LOG_INFO("[Control] Got sensor data, ready for control");
  }

  /* Vector of joint angles to be sent to robot actuators */
  std::vector<float> angles;
  angles.resize(globalController.robot().refJointOrder().size());

  while (interfaceRunning){
    auto start = std::chrono::high_resolution_clock::now();

    if (globalController.running){

      /** CONTROL loop **/
      if (globalController.run()){
        /* Get latest QP result */
        const mc_solver::QPResultMsg& res = globalController.send(0);

        /* Prepare to send desired joint angles as actuator commands */
        for (size_t i = 0; i < globalController.robot().refJointOrder().size(); ++i){
          const auto& jname = globalController.robot().refJointOrder()[i];
          angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
        }

        /* Update gripper state */
        std::map<std::string, std::vector<double>> gQs = globalController.gripperQ();
        // TODO make sure active gripper joint values are correctly put into `angles` vector (otherwise robot won't move grippers)

        /* Send actuator commands to the robot */
        if(host != "simulation"){
          mc_naoqi_dcm.call<void>("setJointAngles", angles);

          /* Prepera wheels speed command */
          if(globalController.robot().name() == "pepper" && moveMobileBase){
            mobileBaseSpeedCommand(0) = globalController.robot().mbc().alpha[0][3];
            mobileBaseSpeedCommand(1) = globalController.robot().mbc().alpha[0][4];
            mobileBaseSpeedCommand(2) = globalController.robot().mbc().alpha[0][2];
            wheelsSpeedCommand = wheelsJacobian * mobileBaseSpeedCommand;
            /* Send wheel speed commands */
            mc_naoqi_dcm.call<void>("setWheelSpeed", wheelsSpeedCommand(0), wheelsSpeedCommand(1), wheelsSpeedCommand(2));
          }
        }

        /* Update bodySensor in control thread if working in simulation mode */
        // TODO update in sensor_thread otherwise
        if(host == "simulation" and useROS){
          globalController.setSensorPosition(t265_position);
          globalController.setSensorOrientation(t265_orientation);
          globalController.setSensorLinearVelocity(t265_linvel);
          globalController.setSensorAngularVelocity(t265_angvel);
        }

        /* Publish contact forces computed by mc_rtc to ROS */
        if(cfp_ptr){
          cfp_ptr->update();
        }
      }
    }else{
      globalController.run(); // keep running the gui and plugins
    }
    /* Wait until next controller run */
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > timestep){
      LOG_WARNING("[Control] Loop time " << elapsed * 1000 << " exeeded timestep of " << timestep << " ms");
    }else{
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
  LOG_INFO("MCControlNAOqi running thread stopped");
}

void MCControlNAOqi::sensor_thread()
{
  while (interfaceRunning)
  {
    auto start = std::chrono::high_resolution_clock::now();

    /* Get all sensor readings from the robot */
    sensors = mc_naoqi_dcm.call<std::vector<float>>("getSensors");

    /* Process the sensor readings common to bith robots */
    /* Encoder values */
    const auto& ref_joint_order = globalController.robot().refJointOrder();
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

    /* Sensors specific to NAO robot */
    std::map<std::string, sva::ForceVecd> wrenches;
    if (globalController.robot().name() == "nao")
    {
      /* Feet force sensors */
      double LFsrTOTAL = sensors[sensorOrderMap["LF_FSR_TotalWeight"]];
      double RFsrTOTAL = sensors[sensorOrderMap["RF_FSR_TotalWeight"]];

      wrenches["LF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, LFsrTOTAL});
      wrenches["RF_TOTAL_WEIGHT"] = sva::ForceVecd({0., 0., 0.}, {0, 0, RFsrTOTAL});
      globalController.setWrenches(wrenches);
    }

    /* Sensors specific to Pepper robot */
    // TODO merge from topic/GenericSensor patch

    /* Send sensor readings to mc_rtc controller */
    globalController.setEncoderValues(qIn);
    if(useRobotIMU){
      globalController.setSensorAcceleration(accIn);
      globalController.setSensorAngularVelocity(rateIn);
    }
    //globalController.controller().realRobot().jointTorques(tauIn);
    globalController.setJointTorques(tauIn);

    /* Update bodySensor from tracking camera */
    if(useROS && !useRobotIMU){
      globalController.setSensorPosition(t265_position);
      globalController.setSensorOrientation(t265_orientation);
      globalController.setSensorLinearVelocity(t265_linvel);
      globalController.setSensorAngularVelocity(t265_angvel);
    }

    /* Start control only once the robot state has been read at least once */
    control_cv.notify_one();
    double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    if (elapsed * 1000 > timestep){
      LOG_WARNING("[Sensors] Loop time " << elapsed * 1000 << " exeeded timestep " << timestep << " ms");
    }
    else if(timestep - elapsed > timestep/2.0 && enableBlinking){
      /* Blink if there is enough time after sensors reading until next DCM cicle */
      auto startExtraAnimation = std::chrono::high_resolution_clock::now();
      msTillBlink -= int(timestep + elapsed * 1000);
      if(msTillBlink<=0){
        mc_naoqi_dcm.call<void>("blink");
        msTillBlink = rand() % 6000 + 1000;
      }
      double elapsedExtraAnimation = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startExtraAnimation).count();
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep - static_cast<unsigned int>(elapsed * 1000) - static_cast<unsigned int>(elapsedExtraAnimation * 1000)));
    }
    else{
      std::this_thread::sleep_for(std::chrono::milliseconds(timestep - static_cast<unsigned int>(elapsed * 1000)));
    }
  }
}

void MCControlNAOqi::servo(const bool state)
{
  if(host != "simulation"){
    bool isConnected2DCM = mc_naoqi_dcm.call<bool>("isPreProccessConnected");

    if (state) // Servo ON
    {
      LOG_WARNING("Turning ON the motors")
      /* Deactivate safety reflexes if ALMotion module is running */
      if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
        LOG_INFO("ALMotion module is active on the robot. Disabling safety reflexes...")
        try{
          qi::AnyObject al_motion = al_broker->service("ALMotion");
          al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", false);
          al_motion.call<void>("setDiagnosisEffectEnabled", false);
          al_motion.call<void>("setSmartStiffnessEnabled", false);
          al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", false);
          al_motion.call<void>("setFallManagerEnabled", false);
          if(globalController.robot().name() == "pepper"){
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
        LOG_INFO("Connected to DCM loop")
      }

      if(globalController.robot().name() == "pepper"){
        /* Set tablet image */
        al_tabletservice.call<bool>("setBrightness", 1.0);
        /* Display a local image located in /opt/aldebaran/www/apps/media/html/
           Custom image needs to be loaded to the robot first
           The ip of the robot from the tablet is 198.18.0.1 */
        al_tabletservice.call<bool>("showImage", "http://198.18.0.1/apps/media/tablet_screen.png");
      }

      /* If controller is not running, set joint angle commands to current joint state from encoders */
      if (!globalController.running)
      {
        std::vector<float> angles;
        angles.resize(globalController.robot().refJointOrder().size());
        for (size_t i = 0; i < globalController.robot().encoderValues().size(); ++i)
        {
          angles[i] = static_cast<float>(globalController.robot().encoderValues()[i]);
        }
        mc_naoqi_dcm.call<void>("setJointAngles", angles);
      }

      /* Make sure Pepper wheels actuators are not commanded to move before turning motors on */
      if(globalController.robot().name() == "pepper"){
        /* Enable mobile base safety reflex */
        if(wheelsOffOnBumperPressed){
          mc_naoqi_dcm.call<void>("bumperSafetyReflex", true);
        }

        mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
      }

      /* Gradually increase stiffness over 1s to prevent initial jerky motion */
      for (int i = 1; i <= 100; ++i)
      {
        mc_naoqi_dcm.call<void>("setStiffness", i / 100.);
        if(globalController.robot().name() == "pepper" && moveMobileBase){
          mc_naoqi_dcm.call<void>("setWheelsStiffness", i / 100.);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      servoState = true;
      servoButtonText_ = "Motors OFF";
      LOG_WARNING("Motors ON")
      // end of servo ON
    }
    else
    { /* Servo OFF */
      LOG_WARNING("Turning OFF the motors")
      /* Gradually decrease stiffness over 1s to prevent jerky motion */
      for (int i = 1; i <= 100; ++i)
      {
        mc_naoqi_dcm.call<void>("setStiffness", 1.0 - i / 100.);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(globalController.robot().name() == "pepper"){
        /* Switch off wheels */
        if(moveMobileBase){
          mc_naoqi_dcm.call<void>("setWheelsStiffness", 0.);
        }

        /* Disable mobile base safety reflex */
        if(wheelsOffOnBumperPressed){
          mc_naoqi_dcm.call<void>("bumperSafetyReflex", false);
        }

        /* Hide tablet image */
        al_tabletservice.call<void>("hideImage");
      }

      /* Disconnect the mc_rtc joint update callback from robot's DCM loop */
      if(isConnected2DCM){
        mc_naoqi_dcm.call<void>("stopLoop");
        LOG_INFO("Disconnected from DCM loop")
      }

      /* Re-activate safety reflexes */
      if(al_launcher.call<bool>("isModulePresent", "ALMotion")){
        LOG_INFO("ALMotion module is active on the robot. Re-activating safety reflexes...")
        qi::AnyObject al_motion = al_broker->service("ALMotion");
        al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", true);
        al_motion.call<void>("setDiagnosisEffectEnabled", true);
        al_motion.call<void>("setSmartStiffnessEnabled", true);
        al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", true);
        al_motion.call<void>("setFallManagerEnabled", true);
        if(globalController.robot().name() == "pepper"){
          al_motion.call<void>("setPushRecoveryEnabled", true);
        }
        LOG_INFO("Safety reflexes reactivated")
      }
      servoState = false;
      servoButtonText_ = "Motors ON";
      LOG_WARNING("Motors OFF")
      // end of servo OFF
    }
  }else{
    LOG_ERROR("Host is virtual robot, cannot turn ON/OFF motors")
  }
}

bool MCControlNAOqi::running() { return interfaceRunning; }

void MCControlNAOqi::startOrStop(const bool state)
{

  if(state){ // don't start if already started
    LOG_INFO("Starting experiment")
    /* Initialize controller with values from the encoders */
    LOG_INFO("[Control] Initializing controller");
    globalController.init(qIn);
    LOG_INFO("[Control] Controller initialized with sensor data from encoders");

    /* Start running controller */
    globalController.running = true;

    /* Change led colors */
    if (globalController.robot().name() == "pepper" && host != "simulation")
    {
      mc_naoqi_dcm.call<void>("setLeds", "eyesCenter", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "eyesPeripheral", 1.0f, 1.0f, 1.0f);
      mc_naoqi_dcm.call<void>("setLeds", "shoulderLeds", 1.0f, 0.5f, 1.0f);
      mc_naoqi_dcm.call<void>("isetLeds", "earsLeds", 1.0);

      mc_naoqi_dcm.call<void>("setWheelSpeed", 0.0, 0.0, 0.0);
    }
    controllerStartedState = true;
    controllerButtonText_ = "Stop";
    LOG_INFO("Controller stated")
    LOG_INFO("Experiment started")
  }else{
    LOG_INFO("Stopping experiment")
    /* Stop running controller */
    globalController.running = false;

    /* Change led colors */
    if (globalController.robot().name() == "pepper" && host != "simulation")
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
    controllerStartedState = false;
    controllerButtonText_ = "Start";
    LOG_INFO("Controller Stopped");
    LOG_INFO("Experiment stopped")
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

mc_control::MCGlobalController& MCControlNAOqi::controller() { return globalController; }
} /* mc_naoqi */
