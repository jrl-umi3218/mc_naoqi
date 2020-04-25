#include "MCControlNAOqi.h"

#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace mc_naoqi;

namespace
{
  /* Open fully all robot grippers */
  bool openGrippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(1);
    return true;
  }

  /* Fully close all robot grippers */
  bool closeGrippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(0);
    return true;
  }

  /* Set particular gripper opening to a given value (e.g. sg r_gripper 0.5) */
  bool setGripper(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string gripper; std::vector<double> v; double tmp;
    args >> gripper;
    while(args.good())
    {
      args >> tmp;
      v.push_back(tmp);
    }
    controller.setGripperTargetQ(gripper, v);
    return true;
  }

  /* Get current position of a particular joint in robot().mbc() (e.g. gjp KneePitch) */
  bool getJointPos(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string jn;
    args >> jn;
    if (controller.robot().hasJoint(jn))
    {
      LOG_INFO(jn << ": " << controller.robot().mbc().q[controller.robot().jointIndexByName(jn)][0]);
    }
    else
    {
      LOG_ERROR("No joint named " << jn << " in the robot");
    }
    return true;
  }

  /* Enable HalfSitPose controller */
  bool GoToHalfSitPose(mc_control::MCGlobalController & controller, std::stringstream &)
  {
    return controller.GoToHalfSitPose_service();
  }

  /* Change controller */
  bool ChangeController(mc_control::MCGlobalController & controller, std::stringstream &args)
  {
    std::string name;
    args >> name;
    return controller.EnableController(name);
  }

  /* Map common functions to brief terminal commands */
  std::map<std::string, std::function<bool(mc_control::MCGlobalController&, std::stringstream&)>> cli_fn = {
    {"gjp", std::bind(&getJointPos, std::placeholders::_1, std::placeholders::_2) },
    {"og", std::bind(&openGrippers, std::placeholders::_1, std::placeholders::_2)},
    {"cg", std::bind(&closeGrippers, std::placeholders::_1, std::placeholders::_2)},
    {"sg", std::bind(&setGripper, std::placeholders::_1, std::placeholders::_2)},
    {"hs", std::bind(&GoToHalfSitPose, std::placeholders::_1, std::placeholders::_2)},
    {"cc", std::bind(&ChangeController, std::placeholders::_1, std::placeholders::_2)}
  };
}

/* Thread that processes commands for the inferface given throught the terminal */
void input_thread(MCControlNAOqi & controlNAOqi)
{
  while(controlNAOqi.running())
  {
    std::string ui;
    std::getline(std::cin, ui);
    std::stringstream ss;
    ss << ui;
    std::string token;
    ss >> token;
    /* Start the controller (if not started), stop otwerwise */
    if(token == "s")
    {
      controlNAOqi.startOrStop(!controlNAOqi.controllerStartedState);
    }
    /* Switch on robot motors */
    else if(token == "on")
    {
      controlNAOqi.servo(true);
    }
    /* Switch off robot motors */
    else if(token == "off")
    {
      controlNAOqi.servo(false);
    }
    /* Change controller (not tested) */
    else if(token == "cc")
    {
      std::string controller_name;
      ss >> controller_name;
      controlNAOqi.startOrStop(false);
      controlNAOqi.controller().running = false;
      controlNAOqi.controller().EnableController(controller_name);
    }
    /* (Re)start slam */
    else if(token == "ss")
    {
      LOG_INFO("Initializing V-SLAM")
      // TODO set realRobot().posW() to Identity (back to world origin)
      // ... Assuming realRobot is not moving
      // ... Get realRobot joints state, compute t265_pose in world frame
      // ... Use t265_pose to set initial position of T265 sensor in world frame
      // ... restart T265 V-SLAM
    }
    /* Run one of the common functions (cli_fn) */
    else if(cli_fn.count(token))
    {
      std::string rem;
      std::getline(ss, rem);
      boost::algorithm::trim(rem);
      std::stringstream ss2;
      ss2 << rem;
      bool ret = cli_fn[token](controlNAOqi.controller(), ss2);
      if (!ret)
      {
        LOG_ERROR("Failed to invoke the previous command");
      }
    }
    /* Any other input from terminal is unknown command */
    else
    {
      LOG_ERROR("Unknow command" << token);
    }
  }
}


/* Main function of the interface */
int main(int argc, char **argv)
{
  /* Prepare to publish desired contact forces to ROS if reqested */
  std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
  auto nh_p = *nh;
  std::unique_ptr<ContactForcePublisher> cfp_ptr = nullptr;

  /* Set command line arguments options */
  /* Usage example: ./src/mc_naoqi -h simulation -f ../etc/mc_rtc_pepper.yaml*/
  std::string conf_file = mc_rtc::CONF_PATH;
  std::string host;
  unsigned int port;
  po::options_description desc("MCControlNAOqi options");
  desc.add_options()
    ("info,i", "display help message")
    ("host,h", po::value<std::string>(&host)->default_value("nao"), "connection host")
    ("port,p", po::value<unsigned int>(&port)->default_value(9559), "connection port")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(mc_rtc::CONF_PATH), "configuration file");

  /* Parse command line arguments */
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if(vm.count("info")){
    std::cout << desc << std::endl;
    return 1;
  }
  LOG_INFO("MCControlNAOqi - Using conf: " << conf_file);

  /* Create global controller */
  mc_control::MCGlobalController controller(conf_file);
  /* Check that the interface can work with the main controller robot */
  if(controller.robot().name() != "NAO" && controller.robot().name() != "pepper"){
    LOG_ERROR("MCControlNAOqi: This program can only handle nao and pepper at the moment");
    return 1;
  }
  /* Create MCControlNAOqi interface */
  MCControlNAOqi mc_control_naoqi(controller, cfp_ptr, host, port);

  /* Create contact force publisher */
  if(mc_control_naoqi.publish_contact_forces){
    cfp_ptr.reset(new ContactForcePublisher(nh_p, controller));
  }

  // Set radom seed for eye blinking
  srand (uint(time(NULL)));

  /* Start terminal input thread */
  std::thread th(std::bind(&input_thread, std::ref(mc_control_naoqi)));
  th.join();

  return 0;
}
