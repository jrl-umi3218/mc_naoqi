#include "MCControlNAOqi.h"
#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <mc_rtc/ros.h>
#include <ros/ros.h>

namespace po = boost::program_options;
using namespace mc_rtc_naoqi;

namespace
{
  bool open_grippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(1);
    return true;
  }

  bool close_grippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(0);
    return true;
  }

  bool set_gripper(mc_control::MCGlobalController & controller, std::stringstream & args)
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

  bool set_joint_pos(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    LOG_INFO("Set joint pos called");
    std::string jn;
    double v;
    args >> jn >> v;
    LOG_INFO("Setting joint pos for " << jn << " with val " << v);
    return controller.set_joint_pos(jn, v);
  }

  bool get_joint_pos(mc_control::MCGlobalController & controller, std::stringstream & args)
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

  bool move_com(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    double x, y, z = 0;
    args >> x >> y >> z;
    return controller.move_com(Eigen::Vector3d(x,y,z));
  }

  bool play_next_stance(mc_control::MCGlobalController & controller, std::stringstream &)
  {
    return controller.play_next_stance();
  }

  bool GoToHalfSitPose(mc_control::MCGlobalController & controller, std::stringstream &)
  {
    return controller.GoToHalfSitPose_service();
  }

  bool ChangeController(mc_control::MCGlobalController & controller, std::stringstream &args)
  {
    std::string name;
    args >> name;
    return controller.EnableController(name);
  }

  bool send_msg(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    return controller.send_msg(args.str());
  }

  bool send_recv_msg(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string out;
    bool r = controller.send_recv_msg(args.str(), out);
    LOG_INFO("Controller response:" << std::endl << out)
    return r;
  }

  std::map<std::string, std::function<bool(mc_control::MCGlobalController&, std::stringstream&)>> cli_fn = {
    {"set_joint_pos", std::bind(&set_joint_pos, std::placeholders::_1, std::placeholders::_2)},
    {"get_joint_pos", std::bind(&get_joint_pos, std::placeholders::_1, std::placeholders::_2) },
    {"open_grippers", std::bind(&open_grippers, std::placeholders::_1, std::placeholders::_2)},
    {"close_grippers", std::bind(&close_grippers, std::placeholders::_1, std::placeholders::_2)},
    {"set_gripper", std::bind(&set_gripper, std::placeholders::_1, std::placeholders::_2)},
    {"move_com", std::bind(&move_com, std::placeholders::_1, std::placeholders::_2)},
    {"play_next_stance", std::bind(&play_next_stance, std::placeholders::_1, std::placeholders::_2)},
    {"hs", std::bind(&GoToHalfSitPose, std::placeholders::_1, std::placeholders::_2)},
    {"cc", std::bind(&ChangeController, std::placeholders::_1, std::placeholders::_2)},
    {"send_msg", std::bind(&send_msg, std::placeholders::_1, std::placeholders::_2)},
    {"send_recv_msg", std::bind(&send_recv_msg, std::placeholders::_1, std::placeholders::_2)}
  };
}

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
    if(token == "stop")
    {
      LOG_INFO("Stopping experiment")
      controlNAOqi.stop();
      LOG_INFO("Experiment stopped")
    }
    else if(token == "off")
    {
      controlNAOqi.servo(false);
    }
    else if(token == "on")
    {
      controlNAOqi.servo(true);
    }
    else if(token == "cc")
    {
      std::string controller_name;
      ss >> controller_name;

      // Stop control
      controlNAOqi.stop();
      controlNAOqi.controller().running = false;
      controlNAOqi.controller().EnableController(controller_name);
    }
    else if(token == "start")
    {
      LOG_INFO("Starting experiment")
      controlNAOqi.start();
    }
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
    else
    {
      LOG_ERROR("Unknow command" << token);
    }
  }
}


int main(int argc, char **argv)
{
  std::string conf_file = mc_rtc::CONF_PATH;
  std::string host;
  unsigned int port;
  po::options_description desc("MCControlNAOqi options");
  desc.add_options()
    ("help", "display help message")
    ("host,h", po::value<std::string>(&host)->default_value("nao"), "connection host")
    ("port,p", po::value<unsigned int>(&port)->default_value(9559), "connection port")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(mc_rtc::CONF_PATH), "configuration file");


  LOG_INFO("MCControlNAOqi - Using conf: " << conf_file);
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  // radom seed for eye blinking
  srand (uint(time(NULL)));

  // Create interface object
  mc_control::MCGlobalController controller(conf_file);
  if(controller.robot().name() != "nao" && controller.robot().name() != "pepper")
  {
    LOG_ERROR("MCControlNAOqi: This program can only handle nao and pepper at the moment");
    return 1;
  }
  MCControlNAOqi mc_control_naoqi(controller, host, port);

  std::thread spin_th;
  #ifdef MC_RTC_HAS_ROS
    spin_th = std::thread([](){
        ros::Rate r(30);
        while(ros::ok())
        {
          ros::spinOnce();
          r.sleep();
        }
      });
    std::thread th(std::bind(&input_thread, std::ref(mc_control_naoqi)));
    th.join();
    spin_th.join();
  #else
    std::thread th(std::bind(&input_thread, std::ref(mc_control_naoqi)));
    th.join();
  #endif

  return 0;
}
