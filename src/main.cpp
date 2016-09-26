#include "MCControlNAO.h"
#include <iostream>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string conf_file = mc_rtc::CONF_PATH;
  std::string host = "nao";
  po::options_description desc("MCControlTCP options");
  desc.add_options()
    ("help", "display help message")
    ("host,h", po::value<std::string>(&host)->default_value("hrp4005c"), "connection host")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(mc_rtc::CONF_PATH), "configuration file");


  LOG_INFO("MCControlNAO - Using conf: " << conf_file);
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  mc_control::MCGlobalController controller(conf_file);
  if(controller.robot().name() != "nao")
  {
    LOG_ERROR("MCControlNAO: This program can only handle nao at the moment");
    return 1;
  }

  mc_nao::MCControlNAO mc_control_nao(host, controller, mc_control::Configuration(conf_file));

  return 0;
}
