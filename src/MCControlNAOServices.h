#pragma once

#include <memory>
#include <vector>

#include <mc_control/mc_global_controller.h>

/* pimpl pattern */
struct MCControlNAOServiceImpl;

/*! This class implements service call for the MCControlNAO executable.
 * Following mc_rtc principle, it is disabled when ROS is not available as the
 * executable is started or if ROS was not available at build time */
class MCControlNAOService
{
 public:
  MCControlNAOService(mc_control::MCGlobalController& controller);

  ~MCControlNAOService();

 private:
  std::unique_ptr<MCControlNAOServiceImpl> impl;
};
