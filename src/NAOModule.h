#pragma once

#include <iostream>
#include <alcommon/almodule.h>

namespace AL
{
  // This is a forward declaration of AL:ALBroker which
  // avoids including <alcommon/albroker.h> in this header
  class ALBroker;
}

namespace mc_nao
{
class MCControlNAO;

/**
 * This class inherits AL::ALModule. This allows it to bind methods
 * and be run as a remote executable within NAOqi
 */
class NAOModule : public AL::ALModule
{
public:
  NAOModule(boost::shared_ptr<AL::ALBroker> broker, const std::string &name);

  virtual ~NAOModule();

  void setController(MCControlNAO *controller);

  /**
   * Overloading ALModule::init().
   * This is called right after the module has been loaded
   */
  virtual void init();

  // After that you may add all your bind method.

  // Function which prints "Hello!" on standard output
  void printHello();
  // Function which prints the word given on parameters
  void printWord(const std::string &word);

  void onRobotFalling();
  void onRobotHasFallen();
  void onRightBumperPressed();

  // Function which returns true
  bool returnTrue();
private:
  MCControlNAO *m_controller;
};

} /* mc_nao */
