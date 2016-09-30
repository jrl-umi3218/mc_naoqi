#include "NAOModule.h"
#include "MCControlNAO.h"

#include <mc_rtc/logging.h>
#include <alcommon/albroker.h>

namespace mc_nao
{

NAOModule::NAOModule(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name), m_controller(nullptr)
{
  LOG_INFO("NAOModule intilializing");
  // Describe the module here. This will appear on the webpage
  setModuleDescription("NAO control module for mc_rtc");

  /**
   * Define callable methods with their descriptions:
   * This makes the method available to other cpp modules
   * and to python.
   * The name given will be the one visible from outside the module.
   * This method has no parameters or return value to describe
   * functionName(<method_name>, <class_name>, <method_description>);
   * BIND_METHOD(<method_reference>);
   */
  functionName("printHello", getName(), "Print hello to the world");
  BIND_METHOD(NAOModule::printHello);

  /**
   * addParam(<attribut_name>, <attribut_descrption>);
   * This enables to document the parameters of the method.
   * It is not compulsory to write this line.
   */
  functionName("printWord", getName(), "Print a given word.");
  addParam("word", "The word to be print.");
  BIND_METHOD(NAOModule::printWord);

  /**
   * setReturn(<return_name>, <return_description>);
   * This enables to document the return of the method.
   * It is not compulsory to write this line.
   */
  functionName("returnTrue", getName(), "Just return true");
  setReturn("boolean", "return true");
  BIND_METHOD(NAOModule::returnTrue);

  /**
   * onRobotFalling
   **/
  functionName("onRobotFalling", getName(), "Handles what to do when NAO is falling.");
  //addParam("word", "The word to be print.");
  BIND_METHOD(NAOModule::onRobotFalling);

  functionName("onRobotHasFallen", getName(), "Handles what to do when NAO has fallen.");
  //addParam("word", "The word to be print.");
  BIND_METHOD(NAOModule::onRobotHasFallen);

  /**
   * onRightBumperPressed
   **/
  functionName("onRightBumperPressed", getName(), "Handles what to do when the right foot bumper is pressed.");
  //addParam("word", "The word to be print.");
  BIND_METHOD(NAOModule::onRightBumperPressed);

  // If you had other methods, you could bind them here...
  /**
   * Bound methods can only take const ref arguments of basic types,
   * or AL::ALValue or return basic types or an AL::ALValue.
   */
  LOG_SUCCESS("NAO module initialized");
}

NAOModule::~NAOModule()
{
}

void NAOModule::setController(MCControlNAO *controller)
{
  m_controller = controller;
}

void NAOModule::init()
{
  /**
   * Init is called just after construction.
   * Do something or not
   */
  std::cout << returnTrue() << std::endl;
}


void NAOModule::printHello()
{
  std::cout << "Hello!" << std::endl;
}

void NAOModule::printWord(const std::string &word)
{
  std::cout << word << std::endl;
}

bool NAOModule::returnTrue()
{
  return true;
}

void NAOModule::onRobotHasFallen()
{
  LOG_WARNING("Robot is falling, stopping controller");
  if(m_controller) {
    m_controller->stop();
  } else {
    LOG_ERROR("NO Controller to stop");
  }
}

void NAOModule::onRobotFalling()
{
  if(m_controller)
  {
    m_controller->servo(false);
  }
}

void NAOModule::onRightBumperPressed()
{
  LOG_INFO("RightBumper pressed");
}

} /* mc_nao */
