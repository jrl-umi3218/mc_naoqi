#include "MCControlNAOServices.h"

#include <mc_rtc/ros.h>

// ROS includes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mc_tcp_msgs/EnableController.h>
#include <mc_tcp_msgs/close_grippers.h>
#include <mc_tcp_msgs/open_grippers.h>
#include <mc_tcp_msgs/play_next_stance.h>
#include <mc_tcp_msgs/send_msg.h>
#include <mc_tcp_msgs/send_recv_msg.h>
#include <mc_tcp_msgs/set_gripper.h>
#include <mc_tcp_msgs/set_joint_pos.h>
#include <ros/ros.h>
#pragma GCC diagnostic pop

struct MCControlNAOServiceImpl
{
  MCControlNAOServiceImpl(mc_control::MCGlobalController &controller)
      : controller(controller), nh(mc_rtc::ROSBridge::get_node_handle())
  {
    if (nh)
    {
      LOG_SUCCESS("Starting ROS services")
      start_services();
    }
    else
    {
      LOG_WARNING("[MCControlNAO] ROS not available, services will not be enabled")
    }
  }

 private:
  void start_services()
  {
    services.push_back(
        nh->advertiseService("mc_rtc_nao/EnableController", &MCControlNAOServiceImpl::EnableController_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/close_grippers", &MCControlNAOServiceImpl::close_grippers_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/open_grippers", &MCControlNAOServiceImpl::open_grippers_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/set_gripper", &MCControlNAOServiceImpl::set_gripper_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/set_joint_pos", &MCControlNAOServiceImpl::set_joint_pos_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/play_next_stance", &MCControlNAOServiceImpl::play_next_stance_callback, this));
    services.push_back(nh->advertiseService("mc_rtc_nao/send_msg", &MCControlNAOServiceImpl::send_msg_callback, this));
    services.push_back(
        nh->advertiseService("mc_rtc_nao/send_recv_msg", &MCControlNAOServiceImpl::send_recv_msg_callback, this));
  }

  bool EnableController_callback(mc_tcp_msgs::EnableController::Request &req,
                                 mc_tcp_msgs::EnableController::Response &resp)
  {
    resp.success = controller.EnableController(req.name);
    return true;
  }

  bool close_grippers_callback(mc_tcp_msgs::close_grippers::Request &, mc_tcp_msgs::close_grippers::Response &resp)
  {
    controller.setGripperOpenPercent(0.);
    resp.success = true;
    return true;
  }

  bool open_grippers_callback(mc_tcp_msgs::open_grippers::Request &, mc_tcp_msgs::open_grippers::Response &resp)
  {
    controller.setGripperOpenPercent(1.);
    resp.success = true;
    return true;
  }

  bool set_gripper_callback(mc_tcp_msgs::set_gripper::Request &req, mc_tcp_msgs::set_gripper::Response &resp)
  {
    controller.setGripperTargetQ(req.gname, req.values);
    resp.success = true;
    return true;
  }

  bool set_joint_pos_callback(mc_tcp_msgs::set_joint_pos::Request &req, mc_tcp_msgs::set_joint_pos::Response &resp)
  {
    resp.success = controller.set_joint_pos(req.jname, req.q);
    return true;
  }

  bool play_next_stance_callback(mc_tcp_msgs::play_next_stance::Request &,
                                 mc_tcp_msgs::play_next_stance::Response &resp)
  {
    resp.success = controller.play_next_stance();
    return true;
  }

  bool send_msg_callback(mc_tcp_msgs::send_msg::Request &req, mc_tcp_msgs::send_msg::Response &resp)
  {
    resp.success = controller.send_msg(req.msg);
    return true;
  }

  bool send_recv_msg_callback(mc_tcp_msgs::send_recv_msg::Request &req, mc_tcp_msgs::send_recv_msg::Response &resp)
  {
    resp.success = controller.send_recv_msg(req.msg, resp.msg);
    return true;
  }

  mc_control::MCGlobalController &controller;
  std::shared_ptr<ros::NodeHandle> nh;
  std::vector<ros::ServiceServer> services;
};

MCControlNAOService::MCControlNAOService(mc_control::MCGlobalController &controller)
    : impl(new MCControlNAOServiceImpl(controller))
{
}

MCControlNAOService::~MCControlNAOService() {}
