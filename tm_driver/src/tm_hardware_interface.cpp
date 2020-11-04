/*********************************************************************
 * tm_hardware_interface.cpp
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 * 
 * Author: Yun-Hsuan Tsai
 */

/* Based on original source from University of Colorado, Boulder. License copied below. */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************

 Author: Dave Coleman
 */

#include "tm_driver/tm_hardware_interface.h"

namespace tm_ros_control {

TmHardwareInterface::TmHardwareInterface(ros::NodeHandle& nh, TmDriver* pRobot):
nh_(nh),
robot(pRobot),
interface_flag(no_interface_running)
{
  // Initialize shared memory and interfaces here
  init(); // this implementation loads from rosparam
  
  ROS_INFO_NAMED("tm_hardware_interface", "Loaded tm_hardware_interface.");
}

void TmHardwareInterface::init() {
  ROS_INFO_STREAM_NAMED("tm_hardware_interface", "Reading rosparams from namespace: " << nh_.getNamespace());
  
  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_names);
  if (joint_names.size() == 0) {
    ROS_FATAL_STREAM_NAMED("ur_hardware_interface",
			   "No joints found on parameter server for controller, did you load the proper yaml file?" << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }
  num_joints = joint_names.size();
  
  // Resize vectors
  joint_position.resize(num_joints);
  joint_velocity.resize(num_joints);
  joint_effort.resize(num_joints);
  joint_position_command.resize(num_joints);
  joint_velocity_command.resize(num_joints);
  pos_buf.resize(num_joints);
  vel_buf.resize(num_joints);
  tor_buf.resize(num_joints);
  
  // Initialize controller
  for (std::size_t i = 0; i < num_joints; ++i) {
    ROS_DEBUG_STREAM_NAMED("tm_hardware_interface", "Loading joint name: " << joint_names[i]);
    
    // Create joint state interface
    joint_state_interface.registerHandle(
      hardware_interface::JointStateHandle(
	joint_names[i], &joint_position[i], &joint_velocity[i], &joint_effort[i]));

    // Create position joint interface
    position_joint_interface.registerHandle(
      hardware_interface::JointHandle(
	joint_state_interface.getHandle(joint_names[i]), &joint_position_command[i]));
    
    // Create velocity joint interface
    velocity_joint_interface.registerHandle(
      hardware_interface::JointHandle(
	joint_state_interface.getHandle(joint_names[i]), &joint_velocity_command[i]));
  }
  
  registerInterface( &joint_state_interface );
  registerInterface( &position_joint_interface );
  registerInterface( &velocity_joint_interface );
}

void TmHardwareInterface::read() {
  std::size_t i;
  double robot_time;
  robot_time = robot->interface->stateRT->getQAct(pos_buf);
  robot_time = robot->interface->stateRT->getQdAct(vel_buf);
  robot_time = robot->interface->stateRT->getQtAct(tor_buf);
  for (i = 0; i < num_joints; i++) {
    joint_position[i] = pos_buf[i];
    joint_velocity[i] = vel_buf[i];
    joint_effort[i] = tor_buf[i];
  }
}

void TmHardwareInterface::write() {
  switch(interface_flag) {
    case velocity_interface_running:
      robot->setSpeedj(joint_velocity_command);
      break;
    case position_interface_running:
      robot->setServoj(joint_position_command);
      break;
  }
}

bool TmHardwareInterface::canSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
				    const std::list< hardware_interface::ControllerInfo >& stop_list
				   ) const {
  //return hardware_interface::RobotHW::canSwitch(start_list, stop_list);
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin();
       controller_it != start_list.end();
       ++controller_it
      ) {
    if (controller_it->type == "hardware_interface::VelocityJointInterface") {
      switch(interface_flag) {
	case velocity_interface_running: {
	  ROS_ERROR("%s: An interface of that type (%s) is already running",
		    controller_it->name.c_str(), controller_it->type.c_str());
	  return false;
	  break;
	}
	case position_interface_running: {
	  bool err = true;
	  for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it = stop_list.begin();
	       stop_controller_it != stop_list.end();
	       ++stop_controller_it
	      ) {
	    if (stop_controller_it->type == "hardware_interface::PositionJointInterface") {
	      err = false;
	      break;
	    }
	    if (err) {
	      ROS_ERROR("%s (type %s) can not be run simultaneously with a PositionJointInterface",
		    controller_it->name.c_str(), controller_it->type.c_str());
	    }
	  }
	  break;
	}
      }
    }
    else if (controller_it->type == "hardware_interface::PositionJointInterface") {
      switch(interface_flag) {
	case velocity_interface_running: {
	  bool err = true;
	  for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it = stop_list.begin();
	       stop_controller_it != stop_list.end();
	       ++stop_controller_it
	      ) {
	    if (stop_controller_it->type == "hardware_interface::VelocityJointInterface") {
	      err = false;
	      break;
	    }
	    if (err) {
	      ROS_ERROR("%s (type %s) can not be run simultaneously with a VelocityJointInterface",
		    controller_it->name.c_str(), controller_it->type.c_str());
	    }
	  }
	  break;
	}
	case position_interface_running: {
	  ROS_ERROR("%s: An interface of that type (%s) is already running",
		    controller_it->name.c_str(), controller_it->type.c_str());
	  return false;
	  break;
	}
      }
    }
  }//for
  return true;
  
}

void TmHardwareInterface::doSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
				   const std::list< hardware_interface::ControllerInfo >& stop_list
				  ) {
  //hardware_interface::RobotHW::doSwitch(start_list, stop_list);
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin();
       controller_it != stop_list.end();
       ++controller_it
      ) {
    if (controller_it->type == "hardware_interface::VelocityJointInterface") {
      ROS_DEBUG("Stopping velocity interface");
    }
    else if (controller_it->type == "hardware_interface::PositionJointInterface") {
      ROS_DEBUG("Stopping position interface");
    }
    interface_flag = no_interface_running;
    robot->setServoClose();
  }
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin();
       controller_it != start_list.end();
       ++controller_it
      ) {
    if (controller_it->type == "hardware_interface::VelocityJointInterface") {
      interface_flag = velocity_interface_running;
      robot->setServoOpen("speedj");
    }
    else if (controller_it->type == "hardware_interface::PositionJointInterface") {
      interface_flag= position_interface_running;
      robot->setServoOpen("servoj");
    }
  }
}

}