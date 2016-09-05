/*********************************************************************
 * tm_ros_wrapper.cpp
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

/* Based on original source from Thomas Timm Andersen */

/*
 * Copyright 2015 Thomas Timm Andersen
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
 */

#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_hardware_interface.h"

#include <stdio.h>
#include <string>
#include <vector>

#ifdef USE_BOOST
  #include <boost/lexical_cast.hpp>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/condition_variable.hpp>
  #include <boost/chrono/chrono.hpp>
#else
  #include <thread>
  #include <mutex>
  #include <condition_variable>
  #include <chrono>
#endif
 
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "tm_msgs/RobotStateMsgRT.h"
#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetIORequest.h"
//#include "tm_msgs/SetIOResponse.h"


class TmRosWrapper {
protected:
  TmDriver* robot;

#ifdef USE_BOOST
  boost::condition_variable msg_condv;
  boost::condition_variable msg_condv_rt;
#else
  std::condition_variable msg_condv;
  std::condition_variable msg_condv_rt;
#endif
  
  ros::NodeHandle nh_;
  
  ros::ServiceServer set_io_srv;
  control_msgs::FollowJointTrajectoryFeedback feedback;
  control_msgs::FollowJointTrajectoryResult result;
  //action_ns: joint_trajectory_action
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle;
  bool has_goal;
  
  bool use_ros_control;
  double servo_timeval;
  double max_velocity;
  double max_payload;
  std::vector<double> joint_offsets;
  std::vector<std::string> joint_names;
  std::string base_frame;
  std::string tool_frame;
  
#ifdef USE_BOOST
  boost::thread publisher_thread_rt;
  boost::thread ros_control_thread;
#else
  std::thread publisher_thread_rt;
  std::thread ros_control_thread;
#endif
  
  boost::shared_ptr<tm_ros_control::TmHardwareInterface> hw_interface;
  boost::shared_ptr<controller_manager::ControllerManager> ctrl_manager;
  
public:
  TmRosWrapper(std::string host):
  as_(nh_, "joint_trajectory_action", 
      boost::bind(&TmRosWrapper::goalCB, this, _1),
      boost::bind(&TmRosWrapper::cancelCB, this, _1),
      false
     )//,
  //robot(msg_condv, msg_condv_rt, host, 0)
  {
    robot = new TmDriver(msg_condv, msg_condv_rt, host, 0);
    
    joint_offsets.assign(6, 0.0);
    
    std::string joint_prefix = "";
    if (ros::param::get("~prefix", joint_prefix)) {
      if (joint_prefix.length() > 0) {
	print_info("Setting prefix to %s", joint_prefix.c_str());
      }
    }
    joint_names.push_back(joint_prefix + "shoulder_1_joint");
    joint_names.push_back(joint_prefix + "shoulder_2_joint");
    joint_names.push_back(joint_prefix + "elbow_1_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");  
    
    base_frame = joint_prefix + "base_link";
    tool_frame = joint_prefix + "tool0_controller";
    /*
    if (ros::param::get("~base_frame", base_frame)) {
    }
    if (ros::param::get("~tool_frame", tool_frame)) {
    }
    */
    
    use_ros_control = false;
    ros::param::get("~use_ros_control", use_ros_control);
    if(use_ros_control) {
      print_info("TM_ROS: ros_control:=True");
      hw_interface.reset(new tm_ros_control::TmHardwareInterface(nh_, robot));
      ctrl_manager.reset(new controller_manager::ControllerManager(hw_interface.get(), nh_));
    }
    else {
      print_info("TM_ROS: ros_control:=False");
    }
    
    servo_timeval = 0.01;
    ros::param::get("~servo_time", servo_timeval);
    print_info("TM_ROS: servo_time:= %.4f [sec]", servo_timeval);
    robot->setServoTimeval(servo_timeval);
    
    max_velocity = 10.0;
    ros::param::get("~max_velocity", max_velocity);
    print_info("TM_ROS: max_velocity:= %.4f [rad/s]", max_velocity);
    
    max_payload = 3.0;
    ros::param::get("~max_payload", max_payload);
    print_info("TM_ROS: max_payload:= %.4f [rad/s]", max_payload);
    
    if(robot->start()) {
      if(use_ros_control) {
#ifdef USE_BOOST
	ros_control_thread = boost::thread(boost::bind(&TmRosWrapper::rosControlLoop, this));
#else
	ros_control_thread = std::thread(boost::bind(&TmRosWrapper::rosControlLoop, this));
#endif
      }
      else {
	has_goal = false;
	as_.start();
#ifdef USE_BOOST
	publisher_thread_rt = boost::thread(boost::bind(&TmRosWrapper::publishMsgRT, this));
#else
	publisher_thread_rt = std::thread(boost::bind(&TmRosWrapper::publishMsgRT, this));
#endif
      }
      //TODO io_srv payload_srv
      set_io_srv =  nh_.advertiseService("tm_driver/set_io", &TmRosWrapper::setIO, this);
      
    }
  }
  
  ~TmRosWrapper() {
    printf("[debug] TM_ROS: TMRosWrapper DEstructor\n");
    halt();
    delete robot;
  }
  
  void halt() {
    printf("[ info] TM_ROS: halt\n");
    robot->halt();
    if(publisher_thread_rt.joinable()) {
      publisher_thread_rt.join();
    }
  }

private:
  void trajThread(std::vector<double> timestamps,
		  std::vector<std::vector<double> > positions,
		  std::vector<std::vector<double> > velocities
		 ) {
    robot->runTraj(timestamps, positions, velocities);
    if(has_goal) {
      result.error_code = result.SUCCESSFUL;
      goal_handle.setSucceeded(result);
      has_goal = false;
    }
    printf("[ info] TM_ROS: trajThread thread finished\n");
  }
  
  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    //std::string buf;
    print_info("on goal");
    
    //TODO return condition
    if(!(robot->interface->getSocketDescription() > 0)) {
      result.error_code = -100;
      result.error_string = "Cannot accept new trajectories. Robot is not connected";
      gh.setRejected(result, result.error_string);
      print_error(result.error_string.c_str());
      return;
    }
    
    unsigned char err;
    if(robot->interface->stateRT->getError(err)) {
      robot->setRobotRun();
      result.error_code = -100;
      result.error_string = "Cannot accept new trajectories. Robot is in error state";
      gh.setRejected(result, result.error_string);
      print_error(result.error_string.c_str());
      return;
    }
    
    if(has_goal) {
      print_warning("Received new goal while still executing previous trajectory. Canceling previous trajectory");
      has_goal = false;
      robot->stopTraj();
      result.error_code = -100;
      result.error_string = "Received another trajectory";
      goal_handle.setAborted(result, result.error_string);
#ifdef USE_BOOST
      boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
#endif
    }
    
    //make a copy that we can modify
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *gh.getGoal();
    goal_handle = gh;
    
    //TODO return condition
    /*
    if(!validateJointNames()) {
      std::string joint_names_op = "";
      for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
	joint_names_op += goal.trajectory.joint_names[i] + " ";
      }
      result.error_code = result.INVALID_JOINTS;
      result.error_string = "Received a goal with incorrect joint names: " + joint_names_op;
      gh.setRejected(result, result.error_string);
      print_error(result.error_string.c_str());
      return;
    }
    */
    
    if(!has_positions()) {
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Received a goal without positions";
      print_error(result.error_string.c_str());
      return;
    }
    
    if(!has_velocities()) {
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Received a goal without velocities";
      print_error(result.error_string.c_str());
      return;
    }
    
    if(!has_limited_velocities()) {
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Received a goal with velocities that are higher than " + CONVERT_TO_STRING(max_velocity);
      print_error(result.error_string.c_str());
      return;
    }
    
    if(!traj_is_finite()) {
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Received a goal with infinities or NaNs";
      print_error(result.error_string.c_str());
      return;
    }
    
    reorder_traj_joints(goal.trajectory);
    
    if(!start_positions_match(goal.trajectory, 0.01)) { //0.573 deg
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Goal start doesn't match current pose";
      gh.setRejected(result, result.error_string);
      print_error(result.error_string.c_str());
      return;
    }
    
    if(goal.trajectory.points.size() > 60) {
      result.error_code = result.INVALID_GOAL;
      result.error_string = "Number of trajectory point is >60";
      gh.setRejected(result, result.error_string);
      print_error(result.error_string.c_str());
      return;
    }
    
    std::vector<double> qVec;
    std::vector<double> timestamps;
    std::vector<std::vector<double> > positions, velocities;
    if(goal.trajectory.points[0].time_from_start.toSec() != 0.0) {
      print_warning("Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory");
      timestamps.push_back(0.0);
      robot->interface->stateRT->getQAct(qVec);
      positions.push_back(qVec);
      robot->interface->stateRT->getQdAct(qVec);
      velocities.push_back(qVec);
    }
    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      timestamps.push_back(goal.trajectory.points[i].time_from_start.toSec());
      positions.push_back(goal.trajectory.points[i].positions);
      velocities.push_back(goal.trajectory.points[i].velocities);
    }
    show_traj_info(timestamps, positions, velocities);
    
    goal_handle.setAccepted();
    has_goal = true;
    
#ifdef USE_BOOST
    boost::thread(boost::bind(&TmRosWrapper::trajThread, this, timestamps, positions, velocities)).detach();
#else
    std::thread(&TmRosWrapper::trajThread, this, timestamps, positions, velocities).detach();
#endif
  }
  
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    // set the action state to preempted
    print_info("on cancel");
    if(has_goal) {
      if(gh == goal_handle) {
	robot->stopTraj();
	has_goal = false;
      }
    }
    result.error_code = -100;
    result.error_string = "Goal cancelled by client";
    gh.setCanceled(result);
  }

  void show_traj_info(std::vector<double> timestamps,
		      std::vector<std::vector<double> > positions,
		      std::vector<std::vector<double> > velocities
		     ) {
    char msg[256];
    if(positions[0].size() >= 6) {
      print_info("----");
      for(unsigned int i = 0; i < timestamps.size(); i++) {
	print_info("traj_pt [%d] T: < %lf >", i, timestamps[i]);
	snprintf(msg, 256, "traj_pt [%d] P: < %.6lf, %.6lf, %.6lf, %.6lf, %.6lf, %.6lf >",
		 i, positions[i][0], positions[i][1], positions[i][2], positions[i][3], positions[i][4], positions[i][5]);
        print_info(msg);
        snprintf(msg, 256, "traj_pt [%d] V: < %.6lf, %.6lf, %.6lf, %.6lf, %.6lf, %.6lf >",
		 i, velocities[i][0], velocities[i][1], velocities[i][2], velocities[i][3], velocities[i][4], velocities[i][5]);
	print_info(msg);
	print_info("----");
      }
    }
  }
  
  void reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
    /* Reorders trajectory - destructive */
    std::vector<unsigned int> mapping;
    mapping.resize(joint_names.size(), joint_names.size());
    for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
      for (unsigned int j = 0; j < joint_names.size(); j++) {
	if (traj.joint_names[i] == joint_names[j])
	  mapping[j] = i;
      }
    }
    std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
    for (unsigned int i = 0; i < traj.points.size(); i++) {
      trajectory_msgs::JointTrajectoryPoint new_point;
      for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
	new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
	new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
	if (traj.points[i].accelerations.size() != 0)
	  new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
      }
      new_point.time_from_start = traj.points[i].time_from_start;
      new_traj.push_back(new_point);
    }
    traj.points = new_traj;
  }
  bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps) {
    std::vector<double> qActual;
    robot->interface->stateRT->getQAct(qActual);
    for(unsigned int i = 0; i < traj.points[0].positions.size(); i++) {
      if(fabs(traj.points[0].positions[i] - qActual[i]) > eps) {
	return false;
      }
    }
    return true;
  }  

  bool validateJointNames() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *goal_handle.getGoal();
    if(goal.trajectory.joint_names.size() != joint_names.size())
      return false;
    
    for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
      unsigned int j;
      for(j = 0; j < joint_names.size(); j++) {
	if(goal.trajectory.joint_names[i] == joint_names[j])
	  break;
      }
      if(goal.trajectory.joint_names[i] == joint_names[j]) {
	joint_names.erase(joint_names.begin() + j);
      }
      else {
	return false;
      }
    }
    return true;
  }
  bool has_velocities() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *goal_handle.getGoal();
    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      if(goal.trajectory.points[i].positions.size() != goal.trajectory.points[i].velocities.size())
	return false;
    }
    return true;
  }
  bool has_positions() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *goal_handle.getGoal();
    if(goal.trajectory.points.size() == 0)
      return false;
    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      if(goal.trajectory.points[i].positions.size() != goal.trajectory.joint_names.size())
	return false;
    }
    return true;
  }
  bool has_limited_velocities() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *goal_handle.getGoal();
    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      for(unsigned int j = 0; j < goal.trajectory.points[i].velocities.size(); j++) {
	if(fabs(goal.trajectory.points[i].velocities[j]) > max_velocity)
	  return false;
      }
    }
    return true;
  }
  bool traj_is_finite() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *goal_handle.getGoal();
    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      for(unsigned int j = 0; j < goal.trajectory.points[i].velocities.size(); j++) {
	if(!std::isfinite(goal.trajectory.points[i].positions[j]))
	  return false;
	if(!std::isfinite(goal.trajectory.points[i].velocities[j]))
	  return false;
      }
    }
    return true;
  }
  
  void rosControlLoop() {
    static const double BILLION = 1000000000.0;
    struct timespec current_time, last_time;
    ros::Duration elapsed_time;
    
    clock_gettime(CLOCK_MONOTONIC, &last_time);
    while(ros::ok()) {
#ifdef USE_BOOST
      boost::mutex msg_lock;
      boost::unique_lock<boost::mutex> locker(msg_lock);
#else
      std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
      std::unique_lock<std::mutex> locker(msg_lock);
#endif
      
      while(!robot->interface->stateRT->getDataUpdated()) {
	msg_condv_rt.wait(locker);
      }
      clock_gettime(CLOCK_MONOTONIC, &current_time);
      elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec +
				   (current_time.tv_nsec - last_time.tv_nsec) / BILLION
				  );
      last_time = current_time;
      // Input
      hw_interface->read();
      // Control
      ctrl_manager->update(ros::Time(current_time.tv_sec, current_time.tv_nsec), elapsed_time);
      // Output
      hw_interface->write();
    }
  }
  
  bool setIO(tm_msgs::SetIORequest& req, tm_msgs::SetIOResponse& res) {
    bool b = false, ret = false;
    if(req.ch < 0) {
      res.success = ret;
      return ret;
    }
    if(req.value != 0)
      b = true;
    switch(req.fun) {
      case 0://tm_msgs::SetIORequest::FUN_SET_MB_DIGITAL_OUT:
	print_info("TM_ROS: setting mb DO...");
	if(req.ch < 12) {
	  ret = robot->setDigitalOutputMB(req.ch, b);
	}
	break;
      case 1://tm_msgs::SetIORequest::FUN_SET_MB_ANALOG_OUT:
	if(req.ch < 2) {
	}
	break;
      case 2://tm_msgs::SetIORequest::FUN_SET_EE_DIGITAL_OUT:
	print_info("TM_ROS: setting ee DO...");
	if(req.ch < 4) {
	  ret = robot->setDigitalOutputEE(req.ch, b);
	}
	break;
      case 3://tm_msgs::SetIORequest::FUN_SET_EE_ANALOG_OUT:
	if(req.ch < 0) {
	}
	break;
    }
    res.success = ret;
    if(!ret)
      print_warning("TM_ROS: set io failed");
    return ret;
  }
  
  void publishMsgRT() {
    static tf::TransformBroadcaster tf_bc;
    
    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
    ros::Publisher tool_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("tool_velocity", 1);
    ros::Publisher tool_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("tool_position", 1);
    
    sensor_msgs::JointState joint_msg;
    joint_msg.name = joint_names;
    geometry_msgs::WrenchStamped wrench_msg;
    geometry_msgs::TwistStamped tool_twist_msg;
    geometry_msgs::PoseStamped tool_pose_msg;
    tf::Quaternion quat;
    tf::Transform transform;
      
    while(ros::ok()) {
      
      double robot_state_time;
      std::vector<double> robot_state_vec;
      
#ifdef USE_BOOST
      boost::mutex msg_lock;
      boost::unique_lock<boost::mutex> locker(msg_lock);
#else
      std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
      std::unique_lock<std::mutex> locker(msg_lock);
#endif
      
      while(!robot->interface->stateRT->getDataUpdated()) {
	msg_condv_rt.wait(locker);
      }
      
      //Publish JointState
      joint_msg.header.stamp = ros::Time::now();
      robot_state_time = robot->interface->stateRT->getQAct(joint_msg.position);
      for (unsigned int i = 0; i < joint_msg.position.size(); i++) {
	joint_msg.position[i] += joint_offsets[i];
      }
      robot_state_time = robot->interface->stateRT->getQdAct(joint_msg.velocity);
      robot_state_time = robot->interface->stateRT->getQtAct(joint_msg.effort);
      joint_pub.publish(joint_msg);
      
      //Publish WrenchStamped
      wrench_msg.header.stamp = joint_msg.header.stamp;
      robot_state_time = robot->interface->stateRT->getTcpForce(robot_state_vec);
      wrench_msg.wrench.force.x = robot_state_vec[0];
      wrench_msg.wrench.force.y = robot_state_vec[1];
      wrench_msg.wrench.force.z = robot_state_vec[2];
      wrench_pub.publish(wrench_msg);
      
      //Publish velocity (tool0) 
      robot_state_time = robot->interface->stateRT->getTool0VelAct(robot_state_vec);
      tool_twist_msg.header.frame_id = base_frame;
      tool_twist_msg.header.stamp = joint_msg.header.stamp;
      tool_twist_msg.twist.linear.x = robot_state_vec[0];
      tool_twist_msg.twist.linear.y = robot_state_vec[1];
      tool_twist_msg.twist.linear.z = robot_state_vec[2];
      tool_twist_msg.twist.angular.x = robot_state_vec[3];
      tool_twist_msg.twist.angular.y = robot_state_vec[4];
      tool_twist_msg.twist.angular.z = robot_state_vec[5];
      tool_vel_pub.publish(tool_twist_msg);
      
      //Broadcast transform (tool0)
      robot_state_time = robot->interface->stateRT->getTool0PosAct(robot_state_vec);
      quat.setRPY(robot_state_vec[3], robot_state_vec[4], robot_state_vec[5]);
      tool_pose_msg.header.frame_id = base_frame;
      tool_pose_msg.header.stamp = joint_msg.header.stamp;
      tool_pose_msg.pose.position.x = robot_state_vec[0];
      tool_pose_msg.pose.position.y = robot_state_vec[1];
      tool_pose_msg.pose.position.z = robot_state_vec[2];
      tool_pose_msg.pose.orientation.x = quat.x();
      tool_pose_msg.pose.orientation.y = quat.y();
      tool_pose_msg.pose.orientation.z = quat.z();
      tool_pose_msg.pose.orientation.w = quat.w();
      tool_pos_pub.publish(tool_pose_msg);
      transform.setOrigin(tf::Vector3(
	robot_state_vec[0],
	robot_state_vec[1],
	robot_state_vec[2]));
      transform.setRotation(quat);
      tf_bc.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame, tool_frame));
      
      robot->interface->stateRT->setDataUpdated();
    }
    printf("[ info] TM_ROS: publishMsgRT thread finished\n");
  }
  
};

int main(int argc, char **argv) {
  std::string host;
  
  ros::init(argc, argv, "tm_driver");
  ros::NodeHandle nh;
  
  if(!(ros::param::get("~robot_ip_address", host))) {
    if(argc > 1) {
      host = argv[1];
    }
    else {
      exit(1);
    }
  }
  //TmRosWrapper TmRosNode(host);
  TmRosWrapper* TmRosNode = new TmRosWrapper(host);
  
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  ros::waitForShutdown();
  
  printf("[ info] TM_ROS: shutdown\n");
  
  //TmRosNode->halt();
  delete TmRosNode;
  
  //exit(0);
  return 0;
}
