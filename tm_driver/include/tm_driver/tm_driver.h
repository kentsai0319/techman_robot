/*********************************************************************
 * tm_driver.h
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

#ifndef _TM_DRIVER_H_
#define _TM_DRIVER_H_

#include "tm_print.h"
#include "tm_robot_state_rt.h"
#include "tm_communication.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <math.h>
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

#ifdef USE_BOOST
  #define CONVERT_TO_STRING boost::lexical_cast<std::string>
  #define THIS_THREAD_NS_NAME boost::this_thread
  #define CHRONO_NS_NAME boost::chrono
#else
  #define CONVERT_TO_STRING std::to_string
  #define THIS_THREAD_NS_NAME std::this_thread
  #define CHRONO_NS_NAME std::chrono
#endif


class TmDriver {
public:
  TmCommunication* interface;
  
#ifdef USE_BOOST
  TmDriver(boost::condition_variable& data_condv,
	   boost::condition_variable& data_condv_rt,
	   std::string robot_ip,
	   int robot_ind
	  );
#else
  TmDriver(std::condition_variable& data_condv,
	   std::condition_variable& data_condv_rt,
	   std::string robot_ip,
	   int robot_ind
	  );
#endif
  ~TmDriver();
  
  bool start();
  void halt();
  
  void setRobotIndex(int index);
  int  getRobotIndex();
  
  bool setCommandData(std::string& cmd_name, char* cmd_data);
  bool setCommandMsg(std::string& cmd_msg);

  bool setRobotRun();
  bool setRobotStop();
  
  bool setMoveJabs(const std::vector<double>& q, double blend = 0.0);
  bool setMoveJrel(const std::vector<double>& dq, double blend = 0.0);
  bool setMoveLabs(const std::vector<double>& pose, double blend = 0.0);
  bool setMoveLrel(const std::vector<double>& dpose, double blend = 0.0);
  
  void setServoTimeval(double timeval);
  bool setServoOpen(std::string type_name);
  bool setServoClose();
  bool setServoStop();
  bool setServoj(const std::vector<double>& q);
  bool setServojSpd(const std::vector<double>& qd);
  bool setServojSrvSpd(const std::vector<double>& q, const std::vector<double>& qd);
  bool setServojCubicAddPt(double time, const std::vector<double>& q, const std::vector<double>& qd);
  bool setServojCubicStart();
  bool setSpeedj(const std::vector<double>& qd);
  
  
  bool setDigitalOutputMB(unsigned char ch, bool b);
  bool setDigitalOutputEE(unsigned char ch, bool b);
  //bool setAnalogOutputMB(unsigned int ch, double val);
  //bool setAnalogOutputEE(unsigned int ch, double val);
  
  bool setPayload(double mass);
  //bool setEndEffector(std::vector<double> cm);
  
  std::vector<double> interp_cubic(double t, double T,
				   const std::vector<double>& p0, const std::vector<double>& p1,
				   const std::vector<double>& v0, const std::vector<double>& v1
				  );
  bool runTraj(std::vector<double> timestamps, 
	       std::vector<std::vector<double> > positions, 
	       std::vector<std::vector<double> > velocities
	      );
  bool runTraj_noInterp(std::vector<double> timestamps, 
			std::vector<std::vector<double> > positions, 
			std::vector<std::vector<double> > velocities
		       );
  void stopTraj();
  
private:
  const float rad2deg;
  const float m2mm;
  
  int robot_index;
  //char robot_ind_str[4];
  std::string robot_ind_str;
  std::string io_ind_str;
  
  double ee_payload;
  double ee_mass_centor[3];
  int servo_timeval;
  
  bool executing_traj;
  
  //std::mutex var_mutex;
  
  bool vector_match(const std::vector<double>& vec_a, const std::vector<double>& vec_b, double eps);
};

#endif
