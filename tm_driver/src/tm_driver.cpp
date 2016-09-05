/*********************************************************************
 * tm_driver.cpp
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

#include "tm_driver/tm_driver.h"

#ifdef USE_BOOST
TmDriver::TmDriver(boost::condition_variable& data_condv,
		   boost::condition_variable& data_condv_rt,
		   std::string robot_ip,
		   int robot_ind
		  ):
#else
TmDriver::TmDriver(std::condition_variable& data_condv,
		   std::condition_variable& data_condv_rt,
		   std::string robot_ip,
		   int robot_ind
		  ):
#endif
m2mm(1000.0f),
rad2deg((float)57.2957795131)
{
  char IP[64];
  char Port[64] = "6188";
  if(robot_ip.length() < 1 || robot_ip.length() >= 64) {
    print_error("err");
    return;
  }
  print_debug("TM_DRV: TMDriver CONstructor");
  strcpy(IP, robot_ip.c_str());
  interface = new TmCommunication(data_condv, data_condv_rt, IP, Port, 5000, 500);
  
  setRobotIndex(robot_ind);
  
  ee_payload = 0.0;
  ee_mass_centor[0] = 0.0;
  ee_mass_centor[1] = 0.0;
  ee_mass_centor[2] = 0.0;
  
  servo_timeval = 10;
  executing_traj = false;
}

TmDriver::~TmDriver() {
  printf("[debug] TM_DRV: TmDriver DEstructor\n");
  delete interface;
}

bool TmDriver::start() {
  if(!interface->start()) {
    return false;
  }
  //show something
  return true;
}

void TmDriver::halt() {
  printf("[ info] TM_DRV: halt\n");
  if(executing_traj){
    stopTraj();
  }
  interface->halt();
}

void TmDriver::setRobotIndex(int index){
  //char ind[4];
  //memset(ind, 0, 4);
  robot_index = index;
  //snprintf(ind, 4, "%d", robot_index);
  //memcpy(robot_ind_str, ind, 4);
  robot_ind_str = CONVERT_TO_STRING(robot_index) + " ";
  io_ind_str = CONVERT_TO_STRING(8 + 8*robot_index) + " ";
}

int TmDriver::getRobotIndex() {
 return robot_index;
}

bool TmDriver::setCommandData(std::string& cmd_name, char* cmd_data) {
  unsigned int n = (unsigned int)cmd_name.size() + (unsigned int)strlen(cmd_data);
  if(n == 0 || n >= g_tmcom_max_command_size) {
    return false;
  }
  return (interface->sendCommandData(cmd_name.c_str(), cmd_data) > 0);
}

bool TmDriver::setCommandMsg(std::string& cmd_msg) {
  unsigned int n = (unsigned int)cmd_msg.length();
  print_debug(cmd_msg.c_str());
  if(n == 0 || n >= g_tmcom_max_command_size) {
    return false;
  }
  return (interface->sendCommandMsg(cmd_msg.c_str()) > 0);
}

bool TmDriver::setRobotRun() {
  return (interface->sendCommandMsg("run") > 0);
}

bool TmDriver::setRobotStop() {
  return (interface->sendCommandMsg("stop") > 0);
}

bool TmDriver::setMoveJabs(const std::vector<double>& q, double blend) {
  unsigned int dof = interface->stateRT->getDOF();
  if(q.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "movj " + robot_ind_str;
  for(unsigned int i = 0; i < dof; i++) {
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.4f ", rad2deg * ((float)q[i]));
    cmd_msg += var_str;
  }
  if(blend != 0) {
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.1f ", 100.0f * (float)blend);
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setMoveJrel(const std::vector<double>& dq, double blend) {
  unsigned int dof = interface->stateRT->getDOF();
  if(dq.size() != dof) {
    return false;
  }
  std::vector<double> q;
  interface->stateRT->getQAct(q);
  for(unsigned int i = 0; i < q.size(); i++) {
    q[i] += dq[i];
  }
  return setMoveJabs(q, blend);
}
bool TmDriver::setMoveLabs(const std::vector<double>& pose, double blend) {
  float fc;
  if(pose.size() != 6) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "movl " + robot_ind_str;
  for(unsigned int i = 0; i < 6; i++) {
    if(i < 3)
      fc = m2mm;
    else
      fc = rad2deg;
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.4f ", fc * (float)pose[i]);
    cmd_msg += var_str;
  }
  if(blend != 0) {
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.1f ", 100.0f * (float)blend);
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setMoveLrel(const std::vector<double>& dpose, double blend) {
  
  if(dpose.size() != 6) {
    return false;
  }
  std::vector<double> pose;
  interface->stateRT->getToolPosAct(pose);
  for(unsigned int i = 0; i < pose.size(); i++) {
    pose[i] += dpose[i];
  }
  return setMoveJabs(pose, blend);
}

void TmDriver::setServoTimeval(double timeval) {
  if(timeval < 0.001)
    servo_timeval = 0;
  else
    servo_timeval = (int)(timeval * 1000.0);
  print_info("TM_DRV: servo_timeval:=%d", servo_timeval);
}

bool TmDriver::setServoOpen(std::string type_name) {
  bool ret = false, fg = false;
  std::string cmd_msg;
  if(type_name.compare(0, 6, "servoj") == 0) {
    cmd_msg = "nrtservo " + robot_ind_str + type_name + " ";
    fg = true;
  }
  else if(type_name.compare(0, 6, "speedj") == 0) {
    cmd_msg = "nrtservo " + robot_ind_str + type_name + " ";
    fg = true;
  }
  if(fg) {
    if(interface->sendCommandMsg(cmd_msg.c_str()) > 0)
      ret = true;
  }
  return ret;
}
bool TmDriver::setServoClose() {
  std::string cmd_msg = "nrtservo " + robot_ind_str + "close ";
  if(interface->sendCommandMsg(cmd_msg.c_str()) > 0)
    return true;
  else
    return false;
}
bool TmDriver::setServoStop() {
  std::string cmd_msg = "nrtservo " + robot_ind_str + "stop ";
  if(interface->sendCommandMsg(cmd_msg.c_str()) > 0)
    return true;
  else
    return false;
  return false;
}
bool TmDriver::setServoj(const std::vector<double>& q) {
  unsigned int dof = interface->stateRT->getDOF();
  if(q.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "srvj " + robot_ind_str + "1 ";
  for(unsigned int i = 0; i < dof; i++) {
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)q[i]);
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setServojSpd(const std::vector<double>& qd) {
  unsigned int dof = interface->stateRT->getDOF();
  if(qd.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "srvj " + robot_ind_str + "2 ";
  for(unsigned int i = 0; i < dof; i++) {
    //memset(var_str, 0, 16);
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)qd[i]);
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setServojSrvSpd(const std::vector<double>& q, const std::vector<double>& qd) {
  unsigned int dof = interface->stateRT->getDOF();
  if(q.size() != dof || qd.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "srvj " + robot_ind_str + "3 ";
  for(unsigned int i = 0; i < dof; i++) {
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)q[i]);
    cmd_msg += var_str;
  }
  for(int i = 0; i < dof; i++) {
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)qd[i]);
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setServojCubicAddPt(double time, const std::vector<double>& q, const std::vector<double>& qd) {
  unsigned int dof = interface->stateRT->getDOF();
  if(q.size() != dof || qd.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "srvjque " + robot_ind_str + "0 ";
  for(unsigned int i = 0; i < dof; i++) {
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)q[i]);
    cmd_msg += var_str;
  }
  for(unsigned int i = 0; i < dof; i++) {
    snprintf(var_str, 16, "%.4f ", rad2deg * (float)qd[i]);
    cmd_msg += var_str;
  }
  snprintf(var_str, 16, "%.4f ", (float)time);
  cmd_msg += var_str;
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setServojCubicStart() {
  std::string cmd_msg = "srvjque " + robot_ind_str + "1 ";
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setSpeedj(const std::vector< double >& qd) {
  unsigned int dof = interface->stateRT->getDOF();
  if(qd.size() != dof) {
    return false;
  }
  char var_str[16];
  std::string cmd_msg = "spdj " + robot_ind_str;
  for(unsigned int i = 0; i < dof; i++) {
    snprintf(var_str, 16, "%.4f ", rad2deg * ((float)qd[i]));
    cmd_msg += var_str;
  }
  return setCommandMsg(cmd_msg);
}

bool TmDriver::setDigitalOutputMB(unsigned char ch, bool b) {
  if(ch >= 12) return false;
  std::string cmd_msg;
  int ich = ch;
  if(b) {
    cmd_msg = "digop 0 1 " + CONVERT_TO_STRING(ich);
  }
  else {
    cmd_msg = "digop 0 0 " + CONVERT_TO_STRING(ich);
  }
  return setCommandMsg(cmd_msg);
}
bool TmDriver::setDigitalOutputEE(unsigned char ch, bool b) {
  if(ch >= 4) return false;
  std::string cmd_msg;
  int ich = ch;
  if(b) {
    cmd_msg = "digop " + io_ind_str + "1 " + CONVERT_TO_STRING(ich);
  }
  else {
    cmd_msg = "digop " + io_ind_str + "0 " + CONVERT_TO_STRING(ich);
  }
  return setCommandMsg(cmd_msg);
}

bool TmDriver::setPayload(double mass) {
  ee_payload = mass;
  return true;
}

std::vector<double> TmDriver::interp_cubic(double t, double T,
					   const std::vector<double>& p0, const std::vector<double>& p1,
					   const std::vector<double>& v0, const std::vector<double>& v1
					  ) {
  std::vector<double> pt;
  double c, d, p;
  for(int i = 0; i < p0.size(); i++) {
    //a = p0[i];
    //b = v0[i];
    c = ((3.0 * (p1[i] - p0[i]) / T) - 2.0 * v0[i] - v1[i]) / T;
    d = ((2.0 * (p0[i] - p1[i]) / T) + v0[i] + v1[i]) / (T*T);
    p = p0[i] + v0[i] * t + c * t*t + d * t*t*t;
    pt.push_back(p);
  }
  return pt;
}

bool TmDriver::runTraj(std::vector<double> timestamps,
		       std::vector<std::vector<double> > positions,
		       std::vector<std::vector<double> > velocities
		      ) {
  //if(!setRobotRun()) return false;
  //THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10));
  
  bool ret = false;
  unsigned int k = 0;
  int timeval = servo_timeval;
  
  if(timeval <= 0) {
    ret = true; //jog command with blending
  }
  else if(timeval == 1) {
    ret = setServoOpen("servoj 3"); //cubic
  }
  else if(timeval < 8) {
  }
  else if(timeval <= 10) {
    ret = setServoOpen("servoj 0"); //zero target velocity
  }
  else {
    ret = setServoOpen("servoj 1"); //non-zero target velocity
  }
  if(!ret) return false;
  
  THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10)); //important delay
  
  CHRONO_NS_NAME::high_resolution_clock::time_point t0, t;
  t0 = CHRONO_NS_NAME::high_resolution_clock::now();
  t = t0;
  k = 0;
  executing_traj = true;
  
  ////////////////////////////////////////////////////////////
  //
  if(timeval <= 0) //jog command with blending
  {
  ////////////////////////////////////////////////////////////
  
  double blend = 1.0;
  
  //TODO set resonable jog speed
  while(executing_traj && k < timestamps.size()) {
    if(k == timestamps.size() - 1) blend = 0.0;
    setMoveJabs(positions[k], blend);
    k += 1;
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10));
  }
  while(executing_traj && (interface->stateRT->getQueCmdCount() != 0)) {
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10));
  }
  if(executing_traj) {
    executing_traj = false;
  }
  else {
    setRobotStop();
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(100));
    setRobotRun();
    //TODO set blend = 0
  }
  
  ////////////////////////////////////////////////////////////
  }
  else if(timeval == 1) //cubic interp. by robot controller
  {
  ////////////////////////////////////////////////////////////
  
  std::vector<double> q_vec(positions[0].size(), 0.0);
  
  THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10)); // 15 important delay
  while(executing_traj && k < timestamps.size()) {
    setServojCubicAddPt(timestamps[k], positions[k], velocities[k]);
    k += 1;
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10)); // 15
    if(executing_traj && k == timestamps.size()) {
      print_info(" TM_DRV: cubic interp. on robot side start...");
      setServojCubicStart();
    }
  }
  t0 = CHRONO_NS_NAME::high_resolution_clock::now();
  t = t0;
  while(executing_traj &&
	((1.0*timestamps[timestamps.size() - 1]) >= CHRONO_NS_NAME::duration_cast<CHRONO_NS_NAME::duration<double> >(t-t0).count())
       ) {
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(5));
    t = CHRONO_NS_NAME::high_resolution_clock::now();
  }
  while(executing_traj &&
	((1.1*timestamps[timestamps.size() - 1] + 0.5) >= CHRONO_NS_NAME::duration_cast<CHRONO_NS_NAME::duration<double> >(t-t0).count())
       ) {
    interface->stateRT->getQCmd(q_vec);
    if(vector_match(positions[timestamps.size() - 1], q_vec, 0.005)) //0.287 deg
      break;
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(2));
    t = CHRONO_NS_NAME::high_resolution_clock::now();
  }
  if(executing_traj) {
    executing_traj = false;
    setMoveJabs(positions[timestamps.size() - 1], 0.0);
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10));
  }
  else {
    setServoStop();
  }
  
  ////////////////////////////////////////////////////////////
  }
  else if(timeval < 8) //cubic interp. by robot controller
  {
  ////////////////////////////////////////////////////////////
  
  executing_traj = false;
  
  ////////////////////////////////////////////////////////////
  }
  else if(timeval >= 8 && timeval <= 125) //(timeval >= 8) //cubic interp. here
  {
  ////////////////////////////////////////////////////////////
  
  std::vector<double> traj_point;
  
  while(executing_traj &&
	(timestamps[timestamps.size() - 1] >= CHRONO_NS_NAME::duration_cast<CHRONO_NS_NAME::duration<double> >(t - t0).count())
       ) {
    while((timestamps[k] <= CHRONO_NS_NAME::duration_cast<CHRONO_NS_NAME::duration<double> >(t - t0).count()) &&
	  (k < timestamps.size() - 1)
	 ) {
      k += 1;
    }
    traj_point = interp_cubic(CHRONO_NS_NAME::duration_cast<CHRONO_NS_NAME::duration<double> >(t - t0).count() - timestamps[k - 1],
			      timestamps[k] - timestamps[k - 1],
			      positions[k - 1], positions[k],
			      velocities[k - 1], velocities[k]
			     );
    setServoj(traj_point);
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(timeval));
    t = CHRONO_NS_NAME::high_resolution_clock::now();
  }
  if(executing_traj) {
    //traj_point = positions[positions.size() - 1];
    //setServoj(traj_point);
    executing_traj = false;
  }
  setServoStop();
  
  ////////////////////////////////////////////////////////////
  }
  else //(timeval > 125) //no cubic interp. here, check if about target reach, then send next traj. point
  {
  ////////////////////////////////////////////////////////////
  
  std::vector<double> position_vec(positions[0].size(), 0.0);
  std::vector<double> velocity_vec(positions[0].size(), 0.0);
  std::vector<bool> target_set(positions[0].size(), false);
  bool timeout = false;
  
  while(executing_traj && k < timestamps.size()) {
    
    interface->stateRT->getQCmd(position_vec);
    interface->stateRT->getQdCmd(velocity_vec);
    bool position_match = vector_match( positions[k], position_vec, 0.01);
    bool velocity_match = vector_match(velocities[k], velocity_vec, 0.01);
    
    if((position_match && velocity_match) || timeout) {
      k += 1;
      if(k < timestamps.size()) {
	if(!target_set[k]) {
	  target_set[k] = false;
	  timeout = false;
	  setServojSrvSpd(positions[k], velocities[k]);
	}
      }
    }
    else {
      t = CHRONO_NS_NAME::high_resolution_clock::now();
    }
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(2));
  }
  if(executing_traj) {
    position_vec = positions[positions.size() - 1];
    velocity_vec.assign(positions[0].size(), 0.0);
    setServojSrvSpd(position_vec, velocity_vec);
    executing_traj = false;
  }
  else {
    setServoStop();
  }
  
  ////////////////////////////////////////////////////////////
  //
  } //end if (timeval...)
  //
  ////////////////////////////////////////////////////////////
  
  if(k < timestamps.size() - 1)
    print_warning(" TM_DRV: k < number of traj. points");
  
  if(timeval > 0) {
    THIS_THREAD_NS_NAME::sleep_for(CHRONO_NS_NAME::milliseconds(10));
    setServoClose();
  }
  return true;
}

void TmDriver::stopTraj() {
  executing_traj = false;
  setServoStop();
}

bool TmDriver::vector_match(const std::vector< double >& vec_a, const std::vector< double >& vec_b, double eps) {
  for(unsigned int i = 0; i < vec_a.size(); i++) {
    if(fabs(vec_a[i] - vec_b[i]) > eps)
      return false;
  }
  return true;
}

