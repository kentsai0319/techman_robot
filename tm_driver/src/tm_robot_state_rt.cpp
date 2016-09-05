/*********************************************************************
 * tm_robot_state_rt.cpp
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

#include "tm_driver/tm_robot_state_rt.h"

#ifdef USE_BOOST
TmRobotStateRT::TmRobotStateRT(boost::condition_variable& var_condv):
#else
TmRobotStateRT::TmRobotStateRT(std::condition_variable& var_condv):
#endif
mm2m(0.001f),
deg2rad((float)0.0174532925199),
num_max_dof(MAX_AXIS_NUM)
{
  num_dof = 6;
  pack_size = 372;
  data_updated = false;
  pVar_condv = &var_condv;
}

TmRobotStateRT::~TmRobotStateRT() {
  data_updated = true;
  pVar_condv->notify_all();
}

int TmRobotStateRT::getDOF() {
  return num_dof;
}

int TmRobotStateRT::getPackSize() {
  return (int)pack_size;
}

void TmRobotStateRT::setDataUpdated() {
  data_updated = false;
}

bool TmRobotStateRT::getDataUpdated() {
  return data_updated;
}

void TmRobotStateRT::var_update() {
  float fc;
  controller_time_sec = 0.001 * (double)controller_time_ms;
  for(unsigned int i = 0; i < num_dof; i++) {
     q_act[i] = (double)(deg2rad *  q_act_deg[i]);
     q_cmd[i] = (double)(deg2rad *  q_cmd_deg[i]);
    qd_act[i] = (double)(deg2rad * qd_act_deg[i]);
    qd_cmd[i] = (double)(deg2rad * qd_cmd_deg[i]);
    qt_act[i] = (double)(   mm2m * qt_act_mNm[i]);
    qt_cmd[i] = (double)(   mm2m * qt_cmd_mNm[i]);
  }
  for(unsigned int i = 0; i < 6; i++) {
    if(i < 3) {
      fc = mm2m;
    }
    else {
      fc = deg2rad;
    }
    tool0_pos_act[i] = (double)(fc *tool0_pos_act_mmdeg[i]);
    tool0_vel_act[i] = (double)(fc *tool0_vel_act_mmdeg[i]);
     tool_pos_act[i] = (double)(fc * tool_pos_act_mmdeg[i]);
     tool_pos_cmd[i] = (double)(fc * tool_pos_cmd_mmdeg[i]);
     tool_vel_act[i] = (double)(fc * tool_vel_act_mmdeg[i]);
     tool_vel_cmd[i] = (double)(fc * tool_vel_cmd_mmdeg[i]);
  }
  for(unsigned int i = 0; i < 4; i++) {
    tcp_force_est[i] = mm2m * tcp_force_est_mNm[i];
  }
  for(unsigned int i = 0; i < 12; i++) {
    DI_bits_mb[i] = digital_input_mb & (1<<i);
    DO_bits_mb[i] = digital_output_mb & (1<<i);
  }
  for(unsigned int i = 0; i < 3; i++) {
    DI_bits_ee[i] = digital_input_ee & (1<<i);
    DO_bits_ee[i] = digital_output_ee & (1<<i);
  }
}

//host is littleEndian
int TmRobotStateRT::deSerialize_littleEndian(uint8_t* buffer) {
  unsigned int boffset = 4;
  unsigned int bsize;
  unsigned int len;

  len = 256 * (unsigned int)buffer[0] + (unsigned int)buffer[1];
  
  if(len != pack_size) {
    return 0;
  }
  
  var_mutex.lock();
  
  //if(buffer[2] == 0)
  //{
  //robot data is littleEndian
  bsize = 8;
  memcpy(&controller_time_ms, buffer + boffset, bsize); boffset += bsize; //12
  bsize = 4 * num_dof;
  memcpy( q_act_deg, buffer + boffset, bsize); boffset += bsize;
  memcpy( q_cmd_deg, buffer + boffset, bsize); boffset += bsize;
  memcpy(qd_act_deg, buffer + boffset, bsize); boffset += bsize;
  memcpy(qd_cmd_deg, buffer + boffset, bsize); boffset += bsize;
  memcpy(qt_act_mNm, buffer + boffset, bsize); boffset += bsize;
  memcpy(qt_cmd_mNm, buffer + boffset, bsize); boffset += bsize; //12+4*6*6 = 156
  bsize = 4 * 6;
  memcpy(tool0_pos_act_mmdeg, buffer + boffset, bsize); boffset += bsize;
  memcpy(tool0_vel_act_mmdeg, buffer + boffset, bsize); boffset += bsize;
  memcpy( tool_pos_act_mmdeg, buffer + boffset, bsize); boffset += bsize;
  memcpy( tool_pos_cmd_mmdeg, buffer + boffset, bsize); boffset += bsize;
  memcpy( tool_vel_act_mmdeg, buffer + boffset, bsize); boffset += bsize;
  memcpy( tool_pos_cmd_mmdeg, buffer + boffset, bsize); boffset += bsize; //156+4*6*6 = 300
  bsize = 4 * 4;
  memcpy(tcp_force_est_mNm, buffer + boffset, bsize); boffset += bsize;
  bsize = 4 * 4;
  memcpy(kine_config, buffer + boffset, bsize); boffset += bsize; //300+4*4*2 = 332
  bsize = 4;
  memcpy(&spd_down_pa, buffer + boffset, bsize); boffset += bsize;
  memcpy(&spd_j_pa, buffer + boffset, bsize); boffset += bsize;
  memcpy(&spd_j_ta_ms, buffer + boffset, bsize); boffset += bsize;
  memcpy(&spd_l_m, buffer + boffset, bsize); boffset += bsize;
  memcpy(&spd_l_ta_ms, buffer + boffset, bsize); boffset += bsize; //332+5*4 = 352
  bsize = 2;
  memcpy(&digital_input_mb, buffer + boffset, bsize); boffset += bsize;
  memcpy(&digital_output_mb, buffer + boffset, bsize); boffset += bsize;
  bsize = 1;
  memcpy(&digital_input_ee, buffer + boffset, bsize); boffset += bsize;
  memcpy(&digital_output_ee, buffer + boffset, bsize); boffset += bsize; //352+2*2+2 = 358
  bsize = 1;
  robot_mode  = *(buffer + boffset); boffset += bsize;
  safety_mode = *(buffer + boffset); boffset += bsize;
  control_mode= *(buffer + boffset); boffset += bsize;
  teach_mode  = *(buffer + boffset); boffset += bsize; //358+4 = 362
  bsize = 4;
  memcpy(&res_que_cmd_count, buffer + boffset, bsize); boffset += bsize;
  memcpy(&buf_empty_flag, buffer + boffset, bsize); boffset += bsize; //362+4+4 = 370
  bsize = 1;
  memcpy(&error_code[0], buffer + boffset, bsize); boffset += bsize;
  memcpy(&error_code[1], buffer + boffset, bsize); boffset += bsize; //370+2 = 372
  //}
  //else {
  //robot data is bigEndian
  //...
  //}
  var_update();

  var_mutex.unlock();
  
  data_updated = true;
  pVar_condv->notify_all();
  
  if(boffset != pack_size) {
    return -1;
  }
  return 1;
}

unsigned long int TmRobotStateRT::getTimerCount() {
  unsigned long int ret;
  var_mutex.lock();
  ret = controller_time_ms;
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getTime() {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(q_act, q_act + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQCmd(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(q_cmd, q_cmd + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQdAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(qd_act, qd_act + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQdCmd(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(qd_cmd, qd_cmd + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQtAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(qt_act, qt_act + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getQtCmd(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(qt_cmd, qt_cmd + num_dof);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getTool0PosAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool0_pos_act, tool0_pos_act + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getTool0VelAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool0_vel_act, tool0_vel_act + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getToolPosAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool_pos_act, tool_pos_act + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getToolPosCmd(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool_vel_act, tool_vel_act + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getToolVelAct(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool_vel_act, tool_vel_act + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getToolVelCmd(std::vector<double>& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tool_vel_cmd, tool_vel_cmd + 6);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getTcpForce(std::vector< double >& vec) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  vec.assign(tcp_force_est, tcp_force_est + 3);
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getTcpForceNorm(double& fnorm) {
  double ret;
  var_mutex.lock();
  ret = controller_time_sec;
  fnorm = tcp_force_est[3];
  var_mutex.unlock();
  return ret;
}
void TmRobotStateRT::getKineConfig(bool& is_left_arm, bool& is_below_elbow, bool& is_down_wrist) {
  var_mutex.lock();
  if(kine_config[0] == 1)
    is_left_arm = true;
  else
    is_left_arm = false;
  if(kine_config[1] == 3)
    is_below_elbow = true;
  else
    is_below_elbow = false;
  if(kine_config[2] == 5)
    is_down_wrist = true;
  else
    is_down_wrist = false;
  var_mutex.unlock();
}
double TmRobotStateRT::getSpdDownRatio() {
  double ret;
  var_mutex.lock();
  ret = (double)spd_down_pa;
  var_mutex.unlock();
  return (0.01 * ret);
}
double TmRobotStateRT::getSpdJRatio() {
  double ret;
  var_mutex.lock();
  ret = (double)spd_j_pa;
  var_mutex.unlock();
  return (0.01 * ret);
}
double TmRobotStateRT::getSpdJTa() {
  double ret;
  var_mutex.lock();
  ret = (double)spd_j_ta_ms;
  var_mutex.unlock();
  return (0.001 * ret);
}
double TmRobotStateRT::getSpdLRatio() {
  double ret;
  var_mutex.lock();
  ret = (double)spd_l_m;
  var_mutex.unlock();
  return ret;
}
double TmRobotStateRT::getSpdLTa() {
  double ret;
  var_mutex.lock();
  ret = (double)spd_l_ta_ms;
  var_mutex.unlock();
  return (0.001 * ret);
}
void TmRobotStateRT::getDigitalInputMB(std::vector<bool>& vec) {
  var_mutex.lock();
  vec.assign(DI_bits_mb, DI_bits_mb + 12);
  var_mutex.unlock();
}
void TmRobotStateRT::getDigitalOutputMB(std::vector<bool>& vec) {
  var_mutex.lock();
  vec.assign(DO_bits_mb, DO_bits_mb + 12);
  var_mutex.unlock();
}
void TmRobotStateRT::getDigitalInputEE(std::vector<bool>& vec) {
  var_mutex.lock();
  vec.assign(DI_bits_ee, DI_bits_ee + 4);
  var_mutex.unlock();
}
void TmRobotStateRT::getDigitalOutputEE(std::vector<bool>& vec) {
  var_mutex.lock();
  vec.assign(DO_bits_ee, DO_bits_ee + 4);
  var_mutex.unlock();
}
int TmRobotStateRT::getRobotMode() {
  int ret;
  //var_mutex.lock();
  ret = robot_mode;
  //var_mutex.unlock();
  return ret;
}
int TmRobotStateRT::getSafetyMode() {
  int ret;
  //var_mutex.lock();
  ret = safety_mode;
  //var_mutex.unlock();
  return ret;
}
int TmRobotStateRT::getControlMode() {
  int ret;
  //var_mutex.lock();
  ret = control_mode;
  //var_mutex.unlock();
  return ret;
}
int TmRobotStateRT::getTeachMode() {
  int ret;
  //var_mutex.lock();
  ret = teach_mode;
  //var_mutex.unlock();
  return ret;
}
int TmRobotStateRT::getQueCmdCount() {
  int ret;
  //var_mutex.lock();
  ret = (int)res_que_cmd_count;
  //var_mutex.unlock();
  return ret;
}
int TmRobotStateRT::getBufEmptyFlag() {
  int ret;
  //var_mutex.lock();
  ret = (int)buf_empty_flag;
  //var_mutex.unlock();
  return ret;
}
bool TmRobotStateRT::getError(unsigned char& err_code) {
  bool ret = false;
  var_mutex.lock();
  if(error_code[0] != 0) {
    ret = true;
  }
  err_code = error_code[1];
  var_mutex.unlock();
  return ret;
}
bool TmRobotStateRT::getError(unsigned char& err_code, double& time_sec) {
  bool ret = false;
  var_mutex.lock();
  time_sec = controller_time_sec;
  if(error_code[0] != 0x00) {
    ret = true;
  }
  err_code = error_code[1];
  var_mutex.unlock();
  return ret;
}
