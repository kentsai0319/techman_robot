/*********************************************************************
 * tm_robot_state_rt.h
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

#ifndef _TM_ROBOT_STATE_RT_H_
#define _TM_ROBOT_STATE_RT_H_
 
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#ifdef USE_BOOST
  //#include <boost/thread/thread.hpp>
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/condition_variable.hpp>
#else
  //#include <thread>
  #include <mutex>
  #include <condition_variable>
#endif 


class TmRobotStateRT {
private:
  
  enum { MAX_AXIS_NUM = 8 };
  
  const float mm2m;
  const float deg2rad;
  const int num_max_dof;
  int num_dof;
  
  uint32_t pack_size;
  
  uint64_t controller_time_ms;
  
  double controller_time_sec;
  
  double  q_act[ MAX_AXIS_NUM ];
  double  q_cmd[ MAX_AXIS_NUM ];
  double qd_act[ MAX_AXIS_NUM ];
  double qd_cmd[ MAX_AXIS_NUM ];
  
  double qt_act[ MAX_AXIS_NUM ];
  double qt_cmd[ MAX_AXIS_NUM ];
  
  double tool0_pos_act[6]; //(x,y,z,rx,ry,rz)
  double tool0_vel_act[6];
  
  double tool_pos_act[6];
  double tool_pos_cmd[6];
  double tool_vel_act[6];
  double tool_vel_cmd[6];
  
  double tcp_force_est[4];
  
  bool digital_input_bits_MB [12];
  bool digital_output_bits_MB[12];
  bool digital_input_bits_EE [4];
  bool digital_output_bits_EE[4];
  
  float  q_act_deg[ MAX_AXIS_NUM ];
  float  q_cmd_deg[ MAX_AXIS_NUM ];
  float qd_act_deg[ MAX_AXIS_NUM ];
  float qd_cmd_deg[ MAX_AXIS_NUM ];
  
  float qt_act_mNm[ MAX_AXIS_NUM ];
  float qt_cmd_mNm[ MAX_AXIS_NUM ];
  
  float tool0_pos_act_mmdeg[6]; //(x,y,z,rx,ry,rz)
  float tool0_vel_act_mmdeg[6];
  
  float tool_pos_act_mmdeg[6]; //(x,y,z,rx,ry,rz)
  float tool_pos_cmd_mmdeg[6]; //(x,y,z,rx,ry,rz)
  float tool_vel_act_mmdeg[6];
  float tool_vel_cmd_mmdeg[6];
  
  float tool0_pose_act_mmdeg[8]; //use Quaternion
  float  tool_pose_act_mmdeg[8]; //use Quaternion
  float  tool_pose_cmd_mmdeg[8]; //use Quaternion
  
  float tcp_force_est_mNm[4];

  int kine_config[4];
  
  float spd_down_pa;
  
  float spd_j_pa;
  float spd_j_ta_ms;
  
  float spd_l_m;
  float spd_l_ta_ms;
  
  uint16_t digital_input_mb;
  uint16_t digital_output_mb;
  uint8_t  digital_input_ee;
  uint8_t  digital_output_ee;
  
  bool DI_bits_mb[12];
  bool DO_bits_mb[12];
  bool DI_bits_ee[3];
  bool DO_bits_ee[3];
  
  uint8_t robot_mode;
  uint8_t safety_mode;
  uint8_t control_mode;
  uint8_t teach_mode;
  
  uint32_t res_que_cmd_count;
  
  uint32_t buf_empty_flag;
  
  uint8_t  error_code[2];

  bool data_updated;
  
#ifdef USE_BOOST
  boost::mutex var_mutex;
  boost::condition_variable* pVar_condv; 
#else
  std::mutex var_mutex;
  std::condition_variable* pVar_condv;
#endif
  
  void var_update();

public:
  
#ifdef USE_BOOST
  TmRobotStateRT(boost::condition_variable& var_condv);
#else
  TmRobotStateRT(std::condition_variable& var_condv);
#endif
  ~TmRobotStateRT();
  
  int getDOF();
  int getPackSize();
  
  void setDataUpdated();
  bool getDataUpdated();
  
  int deSerialize_littleEndian(uint8_t* buffer);
  
  unsigned long int getTimerCount();
  double getTime();
  
  double getQAct(std::vector<double>& vec);
  double getQCmd(std::vector<double>& vec);
  
  double getQdAct(std::vector<double>& vec);
  double getQdCmd(std::vector<double>& vec);
  
  double getQtAct(std::vector<double>& vec);
  double getQtCmd(std::vector<double>& vec);
  
  double getTool0PosAct(std::vector<double>& vec);
  double getTool0VelAct(std::vector<double>& vec);
  
  double getToolPosAct(std::vector<double>& vec);
  double getToolPosCmd(std::vector<double>& vec);
  
  double getToolVelAct(std::vector<double>& vec);
  double getToolVelCmd(std::vector<double>& vec);
  
  double getTcpForce(std::vector<double>& vec);
  double getTcpForceNorm(double& fnorm);
  
  void getKineConfig(bool& is_left_arm, bool& is_below_elbow, bool& is_down_wrist);
  
  double getSpdDownRatio();
  double getSpdJRatio();
  double getSpdJTa();
  double getSpdLRatio();
  double getSpdLTa();
  
  void getDigitalInputMB(std::vector<bool>& vec);
  void getDigitalOutputMB(std::vector<bool>& vec);
  void getDigitalInputEE(std::vector<bool>& vec);
  void getDigitalOutputEE(std::vector<bool>& vec);
  
  int getRobotMode();
  int getSafetyMode();
  int getControlMode();
  int getTeachMode();
  
  int getQueCmdCount();
  int getBufEmptyFlag();
  
  bool getError(unsigned char& err_code);
  bool getError(unsigned char& err_code, double& time_sec);
};

#endif
