/*********************************************************************
 * tm_communication.h
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
 * Author: Pennyer Chang, Yun-Hsuan Tsai
 */

#ifndef _TM_COMMUNICATION_SC_H_
#define _TM_COMMUNICATION_SC_H_

#include "tm_print.h"
#include "tm_robot_state_rt.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <errno.h>

#ifdef USE_BOOST
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

//#define MAX_CMDDATA_SIZE 510
//#define MAX_CMDMSG_LEN   508


const unsigned int g_tmcom_max_command_size = 508; // MAX_SENDDATA_SIZE (512) - 4

class TmCommunication {
private:
  
  enum {
    DEFAULT_TIMEOUT = 5000,
    DEFAULT_TIMEVAL = 500,
    MAX_NUM_OF_PORT = 2,
    MAX_RECVDATA_SIZE = 1024,
    MAX_SENDDATA_SIZE = 512,
    SB_SIZE = 2048
  };
  
public:
  //TmRobotState* state_;
  TmRobotStateRT* stateRT;
  
#ifdef USE_BOOST
  TmCommunication(boost::condition_variable& data_condv,
		  boost::condition_variable& data_condv_rt,
		  const char* ip = "", const char* port = "6188",
		  int timeout_ms = DEFAULT_TIMEOUT, int timeval_ms = DEFAULT_TIMEVAL
		 );
#else
  TmCommunication(std::condition_variable& data_condv,
		  std::condition_variable& data_condv_rt,
		  const char* ip = "", const char* port = "6188",
		  int timeout_ms = DEFAULT_TIMEOUT, int timeval_ms = DEFAULT_TIMEVAL
		 );
#endif
  ~TmCommunication();
  
  int getSocketDescription();
  //void setTimeout();
  //void setTimeval();
  
  bool start();
  void halt();
  
  int sendCommandData(const char* cmd_name, const char* cmd_data);
  int sendCommandMsg(const char* cmd_msg);
private:
  //recv buffer
  int  sb_H;
  int  sb_T;
  char sb_data[SB_SIZE*2];
  //server IP & port
  char server_ip[64];
  char server_port[64];
  //time var
  int connect_timeout_s;
  int connect_timeout_us;
  int thread_timeval_s;
  int thread_timeval_us;
  //Socket Descriptor
  int sockfd;
  int optflag;
  //is keeping thread alive
  bool thread_alive;
  
#ifdef USE_BOOST
  boost::thread recv_thread;
#else
  //std::mutex connect_mutex;
  //std::mutex send_data_mutex;
  //std::mutex send_msg_mutex;
  //socket recv thread
  std::thread recv_thread;
#endif
  
  int  connectWithTimeout(int sock, const char* ip, const char* port);
  bool connectToServer();
  void disConnect();
  //Ring buffer for accept data
  bool receiveDataToSB(char bdata[], int blen);
  void threadFunction();
};

#endif