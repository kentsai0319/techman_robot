/*********************************************************************
 * tm_communication.cpp
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

#include "tm_driver/tm_communication.h"

#ifdef USE_BOOST
TmCommunication::TmCommunication(boost::condition_variable& data_condv,
				 boost::condition_variable& data_condv_rt,
				 const char* ip, const char* port,
				 int timeout_ms, int timeval_ms
				):
#else
TmCommunication::TmCommunication(std::condition_variable& data_condv,
				 std::condition_variable& data_condv_rt,
				 const char* ip, const char* port,
				 int timeout_ms, int timeval_ms
				):
#endif
sockfd(-1),
optflag(1),
thread_alive(false)
{
  print_debug("TM_COM: TMCommunication CONstructor");
  //RobotState = new TmRobotState(data_condv);
  stateRT = new TmRobotStateRT(data_condv_rt);
  sb_H = 0;
  sb_T = 0;
  memset(sb_data, 0, sizeof(sb_data));
  memset(server_ip,  0, sizeof(server_ip));
  memset(server_port,0, sizeof(server_port));
  if((strcmp(ip,"") == 0) || (strlen(ip) > sizeof(server_ip))) {
    print_error("TM_COM: Set IP Error!");
    return;
  }
  if((strcmp(port,"") == 0) || (strlen(port) > sizeof(server_port))) {
    print_error("TM_COM: Set Port Error!");
    return;
  }
  strcpy(server_ip,  ip);
  strcpy(server_port,port);
  connect_timeout_s = (timeout_ms / 1000);
  connect_timeout_us= (timeout_ms % 1000) * 1000;
  thread_timeval_s  = (timeval_ms / 1000);
  thread_timeval_us = (timeval_ms % 1000) * 1000;
  print_debug("TM_COM: TMCommunication CONstruction DONE");
}

TmCommunication::~TmCommunication() {
  printf("[debug] TM_COM: TMCommunication DEstructor\n");
  //halt thread
  halt();
  //desconnet socket
  if(sockfd != -1) {
    close(sockfd);
    sockfd = -1;
  }
  delete stateRT;
  print_debug("TM_COM: TMCommunication DEstruction DONE");
}

int TmCommunication::getSocketDescription() {
  return sockfd;
}

bool TmCommunication::start() {
  bool ret;
  //re start
  halt();
  ret = connectToServer();
  if(ret) {
    thread_alive = true;
#ifdef USE_BOOST
    recv_thread = boost::thread(boost::bind(&TmCommunication::threadFunction, this));
#else
    recv_thread = std::thread(&TmCommunication::threadFunction, this);
#endif
  }
  return ret;
}

void TmCommunication::halt() {
  thread_alive = false;
  if(recv_thread.joinable()) {
    recv_thread.join();
    printf("[ info] TM_COM: halt\n");
  }
}

int TmCommunication::sendCommandData(const char* cmd_name, const char* cmd_data) {
  //static 
  char buf[MAX_SENDDATA_SIZE];
  int ret = -1;
  size_t nsize = strlen(cmd_name);
  size_t dsize = strlen(cmd_data);
  if(nsize + dsize > g_tmcom_max_command_size) {
    return -1;
  }
  //send_data_mutex.lock();
  //memset(buf, 0, sizeof(buf));
  *((unsigned short*)buf) = htons((unsigned short)(nsize + dsize) + 4);
  buf[2] = 0; //reserved
  buf[3] = (char)nsize;
  memcpy(buf + 4, cmd_name, nsize);
  memcpy(buf + nsize + 4, cmd_name, dsize);
  if(sockfd > 0)
    ret = send(sockfd, buf, nsize + dsize + 4, 0) - 4;
  //send_data_mutex.unlock();
  return ret;
}

int TmCommunication::sendCommandMsg(const char* cmd_msg) {
  //static 
  char buf[MAX_SENDDATA_SIZE];
  int ret = -1;
  size_t bsize = strlen(cmd_msg);
  if(bsize > g_tmcom_max_command_size) {
    return -1;
  }
  //send_msg_mutex.lock();
  //memset(buf, 0, sizeof(buf));
  *((unsigned short*)buf) = htons((unsigned short)bsize + 4);
  buf[2] = 0;
  buf[3] = 0;
  memcpy(buf + 4, cmd_msg, bsize);
  if(sockfd > 0)
    ret = send(sockfd, buf, bsize + 4, 0) - 4;
  //send_msg_mutex.unlock();
  return ret;
}

int TmCommunication::connectWithTimeout(int sock, const char* ip, const char* port) {
  //char msg[64];
  int flags = 0;
  int ret = 0;
  int error = 0;
  unsigned int len_err = 0;
  struct sockaddr_in serv_addr;
  struct timeval tv;
  fd_set wset;
  
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(atoi(port));
  inet_pton(AF_INET, ip, &(serv_addr.sin_addr));
  
  tv.tv_sec = connect_timeout_s;
  tv.tv_usec = connect_timeout_us; //unit micro second
  
  FD_ZERO(&wset);
  FD_SET(sock, &wset);
  
  //Get Flag of Fcntl 
  if((flags = fcntl(sock, F_GETFL, 0)) < 0 ) {
	print_warning("TM_COM: Get origin Flag of fcntl fail");
	return -1;
  }
  //snprintf(msg, 64, "TM_COM: Origin Flag of fcntl = %d", flags);
  print_debug("TM_COM: Origin Flag of fcntl = %d", flags);
  
  //Set O_NONBLOCK to Connect
  if(fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
    print_warning("TM_COM: Set O_NONBLOCK to Connect Fail");
    return -1;
  }
  
  if((ret = connect(sock, (struct sockaddr*)&serv_addr, 16)) < 0) {
    if(errno != EINPROGRESS) {
      return -1;
    }
  }
  if(ret == 0) {
    print_info("TM_COM: Connect Immediatly OK");
  }
  else {
    //Wait for Connect OK by checking Write buffer
    if((ret = select(sock + 1, NULL, &wset, NULL, &tv)) < 0) {
      return -1;
    }
    if(ret == 0) {
      print_warning("TM_COM: Connect Timeout");
      errno = ETIMEDOUT;
      return -1;
    }
    if(FD_ISSET(sock, &wset)) {
      if(getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len_err) < 0) {
	print_error("TM_COM: Get Socketopt SO_ERROR FAIL");
	return -1;
      }
    }
    else {
      print_error("TM_COM: undefined socket error");
      return -1;
    }
    if(error != 0) {
      errno = error;
      print_error("TM_COM: Socket Error while connecting");
      return -1;
    }
  }
}

bool TmCommunication::connectToServer() {
  //char msg[128];
  if(sockfd > 0) {
    print_info("TM_COM: Already connect to TM robot");
    return true;
  }
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0) {
    print_error("TM_COM: ERROR opening socket");
    return false;
  }
  optflag = 1;
  setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY,  &optflag, sizeof(int));
  setsockopt(sockfd, IPPROTO_TCP, TCP_QUICKACK, &optflag, sizeof(int));
  setsockopt(sockfd,  SOL_SOCKET, SO_REUSEADDR, &optflag, sizeof(int));
  //snprintf(msg, 128, "TM_COM: Connecting to TM robot... IP:=%s, Port:=%s, sockfd:=%d", strServerIP, strServerPort, sockfd);
  print_info("TM_COM: Connecting to TM robot... IP:=%s, Port:=%s, sockfd:=%d", server_ip, server_port, sockfd);
  if(connectWithTimeout(sockfd, server_ip, server_port) == 0) {
    print_info("TM_COM: O_NONBLOCK Connect OK");
  }
  else {
    sockfd = -1;
    print_info("TM_COM: O_NONBLOCK Connect Fail"); 
  }
  if(sockfd > 0) {
    //snprintf(msg, 128, "TM_COM: TM robot is Connected. sockfd:=%d", sockfd);
    print_info("TM_COM: TM robot is Connected. sockfd:=%d", sockfd);
    return true;
  }
  else {
    return false;
  }
}

void TmCommunication::disConnect() {
  if(sockfd > 0) {
    close(sockfd);
    sockfd = -1;
    print_info("TM_COM: Connection to TM robot CLOSED");
  }
  else {
    print_info("TM_COM: No Conection");
  }    
}

bool TmCommunication::receiveDataToSB(char bdata[], int blen) {
  //static char buf[MAX_RECVDATA_SIZE];
  int size_buf = 0;
  int len_data = 0;
  int space = 0;
  char* pchr = NULL;
  
  if(sb_H >= sb_T) {
    space = SB_SIZE - sb_H + sb_T - 1;
  }
  else {
    space = - sb_H + sb_T - 1;
  }
  if(blen > space) {
    //Sever send Data too Fast
    return false;
  }
  
  //append data
  if((sb_H + blen - 1) >= SB_SIZE) {
    //printf("sb_H surpass sb size\n");
    pchr = &sb_data[sb_H];
    memcpy(pchr, bdata, blen);
    pchr = &sb_data[sb_H + SB_SIZE];
    memcpy(pchr, bdata, SB_SIZE - sb_H);
    pchr = &sb_data[0];
    memcpy(pchr, (bdata + SB_SIZE - sb_H), (sb_H + blen - SB_SIZE));
  }
  else {
    pchr = &sb_data[sb_H];
    memcpy(pchr, bdata, blen);
    pchr = &sb_data[sb_H + SB_SIZE];
    memcpy(pchr, bdata, blen);
  }
  sb_H = (sb_H + blen) % SB_SIZE;
  
  for(;;) {
    len_data = (SB_SIZE + sb_H - sb_T) % SB_SIZE;
    if(len_data <= 4)
      break;
    
    size_buf = 256 * sb_data[sb_T] + sb_data[(sb_T + 1) % SB_SIZE];
    if(len_data < size_buf)
      break;
    
    //extract data
    pchr = &sb_data[sb_T];
    //memset(buf, 0 , sizeof(buf));
    //memcpy(buf, pchr, size_buf);
    sb_T = (sb_T + size_buf) % SB_SIZE;
    
    //unpack state_ data
    
    //unpack stateRT data
    //size_buf = stateRT->deSerialize_littleEndian((unsigned char*)buf);
    size_buf = stateRT->deSerialize_littleEndian((unsigned char*)pchr);
    if(size_buf == 0) {
      print_error("data length wrong!");
    }
    else if(size_buf < 0) {
      print_error("boffset wrong! boffset:=%d", size_buf);
    }
  }
  return true;
}

void TmCommunication::threadFunction() {
  char buf[MAX_RECVDATA_SIZE];
  int rv = 0, n = 0;
  struct timeval tv;
  
  print_info("TM_COM: Recv. thread start running");

  while(thread_alive) { 
    
    fd_set masterfs, readfs;
    FD_ZERO(&masterfs);
    FD_SET(sockfd, &masterfs);
    
    sb_H = 0;
    sb_T = 0;
    
    while(thread_alive && sockfd > 0) {
      
      readfs = masterfs;
      tv.tv_sec = thread_timeval_s;
      tv.tv_usec = thread_timeval_us;
      
      rv = select(sockfd + 1, &readfs, NULL, NULL, &tv);
      if(rv < 0) {
	print_error("TM_COM: Socket select Error");
	break;
      }
      else if(rv == 0) {
	//timeout or no data
	if(!thread_alive)
	  break;
      }
      else if(FD_ISSET(sockfd, &readfs)) {  
	
	memset(buf, 0, MAX_RECVDATA_SIZE);
	
	n = recv(sockfd, buf, MAX_RECVDATA_SIZE, 0);
	if(n < 0) {
	  print_error("TM_COM: ERROR reading from socket");
	  break;
	}
	else if(n == 0) {
	  print_warning("TM_COM: Server is shutdown, reconnection is needed");
	  break;
	}
	optflag = 1;
	setsockopt(sockfd, IPPROTO_TCP, TCP_QUICKACK, &optflag, sizeof(int));
	if(!receiveDataToSB(buf, n)) {
	  print_warning("TM_COM: Server send data too fast");
	  break;
	}
      }
    }
    disConnect();
    
    if(thread_alive) {
      print_info("TM_COM: Will try to reconnect in 5 seconds...");
#ifdef USE_BOOST
      boost::this_thread::sleep_for(boost::chrono::seconds(5));
#else
      std::this_thread::sleep_for(std::chrono::seconds(5));
#endif
    }
    //try to reconect to server
    if(thread_alive) {
      connectToServer();
    }
  }
  disConnect();
  printf("[ info] TM_COM: Recv. thread finished\n");
}
