/*
 * tm_print.cpp
 *
 * Copyright 2016 Copyright 2016 Yun-Hsuan Tsai, Techman Robot Inc.
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

#define MAX_MSG_SIZE 256

int print_debug(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
#ifdef ROS_BUILD
  ROS_DEBUG("%s", msg);
#else
  printf("[DEBUG] %s\n", msg);
#endif
  return n;
}

int print_info(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
#ifdef ROS_BUILD
  ROS_INFO("%s", msg);
#else
  printf("[ INFO] %s\n", msg);
#endif
  return n;
}

int print_warning(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
#ifdef ROS_BUILD
  ROS_WARN("%s\n", msg);
#else
  printf("[ WARN] %s\n", msg);
#endif
  return n;
}

int print_error(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
#ifdef ROS_BUILD
  ROS_ERROR("%s", msg);
#else
  printf("[ERROR] %s\n", msg);
#endif
  return n;
}

int print_fatal(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  int n;
  va_list vl;
  va_start(vl, fmt);
  n = vsnprintf(msg, MAX_MSG_SIZE, fmt, vl);
  va_end(vl);
#ifdef ROS_BUILD
  ROS_FATAL("%s", msg);
  ros::shutdown();
#else
  printf("[FATAL] %s\n", msg);
  exit(1);
#endif
  return n;
}

/*
int print_debug(const char* msg) {
  int n = strlen(msg);
#ifdef ROS_BUILD
  ROS_DEBUG("%s", msg);
#else
  printf("[DEBUG] %s\n", msg);
#endif
  return n;
}

int print_info(const char* msg) {
  int n = strlen(msg);
#ifdef ROS_BUILD
  ROS_INFO("%s", msg);
#else
  printf("[ INFO] %s\n", msg);
#endif
  return n;
}

int print_warning(const char* msg) {
  int n = strlen(msg);
#ifdef ROS_BUILD
  ROS_WARN("%s\n", msg);
#else
  printf("[ WARN] %s\n", msg);
#endif
  return n;
}

int print_error(const char* msg) {
  int n = strlen(msg);
#ifdef ROS_BUILD
  ROS_ERROR("%s", msg);
#else
  printf("[ERROR] %s\n", msg);
#endif
  return n;
}

int print_fatal(const char* msg) {
  int n = strlen(msg);
#ifdef ROS_BUILD
  ROS_FATAL("%s", msg);
  ros::shutdown();
#else
  printf("[FATAL] %s\n", msg);
  exit(1);
#endif
  return n;
}
*/