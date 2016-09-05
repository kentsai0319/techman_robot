/*
 * tm_print.h
 *
 * Copyright 2016 Yun-Hsuan Tsai, Techman Robot Inc.
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

#ifndef _TM_PRINT_H_
#define _TM_PRINT_H_

#ifdef ROS_BUILD
#include <ros/ros.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

int print_debug(const char* fmt, ...);
int print_info(const char* fmt, ...);
int print_warning(const char* fmt, ...);
int print_error(const char* fmt, ...);
int print_fatal(const char* fmt, ...);

/*
int print_debug(const char* msg);
int print_info(const char* msg);
int print_warning(const char* msg);
int print_error(const char* msg);
int print_fatal(const char* msg);
*/
#endif