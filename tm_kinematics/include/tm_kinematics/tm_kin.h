/*********************************************************************
 * tm_kin.h
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

#ifndef TM_KIN_H
#define TM_KIN_H

namespace tm_kinematics {
  
  void forward(const double* q, double* T);
  
  int inverse(const double* T, double* q_sols, const double* q_ref);
  
  int inverse(const double* T, double* q_sols, double q6_des = 0.0);

}

#endif //TM_KIN_H
