/*********************************************************************
 * tm_kin.cpp
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

/*
 * tm_kin.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: yhtsai
 */
#include "tm_kinematics/tm_kin.h"
#include <math.h>
#include <stdio.h>

namespace tm_kinematics {

	namespace {
		const double _PI = M_PI;
		const double _PI_2 = 0.5 * M_PI;
		const double _2_PI = 2.0 * M_PI;

#ifdef TM900_DH_PARAMS
		const double d1 =  0.1451;
		const double a2 =  0.4290;
		const double a3 =  0.4115;
		const double d4 = -0.1222;
		const double d5 =  0.1060;
		const double d6 =  0.1144;
#else
	#define TM700_DH_PARAMS
	#ifdef TM700_DH_PARAMS
		const double d1 =  0.1451;
		const double a2 =  0.3290;
		const double a3 =  0.3115;
		const double d4 = -0.1222;
		const double d5 =  0.1060;
		const double d6 =  0.1144;
	#endif
#endif
		double IkDistTOL = 0.00000001;
		double IkAngTOL  = 0.00000001;

		int inverse_q1(const double* T, double* L0J1J6, double* q1) {
			int num_sols_q1;
			for (int i = 0; i < 3; i++) {
				L0J1J6[i] = T[ 4*i+3 ] - T[ 4*i+2 ] * d6;
			}
			L0J1J6[2] -= d1;
			double r = sqrt(L0J1J6[0] * L0J1J6[0] + L0J1J6[1] * L0J1J6[1]);
			if (r > -d4 + IkDistTOL) {
				double asp = asin(-d4/r);
				q1[0] =  atan2(L0J1J6[1], L0J1J6[0]) + asp;
				q1[1] = atan2(-L0J1J6[1], -L0J1J6[0]) - asp;
				num_sols_q1 = 2;
			}
			else if (r >= -d4 - IkDistTOL && r <= -d4 + IkDistTOL) {
				q1[0] =  atan2(L0J1J6[1], L0J1J6[0]) + _PI_2;
				q1[1] = atan2(-L0J1J6[1], -L0J1J6[0]) - _PI_2;
				num_sols_q1 = -1;
			}
			else {
				num_sols_q1 = 0;
			}
			if(num_sols_q1 != 0)
			{
				// q1 in [-180, +180]
				for (int i = 0; i < 2; i++) {
					if (q1[i] > _PI) {
						q1[i] -= _2_PI;
					}
					else if (q1[i] < -_PI) {
						q1[i] += _2_PI;
					}
				}
			}
			return num_sols_q1;
		}

		int inverse_qp56(const double* T, double q1, double* R10, double* R16, double* qp56 ,const double* q_ref, bool isRA) {
			int num_sols_qp56;
			for (int i = 0; i < 9; i++) {
				R10[i] = 0;
			}
			R10[0] = cos(q1);
			R10[1] = sin(q1);
			R10[5] = -1;
			R10[6] = -R10[1];
			R10[7] = R10[0];
			double c5;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					c5 = 0;
					for (int k = 0; k < 3; k++) {
						c5 += R10[ 3*i+k ] * T[ 4*k+j ];
					}
					R16[ 3*i+j ] = c5;
				}
			}
			if (R16[8] <= -1 + IkAngTOL) {
				c5 = 1;
				qp56[1] = 0;
				num_sols_qp56 = -1;
			}
			else if (R16[8] >= 1 - IkAngTOL) {
				c5 = -1;
				qp56[1] = _PI;
				num_sols_qp56 = -1;
			}
			else {
				if (isRA) {
					qp56[1] = acos(-R16[8]);
				}
				else {
					qp56[1] = -acos(-R16[8]);
				}
				qp56[4] = -qp56[1];
				num_sols_qp56 = 2;
			}
			if(num_sols_qp56 == 2)
			{
				if (isRA) {
					// qp
					qp56[0] = atan2(R16[5], R16[2]);
					qp56[3] = atan2(-R16[5], -R16[2]);
					// q6
					qp56[2] = atan2(-R16[7], R16[6]);
					qp56[5] = atan2(R16[7], -R16[6]);
				}
				else {
					// qp
					qp56[0] = atan2(-R16[5], -R16[2]);
					qp56[3] = atan2(R16[5], R16[2]);
					// q6
					qp56[2] = atan2(R16[7], -R16[6]);
					qp56[5] = atan2(-R16[7], R16[6]);
				}
			}
			else {
				double cq, sq;
				qp56[2] = q_ref[5];
				cq = cos(qp56[2]);
				sq = sin(qp56[2]);
				qp56[0] = atan2(sq * R16[0] + c5 * cq * R16[3], c5 * cq * R16[0] - sq * R16[3]);
				qp56[3] = qp56[0]; qp56[5] = qp56[2];
				num_sols_qp56 = 1;
			}
			return num_sols_qp56;
		}

		int inverse_q234(const double* T, const double* L0J1J6, const double* R10, double qp, double* L1J1J5, double* q234, const double* q_ref, bool isRA) {
			int num_sols_q234;
			int isol;
			double r, s, t;
			for (int i = 0; i < 3; i++) {
				r = 0;
				for (int j = 0; j < 3; j++) {
					r += R10[ 3*i+j ] * L0J1J6[j];
				}
				L1J1J5[i] = r;
			}
			L1J1J5[0] -= sin(qp) * d5;
			L1J1J5[1] += cos(qp) * d5;
			r = sqrt(L1J1J5[0] * L1J1J5[0] + L1J1J5[1] * L1J1J5[1]);
			s = a2 + a3 - r;
			t = r - fabs(a2 - a3);
			if (s >= IkDistTOL && t >= IkDistTOL) {
				double atp = atan2(L1J1J5[1], L1J1J5[0]);
				double acp1 = acos((a2 * a2 + r * r - a3 * a3) / (2.0 * a2 * r));
				double acp2 = acos((a2 * a2 + a3 * a3 - r * r) / (2.0 * a2 * a3));
				if (isRA) {
					// q2
					q234[0] = atp - acp1;
					q234[3] = atp + acp1;
					// q3
					q234[1] = _PI - acp2;
					q234[4] = acp2 - _PI;
				}
				else {
					// q2
					q234[0] = atp + acp1;
					q234[3] = atp - acp1;
					// q3
					q234[1] = acp2 - _PI;
					q234[4] = _PI - acp2;
				}
				for (int i = 0; i < 2; i++) {
					isol = 3*i;
					// q2 in [-180, +180]
					if (q234[ isol ] > _PI) {
						q234[ isol ] -= _2_PI;
					}
					else if (q234[ isol ] < -_PI) {
						q234[ isol ] += _2_PI;
					}
					// q3 = +-(pi - acos) always in [-180, +180]
				}
				num_sols_q234 = 2;
			}
			else if (s > -IkDistTOL && s < IkDistTOL) {
				// q2
				q234[0] = atan2(L1J1J5[1], L1J1J5[0]);
				q234[3] = q234[0];
				// q3
				q234[1] = 0; q234[4] = 0;
				num_sols_q234 = 1;
			}
			else if (t > -IkDistTOL && t < IkDistTOL) {
				// q2
				q234[0] = atan2(L1J1J5[1], L1J1J5[0]);
				q234[3] = q234[0];
				if (q_ref[2] > 0) {
					q234[1] = _PI; q234[4] = _PI;
					num_sols_q234 = 1;
				}
				else if (q_ref[2] < 0) {
					q234[1] = -_PI; q234[4] = -_PI;
					num_sols_q234 = 1;
				}
				else {
					if (isRA) {
						q234[1] = _PI; q234[4] = -_PI;
					}
					else {
						q234[1] = -_PI; q234[4] = _PI;
					}
					num_sols_q234 = 2;
				}
			}
			else if (t <= -IkDistTOL) {
				num_sols_q234 = 0;
			}
			else {
				num_sols_q234 = 0;
			}
			if (num_sols_q234 > 0) {
				for (int i = 0; i < 2; i++) {
					isol = 3*i;
					// q4 = qp - q2 -q3
					q234[ 2+isol ] = qp - q234[ isol ] - q234[ 1+isol ];
					// q4 in [-180, +180]
					if (q234[ 2+isol ] > _PI) {
						q234[ 2+isol ] -= _2_PI;
					}
					else if (q234[ 2+isol ] < -_PI) {
						q234[ 2+isol ] += _2_PI;
					}
				}
			}
			return num_sols_q234;
		}

	}

	void forward(const double* q, double* T) {
		double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6;
		double cp, sp;
		c1 = cos(q[0]); s1 = sin(q[0]);
		c2 = cos(q[1] - _PI_2); s2 = sin(q[1] - _PI_2);
		c3 = cos(q[2]); s3 = sin(q[2]);
		c4 = cos(q[3] + _PI_2); s4 = sin(q[3] + _PI_2);
		c5 = cos(q[4]); s5 = sin(q[4]);
		c6 = cos(q[5]); s6 = sin(q[5]);
		cp = cos(q[1] + q[2] + q[3]);
		sp = sin(q[1] + q[2] + q[3]);

		T[0]  = c1*sp*s6 - s1*s5*c6 + c1*cp*c5*c6;	T[1]  = c1*sp*c6 + s1*s5*s6 - c1*cp*c5*s6;	T[2]  = c1*cp*s5 + s1*c5;

		T[3]  = c1*(a2*c2 + a3*c2*c3 - a3*s2*s3) - d4*s1 + d6*c5*s1 + d5*c1*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + d6*c1*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2));

		T[4]  = s1*sp*s6 + c1*s5*c6 + s1*cp*c5*c6;	T[5]  = s1*sp*c6 - c1*s5*s6 - s1*cp*c5*s6;	T[6]  = s1*cp*s5 - c1*c5;

		T[7]  = s1*(a2*c2 + a3*c2*c3 - a3*s2*s3) + d4*c1 - d6*c1*c5 + d5*s1*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + d6*s1*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2));

		T[8]  = cp*s6 - sp*c5*c6;		T[9]  = cp*c6 + sp*c5*s6;		T[10] = -sp*s5;

		T[11] = d1 - a2*s2 + d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) - a3*c2*s3 - a3*c3*s2 - d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));

		T[12] = 0;		T[13] = 0;		T[14] = 0;

		T[15] = 1;
	}

	int inverse(const double* T, double* q_sols, const double* q_ref) {
		int num_sols = 0;
		int num_sols_q1, num_sols_qp56, num_sols_q234, isol;
		double q_r[6] = {0,-_PI_2,0,_PI_2,0,0};
		if (q_ref != NULL) {
			for (int i = 0; i < 6; i++) {
				q_r[i] = q_ref[i];
			}
			q_r[1] -= _PI_2;
			q_r[3] += _PI_2;
			for (int i = 0; i < 6; i++) {
				// q_r in [-180, +180]
				if (q_r[i] > _PI) {
					q_r[i] -= _2_PI;
				}
				else if (q_r[i] < -_PI) {
					q_r[i] += _2_PI;
				}
			}
		}
		else {
			for (int i = 0; i < 6; i++) {
				q_r[i] = 0;
			}
			q_r[1] -= _PI_2;
			q_r[3] += _PI_2;
		}
		double L0J1J6[3];
		double q1[2];
		num_sols_q1 = inverse_q1(T,  L0J1J6, q1);
		if (num_sols_q1 > 0) {
			double R10[9] = {0};
			double R16[9] = {0};
			double qp56[6] = {0};
			for (int i = 0; i < num_sols_q1; i++) {
				num_sols_qp56 = inverse_qp56(T, q1[i], R10, R16, qp56, q_r, (i==0));
				if (num_sols_qp56 > 0) {
					double L1J1J5[3] = {0};
					double q234[6] = {0};
					for (int j = 0; j < num_sols_qp56; j++) {
						num_sols_q234 = inverse_q234(T, L0J1J6, R10, qp56[ 3*j ], L1J1J5, q234, q_r, (i==0));
						if (num_sols_q234 > 0) {
							for (int k = 0; k < num_sols_q234; k++) {
								isol = 6*num_sols;
								q_sols[ 0+isol ] = q1[i];
								q_sols[ 1+isol ] = q234[   3*k ] + _PI_2;
								if (q_sols[ 1+isol ] > _PI) {
									q_sols[ 1+isol ] -= _2_PI;
								}
								else if (q_sols[ 1+isol ] < -_PI) {
									q_sols[ 1+isol ] += _2_PI;
								}
								q_sols[ 2+isol ] = q234[ 1+3*k ];
								q_sols[ 3+isol ] = q234[ 2+3*k ] - _PI_2;
								if (q_sols[ 3+isol ] > _PI) {
									q_sols[ 3+isol ] -= _2_PI;
								}
								else if (q_sols[ 3+isol ] < -_PI) {
									q_sols[ 3+isol ] += _2_PI;
								}
								q_sols[ 4+isol ] = qp56[ 1+3*j ];
								q_sols[ 5+isol ] = qp56[ 2+3*j ];
								num_sols++;
							}//for k
						}
					}//for j
				}
				else {
					break;
				}
			}//for i
		}
		if (num_sols == 0) {
			for (int i = 0; i < 5; i++) {
				q_sols[i] = q_r[i];
			}
		}
		return num_sols;
	}

	int inverse(const double* T, double* q_sols, double q6_des) {
		//int num_sols = 0;
		double q_r[6] = {0,-_PI_2,0,_PI_2,0,0};
		q_r[5] = q6_des;
		return inverse(T, q_sols, q_r);
	}

}
