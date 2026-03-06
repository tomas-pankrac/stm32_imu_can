/*
 * endpoint_calculation.h
 *
 *  Created on: 26 Feb 2026
 *      Author: tomas
 */

#ifndef INC_ENDPOINT_CALCULATION_H_
#define INC_ENDPOINT_CALCULATION_H_

#include "stdio.h"
#include <math.h>
#include "arm_math.h"


extern float32_t Hn[4][4];
extern float32_t H0[4][4];
extern float32_t H1[4][4];
extern float32_t H2[4][4];

extern float32_t Hn1[4][4];
extern float32_t Hn2[4][4];
extern float32_t Dh[3][4];
extern float32_t endpoints[4][3];
extern float32_t endpoints_velocity[4][3];
extern float32_t torques[4][3];;
extern float32_t J[3][3];
extern float32_t Jinv[3][3];
extern float32_t Ri[3][3];
extern float32_t Ri_col[3];
extern float32_t dn[3];
extern float32_t di[3];
extern float32_t angle_vel[3];
extern float32_t ep_vel[3];


extern arm_matrix_instance_f32 Hn_inst;
extern arm_matrix_instance_f32 H0_inst;
extern arm_matrix_instance_f32 H1_inst;
extern arm_matrix_instance_f32 H2_inst;
extern arm_matrix_instance_f32 Hn1_inst;
extern arm_matrix_instance_f32 Hn2_inst;
extern arm_matrix_instance_f32 J_inst;
extern arm_matrix_instance_f32 Jinv_inst;
extern arm_matrix_instance_f32 Ri_inst;
extern arm_matrix_instance_f32 Ri_col_inst;
extern arm_matrix_instance_f32 dn_inst;
extern arm_matrix_instance_f32 di_inst;
extern arm_matrix_instance_f32 angle_vel_inst;
extern arm_matrix_instance_f32 ep_vel_inst;

void init_ep_matrices();
void gen_Hn_n_1(float32_t th, float32_t alpha, float32_t r, float32_t d);
void gen_denhart2();
void update_endpoints(uint8_t n, float32_t angles[12], float32_t angle_velocities[12], float32_t forces[12]);
void generate_Jacobian();

#endif /* INC_ENDPOINT_CALCULATION_H_ */
