/*
 * endpoint_calculation.h
 *
 *  Created on: 26 Feb 2026
 *      Author: tomas
 */
#pragma once

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
extern float32_t endpoints[12];
extern float32_t endpoints_velocity[12];
extern float32_t torques[12];
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

extern const float32_t angle_multiplier_a2p[12];


void init_ep_matrices();
void gen_Hn_n_1(float32_t th, float32_t alpha, float32_t r, float32_t d);
void gen_denhart2();
void update_endpoints(uint8_t n, float32_t angles[12], float32_t angle_velocities[12], float32_t forces[12]);
void generate_Jacobian();
void check_endpoint_limits(uint8_t* estop);
void generate_footstep_landing_location(float32_t body_velocity[], int8_t foot_scheduler_diff[], float32_t foot_step_landing_locations[], float32_t shoulder_vectors[], float32_t theta, float32_t omega_z, float32_t float_time);
void generate_lift_leg_trajectory(int8_t foot_scheduler_diff[], float32_t foot_step_landing_locations[], float32_t endpoints[], float32_t lift_leg_trajectory[15][12], float32_t lift_leg_trajectory_angles[15][12]);
void calculate_angles_from_endpoint(float32_t x, float32_t y, float32_t z, float32_t angles[]);

#endif /* INC_ENDPOINT_CALCULATION_H_ */
