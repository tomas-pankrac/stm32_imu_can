#include "stdio.h"
#include <math.h>
#include "arm_math.h"
#include "endpoint_calculation.h"

float32_t Hn[4][4];
float32_t H0[4][4];
float32_t H1[4][4];
float32_t H2[4][4];
float32_t Hn1[4][4];
float32_t Hn2[4][4];
float32_t Dh[3][4];
float32_t J[3][3];
float32_t Jinv[3][3];
float32_t Ri[3][3];
float32_t Ri_col[3];
float32_t dn[3];
float32_t di[3];
float32_t angle_vel[3];
float32_t ep_vel[3];

float32_t endpoints[12];
float32_t endpoints_velocity[12];
float32_t torques[12];
float32_t sides[] = {1, 1, -1, -1};


arm_matrix_instance_f32 Hn_inst;
arm_matrix_instance_f32 H0_inst;
arm_matrix_instance_f32 H1_inst;
arm_matrix_instance_f32 H2_inst;
arm_matrix_instance_f32 Hn1_inst;
arm_matrix_instance_f32 Hn2_inst;
arm_matrix_instance_f32 J_inst;
arm_matrix_instance_f32 Jinv_inst;
arm_matrix_instance_f32 Ri_inst;
arm_matrix_instance_f32 Ri_col_inst;
arm_matrix_instance_f32 dn_inst;
arm_matrix_instance_f32 angle_vel_inst;
arm_matrix_instance_f32 ep_vel_inst;

const float32_t angle_multiplier_a2p[12] = {
        -6.0    /( 2 * PI),
        -6.0    /( 2 * PI),
        -6.0025 /( 2 * PI),
         6.0    /( 2 * PI),
        -6.0    /( 2 * PI),
        -6.0025 /( 2 * PI),
        -6.0    /( 2 * PI),
         6.0    /( 2 * PI),
         6.0025 /( 2 * PI),
         6.0    /( 2 * PI),
         6.0    /( 2 * PI),
         6.0025 /( 2 * PI)};

float32_t footstep_landings[4][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};


const float32_t f = 0.2;
const float32_t r[4] = {-0.065, -0.065, 0.065, 0.065};

void init_ep_matrices(){
    Hn[0][0] = 0;  Hn[0][1] = 0;  Hn[0][2] = 0;  Hn[0][3] = 0;
    Hn[1][0] = 0;  Hn[1][1] = 0;  Hn[1][2] = 0;  Hn[1][3] = 0;
    Hn[2][0] = 0;  Hn[2][1] = 0;  Hn[2][2] = 0;  Hn[2][3] = 0;
    Hn[3][0] = 0;  Hn[3][1] = 0;  Hn[3][2] = 0;  Hn[3][3] = 1;

    Dh[0][0] = 0;  Dh[0][1] = PI/2;  Dh[0][2] = 0;   Dh[0][3] = 0;
    Dh[1][0] = 0;   Dh[1][1] = 0;     Dh[1][2] = -f;  Dh[1][3] = 0;
    Dh[2][0] = 0;   Dh[2][1] = 0;     Dh[2][2] = -f;  Dh[2][3] = 0;

    uint8_t rows, cols;

    rows = 4;
    cols = 4;
    arm_mat_init_f32(&Hn_inst, rows, cols, (float32_t *)Hn);
    arm_mat_init_f32(&H0_inst, rows, cols, (float32_t *)H0);
    arm_mat_init_f32(&H1_inst, rows, cols, (float32_t *)H1);
    arm_mat_init_f32(&H2_inst, rows, cols, (float32_t *)H2);
    arm_mat_init_f32(&Hn1_inst, rows, cols, (float32_t *)Hn1);
    arm_mat_init_f32(&Hn2_inst, rows, cols, (float32_t *)Hn2);

    rows = 3;
    cols = 3;
    arm_mat_init_f32(&J_inst, rows, cols, (float32_t *)J);
    arm_mat_init_f32(&Jinv_inst, rows, cols, (float32_t *)Jinv);
    arm_mat_init_f32(&Ri_inst, rows, cols, (float32_t *)Ri);
    for (uint8_t i=0; i<3; i++){
        for (uint8_t j=0; j<3; j++){
            Ri[i][j] = (i==j) ? 1 : 0;
        }
    }

    rows = 3;
    cols = 1;
    arm_mat_init_f32(&Ri_col_inst, rows, cols, (float32_t *)Ri_col);
    arm_mat_init_f32(&dn_inst, rows, cols, (float32_t *)dn);
    arm_mat_init_f32(&dn_inst, rows, cols, (float32_t *)di);
    arm_mat_init_f32(&angle_vel_inst, rows, cols, (float32_t *)angle_vel);
    arm_mat_init_f32(&ep_vel_inst, rows, cols, (float32_t *)ep_vel);
      
}

void gen_Hn_n_1(float32_t th, float32_t alpha, float32_t r, float32_t d){
    float32_t ct = cos(th);
    float32_t st = sin(th);
    float32_t ca = cos(alpha);
    float32_t sa = sin(alpha);
    Hn[0][0] = ct;  Hn[0][1] = -st*ca;  Hn[0][2] = st*sa;       Hn[0][3] = r*ct;
    Hn[1][0] = st;  Hn[1][1] = ct*ca;   Hn[1][2] = -ct*sa;      Hn[1][3] = r*st;
    Hn[2][0] = 0;   Hn[2][1] = sa;      Hn[2][2] = ca;          Hn[2][3] = d;
}

void gen_denhart2(){
//    uint8_t n = 3;
    gen_Hn_n_1(Dh[0][0], Dh[0][1], Dh[0][2], Dh[0][3]);
    memcpy(&H0, &Hn, sizeof(float32_t)*16);
    gen_Hn_n_1(Dh[1][0], Dh[1][1], Dh[1][2], Dh[1][3]);
    memcpy(&Hn1, &Hn, sizeof(float32_t)*16);
    gen_Hn_n_1(Dh[2][0], Dh[2][1], Dh[2][2], Dh[2][3]);
    memcpy(&Hn2, &Hn, sizeof(float32_t)*16);
    arm_mat_mult_f32(&H0_inst, &Hn1_inst, &H1_inst);
    arm_mat_mult_f32(&H1_inst, &Hn2_inst, &H2_inst);
}

void generate_Jacobian(){

    for (uint8_t j=0; j<3; j++){
        dn[j] = H2[j][3];
    }

    J[0][0] = -dn[1];
    J[1][0] = -dn[0]; // *-1
    J[2][0] = 0;

    Ri_col[0] = H0[0][2];
    Ri_col[1] = H0[1][2];
    Ri_col[2] = H0[2][2];

    di[0] = dn[0] - H0[0][3];
    di[1] = dn[1] - H0[1][3];
    di[2] = dn[2] - H0[2][3];

    J[0][1] = -Ri_col[1] * di[2] + Ri_col[2] * di[1]; //*-1
    J[1][1] = Ri_col[2] * di[0] - Ri_col[0] * di[2];
    J[2][1] = -Ri_col[0] * di[1] + Ri_col[1] * di[0]; //*-1

    Ri_col[0] = H1[0][2];
    Ri_col[1] = H1[1][2];
    Ri_col[2] = H1[2][2];

    di[0] = dn[0] - H1[0][3];
    di[1] = dn[1] - H1[1][3];
    di[2] = dn[2] - H1[2][3];

    J[0][2] = -Ri_col[1] * di[2] + Ri_col[2] * di[1]; //*-1
    J[1][2] = Ri_col[2] * di[0] - Ri_col[0] * di[2];
    J[2][2] = -Ri_col[0] * di[1] + Ri_col[1] * di[0]; //*-1
}

void update_endpoints(uint8_t n, float32_t angles[12], float32_t angle_velocities[12], float32_t forces[12]){
    for(uint8_t i = 0; i < n; i++){
        float32_t a0 = angles[0+3*i];
        float32_t a1 = angles[1+3*i];
        float32_t a2 = angles[2+3*i];
        Dh[0][0] = -a0;
        Dh[1][0] = a1;
        Dh[2][0] = a2;
        Dh[1][3] = r[i];

        gen_denhart2();
        endpoints[i*3 + 0] = H2[2][3];
        endpoints[i*3 + 1] = H2[1][3];
        endpoints[i*3 + 2] = H2[0][3];

        generate_Jacobian();

        arm_mat_inverse_f32(&J_inst, &Jinv_inst);

        angle_vel[0] = angle_velocities[0+3*i];
        angle_vel[1] = angle_velocities[1+3*i];
        angle_vel[2] = angle_velocities[2+3*i];
        arm_mat_mult_f32(&J_inst, &angle_vel_inst, &ep_vel_inst);

        endpoints_velocity[i*3 + 0] = ep_vel[2];
        endpoints_velocity[i*3 + 1] = ep_vel[1];
        endpoints_velocity[i*3 + 2] = ep_vel[0];

        angle_vel[0] = forces[2+3*i];
        angle_vel[1] = forces[1+3*i];
        angle_vel[2] = forces[0+3*i];
        arm_mat_mult_f32(&Jinv_inst, &angle_vel_inst, &ep_vel_inst);

        torques[i*3 + 0] = ep_vel[2];
        torques[i*3 + 1] = ep_vel[1];
        torques[i*3 + 2] = ep_vel[0];
    }
}

void check_endpoint_limits(uint8_t* estop){
    for (uint8_t i=0; i<4; i++){
        if((endpoints[i*3 + 0] < -0.3) || (endpoints[i*3 + 0] > +0.3)){
            *estop = 1;
        }
        if((endpoints[i*3 + 1] < -0.3) || (endpoints[i*3 + 1] > +0.3)){
            *estop = 1;
        }
        if((endpoints[i*3 + 2] > -0.15)){
            *estop = 1;
        }
    }
}

void generate_footstep_landing_location(float32_t body_velocity[], int8_t foot_scheduler_diff[], float32_t foot_step_landing_locations[], float32_t shoulder_vectors[], float32_t theta, float32_t omega_z, float32_t float_time){
    float32_t ct = cos(theta);
    float32_t st = sin(theta);

    float32_t cotg = cos(omega_z * float_time / 2);
    float32_t sotg = sin(omega_z * float_time / 2);

    float32_t body_velocity_rotated[3];
    body_velocity_rotated[0] = ct * body_velocity[0] + st * body_velocity[1];
    body_velocity_rotated[1] = -st * body_velocity[0] + ct * body_velocity[1];
    float32_t rotation_component[3];

    for(uint8_t i=0; i<4; i++){
        if(foot_scheduler_diff[i]!= -1) {continue;}
        foot_step_landing_locations[i*3 + 0] = body_velocity_rotated[0] * float_time / 2;
        foot_step_landing_locations[i*3 + 1] = body_velocity_rotated[1] * float_time / 2;
        rotation_component[0] = cotg * shoulder_vectors[i*3 + 0] - sotg * shoulder_vectors[i*3 + 1] - shoulder_vectors[i*3 + 0];
        rotation_component[1] = sotg * shoulder_vectors[i*3 + 0] + cotg * shoulder_vectors[i*3 + 1] - shoulder_vectors[i*3 + 1];
        foot_step_landing_locations[i*3 + 0] += rotation_component[0];
        foot_step_landing_locations[i*3 + 1] += rotation_component[1];
    }
}

void generate_lift_leg_trajectory(int8_t foot_scheduler_diff[], float32_t foot_step_landing_locations[], float32_t endpoints[], float32_t lift_leg_trajectory[15][12], float32_t lift_leg_trajectory_angles[15][12]){
    float32_t leg_travel_distance[3];
    float32_t angles[3];
    float32_t x,y,z;
    for(uint8_t i=0; i<4; i++){
        if(foot_scheduler_diff[i]!= -1) {continue;}
        leg_travel_distance[0] = foot_step_landing_locations[i*3] - endpoints[i*3];
        leg_travel_distance[1] = foot_step_landing_locations[i*3+1] - endpoints[i*3+1];
        leg_travel_distance[2] = 0;

        for(uint8_t step=0; step<4;step++){
            x = endpoints[i*3];
            y = endpoints[i*3+1];
            z = endpoints[i*3+2] + 0.025 * (step+1);
            lift_leg_trajectory[step][i*3]   = x;
            lift_leg_trajectory[step][i*3+1] = y;
            lift_leg_trajectory[step][i*3+2] = z; // create trajectory such that we start lifting imemdiately add 0.025, 0.05, 0.075, 0.1
            calculate_angles_from_endpoint(x, y, z, angles);
            lift_leg_trajectory_angles[step][i*3]   = angles[0];
            lift_leg_trajectory_angles[step][i*3+1] = angles[1];
            lift_leg_trajectory_angles[step][i*3+2] = angles[2];

        }

        for(uint8_t step=4; step<10;step++){
            x = endpoints[i*3] + ((step - 3)/6.0) * leg_travel_distance[0];
            y = endpoints[i*3+1] + ((step - 3)/6.0) * leg_travel_distance[1];
            z = lift_leg_trajectory[i*3+2][3];
            lift_leg_trajectory[step][i*3]   = x; // start moving towards our end position
            lift_leg_trajectory[step][i*3+1] = y;
            lift_leg_trajectory[step][i*3+2] = z;
            calculate_angles_from_endpoint(x, y, z, angles);
            lift_leg_trajectory_angles[step][i*3]   = angles[0];
            lift_leg_trajectory_angles[step][i*3+1] = angles[1];
            lift_leg_trajectory_angles[step][i*3+2] = angles[2];
        }

        for(uint8_t step=10; step<15;step++){
            x = endpoints[i*3];
            y = endpoints[i*3+1];
            z = lift_leg_trajectory[i*3+2][3] - 0.02 * (step-9);
            lift_leg_trajectory[step][i*3]   = x; // start moving towards our end position
            lift_leg_trajectory[step][i*3+1] = y;
            lift_leg_trajectory[step][i*3+2] = z;
            calculate_angles_from_endpoint(x, y, z, angles);
            lift_leg_trajectory_angles[step][i*3]   = angles[0];
            lift_leg_trajectory_angles[step][i*3+1] = angles[1];
            lift_leg_trajectory_angles[step][i*3+2] = angles[2];
        }
    }
}

void calculate_angles_from_endpoint(float32_t x, float32_t y, float32_t z, float32_t angles[]){

    float32_t d;
    float32_t h;
    float32_t hv;
    float32_t lv;
    float32_t q2;
    float32_t qpom;
    float32_t q1;
    float32_t lv_p;
    float32_t zy;
    float32_t qz;
    float32_t qh;
    float32_t q0;

    for (uint8_t i=0; i<4; i++) {
        d = 0.2;
        h = 0.065;
        hv = x*x + y*y + z*z;
        lv = hv - h*h;
        q2 = asin(sqrt(lv)/2/d)*2 - PI;
        qpom = asin(x/sqrt(lv));
        q1 = acos(sqrt(lv)/2/d) - qpom;
        lv_p = cos(qpom) * sqrt(lv);
        zy = sqrt(z*z + y*y);
        qz = asin(y/zy);
        qh = asin(lv_p/zy);

        if (sides[i] == 1) {
            q0 = + qz + qh - PI/2;
        } else {
            q0 = + qz - qh + PI/2;
        }
        q0 *= angle_multiplier_a2p[i*3];
        q1 *= angle_multiplier_a2p[i*3+1];
        q2 *= angle_multiplier_a2p[i*3+2];
    }
    angles[0] = q0;
    angles[1] = q1;
    angles[2] = q2;
}
