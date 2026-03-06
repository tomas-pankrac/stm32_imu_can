#include "stdio.h"
#include <math.h>
#include "arm_math.h"


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

float32_t endpoints[4][3];
float32_t endpoints_velocity[4][3];
float32_t torques[4][3];

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
        endpoints[i][0] = H2[2][3];
        endpoints[i][1] = H2[1][3];
        endpoints[i][2] = H2[0][3];

        generate_Jacobian();

        arm_mat_inverse_f32(&J_inst, &Jinv_inst);

        angle_vel[0] = angle_velocities[0+3*i];
        angle_vel[1] = angle_velocities[1+3*i];
        angle_vel[2] = angle_velocities[2+3*i];
        arm_mat_mult_f32(&J_inst, &angle_vel_inst, &ep_vel_inst);

        endpoints_velocity[i][0] = ep_vel[2];
        endpoints_velocity[i][1] = ep_vel[1];
        endpoints_velocity[i][2] = ep_vel[0];



        angle_vel[0] = forces[0+3*i];
        angle_vel[1] = forces[1+3*i];
        angle_vel[2] = forces[2+3*i];
        arm_mat_mult_f32(&Jinv_inst, &angle_vel_inst, &ep_vel_inst);

        torques[i][0] = ep_vel[2];
        torques[i][1] = ep_vel[1];
        torques[i][2] = ep_vel[0];
    }
}
