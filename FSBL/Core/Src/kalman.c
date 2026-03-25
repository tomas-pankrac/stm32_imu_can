#include "stdio.h"
#include <math.h>
#include "arm_math.h"
#include "kalman.h"

float32_t ang[3];
float32_t Cbw[3][3];
float32_t Cwb[3][3];
float32_t Cwbn[3][3];
float32_t Rx[3][3];
float32_t Ry[3][3];
float32_t Rz[3][3];
float32_t Rxy[3][3];
float32_t Ryz[3][3];
float32_t Q[18][18];
float32_t F[18][18];
float32_t Ft[18][18];
float32_t P[18][18];
float32_t Pp[18][18];
float32_t FP[18][18];
float32_t G[18][3];
//int fs[4];
//float var_R =  1e-5;
//float var_V = 1e-5;
float32_t R[24][24];
float32_t H[24][18];
float32_t Ht[18][24];
float32_t HPp[24][18];
float32_t HPpHt[24][24];
float32_t HPpHtR[24][24];
float32_t HPpHtI[24][24];
float32_t PpHt[18][24];
float32_t K[18][24];
float32_t I[18][18];
float32_t KH[18][18];
float32_t IKH[18][18];
float32_t IKHt[18][18];
float32_t IKHPp[18][18];
float32_t IKHPpIKHt[18][18];
float32_t KR[18][24];
float32_t KRKt[18][18];
float32_t Kt[24][18];
float32_t xn[18];
float32_t xnp[18];
float32_t zn[24];
float32_t Hxnp[24];
float32_t KznHxnp[18];
float32_t accel[3];
float32_t Fxn[18];
float32_t Ga[18];

arm_matrix_instance_f32 Cbw_inst;
arm_matrix_instance_f32 Cwb_inst;
arm_matrix_instance_f32 Cwbn_inst;
arm_matrix_instance_f32 Rx_inst;
arm_matrix_instance_f32 Ry_inst;
arm_matrix_instance_f32 Rz_inst;
arm_matrix_instance_f32 Rxy_inst;
arm_matrix_instance_f32 Ryz_inst;
arm_matrix_instance_f32 Q_inst;
arm_matrix_instance_f32 F_inst;
arm_matrix_instance_f32 Ft_inst;
arm_matrix_instance_f32 P_inst;
arm_matrix_instance_f32 Pp_inst;
arm_matrix_instance_f32 FP_inst;
arm_matrix_instance_f32 G_inst;
arm_matrix_instance_f32 R_inst;
arm_matrix_instance_f32 H_inst;
arm_matrix_instance_f32 Ht_inst;
arm_matrix_instance_f32 HPp_inst;
arm_matrix_instance_f32 HPpHt_inst;
arm_matrix_instance_f32 HPpHtR_inst;
arm_matrix_instance_f32 HPpHtI_inst;
arm_matrix_instance_f32 PpHt_inst;
arm_matrix_instance_f32 K_inst;
arm_matrix_instance_f32 I_inst;
arm_matrix_instance_f32 KH_inst;
arm_matrix_instance_f32 IKH_inst;
arm_matrix_instance_f32 IKHt_inst;
arm_matrix_instance_f32 IKHPp_inst;
arm_matrix_instance_f32 IKHPpIKHt_inst;
arm_matrix_instance_f32 KR_inst;
arm_matrix_instance_f32 KRKt_inst;
arm_matrix_instance_f32 Kt_inst;
//arm_matrix_instance_f32 ep_inst;
//arm_matrix_instance_f32 ep_vel_inst;

arm_status arm_status_temp;


void init_matrices() {
    uint32_t rows, cols;

    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Cbw_inst, rows, cols, (float32_t *)Cbw);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Cwb_inst, rows, cols, (float32_t *)Cwb);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Cwbn_inst, rows, cols, (float32_t *)Cwbn);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Rx_inst, rows, cols, (float32_t *)Rx);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Ry_inst, rows, cols, (float32_t *)Ry);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Rz_inst, rows, cols, (float32_t *)Rz);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Rxy_inst, rows, cols, (float32_t *)Rxy);
    rows = 3;
    cols = 3;
    arm_mat_init_f32(&Ryz_inst, rows, cols, (float32_t *)Ryz);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&Q_inst, rows, cols, (float32_t *)Q);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&F_inst, rows, cols, (float32_t *)F);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&Ft_inst, rows, cols, (float32_t *)Ft);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&P_inst, rows, cols, (float32_t *)P);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&Pp_inst, rows, cols, (float32_t *)Pp);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&FP_inst, rows, cols, (float32_t *)FP);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&G_inst, rows, cols, (float32_t *)G);
    rows = 24;
    cols = 3;
    arm_mat_init_f32(&R_inst, rows, cols, (float32_t *)R);
    rows = 24;
    cols = 18;
    arm_mat_init_f32(&H_inst, rows, cols, (float32_t *)H);
    rows = 18;
    cols = 24;
    arm_mat_init_f32(&Ht_inst, rows, cols, (float32_t *)Ht);
    rows = 24;
    cols = 18;
    arm_mat_init_f32(&HPp_inst, rows, cols, (float32_t *)HPp);
    rows = 24;
    cols = 24;
    arm_mat_init_f32(&HPpHt_inst, rows, cols, (float32_t *)HPpHt);
    rows = 24;
    cols = 24;
    arm_mat_init_f32(&HPpHtR_inst, rows, cols, (float32_t *)HPpHtR);
    rows = 24;
    cols = 24;
    arm_mat_init_f32(&HPpHtI_inst, rows, cols, (float32_t *)HPpHtI);
    rows = 18;
    cols = 24;
    arm_mat_init_f32(&PpHt_inst, rows, cols, (float32_t *)PpHt);
    rows = 18;
    cols = 24;
    arm_mat_init_f32(&K_inst, rows, cols, (float32_t *)K);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&I_inst, rows, cols, (float32_t *)I);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&KH_inst, rows, cols, (float32_t *)KH);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&IKH_inst, rows, cols, (float32_t *)IKH);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&IKHt_inst, rows, cols, (float32_t *)IKHt);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&IKHPp_inst, rows, cols, (float32_t *)IKHPp);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&IKHPpIKHt_inst, rows, cols, (float32_t *)IKHPpIKHt);
    rows = 18;
    cols = 24;
    arm_mat_init_f32(&KR_inst, rows, cols, (float32_t *)KR);
    rows = 18;
    cols = 18;
    arm_mat_init_f32(&KRKt_inst, rows, cols, (float32_t *)KRKt);
    rows = 24;
    cols = 18;
    arm_mat_init_f32(&Kt_inst, rows, cols, (float32_t *)Kt);
//    rows = 3;
//    cols = 4;
//    arm_mat_init_f32(&ep_inst, rows, cols, (float32_t *)ep);
//    rows = 3;
//    cols = 4;
//    arm_mat_init_f32(&ep_vel_inst, rows, cols, (float32_t *)ep_vel);
    init_P_Pp();
}

void gen_F(float32_t dt){
    for(uint8_t i=0; i<18; i++){
        F[i][i] = 1.0;
    }
    for(uint8_t i=0; i<3; i++){
        F[i][i+3] = dt;
    }
}

void gen_G(float32_t dt){
    for(uint8_t i=0; i<3; i++){
        G[i][i] = 0.5 * dt * dt;
        G[i+3][i] = dt;
    }
}

void init_P_Pp(){
    for(uint8_t i=0; i<18; i++){
        P[i][i] = 1;
        Pp[i][i] = 1;
    }
}

void rot_matrices(float32_t ang[3], 
                  float32_t Rz[3][3], 
                  float32_t Ry[3][3], 
                  float32_t Rx[3][3], 
                  arm_matrix_instance_f32* Rx_inst,
                  arm_matrix_instance_f32* Ry_inst, 
                  arm_matrix_instance_f32* Rz_inst, 
                  arm_matrix_instance_f32* Rxy_inst, 
                  arm_matrix_instance_f32* Ryz_inst, 
                  arm_matrix_instance_f32* Cbw_inst, 
                  arm_matrix_instance_f32* Cwb_inst) {
                    
    float cz = cos(ang[2]);
    float sz = sin(ang[2]);
    float cy = cos(ang[1]);
    float sy = sin(ang[1]);
    float cx = cos(ang[0]);
    float sx = sin(ang[0]);
    
    Rz[2][2] = 1;
	Rz[0][0] = cz;
	Rz[0][1] = -sz;
	Rz[1][0] = sz;
	Rz[1][1] = cz;
    
    Ry[1][1] = 1;
    Ry[0][0] = cy;
    Ry[0][2] = sy;
    Ry[2][0] = -sy;
    Ry[2][2] = cy;
    
    Rx[0][0] = 1;
    Rx[1][1] = cx;
    Rx[1][2] = -sx;
    Rx[2][1] = sx;
    Rx[2][2] = cx;

    arm_mat_mult_f32(Rx_inst, Ry_inst, Rxy_inst);
    arm_mat_mult_f32(Rxy_inst, Rz_inst, Ryz_inst);
    arm_mat_mult_f32(Rxy_inst, Ryz_inst, Cbw_inst);
    arm_mat_trans_f32(Cbw_inst, Cwb_inst);
}

void gen_H(float32_t H[24][18], 
           float32_t Cwb[3][3], 
           float32_t Cwbn[3][3],
           arm_matrix_instance_f32* Cwb_inst,
           arm_matrix_instance_f32* Cwbn_inst){

    int i, j, k;
    float s = -1;
    arm_mat_scale_f32(Cwb_inst, s, Cwbn_inst);
    for(i=0; i<4; i++){
        for(j=0; j<3; j++){
            for(k=0; k<3; k++){
                H[i*3 + j][k] = Cwbn[j][k];
                H[i*3 + j][6+i*3 + k] = Cwb[j][k];
                H[(i+4)*3 + j][3 + k] = Cwbn[j][k];
            }
        }
    }
}

void gen_R(uint8_t fs[4],
            float32_t R[24][24], 
            float var_R, 
            float var_V){
    int i;
    
    for(i=0; i<4; i++){
        if (fs[i] == 0){
            R[i*3][i*3] = 1e9;
            R[i*3+1][i*3+1] = 1e9;
            R[i*3+2][i*3+2] = 1e9;
            R[(i+4)*3][(i+4)*3] = 1e9;
            R[(i+4)*3+1][(i+4)*3+1] = 1e9;
            R[(i+4)*3+2][(i+4)*3+2] = 1e9;
        }
        else {
            R[i*3][i*3] = var_R;
            R[i*3+1][i*3+1] = var_R;
            R[i*3+2][i*3+2] = var_R/1e3;
            R[(i+4)*3][(i+4)*3] = var_V;
            R[(i+4)*3+1][(i+4)*3+1] = var_V;
            R[(i+4)*3+2][(i+4)*3+2] = var_V/1e4;
        }
    }
}

void gen_Q(float32_t Q[18][18], 
           float var_Axyz, 
           float var_Wxyz, 
           float dt){
    int i;

    for(i=0; i<18; i++){
        Q[i][i] = 1;
    }

    for(i=0; i<3; i++){
        Q[i][i] = (dt * dt * dt * dt) / 4 * var_Axyz;
        Q[i][i+3] = (dt * dt * dt) / 2  * var_Axyz;
        Q[i+3][i] = (dt * dt * dt) / 2 * var_Axyz;
        Q[i+3][i+3] = (dt * dt) * var_Axyz;
    }
    for(i=6; i<18; i++){
        Q[i][i] *= var_Wxyz;
    }
}

void calculate_Pp(arm_matrix_instance_f32* F_inst,
                  arm_matrix_instance_f32* Ft_inst,
                  arm_matrix_instance_f32* P_inst,
                  arm_matrix_instance_f32* FP_inst,
                  arm_matrix_instance_f32* Q_inst,
                  arm_matrix_instance_f32* Pp_inst){
    arm_mat_mult_f32(F_inst, P_inst, FP_inst);
    arm_mat_trans_f32(F_inst, Ft_inst);
    arm_mat_mult_f32(FP_inst, Ft_inst, Pp_inst);
    arm_mat_add_f32(Pp_inst, Q_inst, Pp_inst);
}

void calculate_K(arm_matrix_instance_f32* H_inst, 
                 arm_matrix_instance_f32* Pp_inst, 
                 arm_matrix_instance_f32* R_inst, 
                 arm_matrix_instance_f32* HPp_inst, 
                 arm_matrix_instance_f32* Ht_inst, 
                 arm_matrix_instance_f32* HPpHt_inst, 
                 arm_matrix_instance_f32* HPpHtR_inst,
                 arm_matrix_instance_f32* HPpHtI_inst, 
                 arm_matrix_instance_f32* PpHt_inst,  
                 arm_matrix_instance_f32* K_inst){

    arm_status status;

    status = arm_mat_mult_f32(H_inst, Pp_inst, HPp_inst);
    status = arm_mat_trans_f32(H_inst, Ht_inst);
    status = arm_mat_mult_f32(HPp_inst, Ht_inst, HPpHt_inst);
    status = arm_mat_add_f32(HPpHt_inst, R_inst, HPpHtR_inst);
    status = arm_mat_inverse_f32(HPpHtR_inst, HPpHtI_inst);
    status = arm_mat_mult_f32(Pp_inst, Ht_inst, PpHt_inst);
    status = arm_mat_mult_f32(PpHt_inst, HPpHtI_inst, K_inst);
}

void calculate_P(arm_matrix_instance_f32* K_inst, 
                 arm_matrix_instance_f32* KH_inst, 
                 arm_matrix_instance_f32* H_inst, 
                 arm_matrix_instance_f32* I_inst, 
                 arm_matrix_instance_f32* IKH_inst, 
                 arm_matrix_instance_f32* IKHt_inst, 
                 arm_matrix_instance_f32* IKHPp_inst, 
                 arm_matrix_instance_f32* Pp_inst, 
                 arm_matrix_instance_f32* R_inst, 
                 arm_matrix_instance_f32* KR_inst, 
                 arm_matrix_instance_f32* IKHPpIKHt_inst, 
                 arm_matrix_instance_f32* KRKt_inst, 
                 arm_matrix_instance_f32* Kt_inst, 
                 arm_matrix_instance_f32* P_inst){
    arm_mat_mult_f32(K_inst, H_inst, KH_inst);
    arm_mat_sub_f32(I_inst, KH_inst, IKH_inst);
    arm_mat_trans_f32(IKH_inst, IKHt_inst);
    arm_mat_trans_f32(K_inst, Kt_inst);
    arm_mat_mult_f32(IKH_inst, Pp_inst, IKHPp_inst);
    arm_mat_mult_f32(IKHPp_inst, IKHt_inst, IKHPpIKHt_inst);
    arm_mat_mult_f32(K_inst, R_inst, KR_inst);
    arm_mat_mult_f32(KR_inst, Kt_inst, KRKt_inst);
    arm_mat_add_f32(IKHPpIKHt_inst, KRKt_inst, P_inst);
}


void calculate_xn(float32_t xn[18], 
                   float32_t xnp[18], 
                   arm_matrix_instance_f32* K_inst, 
                   float32_t zn[24], 
                   arm_matrix_instance_f32* H_inst, 
                   float32_t Hxnp[24], 
                   float32_t KznHxnp[18]){
    arm_mat_vec_mult_f32(H_inst, xnp, Hxnp);
    arm_sub_f32(zn, Hxnp, zn, 24);
    arm_mat_vec_mult_f32(K_inst, zn, KznHxnp);
    arm_add_f32(xnp, KznHxnp, xn, 18);
}
 
void calculate_xn_pred(arm_matrix_instance_f32* F_inst, 
                        float32_t xn[18], 
                        arm_matrix_instance_f32* G_inst, 
                        float32_t accel[3], 
                        float32_t Fxn[18], 
                        float32_t xnp[18], 
                        float32_t Ga[18]){
    arm_mat_vec_mult_f32(G_inst, accel, Ga);
    arm_mat_vec_mult_f32(F_inst, xn, Fxn);
    arm_add_f32(Fxn, Ga, xnp, 18);
}
    
void update_zn(float32_t zn[24], 
               float32_t ep[4][3],
               float32_t ep_vel[4][3]){
                    
    int i, j;
    for(i=0; i<4; i++){
        for(j=0; j<3; j++){
            zn[i*3+j] = ep[i][j];
            zn[(i+4)*3+j] = ep_vel[i][j];
        }
    }
}

void step(float32_t ang[3], 
          uint8_t fs[4],
          float32_t var_R,
          float32_t var_V,
          float32_t var_A,
          float32_t var_W,
          float32_t ep[4][3],
          float32_t ep_vel[4][3],
          float32_t accel[3],
          float32_t dt){
    gen_F(dt);
    gen_G(dt);
    gen_Q(Q, var_A, var_W, dt);
    rot_matrices(ang, Rz, Ry, Rx, &Rx_inst, &Ry_inst, &Rz_inst, &Rxy_inst, &Ryz_inst, &Cbw_inst, &Cwb_inst);
    gen_H(H, Cwb, Cwbn, &Cwb_inst, &Cwbn_inst);
    gen_R(fs, R, var_R, var_V);
    calculate_K(&H_inst, &Pp_inst, &R_inst, &HPp_inst, &Ht_inst, &HPpHt_inst, &HPpHtR_inst, &HPpHtI_inst, &PpHt_inst, &K_inst);
    update_zn(zn, ep, ep_vel);
    calculate_xn(xn, xnp, &K_inst, zn, &H_inst, Hxnp, KznHxnp);
    calculate_P(&K_inst, &KH_inst, &H_inst, &I_inst, &IKH_inst, &IKHt_inst, &IKHPp_inst, &Pp_inst, &R_inst, &KR_inst, &IKHPpIKHt_inst, &KRKt_inst, &Kt_inst, &P_inst);
    calculate_xn_pred(&F_inst, xn, &G_inst, accel, Fxn, xnp, Ga);
    calculate_Pp(&F_inst, &Ft_inst, &P_inst, &FP_inst, &Q_inst, &Pp_inst);
}
