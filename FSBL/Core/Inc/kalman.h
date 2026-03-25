#include "stdio.h"
#include <math.h>
#include "arm_math.h"

extern float32_t mat_A[18][18];
extern float32_t mat_B[18][18];
extern float32_t mat_C[18][18];

extern float32_t ang[3];
extern float32_t Cbw[3][3];
extern float32_t Cwb[3][3];
extern float32_t Cwbn[3][3];
extern float32_t Rx[3][3];
extern float32_t Ry[3][3];
extern float32_t Rz[3][3];
extern float32_t Rxy[3][3];
extern float32_t Ryz[3][3];
extern float32_t Q[18][18];
extern float32_t F[18][18];
extern float32_t Ft[18][18];
extern float32_t P[18][18];
extern float32_t Pp[18][18];
extern float32_t FP[18][18];
extern float32_t G[18][3];
extern uint8_t fs[4];
extern float32_t R[24][24];
extern float32_t H[24][18];
extern float32_t Ht[18][24];
extern float32_t HPp[24][18];
extern float32_t HPpHt[24][24];
extern float32_t HPpHtR[24][24];
extern float32_t HPpHtI[24][24];
extern float32_t PpHt[18][24];
extern float32_t K[18][24];
extern float32_t I[18][18];
extern float32_t KH[18][18];
extern float32_t IKH[18][18];
extern float32_t IKHt[18][18];
extern float32_t IKHPp[18][18];
extern float32_t IKHPpIKHt[18][18];
extern float32_t KR[18][24];
extern float32_t KRKt[18][18];
extern float32_t Kt[24][18];
extern float32_t xn[18];
extern float32_t xnp[18];
extern float32_t zn[24];
extern float32_t Hxnp[24];
extern float32_t KznHxnp[18];
//extern float32_t accel[3];
extern float32_t Fxn[18];
extern float32_t Ga[18];


extern arm_matrix_instance_f32 mat_A_instance;
extern arm_matrix_instance_f32 mat_B_instance;
extern arm_matrix_instance_f32 mat_C_instance;

extern arm_matrix_instance_f32 Cbw_inst;
extern arm_matrix_instance_f32 Cwb_inst;
extern arm_matrix_instance_f32 Cwbn_inst;
extern arm_matrix_instance_f32 Rx_inst;
extern arm_matrix_instance_f32 Ry_inst;
extern arm_matrix_instance_f32 Rz_inst;
extern arm_matrix_instance_f32 Rxy_inst;
extern arm_matrix_instance_f32 Ryz_inst;
extern arm_matrix_instance_f32 Q_inst;
extern arm_matrix_instance_f32 F_inst;
extern arm_matrix_instance_f32 Ft_inst;
extern arm_matrix_instance_f32 P_inst;
extern arm_matrix_instance_f32 Pp_inst;
extern arm_matrix_instance_f32 FP_inst;
extern arm_matrix_instance_f32 G_inst;
extern arm_matrix_instance_f32 R_inst;
extern arm_matrix_instance_f32 H_inst;
extern arm_matrix_instance_f32 Ht_inst;
extern arm_matrix_instance_f32 HPp_inst;
extern arm_matrix_instance_f32 HPpHt_inst;
extern arm_matrix_instance_f32 HPpHtR_inst;
extern arm_matrix_instance_f32 HPpHtI_inst;
extern arm_matrix_instance_f32 PpHt_inst;
extern arm_matrix_instance_f32 K_inst;
extern arm_matrix_instance_f32 I_inst;
extern arm_matrix_instance_f32 KH_inst;
extern arm_matrix_instance_f32 IKH_inst;
extern arm_matrix_instance_f32 IKHt_inst;
extern arm_matrix_instance_f32 IKHPp_inst;
extern arm_matrix_instance_f32 IKHPpIKHt_inst;
extern arm_matrix_instance_f32 KR_inst;
extern arm_matrix_instance_f32 KRKt_inst;
extern arm_matrix_instance_f32 Kt_inst;
//extern arm_matrix_instance_f32 ep_inst;
//extern arm_matrix_instance_f32 ep_vel_inst;

extern arm_status arm_status_temp;

void init_matrices();
void gen_F(float32_t dt);
void gen_G(float32_t dt);
void init_P_Pp();
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
                  arm_matrix_instance_f32* Cwb_inst);

void gen_H(float32_t H[24][18], 
            float32_t Cwb[3][3], 
            float32_t Cwbn[3][3],
            arm_matrix_instance_f32* Cwb_inst,
            arm_matrix_instance_f32* Cwbn_inst);

void gen_R(uint8_t fs[4],
            float32_t R[24][24], 
            float var_R, 
            float var_V);

void gen_Q(float32_t Q[18][18], float var_Axyz, float var_Wxyz, float dt);

void calculate_Pp(arm_matrix_instance_f32* F_inst,
                  arm_matrix_instance_f32* Ft_inst,
                  arm_matrix_instance_f32* P_inst,
                  arm_matrix_instance_f32* FP_inst,
                  arm_matrix_instance_f32* Q_inst,
                  arm_matrix_instance_f32* Pp_inst);

void calculate_K(arm_matrix_instance_f32* H_inst, 
                 arm_matrix_instance_f32* Pp_inst, 
                 arm_matrix_instance_f32* R_inst, 
                 arm_matrix_instance_f32* HPp_inst, 
                 arm_matrix_instance_f32* Ht_inst, 
                 arm_matrix_instance_f32* HPpHt_inst, 
                 arm_matrix_instance_f32* HPpHtR_inst,
                 arm_matrix_instance_f32* HPpHtI_inst, 
                 arm_matrix_instance_f32* PpHt_inst,  
                 arm_matrix_instance_f32* K_inst);

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
                 arm_matrix_instance_f32* P_inst);

void calculate_xn(float32_t xn[18], 
                   float32_t xnp[18], 
                   arm_matrix_instance_f32* K_inst, 
                   float32_t zn[24], 
                   arm_matrix_instance_f32* H_inst, 
                   float32_t Hxnp[24], 
                   float32_t KznHxnp[18]);

void calculate_xn_pred(arm_matrix_instance_f32* F_inst, 
                        float32_t xn[18], 
                        arm_matrix_instance_f32* G_inst, 
                        float32_t accel[3], 
                        float32_t Fxn[18], 
                        float32_t xnp[18], 
                        float32_t Ga[18]);

void update_zn(float32_t zn[24], 
               float32_t ep[4][3],
               float32_t ep_vel[4][3]);

void step(float32_t ang[3], uint8_t fs[4], float32_t var_R, float32_t var_V, float32_t var_A, float32_t var_W, float32_t ep[4][3], float32_t ep_vel[4][3], float32_t accel[3], float32_t dt);
