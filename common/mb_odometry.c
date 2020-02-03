/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
mb_odometry->x = x;
mb_odometry->y = y;
mb_odometry->psi = theta;

}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
double E_b=0.9752;
double r=0.0418;
double b=0.208;

double c_L=0.9991;
double delta_phi_left = 2 * M_PI * (mb_state->left_encoder - mb_state->pre_left_encoder)/(GEAR_RATIO*ENCODER_RES);
double delta_s_left = r * delta_phi_left;

double c_R=1.0009;
double delta_phi_right = 2 * M_PI * (mb_state->right_encoder - mb_state->pre_right_encoder)/(GEAR_RATIO*ENCODER_RES);
double delta_s_right = r * delta_phi_right;

double delta_gyro=mpu_data.gyro[2]*mb_state->small_dt*M_PI/180;
double delta_psi = (delta_s_right+delta_s_left) / b;
double delta_go = delta_gyro-delta_psi;

double delta_s = (delta_s_right-delta_s_left) / 2;
double delta_x = delta_s * cos( mb_odometry->psi + delta_psi/2);
double delta_y = delta_s * sin( mb_odometry->psi + delta_psi/2);

mb_odometry->x = mb_odometry->x + delta_x;
mb_odometry->y = mb_odometry->y + delta_y;
/*if (fabs(delta_go)>(1.0*M_PI/180)){
    mb_odometry->psi = mb_odometry->psi + delta_gyro;
}
else{
    mb_odometry->psi = mb_odometry->psi + delta_psi;
}*/
mb_odometry->psi = mb_odometry->psi + delta_psi;

}

float mb_clamp_radians(float angle){
    return 0;
}
