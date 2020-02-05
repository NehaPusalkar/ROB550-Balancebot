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
double b=0.2095;

double c_L=0.9991;
double c_R=1.0009;
double delta_phi_left = 2 * M_PI * (mb_state->left_encoder - mb_state->pre_left_encoder)/(GEAR_RATIO*ENCODER_RES);
double delta_s_left = r * delta_phi_left;

double delta_phi_right = 2 * M_PI * (mb_state->right_encoder - mb_state->pre_right_encoder)/(GEAR_RATIO*ENCODER_RES);
double delta_s_right = r * delta_phi_right;

double prev_tb = mb_state->prev_tb;
prev_tb = mb_clamp_radians(prev_tb);
double curr_tb =  mb_state->yaw;
curr_tb = mb_clamp_radians(curr_tb);
double delta_gyro = curr_tb - prev_tb;
mb_state->prev_tb = curr_tb;

//delta_s_right = mb_clamp_radians(delta_s_right);
//delta_s_left = mb_clamp_radians(delta_s_left);
double delta_psi = (delta_s_right+delta_s_left) / b;
double delta_go = delta_gyro-delta_psi;

double delta_s = (delta_s_right-delta_s_left) / 2;
double delta_x = delta_s * cos( mb_odometry->psi + delta_psi/2);
double delta_y = delta_s * sin( mb_odometry->psi + delta_psi/2);

mb_odometry->x = mb_odometry->x + delta_x;
mb_odometry->y = mb_odometry->y + delta_y;
if (fabs(delta_go)>(3.0*M_PI/180)&&fabs(delta_go)<(M_PI)){
    mb_odometry->psi = mb_odometry->psi + delta_gyro;
}
else{
    mb_odometry->psi = mb_odometry->psi + delta_psi;
    
}
//mb_odometry->psi = mb_clamp_radians(mb_odometry->psi);
//mb_odometry->psi = mb_odometry->psi + delta_psi;

}

float mb_clamp_radians(float angle){
    double clamped_angle = 0;
    if(angle > M_PI){
        clamped_angle = angle - 2*M_PI;
    }
    else if(angle < -M_PI){
        clamped_angle = angle + 2*M_PI;
    }
    else{
        clamped_angle = angle;
    }

    return clamped_angle;
}
