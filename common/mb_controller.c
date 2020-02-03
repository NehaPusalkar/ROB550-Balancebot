#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <rc/math/filter.h>
#include <unistd.h>
#include <rc/pwm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include "../balancebot/balancebot.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
double pid_1[4], pid_2[4], pid_3[4], pid_4[4];

double TIME_CONSTANT = 0.1;

    rc_filter_t D1 = RC_FILTER_INITIALIZER;
    rc_filter_t D2 = RC_FILTER_INITIALIZER;
    rc_filter_t D3 = RC_FILTER_INITIALIZER;
    rc_filter_t D4 = RC_FILTER_INITIALIZER;

int mb_controller_init(){

    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    printf("inner loop controller- kp: %f, ki: %f, kd: %f\n", pid_1[0], pid_1[1], pid_1[2]);
    printf("outer loop controller- kp: %f, ki: %f, kd: %f\n", pid_2[0], pid_2[1], pid_2[2]);
    rc_filter_pid(&D1, pid_1[0], pid_1[1], pid_1[2], pid_1[3], DT);
    rc_filter_enable_saturation(&D1,-1.0,1.0);
    rc_filter_enable_soft_start(&D1,0.7);

    rc_filter_pid(&D2, pid_2[0], pid_2[1], pid_2[2], pid_2[3], DT);
    rc_filter_enable_saturation(&D2,-0.3,0.3);
    rc_filter_enable_soft_start(&D2,0.7);

    rc_filter_pid(&D3, pid_3[0], pid_3[1], pid_3[2], 4*DT, DT);
    rc_filter_enable_saturation(&D3,-0.5,0.5);

    rc_filter_pid(&D4, pid_4[0], pid_4[1], pid_4[2], 4*DT, DT);
    rc_filter_enable_saturation(&D4,-0.5,0.5);
    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
* 
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    double val;
    int i,j;
    double* M = (double*) malloc(4*4*sizeof(double));
    for(i = 0; i < 4; i++){
	for(j = 0; j<4; j++){
	    if(fscanf(file, "%lf", &val) != 1){
		fprintf(stderr, "Couldn't read value. \n");
		return NULL;
	    }
	    fgetc(file);
	    M[i*4+j] = val;
	}
    }
    printf("loaded gains- kp: %f, ki: %f, kd: %f\n", M[0], M[1], M[2]);

    pid_1[0] = M[0];
    pid_1[1] = M[1];
    pid_1[2] = M[2];
    pid_1[3] = M[3];

    pid_2[0] = M[4];
    pid_2[1] = M[5];
    pid_2[2] = M[6];
    pid_2[3] = M[7];

    pid_3[0] = M[8];
    pid_3[1] = M[9];
    pid_3[2] = M[10];
    pid_3[3] = M[11];

    pid_4[0] = M[12];
    pid_4[1] = M[13];
    pid_4[2] = M[14];
    pid_4[3] = M[15];
    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    double cr_theta = mb_state->theta;

    double sp_gamma;
    double cr_gamma;
    //cr_gamma = (mb_state->right_angle - mb_state->left_angle) * (0.0418/0.208);
    cr_gamma = mb_state->yaw;

    double cr_phi= -(mb_state->phi);
    double D2_duty = rc_filter_march(&D2, (0+mb_state->angle-cr_phi));
    double D1_duty = rc_filter_march(&D1, (D2_duty-cr_theta));


    //double diff_velocity = (mb_state->left_speed-mb_state->right_speed) * mb_state->small_dt;

    /*double delta_phi_left = 2 * M_PI * (mb_state->left_encoder - mb_state->pre_left_encoder)/(GEAR_RATIO*ENCODER_RES);
     double delta_s_left = 0.0418 * delta_phi_left;

    double delta_phi_right = 2 * M_PI * (mb_state->right_encoder - mb_state->pre_right_encoder)/(GEAR_RATIO*ENCODER_RES);
    double delta_s_right = 0.0418 * delta_phi_right;

    double delta_psi = (delta_s_right+delta_s_left) /0.208;  //baseline
    
    mb_state->heading=mb_state->heading+delta_psi;
    double D3_duty = rc_filter_march(&D3, ( 0+mb_state->turn_angle - mb_state->heading));*/
   
    double D3_duty = rc_filter_march(&D3, ( 0+mb_state->turn_angle - mb_odometry.psi));

    double left_duty = D1_duty-D3_duty;
    double right_duty = D1_duty+D3_duty;
    
    mb_motor_set(LEFT_MOTOR,left_duty);
    mb_motor_set(RIGHT_MOTOR,right_duty);
    //mb_motor_set_all(D1_duty);
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    rc_filter_free(&D1);
    rc_filter_free(&D2);
    rc_filter_free(&D3);
    rc_filter_free(&D4);
    mb_motor_set_all(0);
    rc_dsm_cleanup();
    rc_mpu_power_off();
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file();
    return 0;
}
