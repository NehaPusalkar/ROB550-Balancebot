#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
double pid_1[], pid_2[], pid_3[], pid_4[];
double DT = 1/SAMPLE_RATE_HZ;

int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    rc_filter_t D1 = RC_FILTER_INITIALIZER;
    rc_filter_t D2 = RC_FILTER_INITIALIZER;
    rc_filter_t D3 = RC_FILTER_INITIALIZER;
    rc_filter_t D4 = RC_FILTER_INITIALIZER;
    
    rc_filter_pid(&D1, pid_1[0], pid_1[1], pid_1[2], 4*DT, DT);
    rc_filter_pid(&D2, pid_2[0], pid_2[1], pid_2[2], 4*DT, DT);
    rc_filter_pid(&D3, pid_3[0], pid_3[1], pid_3[2], 4*DT, DT);
    rc_filter_pid(&D4, pid_4[0], pid_4[1], pid_4[2], 4*DT, DT);
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
    pid_1 = [M[0],M[1],M[2]];
    pid_2 = [M[4],M[5],M[6]];
    pid_3 = [M[8],M[9],M[10]];
    pid_4 = [M[12],M[13],M[14]];
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
    return 0;
}
