/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include "../common/mb_defs.h"



/*******************************************************************************
* int main() 
*
*******************************************************************************/
rc_mpu_data_t data;


//Callback function
void print_data(void){
        rc_mpu_read_accel(&data);
        rc_mpu_read_gyro(&data);
	printf("accel:%6.2f %6.2f %6.2f |", data.accel[0], data.accel[1], data.accel[2]);
	printf("gyro: %6.1f %6.1f %6.1f |", data.gyro[0], data.gyro[1], data.gyro[2]);
	printf("taitByran:%6.1f %6.1f %6.1f \n", data.dmp_TaitBryan[TB_PITCH_X],data.dmp_TaitBryan[TB_ROLL_Y],
		data.dmp_TaitBryan[TB_YAW_Z]);
        
}

int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


	FILE* f1 = fopen("InertialDataPitch4_0.csv","w");
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.dmp_sample_rate = SAMPLE_RATE_HZ;
    conf.orient = ORIENTATION_Z_DOWN;

    rc_mpu_initialize(&data, conf);
    rc_mpu_initialize_dmp(&data, conf);
    	
    rc_set_state(RUNNING);
    rc_mpu_set_dmp_callback(&print_data);
    while(rc_get_state()!=EXITING){

	fprintf(f1, "%6.2f %6.2f %6.2f", data.accel[0], data.accel[1], data.accel[2]);
	fprintf(f1, "%6.1f %6.1f %6.1f", data.gyro[0], data.gyro[1], data.gyro[2]);
	fprintf(f1, "%6.1f %6.1f %6.1f \n", data.dmp_TaitBryan[TB_PITCH_X],data.dmp_TaitBryan[TB_ROLL_Y],
		data.dmp_TaitBryan[TB_YAW_Z]);
	
    	rc_nanosleep(1E9/SAMPLE_RATE_HZ);
    }
	
	// exit cleanly
	fclose(f1);
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
