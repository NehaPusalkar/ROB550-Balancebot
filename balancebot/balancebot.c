/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>


#include "balancebot.h"


int turn_flag=0;
int num_setpoints=0;
int stop_flag=0;
int num_circle=0;
/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);

	
	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);
       // calculate the loop time
        mb_state.angle=0;
        mb_state.turn_angle=0;
        mb_state.heading=0;
        mb_state.start_time = (double)(rc_nanos_since_epoch())*1.0E-9;
        mb_state.time_elapse = 0.0;                      // Current loop time    
        mb_state.prevtime = 0.0;                         // Prev loop time, needed to get dt

	printf("attaching imu interupt...\n");
        mb_state.left_encoder = 0;
	mb_state.right_encoder = 0;
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
        FILE* f1 = fopen("Odometry_sqr2.csv","w");
	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		fprintf(f1,"%7.3f %7.3f %7.3f \n", mb_odometry.x,mb_odometry.y,mb_odometry.psi);
		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9/SAMPLE_RATE_HZ);
	}
	
	// exit cleanly
        mb_controller_cleanup();
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
        mb_state.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];
	// Read Previous encoders
	mb_state.pre_left_encoder = mb_state.left_encoder;
  	mb_state.pre_right_encoder = mb_state.right_encoder;
	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(2);
	mb_state.right_encoder = rc_encoder_eqep_read(1);

	// Collect wheel angle
	mb_state.left_angle = (mb_state.left_encoder*2.0*M_PI)/(ENC_1_POL*GEAR_RATIO*ENCODER_RES);
	mb_state.right_angle = (mb_state.right_encoder*2.0*M_PI)/(ENC_2_POL*GEAR_RATIO*ENCODER_RES);
        mb_state.phi = (mb_state.left_angle + mb_state.right_angle) / 2;
        

        //Calculate speed
        double current_time = (double)(rc_nanos_since_epoch())*1.0E-9;
        mb_state.time_elapse = current_time - mb_state.start_time;
        mb_state.small_dt = mb_state.time_elapse - mb_state.prevtime;
        mb_state.prevtime = mb_state.time_elapse;
        mb_state.left_speed = ENC_2_POL*(mb_state.left_encoder-mb_state.pre_left_encoder) * (2.0*M_PI) / (GEAR_RATIO * ENCODER_RES) / mb_state.small_dt;
        mb_state.right_speed = ENC_1_POL*(mb_state.right_encoder-mb_state.pre_right_encoder) * (2.0*M_PI) / (GEAR_RATIO * ENCODER_RES) / mb_state.small_dt;
        double avg_velocity=(mb_state.left_speed+mb_state.right_speed)/2;
        
    // Update odometry
     mb_odometry_update(&mb_odometry, &mb_state); 


    // Calculate controller outputs
 
    if(!mb_setpoints.manual_ctl){
    	//send motor commands
		mb_state.angle= mb_state.angle+0.03;
		if(mb_odometry.x <= 0.9&&num_setpoints==0){
			mb_state.turn_angle=0+num_circle*2*M_PI;
		}
		if(mb_odometry.x >= 0.9&&num_setpoints==0){
			mb_state.turn_angle=M_PI/2+num_circle*2*M_PI;
			num_setpoints++;
		}

		if(num_setpoints==1&&mb_odometry.y>=0.9){
			mb_state.turn_angle=M_PI+num_circle*2*M_PI;
			num_setpoints++;
		}

		if(mb_odometry.x <= 0.05&&num_setpoints==2&&!turn_flag){
			mb_state.turn_angle=M_PI*1.5+num_circle*2*M_PI;
			turn_flag=1;
			num_setpoints++;
		}

		if(num_setpoints==3&&mb_odometry.y<=0.05&&turn_flag){
			mb_state.turn_angle=M_PI*2+num_circle*2*M_PI;
			turn_flag=0;
			num_setpoints=0;
			num_circle=num_circle+1;
		}
      mb_controller_update(&mb_state);
	  ////start of 11 m 
      /*mb_state.turn_angle=0;
      if(stop_flag==0){
      mb_state.angle=mb_state.angle+0.2;
      }
	  if(mb_odometry.x>6&&stop_flag==0){
		  mb_state.angle=mb_state.angle+0.001;
	  }
      if(mb_odometry.x>11.5&&stop_flag==0);
      stop_flag=1;
      }
	  mb_controller_update(&mb_state);*/
     ///Start of open loop controller that sorta worked

	 //double corner_x[4]={0.984,0.995,0.0,0.054};
	 //double corner_y[4]={-0.021,0.961,0.946,-0.039};
	 /*double corner_x[4]={1, 1, 0, 0};
	 double corner_y[4]={0, 1, 1, 0};
	 double corner_angle[4]={M_PI/2, M_PI, M_PI*1.5, M_PI*2};

		double t_x=corner_x[num_setpoints];
		double t_y=corner_y[num_setpoints];
		double t_angle=corner_angle[num_setpoints]+num_circle*2*M_PI;
	 
		if(!turn_flag){
		 mb_state.angle= mb_state.angle+0.03;
	    }

		if((pow(mb_odometry.x-t_x,2)+pow(mb_odometry.y-t_y,2)) <= 0.01 && !turn_flag){
			printf("Setpoint: %d \n", num_setpoints);
			turn_flag =1;
			double deltax = corner_x[num_setpoints+1]-mb_odometry.x;
			double deltay = corner_y[num_setpoints+1]-mb_odometry.y;
			if(num_setpoints==0){
				mb_state.turn_angle= atan2(deltay, deltax)+mb_odometry.psi;
				//mb_state.turn_angle = M_PI/2+num_circle*2*M_PI;
			}
			if(num_setpoints==1){
				mb_state.turn_angle= -atan2(deltax, deltay)+mb_odometry.psi;
				//mb_state.turn_angle = M_PI+num_circle*2*M_PI;
			}
			if(num_setpoints==2){
				mb_state.turn_angle= -atan2(deltay, deltax)+mb_odometry.psi;
				//mb_state.turn_angle = 1.5*M_PI+num_circle*2*M_PI;
			}
			if(num_setpoints==3){
				deltax = corner_x[0]-mb_odometry.x;
				deltay = corner_y[0]-mb_odometry.y;
				mb_state.turn_angle= atan2(deltax, deltay)+mb_odometry.psi;
				
				//mb_state.turn_angle = 2*M_PI+num_circle*2*M_PI;
			}

			printf("turn_angle: %f \n",mb_state.turn_angle);
        }

		if(fabs(mb_odometry.psi-t_angle)<0.05){
			turn_flag=0;
			num_setpoints=num_setpoints+1;
			if(num_setpoints==4){num_setpoints=0;num_circle=num_circle+1;}
	    }	
		mb_controller_update(&mb_state);*/
		
	  
    }

    if(mb_setpoints.manual_ctl){
    	//send motor commands
     mb_state.angle=mb_state.angle+mb_setpoints.fwd_velocity*DT * FWD_VEL_SENSITIVITY;
     mb_state.turn_angle=mb_state.turn_angle+mb_setpoints.turn_velocity*DT * TURN_VEL_SENSITIVITY;
     mb_controller_update(&mb_state);
   	}
	
	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	//printf("X = %f, Y = %f, psi = %f \n", mb_odometry.x, mb_odometry.y, mb_odometry.psi);
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	while(1){

		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.
                  mb_setpoints.fwd_velocity = rc_dsm_ch_normalized(3);
                  mb_setpoints.turn_velocity = rc_dsm_ch_normalized(4);
                  mb_setpoints.manual_ctl = rc_dsm_ch_normalized(7);
                  
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;

		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			/*printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);*/
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.psi);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
