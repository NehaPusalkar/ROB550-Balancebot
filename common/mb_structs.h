#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   yaw;
    float   phi;               // average wheel angle (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    float   left_angle;
    float   right_angle;

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needede
    int pre_left_encoder;
    int pre_right_encoder;
    double start_time;
    double time_elapse;                      // Current loop time    
    double prevtime;                         // Prev loop time, needed to get dt
    double small_dt;                         //time between two mian loop
    double angle;                            //reference wheel angle
    double left_speed;
    double right_speed;
    double turn_angle;
    double heading;
    double curr_tb;
    double prev_tb;
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
};

#endif
