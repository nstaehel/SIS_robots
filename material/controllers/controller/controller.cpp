// Controller for the robot robot

// Provided libraries 
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions  
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"
#include <string.h>
#include <stdio.h>
#define TIME_INIT_ACC 15
#define VERBOSE_ACC_MEAN true
#define VERBOSE_ACC true

typedef struct 
{
  double acc_mean[6]={0,0,0,0,0,0};
  double acc[6];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
  double dt;
} measurement_t;


static void controller_compute_mean_acc(Pioneer& robot);
static void controller_get_acc(Pioneer& robot);
static pose_t _pose_result={0,0,0};
static pose_t _pose_result_acc={0,0,0};
static measurement_t _meas;



int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  odo_reset(robot.get_timestep());
  controller_get_acc(robot);
  
  double time_prev=robot.get_time();
  _meas.prev_left_enc=robot.get_encoders()[0];
  _meas.prev_right_enc=robot.get_encoders()[1];

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!
  std::string f_odometry = "odometry.csv";
  int         f_odometry_cols = init_csv(f_odometry, "time, posx, posy, heading, posx_acc, posy_acc, heading_acc,");
  std::string f_temp = "temperature.csv";     
  int         f_temp_cols = init_csv(f_temp, "time,sensor_id, x_location, y_location, in_temp, out_temp,");  
  
  while (robot.step() != -1) {
    double time_next=robot.get_time();
    _meas.dt = time_next-time_prev;  
    controller_get_acc(robot);
    if(robot.get_time()<TIME_INIT_ACC){
      controller_compute_mean_acc(robot);

    }
    else{
    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////

    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    _meas.left_enc=wheel_rot[0];
    _meas.right_enc=wheel_rot[1];

    double increment_rad_left=_meas.left_enc-_meas.prev_left_enc;
    double increment_rad_right=_meas.right_enc-_meas.prev_right_enc;
    // DEBUG: Print encoder values and increments
    printf("Encoders: prev_left=%f, left=%f, prev_right=%f, right=%f\n",
       _meas.prev_left_enc, _meas.left_enc, _meas.prev_right_enc, _meas.right_enc);
    printf("Increments: left=%f, right=%f, dt=%f\n",
       increment_rad_left, increment_rad_right, _meas.dt);

    odo_compute_encoders(&_pose_result, increment_rad_left, increment_rad_right, imu,_meas.acc_mean,_meas.dt);
  
    

    ////////////////////
    // Implementation //
    ////////////////////

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);
    if (signal_strength > 0.0){
      log_csv(f_temp,f_temp_cols,time,data[0],data[1],data[2],data[3],data[4]);
      }
      
    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities

    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);
    log_csv(f_odometry, f_odometry_cols, time, _pose_result.x, _pose_result.y, _pose_result.heading,_pose_result_acc.x,_pose_result_acc.y,_pose_result_acc.heading);

    _meas.prev_left_enc=_meas.left_enc;
    _meas.prev_right_enc=_meas.right_enc;
    
    }
    time_prev=time_next;
    time_next=robot.get_time();
  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}


void controller_get_acc(Pioneer& robot)
{
  // Call the function to get the accelerometer measurements. Uncomment and complete the following line. Note : Use _robot.acc
  const double * acc_values = robot.get_imu();

  // Copy the acc_values into the measurment structure (use memcpy)
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if(VERBOSE_ACC)
    printf("ROBOT acc : [x=%g y=%g z=%g wx=%g wy=%g wz=%g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2], _meas.acc[3], _meas.acc[4], _meas.acc[5]);
}



void controller_compute_mean_acc(Pioneer& robot)
{
  static int count = 0;
  
  count++;
  
  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 6; i++)  
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 1) + _meas.acc[i]) / (double) count;
  }
  
  if( count == (int) (TIME_INIT_ACC / (double) robot.get_timestep() * 1000) )
    printf("Accelerometer initialization Done ! \n");

  if(VERBOSE_ACC_MEAN)
        printf("ROBOT acc mean : %g %g %g %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2], _meas.acc_mean[3], _meas.acc_mean[4], _meas.acc_mean[5]);
}



