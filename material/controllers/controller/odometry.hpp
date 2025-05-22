#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees
#define VERBOSE_ODO_ACC true 
#define VERBOSE_ODO_ENC true
#define VERBOSE_DEBUG_ODO_ENC false



typedef struct 
{
  double x=0;
  double y=0;
  double heading=0;
} pose_t;


static double _T;
static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
void odo_compute_heading(pose_t* odo, const double acc[6], const double acc_mean[6], double delta_t);
// TODO: You can implement your wheel odometry here if relevant for your project

void odo_reset(int time_step) {
    _odo_pose_enc.x = 0;
    _odo_pose_enc.y = 0;
    _odo_pose_enc.heading = 0;  // Explicitly initialize to 0
    _T = time_step / 1000.0;
    printf("Reset: heading=%f\n", _odo_pose_enc.heading);  // Should print 0.000
}
/**
 * @brief      Compute the odometry using the encoders. Use the motion model proposed in bonus question
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc, const double acc[6], const double acc_mean[6],double delta_t)
{
	printf("Wheel radius: %f, Axis length: %f\n",
	pioneer_info.wheel_radius, pioneer_info.axis_length);
	if (pioneer_info.axis_length <= 0.0 || _T <= 0.0) {
    	printf("ERROR: Invalid axis_length or dt!\n");
    return;  // Skip calculation
}
	_T=delta_t;
	// Rad to meter
	Aleft_enc  *= pioneer_info.wheel_radius;

	Aright_enc *= pioneer_info.wheel_radius;
	
	if (isnan(Aleft_enc) || isnan(Aright_enc) || isnan(delta_t)) {
        printf("ERROR: NaN in inputs!\n");
        return;
    }

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );
	/*
	// Compute forward speed and angular speed
	double omega = ( Aright_enc - Aleft_enc ) / ( pioneer_info.axis_length * _T );
	if (isnan(omega)) {
        printf("ERROR: omega is NaN!\n");
        omega = 0.0;
    }
	
	// After computing speed and omega:
	if(VERBOSE_DEBUG_ODO_ENC){
	printf("Before heading update: omega=%f, _T=%f\n", omega, _T);
	// Apply rotation (Body to World)
	}
	// smaller integration step for the angle (1/2)
	_odo_pose_enc.heading += omega * _T / 2.0;
	if(VERBOSE_DEBUG_ODO_ENC){
	printf("Mid-heading: %f\n", _odo_pose_enc.heading);  // Debug
	}
	double a = _odo_pose_enc.heading;
	if (isnan(a)) {
    printf("ERROR: heading is NaN!\n");
    a = 0.0;  // Reset to avoid corruption
	}*/
	odo_compute_heading(odo,acc,acc_mean,delta_t);
	
	double a = odo->heading;
	
	double speed_wx = speed * cos(a);

	double speed_wy = speed * sin(a);

	// Integration : Euler method
	_odo_pose_enc.x += speed_wx * _T;

	_odo_pose_enc.y += speed_wy * _T;
	if(VERBOSE_DEBUG_ODO_ENC){
	printf("x=%f, y=%f\n", _odo_pose_enc.x, _odo_pose_enc.y);
	}
	// smaller integration step for the angle (2/2)
	/*_odo_pose_enc.heading += omega * _T / 2.0;
	if(VERBOSE_DEBUG_ODO_ENC){
	printf("Final-heading: %f\n", _odo_pose_enc.heading);  // Debug
	}
    // Validate final heading
    if (isnan(_odo_pose_enc.heading)) {
        printf("ERROR: heading became NaN!\n");
        _odo_pose_enc.heading = 0.0;  // Reset
    }
	*/
	_odo_pose_enc.heading=odo->heading;
	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	if(VERBOSE_ODO_ENC)
    	printf("ODO with wheel encoders using half-step: %g %g \n", odo->x , odo->y );
}
//Odometry based on the accelerometer
void odo_compute_heading(pose_t* odo, const double acc[6], const double acc_mean[6], double delta_t)  // Add delta_t
{
    // 1. Update time step
    _T = delta_t;
    if (_T <= 0.0) {
        printf("ERROR: Invalid delta_t in acc odometry!\n");
        return;
    }

    // 2. Remove bias and rotate acceleration to world frame if needed
    double acc_wo_bias[6];
    for(int i = 0; i < 6; i++) {
        acc_wo_bias[i] = acc[i] - acc_mean[i];
    }

    // 3. Integrate angular velocity (gyro z-axis) for heading
    double angular_velocity = acc_wo_bias[5];  // Assuming acc_wo_bias[5] is gyro-z
    _odo_pose_acc.heading += angular_velocity * _T;
    
    // Optional: Normalize heading to [-π, π]
    _odo_pose_acc.heading = atan2(sin(_odo_pose_acc.heading), cos(_odo_pose_acc.heading));
	/*
    // 4. Rotate linear acceleration to world frame using current heading
    double a = _odo_pose_acc.heading;
	
    double acc_world_x = acc_wo_bias[0] * cos(a) - acc_wo_bias[1] * sin(a);  // Body to world
    double acc_world_y = acc_wo_bias[0] * sin(a) + acc_wo_bias[1] * cos(a);

    // 5. Integrate acceleration to velocity (with damping to reduce drift)
    _odo_speed_acc.x = _odo_speed_acc.x + acc_world_x * _T;  // Damping factor
    _odo_speed_acc.y = _odo_speed_acc.y + acc_world_y * _T;

    // 6. Integrate velocity to position
    _odo_pose_acc.x += _odo_speed_acc.x * _T;
    _odo_pose_acc.y += _odo_speed_acc.y * _T;
	*/
    // 7. Copy results
	_odo_pose_acc.x=odo->x;
	_odo_pose_acc.y=odo->y;
    memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
    
    if(VERBOSE_ODO_ACC) {
        printf("Acc ODO: x=%.3f y=%.3f heading=%.3f°\n", 
               odo->x, odo->y, RAD2DEG(odo->heading));
    }
}