#pragma once 

/**
 * TODO: Implement your FSM here
 * 
 * This file implements a very basic FSM that initially lets the robot go forward
 * and stops it when an obstacle is detected.
 * 
 * You can modify this file to implement your own FSM and implement your own behaviors.
*/

#include "pioneer_interface/pioneer_interface.hpp" // contains the NUM_SENSORS constant 
#include "braitenberg.hpp" // you may want to use the braitenberg function to control the robot in some cases

#define THRESHOLD   1010  // Arbitrary threshold value for the proximity sensors
#define FRONT_CLEAR_THRESHOLD 800
#define SIDE_CLEAR_THRESHOLD 500
#define TRANSITION_DURATION 150 // nb de steps à tourner
#define MAX_TRAVERSALS 4


//////////////////////
// Global variables //
//////////////////////

// enum used to store the current behavior
enum basicBehaviors {GOFORWARD,TURN_LEFT,TURN_RIGHT, STOP} behavior;

int turn_counter = 0;
bool turn_left_next = true;
int traversals_done = 0;
////////////////////////
// Behavior Functions //
////////////////////////

void goForwardBehavior(double* ps_values, double &vel_left, double &vel_right){
  braitenberg(ps_values,vel_left,vel_right);
  return;
}

void stopBehavior(double &vel_left, double &vel_right){
  vel_left = 0;
  vel_right = 0;
  return;
}
void turnLeftBehavior(double &vel_left, double &vel_right){
  vel_left = 1.5;
  vel_right = 5.0;
  return;
}

void turnRightBehavior(double &vel_left, double &vel_right){
  vel_left = 5.0;
  vel_right = 1.5;
  return;
}
///////////////////////
// Main FSM function //
///////////////////////

/**
 * @brief Finite State Machine that manages the robot's behavior
 * @param ps_values array of proximity values from the robot's proximity sensors
 * @param[out] vel_lef left wheels velocity
 * @param[out] vel_right right wheels velocity
*/
void fsm(double* ps_values, double &vel_left, double &vel_right){

  switch(behavior){

      case GOFORWARD:
        
        // Perform the goForward behavior
        goForwardBehavior(ps_values, vel_left, vel_right);
        // Check for transition criteria 
        
        if (ps_values[0] < SIDE_CLEAR_THRESHOLD && ps_values[15] < SIDE_CLEAR_THRESHOLD &&
            ps_values[1] < FRONT_CLEAR_THRESHOLD) {
            traversals_done++;
            if (traversals_done >= MAX_TRAVERSALS) {
            behavior = STOP;
            break;
          }
          turn_counter = TRANSITION_DURATION;
          behavior = turn_left_next ? TURN_LEFT : TURN_RIGHT;
          turn_left_next = !turn_left_next;
          
         }
        break;
      case TURN_LEFT:
        turnLeftBehavior(vel_left, vel_right);
        if (--turn_counter <= 0) {
          behavior = GOFORWARD;
        }
        break;
      case TURN_RIGHT:
          turnRightBehavior(vel_left, vel_right);
        if (--turn_counter <= 0) {
          behavior = GOFORWARD;
        }
        break;
        
      case STOP:
        
        // Perform the stop behavior
        stopBehavior(vel_left, vel_right);

        // No transition criteria, the robot stays in this state forever
        break;

      default:
        printf("This behavior is not implemented.\n");
        break;
    }
    return;
}