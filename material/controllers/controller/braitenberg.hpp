#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

#define RANGE 1024
#define MAX_SPEED 6.4
#define BASE_SPEED 2.0
#define CRITICAL_THRESHOLD 980

/*

Pioneer sensor layout:

                Front
                3   4
        1   2           5   6
    0                           7
Left                             Right
    15                          8
        14  13          10  9
                12  11
                 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
*/
void braitenberg(double* ps, double &vel_left, double &vel_right){
    vel_left = BASE_SPEED;
    vel_right = BASE_SPEED;
    
    double braitenberg_coeff[16][2] = {
  { 2.0,  -0.5 },  // 0 - avant gauche extrême
  { 5.0,  -3.0 },  // 1
  { 7.0,  -5.0 },  // 2
  { 9.0,  -7.0 },  // 3 - centre avant gauche

  { -7.0, 9.0 },   // 4 - centre avant droit
  { -5.0, 7.0 },   // 5
  { -3.0, 5.0 },   // 6
  { -0.5, 2.0 },   // 7 - avant droit extrême

  {  0.5, -0.5 },  // 8 - arrière droit (léger recentrage)
  {  0.8, -0.6 },  // 9
  {  1.0, -0.8 },  // 10
  {  0.5, -0.5 },  // 11

  { -0.5, 0.5 },   // 12 - arrière gauche
  { -0.8, 0.6 },   // 13
  { -1.0, 0.8 },   // 14
  { -0.5, 0.5 }    // 15
};

  for (int i = 0; i < 16; ++i) {
      double activation = ps[i]/RANGE;
        vel_left += braitenberg_coeff[i][0]/4 * activation;
        vel_right += braitenberg_coeff[i][1]/4* activation;
    }

// Clamp les vitesses
    if (vel_left > MAX_SPEED) vel_left = MAX_SPEED;
    if (vel_left < -MAX_SPEED) vel_left = -MAX_SPEED;
    if (vel_right > MAX_SPEED) vel_right = MAX_SPEED;
    if (vel_right < -MAX_SPEED) vel_right = -MAX_SPEED;


    bool obstacle_gauche = false;
    for (int i = 0; i <= 3; ++i) {
        if (ps[i] > CRITICAL_THRESHOLD) {
            obstacle_gauche = true;
            break;
        }
    }
    

    bool obstacle_droite = false;
    for (int i = 4; i <= 7; ++i) {
        if (ps[i] > CRITICAL_THRESHOLD) {
            obstacle_droite = true;
            break;
        }
    }
    

    if (obstacle_gauche && !obstacle_droite) {
        vel_right = -MAX_SPEED+1.0;
        vel_left = MAX_SPEED-1.0;
    }
    else if (obstacle_droite && !obstacle_gauche) {
        vel_left = -MAX_SPEED+1.0;
        vel_right = MAX_SPEED-1.0;
    }
    // sinon: BASE_SPEED
}
