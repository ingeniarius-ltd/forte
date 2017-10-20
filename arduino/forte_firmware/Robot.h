/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Ingeniarius,Ltd.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the company Ingeniarius, Ltd nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Version: 1.2
* Last change: 20/10/2017
* Author: Ingeniarius, Ltd
*********************************************************************/
/*
                 LEFT
   
    -     |##2##|       |##4##|        
F   :      H ################        B
R   L1     E ################        A    
O   :      A ################        C    
N   :      D ################        K
T   -     |##1##|       |##3##|
           
                 RIGHT
           :----- L2 -----:
         
   L1= AXIS_LENGTH_L1
   L2= AXIS_LENGTH_L2


*********************************************************************/

#include "Arduino.h"
#include <AdafruitNeoPixel.h>
#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>

/* Robot firmware version */
#define ROBOT_FIRMWARE_H_VERSION 1
#define ROBOT_FIRMWARE_L_VERSION 2

#define STARTUP_MODE true   // (true) it will start in ROS mode streaming ON | (false) it will start in Debug mode streaming OFF
#define SONARS_ENABLE false  // Enable or disable sonars

/***************************************************************
*                       !!Attention!!!                         *
****************************************************************/ 
/* Protections */
#define CMD_VEL_TIMEOUT 1000  // ms
#define MAX_LINEAR_SPEED 0.50 // m/s
#define MAX_ANGULAR_SPEED 1.57079 // rad/s  approx. 90 deg/s
#define STOP_BUTTON_PID_TIMEOUT 5000 // m/s  Timeout to reset PID errors gain when stop button is activated

/***************************************************************
*                       !!Attention!!!                         *
****************************************************************/ 
#define MAX_TORQUE_MOTORS 15   // MAX RANGE 0 - 30  
#define MAX_THRESHOLD_MOTORS (90 + MAX_TORQUE_MOTORS)
#define MIN_THRESHOLD_MOTORS (90 - MAX_TORQUE_MOTORS)

/* PID Gains controller */
#define Kp 0.1 //0.7
#define Kd 0.07 //1.0
#define Ki 0.002 //0.0007

/* ROS */
#define ROS_PUB_FREQ 10 // Hz
#define ROS_PUB_FREQ_MS (1000/ROS_PUB_FREQ)  // 1000ms / freq(Hz)

/* Sonars */
#define SONAR_SRF02_DEADZONE_DIST 15 // cm
#define SONAR_SRF02_MAX_DIST 600 // cm
#define NUMBER_OF_SONARS 16

/* General Purpose */
#define BAUD_RATE 115200

/* Robot phisical parameters */
#define WHEEL_DIAMETER 0.2232  // m
#define AXIS_LENGTH_L1 0.495   // m
#define AXIS_LENGTH_L2 0.495   // m

/* Encoders */
#define PULSES_PER_ROTATION 12750
#define PULSES_PER_METER 19982 //( PULSES_PER_ROTATION / (3.14159*WHEEL_DIAMETER))

/* Motors Pins */
// create servo objects to control the four motors (R - right; L - left; F - front; B - back)
#define MOTOR_RF_PIN 8
#define MOTOR_RB_PIN 9
#define MOTOR_LF_PIN 10
#define MOTOR_LB_PIN 11

/* Encoders Pins */
#define ENCODER_LB_A_PIN 19
#define ENCODER_LB_B_PIN 16
#define ENCODER_RF_A_PIN 2
#define ENCODER_RF_B_PIN 4
#define ENCODER_RB_A_PIN 3
#define ENCODER_RB_B_PIN 5
#define ENCODER_LF_A_PIN 18
#define ENCODER_LF_B_PIN 17

/* RGB Leds pin */
#define LEDS_RGB_NEOPIXEL_PIN 6

/* Bricket Buzzer pin */
#define BUZZ_PIN 7

/* Stop button Pin */
#define EMERGENCY_STOP_PIN  13


class Robot
{
  public:
  
    /* Robot Setup */
    void robotSetup(void);
    
    /* Robot Kinematics */
    void mecanumcinematics(float linearx, float lineary, float angularz, float velocities[]);
    void Pcart(float vel0, float vel1, float vel2, float vel3, float velocidades[]);
    boolean odometry_calculation(float &pose_x, float &pose_y, float &pose_w, float velocities[]);
    
    /* Encoders */
    void encoders_update(long int &encoderRF,long int &encoderRB, long int &encoderLF, long int &encoderLB);
    void encoders_reset(void);
    
    /* Motors */
    void move_PID_odometry(float linearx, float lineary, float angularz, int pwm_vel_motors[], float &pose_x, float &pose_y, float &pose_w, float velocities[]);
    void move_noPID(int RF, int RB, int LF, int LB);
    
    /* Sonars */
    boolean readAllSonar_nonBlocking(uint16_t sonars_array[]);
    uint16_t readSonar(int index);
    void request_sonarRead(int index);
    uint16_t requested_sonarRead(int index);
    
    /* RGB Leds */
    void statusLED(int R, int G, int B, int R2, int G2, int B2);
    
    /* General purpose */
    void resetTimers(void);
    int remap_vel(float vel);    
    
  private:
  
    /* General purpose */
    int sonarIN = 0;                // SonarID inc counter
    long int encRF_old = 0, encRB_old = 0, encLF_old = 0, encLB_old = 0;
    long int encRF_old_odm = 0, encRB_old_odm = 0, encLF_old_odm = 0, encLB_old_odm = 0;
    const int sonarID[16] = {112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127};
    
    /* Timing */
    unsigned long int last_vel_update=0;
    unsigned long int sonarSync = 0;
    long int dif_T=millis();        // PID
    
    /* PIDs erros */
    float EpRF = 0.0, EpRF_old = 0.0, EdRF = 0.0, EiRF = 0.0;    // RF - Right Front motor controller error
    float EpRB = 0.0, EpRB_old = 0.0, EdRB = 0.0, EiRB = 0.0;    // RB - Right Back motor controller error
    float EpLF = 0.0, EpLF_old = 0.0, EdLF = 0.0, EiLF = 0.0;    // RF - Right Front motor controller error
    float EpLB = 0.0, EpLB_old = 0.0, EdLB = 0.0, EiLB = 0.0;    // LB - Left Front motor controller error
};


