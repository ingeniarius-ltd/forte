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
 * Version: 1.3
 * Last change: 20/10/2017
 * Author: Ingeniarius, Ltd
 *********************************************************************/

/************************************************************************
 *
 * Load libraries
 *
 *************************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <AdafruitNeoPixel.h>
#include "RobotSerialComm.h"
#include "Robot.h"
#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>

/***************************************************************
 *
 * Classes and objects initialization
 *
 ***************************************************************/
Robot robot;              // Robot initialization
RobotSerialComm port;     // RobotSerial Communication


/***************************************************************
 *
 * Define global variables
 *
 ***************************************************************/

/* General purpose */
boolean STREAM = STARTUP_MODE;     // Streaming flag
boolean newSonar_data = false;     // Flag to confirm that exist new update sonar data
int timer4_counter;                // Counter 
uint8_t cmdVel_zero_counter = 0;

/* Timing */
unsigned long int rosSync=millis();

/* Status RGB Leds */
int statusLEDmode=0;

/* Robot shared data */
struct {
  float pose_x=0;  
  float pose_y=0;
  float pose_w=0;
  float cmd_vel_Vx=0; // Input velocities             
  float cmd_vel_Vy=0;
  float cmd_vel_Vw=0;
  float velocities[3]={0};  // Output Vx, Vy, Vw velocities
  int motor_pwm_vel[4]={0}; // Velocites pwm for each motor
} robotData;


 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************/ 
 /* Protections */   
 volatile unsigned long int cmd_vel_timeout = 0;      // Protection to timeout ROS cmd_vel
 volatile unsigned long int odometry_update = millis()+1000;      // Rate refresh PID
 volatile unsigned long int emergencyStop_loopUpdate = millis();      // Rate refresh PID
 volatile boolean ACTIVATE_TIMEOUT = false;
 volatile boolean HEART_BEAT = false;
 unsigned long int loopUpdate_time = millis();               // Rate refresh PID

/***************************************************************
 *
 * Prototypes Functions
 *
 ***************************************************************/
void enable_timeout_motors(void);                                 //Enable motors timeout protection
void messageCb (const geometry_msgs::Twist &incoming_msg);        // ROS cmd_vel callback
void messageRGBleds_Cb (const std_msgs::UInt8MultiArray &msg);    // ROS RGB_leds callback
void stopmotors(void);


/***************************************************************
 *
 * ROS Classes, objects and global variables initialization
 *
 ***************************************************************/
ros::NodeHandle  nh;


/* Odometry TF */
char base_link_frame[] = "/base_link";
char odom_frame[] = "/odom";
tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped odom_trans;


/* Odometry */
nav_msgs::Odometry odom;

/* Sonars */
std_msgs::UInt16MultiArray data_sonars;

/* cmd_vel */
geometry_msgs::Twist message_cmd_vel;

/* Subscribers */
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );                        //subscribe rostopic cmd_vel
ros::Subscriber<std_msgs::UInt8MultiArray> sub_RGBleds("RGB_leds", &messageRGBleds_Cb );  //subscribe rostopic RGB_leds

/* Publishers */
ros::Publisher odometry("odom", &odom);
ros::Publisher sonars("sonars", &data_sonars);

/************************************************************************
 *
 * Function:  Initial setup.
 * Objective: Open Serial ports
 *            Open I2C
 *            advertise/subscribe topics
 * Issues:    None to report so far.
 *
 *************************************************************************/
void setup(){


  // Robot initial setup
  robot.robotSetup();
   
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  broadcaster.init(nh);

  //ROS topics
  nh.subscribe(sub);
  nh.subscribe(sub_RGBleds);
  #if SONARS_ENABLE
  nh.advertise(sonars);

  // Sonars initialization
  data_sonars.data_length = NUMBER_OF_SONARS;  //sonar array length
  data_sonars.data = (uint16_t *)malloc(sizeof(uint16_t)*NUMBER_OF_SONARS);
  #endif 
  

 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************/ 
  // Timer interrupt for ROS timeout cmd_vel protection
  
  // initialize timer4 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;               // set entire TCCR3A register to 0
  TCCR4B = 0;               // same for TCCR3B

  // Set timer1_counter to the correct value for our interrupt interval
  timer4_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  
  TCNT4 = timer4_counter;   // preload timer
  TCCR4B |= (1 << CS42);    // 256 prescaler 
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

}


/************************************************************************
 *
 * Function:  Main loop.
 * Objective: 
 * Issues:    None to report so far.
 *
 *************************************************************************/


void loop() {

  if (digitalRead(EMERGENCY_STOP_PIN)) {    //  Check if stop button is active
  
   
      if(millis()-rosSync > ROS_PUB_FREQ_MS){
        
        rosSync = millis();  // update rosSync time 
        
        status_mode_RGBleds(3);
        
        ////////////////////////////////////////////////////////  PUB ODOM TF
        ros::Time current_time = nh.now();
        // Publish the transform over tf
        odom_trans.header.frame_id = odom_frame;
        odom_trans.child_frame_id = base_link_frame;
        odom_trans.header.stamp = current_time;

        odom_trans.transform.translation.x = robotData.pose_x;
        odom_trans.transform.translation.y = robotData.pose_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionFromYaw(robotData.pose_w);

       broadcaster.sendTransform(odom_trans);     // odom->base_link transform
       nh.spinOnce();
        
        ////////////////////////////////////////////////////////  PUB Sonars
        #if SONARS_ENABLE
        if(newSonar_data){  // After update all 16 sonars readings, publish it
          sonars.publish( &data_sonars);
          newSonar_data=false;     //Reset flag of new sonar data
        }
        #endif

      }  // END OF IF rosSync
      
      
      #if SONARS_ENABLE
      
      newSonar_data = robot.readAllSonar_nonBlocking( data_sonars.data );  /// Update sonar values and verify if all array is ready to be published
      
      #endif 

    
    
  }  // END OF IF EMERGENCY_STOP

  else{  //stop button pushed

    stopmotors();
    status_mode_RGBleds(1);


      if(millis()-rosSync > ROS_PUB_FREQ_MS){        
        rosSync = millis();  // update rosSync time

        nh.logwarn("Emergency button is pushed! Unlock the button to move the motor.");
        
        ////////////////////////////////////////////////////////  PUB ODOM TF
        ros::Time current_time = nh.now();
        // Publish the transform over tf
        odom_trans.header.frame_id = odom_frame;
        odom_trans.child_frame_id = base_link_frame;
        odom_trans.header.stamp = current_time;

        odom_trans.transform.translation.x = robotData.pose_x;
        odom_trans.transform.translation.y = robotData.pose_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionFromYaw(robotData.pose_w);

       // Send the transform and odom
       broadcaster.sendTransform(odom_trans);     // odom->base_link transform
       nh.spinOnce();
      }
 
  }
  
  if(millis()-loopUpdate_time > 10){       // Update every 10ms
   loopUpdate_time = millis();
   emergencyStop_loopUpdate  = millis();   // Update time to security routine, if the loop is blocking, timeout 200ms
   HEART_BEAT = true;                      // Flag to confirm if the loop is blocking, timeout 200ms
  }   
}

/************************************************************************
 *
 * Function:  enable_timeout_motors
 * Objective: Enable motors timeout protection
 * Issues:    None to report so far.
 *
 *************************************************************************/
void enable_timeout_motors(){
  cmd_vel_timeout = millis();
  ACTIVATE_TIMEOUT = true;   // Activate timeout protection flag
}

/************************************************************************
 *
 * Function:  messageCb
 * Objective: callback function to cmd_vel topic, update the velocities to send to motors 
 * Issues:    None to report so far.
 *
 *************************************************************************/
void messageCb ( const geometry_msgs::Twist &incoming_msg ){
 
 robotData.cmd_vel_Vx = incoming_msg.linear.x; 
 robotData.cmd_vel_Vy = incoming_msg.linear.y;
 robotData.cmd_vel_Vw = incoming_msg.angular.z;
  
 enable_timeout_motors();
 
 if(robotData.cmd_vel_Vx==0.0 && robotData.cmd_vel_Vy==0.0 && robotData.cmd_vel_Vw==0.0){
   if(cmdVel_zero_counter >= 50){  // 10hz -> 50= 5seconds
     cmdVel_zero_counter = 0;
     stopmotors();
   }
   cmdVel_zero_counter++;
 }
}

/************************************************************************
 *
 * Function:  SIGNAL
 * Objective: Interrupt is called once a millisecond, to security system to protection of timeout ROS cmd_vel
 * Issues:    None to report so far.
 *
 *************************************************************************/
 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************/ 
ISR(TIMER4_OVF_vect)        // interrupt service routine 
{
  TCNT4 = timer4_counter;   // preload timer
  
  if(millis()-odometry_update > 50){    // Odometry in arduino will be updated every 50ms
    odometry_update = millis();
    robot.move_PID_odometry(robotData.cmd_vel_Vx, robotData.cmd_vel_Vy, robotData.cmd_vel_Vw,robotData.motor_pwm_vel,robotData.pose_x, robotData.pose_y, robotData.pose_w, robotData.velocities);
  }

  if(ACTIVATE_TIMEOUT && ( millis()-cmd_vel_timeout > CMD_VEL_TIMEOUT )){
     ACTIVATE_TIMEOUT=false;
     stopmotors();
     nh.logwarn("CMD_VEL timeout! I will stop the motors for safe issues.");
  }
  
  if(millis()- emergencyStop_loopUpdate > 200 && HEART_BEAT){
    HEART_BEAT = false;
    robot.move_noPID(90,90,90,90);
    nh.logwarn("Loop Stop emergency");
  }

}

/************************************************************************
 *
 * Function:  messageRGBleds_Cb
 * Objective: callback function to RGB_leds topic
 * Issues:    None to report so far.
 *
 *************************************************************************/
void messageRGBleds_Cb  ( const std_msgs::UInt8MultiArray &msg ){
  robot.statusLED(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
}


/************************************************************************
 *
 * Function:  Initialize motors (stop position).
 * Objective: Reset encoders and activate the motors
 * Issues:    None to report so far.
 *
 *************************************************************************/
void stopmotors() {
 robotData.cmd_vel_Vx = 0; 
 robotData.cmd_vel_Vy = 0;
 robotData.cmd_vel_Vw = 0;
 robot.resetTimers();
}


/************************************************************************
 *
 * Function:  messageRGBleds_Cb
 * Objective: callback function to RGB_leds topic
 * Issues:    None to report so far.
 *
 *************************************************************************/
void status_mode_RGBleds (int mode){
  
  if(mode != statusLEDmode){
    statusLEDmode=mode;
    
    if(mode==1){                         // If Stop button pushed
      robot.statusLED(255,0,0,255,0,0);  // RED
    } else if (mode==2) {                // If in Debug mode
      robot.statusLED(0,0,120,0,0,120);  // BLUE
    } else if (mode==3) {                // If in ROS mode
      robot.statusLED(0,120,0,0,120,0);  // GREEN
    }
  }
  
}



