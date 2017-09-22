/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Ingeniarius,Ltd.
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
 * Last change: 22/09/2017
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
int timer5_counter;                // Counter 
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
void streamMode_ON_OFF(void);
void pingpong_debug(void);
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
  nh.advertise(odometry);
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
  
  // initialize timer5 
  noInterrupts();           // disable all interrupts
  TCCR5A = 0;               // set entire TCCR3A register to 0
  TCCR5B = 0;               // same for TCCR3B

  // Set timer1_counter to the correct value for our interrupt interval
  timer5_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  
  TCNT5 = timer5_counter;   // preload timer
  TCCR5B |= (1 << CS52);    // 256 prescaler 
  TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt
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
  

    if (!STREAM){     // Stream OFF - Debug mode  
   
      pingpong_debug();
      status_mode_RGBleds(2);

    } else  {            // Stream ON - ROS mode
    
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


        ////////////////////////////////////////////////////////  PUB ODOM
        // Publish the odometry message over ROS
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_link_frame;
        odom.header.stamp = current_time;

        // Pose
        odom.pose.pose.position.x = odom_trans.transform.translation.x;//robotData.pose_x;
        odom.pose.pose.position.y = odom_trans.transform.translation.y;//robotData.pose_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        
        // Set the velocities
        odom.twist.twist.linear.x = robotData.velocities[0];
        odom.twist.twist.linear.y = robotData.velocities[1];
        odom.twist.twist.angular.z = robotData.velocities[2];
  
       
       // Send the transform and odom
       odometry.publish(&odom);		                // odometry publisher
       broadcaster.sendTransform(odom_trans); 		// odom->base_link transform
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

      pingpong_debug();               // Verify if flagStream ON  - ROS mode | If flagStream OFF - Debug mode
      
    
    }  // END IF ROS MODE
    
    
  }  // END OF IF EMERGENCY_STOP

  else{  //stop button pushed

    stopmotors();
    status_mode_RGBleds(1);
    
    if (STREAM){
      if(millis()-rosSync > ROS_PUB_FREQ_MS){        
        rosSync = millis();  // update rosSync time
        
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


        ////////////////////////////////////////////////////////  PUB ODOM
        // Publish the odometry message over ROS
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_link_frame;
        odom.header.stamp = current_time;

        // Pose
        odom.pose.pose.position.x = odom_trans.transform.translation.x;//robotData.pose_x;
        odom.pose.pose.position.y = odom_trans.transform.translation.y;//robotData.pose_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        
        // Set the velocities
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
  
       
       // Send the transform and odom
       odometry.publish(&odom);		                // odometry publisher
       broadcaster.sendTransform(odom_trans); 		// odom->base_link transform
       nh.spinOnce();
      }
    }
 
  }
  
  if(millis()-loopUpdate_time > 10){       // Update every 10ms
   loopUpdate_time = millis();
   emergencyStop_loopUpdate  = millis();   // Update time to security routine, if the loop is blocking, timeout 200ms
   HEART_BEAT = true;                      // Flag to confirm if the loop is blocking, timeout 200ms
   //nh.spinOnce();                          // ROS SpinOnce
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
ISR(TIMER5_OVF_vect)        // interrupt service routine 
{
  TCNT5 = timer5_counter;   // preload timer
  
  if(millis()-odometry_update > 50){    // Odometry in arduino will be updated every 50ms
    odometry_update = millis();
    robot.move_PID_odometry(robotData.cmd_vel_Vx, robotData.cmd_vel_Vy, robotData.cmd_vel_Vw,robotData.motor_pwm_vel,robotData.pose_x, robotData.pose_y, robotData.pose_w, robotData.velocities);
  }

  if(ACTIVATE_TIMEOUT && ( millis()-cmd_vel_timeout > CMD_VEL_TIMEOUT )){
     ACTIVATE_TIMEOUT=false;
     stopmotors();
     Serial3.println("ACTIVATE_TIMEOUT");
  }
  
  if(millis()- emergencyStop_loopUpdate > 200 && HEART_BEAT){
    HEART_BEAT = false;
    robot.move_noPID(90,90,90,90);
    Serial3.println("Loop Stop emergency");
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


/************************************************************************
 *
 * Function:  pingpong_debug.
 * Objective: send command, get action
 * Issues:    None to report so far.
 *
 *************************************************************************/
void pingpong_debug() {

  if (Serial3.available() > 0) {
    
      unsigned int arg[5];
      int action = port.getMsg(arg);
      switch (action) {
    
      case ACTION_START_STREAM:             //@1e, no reply
        STREAM = true;
        break;
    
      case ACTION_STOP_STREAM:             //@2e, no reply
        STREAM = false;
        break;
    
      case ENCODER_TEST:                    //@3e, reply: @3<LFsec><RFsec><LBsec><RBsec>e
        robot.ExtDebug_test_MotorsEncoders();
        STREAM = false;
        break;
    
      case READ_ENCODERS:                    //@4e, reply: @4<LF><RF><LB><RB>e
        robot.ExtDebug_encoders_read();
        STREAM = false;
        break;
    
      case READ_SONARS_FRONT:                //@5e, reply: @5<S1><S2><S3><S4><S5>e
        robot.ExtDebug_readAllSonars(1);
        STREAM = false;
        break;
    
      case READ_SONARS_RIGHT:                //@6e, reply: @6<S5><S6><S7><S8><S9>e
        robot.ExtDebug_readAllSonars(2);
        STREAM = false;
        break;
    
      case READ_SONARS_BACK:                //@7e, reply: @7<S9><S10><S11><S12><S13>e
        robot.ExtDebug_readAllSonars(3);
        STREAM = false;
        break;
    
      case READ_SONARS_LEFT:                //@8e, reply: @8<S13><S14><S15><S16><S1>e
        robot.ExtDebug_readAllSonars(4);
        STREAM = false;
        break;
    
      case READ_SONARS_ALL:                 //@9e, reply: @9<S1><S2><S3><S4><S5><S6><S7><S8><S9><S10><S11><S12><S13><S14><S15><S16>e
        robot.ExtDebug_readAllSonars(5);
        STREAM = false;
        break;
    
      case MOVE_PID:                        //@10,Vx,Vy,Vwe, no reply
        //robot.move_PID((float)(arg[0]/100.0), (float)(arg[1]/100.0), (float)(arg[2]/100.0));
        robotData.cmd_vel_Vx = (float)(arg[0]/100.0); 
        robotData.cmd_vel_Vy = (float)(arg[1]/100.0);
        robotData.cmd_vel_Vw = (float)(arg[2]/100.0);
        enable_timeout_motors();
        STREAM = false;
        break;
    
      case MOVE_NOPID:                      //@11,RF_val,RB,_val,LF_vale,LB_val,e , no reply
        //robot.move_noPID(arg[0], arg[1], arg[2], arg[3]);
        enable_timeout_motors();
        STREAM = false;
        break;
    
      case STOP_MOTORS:                     //@12e, no reply
        stopmotors();
        STREAM = false;
        break;
    
      case ENCODERS_RESET:                  //@13e, no reply
        robot.encoders_reset();
        STREAM = false;
        break;
    
      case LED_STATE:                    //@14,R1,G1,B1,R2,G2,B2,e, no reply
        robot.statusLED(arg[0], arg[1], arg[2], arg[3], arg[4], arg[5]);
        STREAM = false;
        break;
       
      case ODOM:                        //@15,e, //@15e, reply: @15<PoseX><PoseY><PoseW><VelX><VelY><VelW>e
        {
          int pose_data[]={(int)(robotData.pose_x * 100.0), 
                         (int)(robotData.pose_y * 100.0), 
                         (int)(robotData.pose_w * 100.0), 
                         (int)(robotData.velocities[0] * 100.0),
                         (int)(robotData.velocities[1] * 100.0), 
                         (int)(robotData.velocities[1] * 100.0)};
        
          robot.ExtDebug_generic(pose_data,6);

          STREAM = false;
        }
        break; 
        
      case ODOM_RESET:                    //@16,e, no reply
        robotData.pose_x=0, robotData.pose_y=0, robotData.pose_w=0, robotData.velocities[0]=0, robotData.velocities[1]=0, robotData.velocities[1]=0;
        STREAM = false;
        break; 
    
      default:
        break;
    
      }
  }
}

