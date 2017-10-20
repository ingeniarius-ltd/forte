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
#include "Arduino.h"
#include "Robot.h"

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2, LEDS_RGB_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); //2 Status LED 

Encoder encRF(ENCODER_RF_A_PIN,ENCODER_RF_B_PIN),   // 
        encRB(ENCODER_RB_A_PIN,ENCODER_RB_B_PIN), 
        encLF(ENCODER_LF_A_PIN,ENCODER_LF_B_PIN), 
        encLB(ENCODER_LB_A_PIN, ENCODER_LB_B_PIN);

Servo motorRF, motorRB, motorLF, motorLB;        // create servo objects to control the four motors (R - right; L - left; F - front; B - back)


/************************************************************************
 *
 * Function:  robotSetup
 * Objective: Robot initial setup
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::robotSetup() {

  // initialize Brick Buzzer
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH); 
  
  // initializes the NeoPixel library.
  pixels.begin(); 
   
  // create i2c bus (sonars)
  Wire.begin();
  
 // attach motors to the servo objects (See VEVs manual)
 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************       
 * Do not turnarround this protetion, to avoid a
 *
 ****************************************************************/
 /* Vex manual min:1000 max:2000 */
//  motorRF.attach(MOTOR_RF_PIN, 1400, 1600); // attaches the right-front motor on pin 8 to the servo object
//  motorRB.attach(MOTOR_RB_PIN, 1400, 1600); // attaches the right-back motor on pin 9 to the servo object
//  motorLF.attach(MOTOR_LF_PIN, 1400, 1600); // attaches the left-front motor on pin 10 to the servo object
//  motorLB.attach(MOTOR_LB_PIN, 1400, 1600); // attaches the left-back motor on pin 11 to the servo object

  motorRF.attach(MOTOR_RF_PIN, 1000, 2000); // attaches the right-front motor on pin 8 to the servo object
  motorRB.attach(MOTOR_RB_PIN, 1000, 2000); // attaches the right-back motor on pin 9 to the servo object
  motorLF.attach(MOTOR_LF_PIN, 1000, 2000); // attaches the left-front motor on pin 10 to the servo object
  motorLB.attach(MOTOR_LB_PIN, 1000, 2000); // attaches the left-back motor on pin 11 to the servo object
}

/************************************************************************
 *
 * Function:  mecanumcinematics
 * Objective: define velocities to the 4 engines
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::mecanumcinematics(float linearx, float lineary, float angularz, float velocities[]) {

  //wheel Front Right
  velocities[0] = 2 / WHEEL_DIAMETER * ( linearx - lineary - (AXIS_LENGTH_L1 + AXIS_LENGTH_L2) / 2 * angularz);
  //wheel Front Left
  velocities[1] = 2 / WHEEL_DIAMETER * ( linearx + lineary + (AXIS_LENGTH_L1 + AXIS_LENGTH_L2) / 2 * angularz);
  //wheel Back Right
  velocities[2] = 2 / WHEEL_DIAMETER * ( linearx + lineary - (AXIS_LENGTH_L1 + AXIS_LENGTH_L2) / 2 * angularz);
  //wheel Back Left
  velocities[3] = 2 / WHEEL_DIAMETER * ( linearx - lineary + (AXIS_LENGTH_L1 + AXIS_LENGTH_L2) / 2 * angularz);
}

/************************************************************************
 *
 * Function:  Pcart
 * Objective: Determine robot velocities components (Vx Vy Wz)
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::Pcart(float vel0, float vel1, float vel2, float vel3, float velocities[]) {
  velocities[0] = (vel0 +  vel1 + vel2 +  vel3) / 4;  // Linear velocity - Vx
  velocities[1] = (vel1 -  vel0 - vel3 +  vel2) / 4;  // Linear velocity - Vy
  velocities[2] = (-vel0 + vel1 - vel2 + vel3) / 2 / (AXIS_LENGTH_L1 + AXIS_LENGTH_L2); // Angular velocity -  Wz
}

/************************************************************************
 *
 * Function:  STATUS LED.
 * Objective:
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::statusLED(int R, int G, int B, int R2, int G2, int B2) {
  pixels.setPixelColor(0, pixels.Color(R, G, B)); // RED / GREEN / BLUE
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setPixelColor(1, pixels.Color(R2, G2, B2)); // RED / GREEN / BLUE
  pixels.show(); // This sends the updated pixel color to the hardware.
}

/************************************************************************
 *
 * Function:  remap
 * Objective: remap velocities to motor input
 * Issues:    None to report so far.
 *
 *************************************************************************/
int Robot::remap_vel(float vel) {
  vel = vel * 100.0;
  int pulse = map((int)vel, -5000, 5000, 0, 18000);

  return pulse / 100;
}


/************************************************************************
 *
 * Function:  Reset encoder count
 * Objective:
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::encoders_reset() {
  encRF.write(0);
  encRB.write(0);
  encLF.write(0);
  encLB.write(0);
}

/************************************************************************
 *
 * Function:  readSonar.
 * Objective: Read X(index) sonar
 * Issues:    None to report so far.
 *
 *************************************************************************/
uint16_t Robot::readSonar(int index) {
  uint16_t reading=0;

  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(index); 

  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in centimeters
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(70);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(index); 
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(index, 2);    // request 2 bytes from slave device #index

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}


/************************************************************************
 *
 * Function:  readSonar.
 * Objective: Read X(index) sonar
 * Issues:    None to report so far.
 *
 *************************************************************************/
boolean Robot::readAllSonar_nonBlocking( uint16_t sonars_array[] ) {
  
  /// Update sonar values
  // Since it will take 65ms for each sonar, for the total of 16 sonars (16*60ms = 1040ms)
  if (sonarSync == 0) { // only move on to another sensor when we're sure we've read the measurement from the current one

  // step 1: instruct sensor to read echoes
    sonarSync = millis();  // start counting time for reading to happen and avoid to be loop blocking
  }
  // step 2: wait for readings to happen
  if ((millis() - sonarSync) > 65) { // datasheet suggests at least 65 milliseconds

    // step 3: instruct sensor to return a particular echo reading
    // step 4: request reading from sensor
    // step 5: receive reading from sensor

    sonars_array[sonarIN] = sonarIN;

    sonarIN++;    // move on to next ssonar
    sonarSync=0;  // Reset time sync counter

    if (sonarIN >= 16) { 
      sonarIN = 0;
      return true; 
    }  //if sonar reachs the index 16 reset to zero
  }
  
  return false;

}

/************************************************************************
 *
 * Function:  request_sonarRead.
 * Objective: Request sonar read and wait 65ms to get the value using requested_sonarRead()
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::request_sonarRead(int index) {

  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(index); 
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in centimeters
  Wire.endTransmission();      // stop transmitting
  
}


/************************************************************************
 *
 * Function:  requested_sonarRead.
 * Objective: Read available data from requested sonar in request_sonarRead()
 * Issues:    None to report so far.
 *
 *************************************************************************/
uint16_t Robot::requested_sonarRead(int index) {
  
  uint16_t reading=0;

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(index); 
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(index, 2);    // request 2 bytes from slave device #index

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}

/************************************************************************
 *
 * Function:  encoders readings.
 * Objective: Stop motors and turn on warning led
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::encoders_update(long int &encoderRF,long int &encoderRB, long int &encoderLF, long int &encoderLB) {
  encoderRF = encRF.read();
  encoderRB = encRB.read();
  encoderLF = encLF.read();
  encoderLB = encLB.read();
}

/************************************************************************
 *
 * Function:  move_noPID
 * Objective: move motors without PID controller   
 * Issues:    None to report so far.
 * Inputs:    Motor Speed Range [0 to 180]
 *
 *************************************************************************/
void Robot::move_noPID(int RF, int RB, int LF, int LB) {

 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************       
 * Do not turnarround this protetion, to avoid a
 *
 ****************************************************************/
 
 int VelRF_final=RF, 
     VelRB_final=RB, 
     VelLF_final=LF, 
     VelLB_final=LB;
 
  if (VelRF_final < MIN_THRESHOLD_MOTORS){  VelRF_final = MIN_THRESHOLD_MOTORS; }
  else if (VelRF_final > MAX_THRESHOLD_MOTORS){  VelRF_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelRB_final < MIN_THRESHOLD_MOTORS){  VelRB_final = MIN_THRESHOLD_MOTORS; }
  else if (VelRB_final > MAX_THRESHOLD_MOTORS){  VelRB_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelLF_final < MIN_THRESHOLD_MOTORS){  VelLF_final = MIN_THRESHOLD_MOTORS; }
  else if (VelLF_final > MAX_THRESHOLD_MOTORS){  VelLF_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelLB_final < MIN_THRESHOLD_MOTORS){  VelLB_final = MIN_THRESHOLD_MOTORS; }
  else if (VelLB_final > MAX_THRESHOLD_MOTORS){  VelLB_final = MAX_THRESHOLD_MOTORS; }
    

  motorRF.write(VelRF_final);
  motorRB.write(VelRB_final);
  motorLF.write(VelLF_final);
  motorLB.write(VelLB_final);
}


/************************************************************************
 *
 * Function:  move_PID
 * Objective: robot moves 2 meters - PID ASSISTED
 * Issues:    None to report so far.
 * Inputs:
 *
 *************************************************************************/
void Robot::move_PID_odometry(float linearx, float lineary, float angularz, int pwm_vel_motors[], float &pose_x, float &pose_y, float &pose_w, float veloc[]) {
  
  
    long int encRF_new=0, encRB_new=0, encLF_new=0, encLB_new=0;
    encoders_update(encRF_new,encRB_new, encLF_new, encLB_new);  //update encoder counts

    encLF_new = encLF_new * (-1);
    encLB_new = encLB_new * (-1);
    
    long int delta_T=millis()-dif_T;
    dif_T=millis();
    
    
    //real speed from each wheel
    float TspeedRB = -(((float)(encRB_new - encRB_old)) / PULSES_PER_METER) / ((float)delta_T / 1000000.0) ; //19972 pulses = 1meter
    float TspeedLB = -(((float)(encLB_new - encLB_old)) / PULSES_PER_METER) / ((float)delta_T / 1000000.0) ;
    float TspeedRF = -(((float)(encRF_new - encRF_old)) / PULSES_PER_METER) / ((float)delta_T / 1000000.0) ;
    float TspeedLF = -(((float)(encLF_new - encLF_old)) / PULSES_PER_METER) / ((float)delta_T / 1000000.0) ;
    
    //Pose calculation
    Pcart (TspeedRB, TspeedLB, TspeedRF, TspeedLF, veloc);
    
    

    
    if (delta_T < 200 && delta_T !=0) {
      
    	//positions:
  	double dt = (double) (delta_T/1000000.0);
  	pose_x = pose_x + (veloc[0] * cos(pose_w) - veloc[1] * sin(pose_w)) * dt;
  	pose_y = pose_y + (veloc[0] * sin(pose_w) + veloc[1] * cos(pose_w)) * dt;
  	pose_w = pose_w + veloc[2] * dt;
   
       // Normalize the vel
       veloc[0] = veloc[0] / 1000.0;
       veloc[1] = veloc[1] / 1000.0;
       veloc[2] = veloc[2] / 1000.0;
    }

    
    //Serial3.println(pose_w*(180.0/3.1415));
    
 
  float velocities[4] = {0};

/***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************       
 * Do not turnarround this protetion, to avoid demage the motors
 *
 ****************************************************************/
 
  if (linearx > MAX_LINEAR_SPEED){  linearx = MAX_LINEAR_SPEED; }
  else if (linearx < -MAX_LINEAR_SPEED){  linearx = -MAX_LINEAR_SPEED;}
  
  if (lineary > MAX_LINEAR_SPEED){  lineary = MAX_LINEAR_SPEED; }
  else if (lineary < -MAX_LINEAR_SPEED){  lineary = -MAX_LINEAR_SPEED;}
  
  if (angularz > MAX_ANGULAR_SPEED){  angularz = MAX_ANGULAR_SPEED; }
  else if (angularz < -MAX_ANGULAR_SPEED){  angularz = -MAX_ANGULAR_SPEED;}

  // check position
  mecanumcinematics(-linearx, -lineary, -angularz, velocities);


//  Serial3.println(linearx);
//  Serial3.println(lineary);
//  Serial3.println(angularz);
//  Serial3.println("-");

  int VelRF_des = remap_vel(velocities[2]);
  int VelRB_des = remap_vel(velocities[0]);
  int VelLF_des = remap_vel(velocities[3]);
  int VelLB_des = remap_vel(velocities[1]);

  //calculate real speed with encoder reading & time
  int VelRF_real = (int) (90 - ((float)(encRF_old - encRF_new) / (float)(delta_T)));
  int VelRB_real = (int) (90 - ((float)(encRB_old - encRB_new) / (float)(delta_T)));
  int VelLF_real = (int) (90 - ((float)(encLF_old - encLF_new) / (float)(delta_T)));
  int VelLB_real = (int) (90 - ((float)(encLB_old - encLB_new) / (float)(delta_T)));

  
  //PID aplication
  EpRF = (float) (VelRF_des - VelRF_real);
  EdRF = (EpRF - EpRF_old)/(float)(delta_T);
  EiRF = EiRF + EpRF*(float)(delta_T);
  EpRF_old = EpRF;
  int VelRF_add = 90 + (int) (Kp * EpRF + Kd * EdRF + Ki * EiRF);

  EpRB = (float) (VelRB_des - VelRB_real);
  EdRB = (EpRB - EpRB_old)/(float)(delta_T);
  EiRB = EiRB + (float)(delta_T)*EpRB;
  EpRB_old = EpRB;
  int VelRB_add = 90 + (int) (Kp * EpRB + Kd * EdRB + Ki * EiRB);

  EpLF = (float) (VelLF_des - VelLF_real);
  EdLF = (EpLF - EpLF_old)/(float)(delta_T);
  EiLF = EiLF + (float)(delta_T)*EpLF;
  EpLF_old = EpLF;
  int VelLF_add = 90 + (int) (Kp * EpLF + Kd * EdLF + Ki * EiLF);

  EpLB = (float) (VelLB_des - VelLB_real);
  EdLB = (EpLB - EpLB_old)/(float)(delta_T);
  EiLB = EiLB + (float)(delta_T)*EpLB;
  EpLB_old = EpLB;
  int VelLB_add = 90 + (int) (Kp * EpLB + Kd * EdLB + Ki * EiLB);

  encRF_old = encRF_new;
  encRB_old = encRB_new;
  encLF_old = encLF_new;
  encLB_old = encLB_new;
  
 /***************************************************************
 *                       !!Attention!!!                         *
 ****************************************************************       
 * Do not turnarround this protetion, to avoid a
 *
 ****************************************************************/
 int VelRF_final=VelRF_add, 
     VelRB_final=VelRB_add, 
     VelLF_final=VelLF_add, 
     VelLB_final=VelLB_add;


  // Invert L wheels
  VelLF_final = VelLF_final - (VelLF_final-90) * 2;
  VelLB_final = VelLB_final - (VelLB_final-90) * 2;
   
  if (VelRF_final < MIN_THRESHOLD_MOTORS){  VelRF_final = MIN_THRESHOLD_MOTORS; }
  else if (VelRF_final > MAX_THRESHOLD_MOTORS){  VelRF_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelRB_final < MIN_THRESHOLD_MOTORS){  VelRB_final = MIN_THRESHOLD_MOTORS; }
  else if (VelRB_final > MAX_THRESHOLD_MOTORS){  VelRB_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelLF_final < MIN_THRESHOLD_MOTORS){  VelLF_final = MIN_THRESHOLD_MOTORS; }
  else if (VelLF_final > MAX_THRESHOLD_MOTORS){  VelLF_final = MAX_THRESHOLD_MOTORS; }
  
  if (VelLB_final < MIN_THRESHOLD_MOTORS){  VelLB_final = MIN_THRESHOLD_MOTORS; }
  else if (VelLB_final > MAX_THRESHOLD_MOTORS){  VelLB_final = MAX_THRESHOLD_MOTORS; }
    
  pwm_vel_motors[0] = VelRF_final;
  pwm_vel_motors[1] = VelRB_final;
  pwm_vel_motors[2] = VelLF_final;
  pwm_vel_motors[3] = VelLB_final;

  move_noPID( VelRF_final, VelRB_final , VelLF_final, VelLB_final);
  
}


/************************************************************************
 *
 * Function:  stopButton_resetTimers
 * Objective: Reset timers when the stop button is pushed
 * Issues:    None to report so far.
 *
 *************************************************************************/
void Robot::resetTimers(){
  
  dif_T = millis();                                             // Update dif_T from move PID, once it is stop
  encoders_update(encRF_old, encRB_old, encLF_old, encLB_old);  // Update previous encoders counters, if the wheels where manualy moved, while the stop buttos is enable
  
  /* Reset PIDs errors */
  EpRF = 0.0, EpRF_old = 0.0, EdRF = 0.0, EiRF = 0.0;    // RF - Right Front motor controller error
  EpRB = 0.0, EpRB_old = 0.0, EdRB = 0.0, EiRB = 0.0;    // RB - Right Back motor controller error
  EpLF = 0.0, EpLF_old = 0.0, EdLF = 0.0, EiLF = 0.0;    // RF - Right Front motor controller error
  EpLB = 0.0, EpLB_old = 0.0, EdLB = 0.0, EiLB = 0.0;    // LB - Left Front motor controller error
}

/************************************************************************
 *
 * Function:  odometry_calculation
 * Objective: Convert wheel velocities to odometry
 * Issues:    None to report so far.
 *
 *************************************************************************/
boolean Robot::odometry_calculation(float &pose_x, float &pose_y, float &pose_w, float velocities[]){

  long int encRF_new=0, encRB_new=0, encLF_new=0, encLB_new=0;
  encoders_update(encRF_new,encRB_new, encLF_new, encLB_new);  //update encoder counts

  //real speed calculation
  if(last_vel_update == 0){  // Just occur in the first iteration
    
    last_vel_update = micros();
    //refresh encoder count
    encRF_old_odm = encRF_new;
    encRB_old_odm = encRB_new;
    encLF_old_odm = encLF_new;
    encLB_old_odm = encLB_new;
    
    return false;
  
  } else {
    
    long int deltat = (micros() - last_vel_update);
    last_vel_update = micros();
    
    
    //real speed from each wheel
    float TspeedRB = -(((float)(encRB_new - encRB_old_odm)) / PULSES_PER_METER) / ((float)deltat / 1000000.0) ; //19972 pulses = 1meter
    float TspeedLB = -(((float)(encLB_new - encLB_old_odm)) / PULSES_PER_METER) / ((float)deltat / 1000000.0) ;
    float TspeedRF = -(((float)(encRF_new - encRF_old_odm)) / PULSES_PER_METER) / ((float)deltat / 1000000.0) ;
    float TspeedLF = -(((float)(encLF_new - encLF_old_odm)) / PULSES_PER_METER) / ((float)deltat / 1000000.0) ;
    
    //Pose calculation
    Pcart (TspeedRF, TspeedLF, TspeedRB, TspeedLB, velocities);                      // Convert wheels velocities to cartesian pose
   
    pose_x = pose_x + ( velocities[0] * ((float)deltat / 1000000.0));
    pose_y = pose_y + ( velocities[1] * ((float)deltat / 1000000.0));
    pose_w = pose_w + ( velocities[2] * ((float)deltat / 1000000.0));
    
   //Serial3.println(pose_w*(180.0/3.1415));
    
    //refresh encoder count
    encRF_old_odm = encRF_new;
    encRB_old_odm = encRB_new;
    encLF_old_odm = encLF_new;
    encLB_old_odm = encLB_new;
    
    return true;
  }
}







