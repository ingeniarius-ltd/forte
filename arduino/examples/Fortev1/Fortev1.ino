/**
 *
 * Copyright (c) 2015 Ingeniarius, Lda. All rights reserved.
 *
 * This file is part of the project Forte. This code handles the low level control of the Forte platform, including navigation.
 *
 * The Arduino Mega ADK has the following tasks:
 *
 *  1. Start the platform;
 *  2. Omnidirectional driving controlling motors, while reading encoders;
 *  3. Receive Serial requests and adequately reply (e.g., sonar readings).
 *
**/

/************************************************************************
 *
 * Load libraries
 *
*************************************************************************/
#include <Servo.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>




/************************************************************************
 *
 * Define global variables
 *
*************************************************************************/

// auxiliary variable
boolean ONCE = false;

// create servo objects to control the four motors (R - right; L - left; F - front; B - back)
#define motorRFpin 8
#define motorRBpin 9
#define motorLFpin 10
#define motorLBpin 11
Servo motorRF, motorRB, motorLF, motorLB;

// define encoders. Note that only the first pin of each encoder has interrupt capability (i.e., 2, 3, 18 and 19). 
// Also note that the first pin should always correspond to the first pin that detects changes when moving forward, 
// meaning that motors from left wheels will have their encoders with the phases switched when compared to the motors from the right
Encoder encRF(2, 4), encRB(3, 5), encLF(18, 17), encLB(19, 16);
long int encRF_old = 0, encRB_old = 0, encLF_old = 0, encLB_old = 0, encRF_new = 0, encRB_new = 0, encLF_new = 0, encLB_new = 0;

// define sonar IDs
int sonarID[16] = { 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127};

// define NeoPixel RGB LED
#define LED            6
float fade = 0;
boolean cycle = true;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, LED, NEO_GRB + NEO_KHZ800);

// define Bricket Buzzer
#define BUZZ           7

// define controller gains for the motors, as well as errors
float Kp = 0.1, Kd = 0.07, Ki = 0.002;
float EpRF = 0.0, EpRF_old = 0.0, EdRF = 0.0, EiRF = 0.0;
float EpRB = 0.0, EpRB_old = 0.0, EdRB = 0.0, EiRB = 0.0;
float EpLF = 0.0, EpLF_old = 0.0, EdLF = 0.0, EiLF = 0.0;
float EpLB = 0.0, EpLB_old = 0.0, EdLB = 0.0, EiLB = 0.0;

int *readAllSonars(){
  int readingHB = 0, readingLB = 0, reading;
  int sonarReadings[16];
  
  Serial.write(0x64);
  Serial.write(0x21);
  for(int index=0;index<16;index++){
    // step 1: instruct sensor to read echoes
    Wire.beginTransmission(sonarID[index]); // transmit to device #112 (0x70)
                                 // the address specified in the datasheet is 224 (0xE0)
                                 // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
                                 // use 0x51 for centimeters
                                 // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting
  
    // step 2: wait for readings to happen
    delay(70);                   // datasheet suggests at least 65 milliseconds
  
    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(sonarID[index]); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting
  
    // step 4: request reading from sensor
    Wire.requestFrom(sonarID[index], 2);    // request 2 bytes from slave device #112

    // step 5: receive reading from sensor
    if(2 <= Wire.available())    // if two bytes were received
    {
      readingHB = Wire.read();  // receive high byte (overwrites previous reading)
      Serial.write(readingHB);
      //reading = reading << 8;    // shift high byte to be high 8 bits
      readingLB = Wire.read(); // receive low byte as lower 8 bits
      Serial.write(readingLB);
      reading = (readingHB << 8) | readingLB; // receive low byte as lower 8 bits
    }
    delay(5);
  }
  
  return sonarReadings;
}


int moveForward(int distObj){
  int distDone = 0;
  
  Serial.write(0x64);
  Serial.write(0x11);


  while(distDone<distObj){
    distDone = (int)((float)(((encRF.read() + encRB.read() + encLF.read() + encLB.read()) / 4) * 2*PI*20.32) / 3000) ;
    // start moving - This is only here because we intend to implement a speed controller at some point!
    motorRF.write(100);
    motorRB.write(100);
    motorLF.write(80);
    motorLB.write(80);
  }
  
  // stop motors
  motorRF.write(90);
  motorRB.write(90);
  motorLF.write(90);
  motorLB.write(90);
  
  return distDone;
}


/************************************************************************
 *
 * Function:  Initialize motors (stop position).
 * Objective: 
 * Issues:    None to report so far.
 *
*************************************************************************/
void startEngines(){
  // initialize motors in the stop position (90 - stop; 0 - high speed reverse; 180 - high speed foward)
  motorRF.write(90);
  motorRB.write(90);
  motorLF.write(90);
  motorLB.write(90);
  
  // blink red LED and Buzz to identify the start
  for(int i=0;i<3;i++){
    digitalWrite(BUZZ,LOW);
    pixels.setPixelColor(0, pixels.Color(255,0,0)); 
    pixels.show(); // This sends the updated pixel color to the hardware
    delay(500);
    digitalWrite(BUZZ,HIGH);
    pixels.setPixelColor(0, pixels.Color(0,0,0)); 
    pixels.show(); // This sends the updated pixel color to the hardware
    delay(200);
  }
  
  pixels.setPixelColor(0, pixels.Color(255,180,0)); 
  pixels.show(); // This sends the updated pixel color to the hardware
  
  // reset encoders
  encRF.write(0);
  encRB.write(0);
  encLF.write(0);
  encLB.write(0);
 
}

/************************************************************************
 *
 * Function:  Blink yellow LED.
 * Objective: 
 * Issues:    None to report so far.
 *
*************************************************************************/
void warningLED(){
  if(cycle==true){
    if(fade<255)
      fade=fade+0.1;
    else
      cycle=false;
  }else{
    if(fade>0)
      fade=fade-0.1;
    else
      cycle=true;
  }
  pixels.setPixelColor(0, pixels.Color(round(fade),round(fade),0)); 
  pixels.show(); // This sends the updated pixel color to the hardware
}


/************************************************************************
 *
 * Function:  Initial setup.
 * Objective: Open Serial port
 *            Open I2C
 *            Start motors
 * Issues:    None to report so far.
 *
*************************************************************************/
void setup() {
  
  // start serial communication at 115200bps
  Serial.begin(115200); 

  // create i2c bus (master)  
  Wire.begin();            

  // initialize Brick Buzzer
  pinMode(BUZZ, OUTPUT);
  digitalWrite(BUZZ, HIGH);  
  
  // attach motors to the servo objects
  motorRF.attach(motorRFpin,1000,2000);  // attaches the right-front motor on pin 8 to the servo object 
  motorRB.attach(motorRBpin,1000,2000);  // attaches the right-back motor on pin 9 to the servo object 
  motorLF.attach(motorLFpin,1000,2000);  // attaches the left-front motor on pin 10 to the servo object 
  motorLB.attach(motorLBpin,1000,2000);  // attaches the left-back motor on pin 11 to the servo object 

  // initialize NeoPixel library
  pixels.begin();
  

  pixels.setPixelColor(0, pixels.Color(255,0,0)); 
  pixels.show(); // This sends the updated pixel color to the hardware
  
  delay(100);
  startEngines();
  
  /*
  motorRF.write(98);
  motorRB.write(98);
  motorLF.write(82);
  motorLB.write(82);
  
  while(abs(encLF.read())<12500);
  */
  motorRF.write(90);
  motorRB.write(90);
  motorLF.write(90);
  motorLB.write(90);


  
}


int VelRF_des = 90, VelRB_des = 90, VelLF_des = 90, VelLB_des = 90;
long int cntT = 0;

/************************************************************************
 *
 * Function:  Main loop.
 * Objective: Receive data from main CPU (Serial communication)
 *            Control platform
 * Issues:    None to report so far.
 *
*************************************************************************/
void loop() {
  unsigned long timeout = 0;
  if(Serial.available()>0){
      unsigned int C = Serial.read();
      if (C==0x64){  // it is a simple command
        C = 0xFF;
        while(Serial.available()==0 && timeout<10000);timeout++;
        timeout = 0;
        C = Serial.read();
        switch(C){
          case 0x10:
            // move forward
            C = 0xFF;
            while(Serial.available()==0 && timeout<10000);timeout++;
            timeout = 0;
            C = Serial.read(); // number of centimeters to move [0 to 200]
            if (C!=0xFF)
              int distDone = moveForward(C); // return distance travelled in centimeters
            break;
          case 0x20:
            // read all sonars (one cycle)
            int *sonarReadings = readAllSonars();
            break; 
        }
      }
    }
    
    encRF_new = encRF.read();
    encRB_new = encRB.read();
    encLF_new = -encLF.read();
    encLB_new = -encLB.read();
    
    long int encRF_diff = encRF_old - encRF_new;
    long int encRB_diff = encRB_old - encRB_new;
    long int encLF_diff = encLF_old - encLF_new;
    long int encLB_diff = encLB_old - encLB_new;
    
    int VelRF_real = (int) (90 - encRF_diff/0.8);
    int VelRB_real = (int) (90 - encRB_diff/0.8);
    int VelLF_real = (int) (90 + encLF_diff/0.8);
    int VelLB_real = (int) (90 + encLB_diff/0.8);
  
    EpRF = (float) (VelRF_des - VelRF_real);
    EdRF = EpRF - EpRF_old;
    EiRF = EiRF + EpRF;
    EpRF_old = EpRF;
    int VelRF_add = 90 + (int) (Kp*EdRF + Kd*EdRF + Ki*EiRF);
    
    EpRB = (float) (VelRB_des - VelRB_real);
    EdRB = EpRB - EpRB_old;
    EiRB = EiRB + EpRB;
    EpRB_old = EpRB;
    int VelRB_add = 90 + (int) (Kp*EdRB + Kd*EdRB + Ki*EiRB);
  
    EpLF = (float) (VelLF_des - VelLF_real);
    EdLF = EpLF - EpLF_old;
    EiLF = EiLF + EpLF;
    EpLF_old = EpLF;
    int VelLF_add = 90 + (int) (Kp*EdLF + Kd*EdLF + Ki*EiLF);// + (int) fopid(EpLF, EdLF, EiLF, Kp, Kd, Ki, miu, gama);
    
    EpLB = (float) (VelLB_des - VelLB_real);
    EdLB = EpLB - EpLB_old;
    EiLB = EiLB + EpLB;
    EpLB_old = EpLB;
    int VelLB_add = 90 + (int) (Kp*EdLB + Kd*EdLB + Ki*EiLB);
    
//    Serial.print("VelLF_real = ");
//    Serial.println(VelLF_real);
//    Serial.print("VelLF_des = ");
//    Serial.println(VelLF_des);
//    Serial.println("VelLF_add = ");
//    Serial.println(VelLF_add);
    
    encRF_old = encRF_new;
    encRB_old = encRB_new;
    encLF_old = encLF_new;
    encLB_old = encLB_new;
    
    if ((VelRF_add>60) && (VelRF_add<120))
      motorRF.write(VelRF_add);
    if ((VelRB_add>60) && (VelRB_add<120))
      motorRB.write(VelRB_add); 
    if ((VelLF_add>60) && (VelLF_add<120))
      motorLF.write(VelLF_add);
    if ((VelLB_add>60) && (VelLB_add<120))
      motorLB.write(VelLB_add);  
    
    cntT++;
    
//    // Forward
//    if (cntT==10){
//      VelRF_des = 80;
//      VelRB_des = 80;
//      VelLF_des = 100;
//      VelLB_des = 100;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
    
//    // Stop
//    if (cntT==8000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
    
//    // Backward
//    if (cntT==6000){
//      VelRF_des = 100;
//      VelRB_des = 100;
//      VelLF_des = 80;
//      VelLB_des = 80;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Stop
//    if (cntT==10000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Rotate Right
//    if (cntT==10000){
//      VelRF_des = 100;
//      VelRB_des = 100;
//      VelLF_des = 100;
//      VelLB_des = 100;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Stop
//    if (cntT==14000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
    
//    // Forward
//    if (cntT==16000){
//      VelRF_des = 80;
//      VelRB_des = 80;
//      VelLF_des = 100;
//      VelLB_des = 100;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Rotate Left
//    if (cntT==18000){
//      VelRF_des = 80;
//      VelRB_des = 80;
//      VelLF_des = 80;
//      VelLB_des = 80;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Stop
//    if (cntT==22000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//
//    // Move Left
//    if (cntT==24000){
//      VelRF_des = 110;
//      VelRB_des = 70;
//      VelLF_des = 110;
//      VelLB_des = 70;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Stop
//    if (cntT==27000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Move Right
//    if (cntT==29000){
//      VelRF_des = 70;
//      VelRB_des = 110;
//      VelLF_des = 70;
//      VelLB_des = 110;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
//    
//    // Stop
//    if (cntT==32000){
//      VelRF_des = 90;
//      VelRB_des = 90;
//      VelLF_des = 90;
//      VelLB_des = 90;
//      EiRF = 0;
//      EiRB = 0;
//      EiLF = 0;
//      EiLB = 0;
//    }
    
    
    

    
//    long int diffEnc = prevEnc - encLF.read();
//    
//    ep = (-1 - diffEnc);
//    ei = ei + ep;
//    
//    int Vel = 98 - 1*ep - 0.01*ei;
//    motorLF.write(Vel);
//    
//    Serial.println(Vel);
//    
//    prevEnc = encLF.read();
//  Serial.println("######");
//  Serial.println(encRF.read());
//  Serial.println(encRB.read());
//  Serial.println(encLF.read());
//  Serial.println(encLB.read());
//  delay(1000);
  
}




