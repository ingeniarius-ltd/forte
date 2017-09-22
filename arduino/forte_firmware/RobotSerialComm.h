/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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
* Originally by: Gonçalo Cabrita on 22/09/2011
* Modified by:   Andre Araujo & David Portugal on 04/01/2013
* Modified by:   Ingeniarius on 20/05/2016
*********************************************************************/

#include "Arduino.h"

#define SERIAL_BUFFER_SIZE  32

#define START            '@'
#define SEPARATOR        ','
#define END              'e'

enum RobotAction
{
  
  //COMM
  ACTION_START_STREAM = 1,
  ACTION_STOP_STREAM = 2,
  
  //Reading routines
  ENCODER_TEST = 3,
  READ_ENCODERS = 4,
  READ_SONARS_FRONT = 5,
  READ_SONARS_RIGHT = 6,
  READ_SONARS_BACK = 7,
  READ_SONARS_LEFT = 8,
  READ_SONARS_ALL = 9,
  
  // Movement routines
  MOVE_PID = 10,
  MOVE_NOPID = 11,
  STOP_MOTORS = 12,
  ENCODERS_RESET = 13,
  
  // Visual
  LED_STATE = 14,
  
  //Odometry
  ODOM = 15,
  ODOM_RESET = 16,
      
  ACTION_COUNT = 17

};

extern int ACTION_PARAM_COUNT[];

enum
{
    AWATING_DATA = 0,
    GETTING_DATA = 1
};

class RobotSerialComm
{
    public:
    RobotSerialComm();
    
    int getMsg(unsigned int * argv);
    
    void reply(unsigned int action, unsigned int * argv, int argc);
    
    void sendDebugMsg();
    int getDebug();
    void setDebug(int d);
    
    // Debug message buffer
    String debug_msg;
    // Debug ON (1) or OFF (0)
    byte debug;
    
    private:
    // Buffer for serial input
    char serial_buffer[SERIAL_BUFFER_SIZE];
    // The current size of serial input buffer
    int serial_buffer_size;
    // Status of the serial port, AWATING_DATA or GETTING_DATA
    byte serial_port_status;
    
    // Used by the function getValue to store the index of the next separator byte
    int next_separator;
    
    int getValue(int start_index);
};

// EOF

