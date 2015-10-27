**FORTE** is a mobile robotic platform ideal for research and industrial applications. **FORTE** is a robust omnidirectional platform that can carry more than 100 Kg, ideal to move materials around a manufacturing facility or a warehouse. Moreover, **FORTE** is equipped with a 7 degree of freedom manipulator that enables the interaction with the surrounding environment so as to, for instance, deliver the load.

# 1.	Install
To extend **FORTE**’s functionalities, you must first connect to it and program it. **FORTE** comes with an Arduino MEGA ADK controller so, to program it, you can simply download Arduino IDE, connect a USB cable to **FORTE**’s board, and that’s it. 
Here is the installation process explained step-by-step:
* Go to https://www.arduino.cc/en/Main/Software and download the latest release of Arduino IDE
* Download the Adafruit_NeoPixel and the Servo Arduino libraries [here](arduino/library) and install it as any other Arduino library
* Connect to **FORTE**’s Arduino MEGA ADK board using a USB cable
* Install any regular Arduino drivers if necessary (those are available in the Arduino IDE folder installed on your computer)

For high-level programming, and to further extend **FORTE**’s functionalities, one can benefit from the Intel NUC equipped on the platform. To have access to the Intel NUC, follow these steps: 
* Connect to WiFi Access Point with the following details:
  * SSID: FORTEX_AP, with X = identification of the robot (ONE, TWO, etc…)
  * Password: ingeniarius
* Go to http://www.tightvnc.com and download the latest release of TightVNC and connect to the following IP 192.168.137.1

For instance, the user can use the Robot Operating System (ROS) for robotic purposes (e.g., navigation and planning), benefiting from **FORTE**’s additional features (e.g., Microsoft Kinect v2).

That's it - Enjoy!

# 2.	Quick start
After following the installation steps, test the [Fortev1.ino](arduino/examples/Fortev1/Fortev1.ino) firmware located in arduino/examples/ to make sure all libraries are adequately installed. The communication protocol of this firmware version is the following:

<table>
<tr>
<td>Sent from CPU</td>
<td>Received from Arduino</td>
<td>Status</td>
</tr>
<tr>
<td>0x64 0x10 0xDIST</td>
<td>-</td>
<td>Move DIST centimetres forward</td>
</tr>
<tr>
<td>0x64 0x20</td>
<td>0x64 0x21<br>S0_HB S0_LB<br>-<br>S15_HB S15_LB</td>
<td>Sending one cycle of all sonar readings</td>
</tr>
</table>
