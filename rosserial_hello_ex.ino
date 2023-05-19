/* HelloWorld.ino 
   Gavin Haynes
   
   **EXAMPLE PROVIDED BY ROS**
   
   wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World

   Usage:
   - this Arduino is treated as a client (or multiple clients) and responds to the RPi
   - on RPi, run roscore
   - on RPi, rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 
   - to view the message being sent back and forth, rostopic echo chatter */

#include <ArduinoHardware.h> // device-specific hardware, such as RAM size
#include <ros.h> // lightweight ROS interface, such as message topics and nodes
#include <std_msgs/String.h> // String message format

// Use this line when working with after-market Arduino 
#define USE_USBCON

ros::NodeHandle nh; // Contains Node identifier information like names and topics

std_msgs::String str_msg; // Creates a message object which is a pointer to the data itself
ros::Publisher chatter("chatter", &str_msg); // Init a node with a name and message types

char hello[13] = "Hi Shariar!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter); // Initialize a new node along the ROS network
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg ); // ros::Publisher.publish(pointer_to_message);
  nh.spinOnce(); // Call back to roscore service 
  delay(1000);
}
