/* mover.ino
   Team Vandal Robotics
   May 22, 2023

    OVERVIEW:
    - Init a Node Handler, which integrates a Node which we create on the Arduino 
      so that it can easily interface with the Nodes we create on the RPi
    - Create a subscriber node, which is able to recieve messages along the 
      ROS network of a specified data type
    - Read the data sent via a message and move the motors */

#include <ros.h>
#include <ArduinoHardware.h> // accomidate for different Arduino hardware specs
#include <std_msgs/Empty.h> // the message format we are sending info over

#define M0_F 6;     // Motor 0 Forwards (front)
#define M1_F 9;     // Motor 1 Forwards (back left)
#define M2_F 11;    // Motor 2 Forwards (back right)
#define M0_B 7;     // Motor 0 Backwards
#define M1_B 8;
#define M2_B 10;

ros::NodeHandle nh;

/* void messageCb(ROS_MSG*)
   Sends an empty message to roscore and flashes the built-in LED when the message has been recieved
   successfully. */
void messageCb(const std_msgs::Empty& toggle_msg)
{
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

/* Init a ROS node that is also a Subscriber (recieves messages)
   and sends a message of type Empty */
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

double mr1, mr2, mr3;         // proportional motor output

void calc_motor_ratios(double x, double y, double w)
{
  // Vector ratios that someone else figured out. See docs.
  double m1[] = { 0.67,     0, 0.33};
  double m2[] = {-0.33, -0.58, 0.33};
  double m3[] = {-0.33,  0.58, 0.33};
}

// Moves the entire robot base motors
void move_base(double m0, double m1, double m2, double mag){
  if (m1 >= 0) analogWrite(M0_F, m0 * mag);
  else analogWrite(M0_B, m0 * -1 * mag);
  if (m2 >= 0) analogWrite(M1_F, m1 * mag);
  else analogWrite(M1_B, m1 * -1 * mag);
  if (m1 >= 0) analogWrite(M2_F, m2 * mag);
  else analogWrite(M2_B, m2 * -1 * mag);
}

// Initialize ROS Nodes and Arduino outputs 
void setup() 
{
  nh.initNode(); // Init a new node in the ROS network (on the RPi)
  nh.subscribe(sub); // Subscribe Node sub to the ROS network

  // Set the pins which move the motors "forwards"
  pinMode(M1_F, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M3_F, OUTPUT);
  digitalWrite(M1_F, LOW);
  digitalWrite(M2_F, LOW);
  digitalWrite(M3_F, LOW);

  // Set backwards motor pins
  pinMode(M1_B, OUTPUT);
  pinMode(M2_B, OUTPUT);
  pinMode(M3_B, OUTPUT);
  digitalWrite(M1_B, LOW);
  digitalWrite(M2_B, LOW);
  digitalWrite(M3_B, LOW);
}

// Main control loop
void loop() 
{
  move_base(1, 1, 1, 250);
}
