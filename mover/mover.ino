/* mover.ino
   Team Vandal Robotics
   May 22, 2023 */

#include <ros.h>
#include <ArduinoHardware.h> // accomidate for different Arduino hardware specs
#include <std_msgs/Empty.h> // the message format we are sending info over

#define M0_F 6     // Motor 0 Forwards (front)
#define M1_F 9     // Motor 1 Forwards (back left)
#define M2_F 11    // Motor 2 Forwards (back right)
#define M0_B 7     // Motor 0 Backwards
#define M1_B 8
#define M2_B 10

ros::NodeHandle nh;

/* Sends an empty message to roscore and flashes the built-in LED when the message has been recieved
   successfully. */
void messageCb(const std_msgs::Empty& toggle_msg)
{
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

/* Init a ROS node that is also a Subscriber (recieves messages)
   and sends a message of type Empty */
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

double M0 = 0, M1 = 0, M2 = 0;

// Initialize ROS Nodes and Arduino outputs 
void setup() 
{
  nh.initNode(); // Init a new node in the ROS network (on the RPi)
  nh.subscribe(sub); // Subscribe Node sub to the ROS network

  // Set the pins which move the motors "forwards"
  pinMode(M0_F, OUTPUT);
  pinMode(M1_F, OUTPUT);
  pinMode(M2_F, OUTPUT);
  digitalWrite(M0_F, LOW);
  digitalWrite(M1_F, LOW);
  digitalWrite(M2_F, LOW);

  // Set backwards motor pins
  pinMode(M0_B, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_B, OUTPUT);
  digitalWrite(M0_B, LOW);
  digitalWrite(M1_B, LOW);
  digitalWrite(M2_B, LOW);

  if (0) // test motor configuration on power on
  {
    analogWrite(M0_F, 150);
    analogWrite(M1_F, 150);
    analogWrite(M2_F, 150);
    delay(3000);
    digitalWrite(M0_F, LOW);
    digitalWrite(M1_F, LOW);
    digitalWrite(M2_F, LOW);
    analogWrite(M0_B, 150);
    analogWrite(M1_B, 150);
    analogWrite(M2_B, 150);
    delay(3000);
    digitalWrite(M0_B, LOW);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_B, LOW);
  }
}

// Main control loop
void loop() 
{
  double M0 = 0, M1 = 0, M2 = 0; // motor power values

  // PRE-CALCULATED -- SEE DOCS --
  // components:     x      y     w
  double m0c[] = { 0.67,     0, 0.33};
  double m1c[] = {-0.33, -0.58, 0.33};
  double m2c[] = {-0.33,  0.58, 0.33};
  
  double movestr[] = {0, 1, 0, 0.5};
  
  /* */
  for (int i = 0; i <= 2; i++) {
    M0 += m0c[i] * movestr[i];
    M1 += m1c[i] * movestr[i];
    M2 += m2c[i] * movestr[i];
  }
  M0 *= movestr[3];
  M1 *= movestr[3];
  M2 *= movestr[3];
}
