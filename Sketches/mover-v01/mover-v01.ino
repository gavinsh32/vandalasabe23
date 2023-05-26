/*  mover-v10.ino
    Gavin, Shariar, Amesh
    Team Vandal Robotics
    May 24, 2023

    Description:
    - Move the bot using 3 omniwheels, so there's
      a little bit of math done using calc_base().
    - Interface with a RPi, which is running ROS Noetic
      on Ubuntu 20.04 using rosserial-arduino and
      the ROS interface ros.h */

#define M0_F 6    // Motor 0 Forwards (front)
#define M1_F 9    // Motor 1 Forwards (back left)
#define M2_F 11   // Motor 2 Forwards (back right)
#define M0_B 7    // Motor 0 Backwards
#define M1_B 8    // etc 
#define M2_B 10   // etc

// Prototypes
void calc_base(const double *);
void move_base();
void stop_base();
void test();

// Global variables
double M0 = 0, M1 = 0, M2 = 0;  //  Analog motor speed (0-255)

/*  Initialize motor base configuration and set all speeds to 0  */
void setup() {

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
}

void loop()
{

}

/*  Calculate the power vector for each motor required to move
    in the direction specified by movestr[x, y, w, speed].
    Each motor contributes a certain fraction of overall power
    for achieving movement of the base.

    This is done by drawing a lot of right triangles and finding the
    ratios for each motor along the axis x, y, and w (rotation).
    The values below are the result of an inverse matrix transformation,
    which is explained in more detail in this bot's documentation. */
void calc_base(const double * movestr)
{
  M0 = 0; M1 = 0; M2 = 0;     // set all motors to 0

  // Components:       x      y     w
  double m0c[] = { -0.67,     0, 0.33 };
  double m1c[] = {  0.33, -0.58, 0.33 };
  double m2c[] = {  0.33,  0.58, 0.33 };

  /*  Multiply each motor component by the desired
      overall vector component. Add each product to
      each motor power to find the resulting vector. */
  for (int i = 0; i <= 2; i++) {
    M0 += m0c[i] * movestr[i];
    M1 += m1c[i] * movestr[i];
    M2 += m2c[i] * movestr[i];
  }

  //  Multiply by speed
  M0 *= movestr[3];
  M1 *= movestr[3];
  M2 *= movestr[3];
}

/*  Move each motor with the speed specified in
    M0, M1, and M2, and flip the pinouts if the
    values are negative. Our pinout configuration
    is with the bot spinning counterclockwise
    when all wheels are moving "forwards".  */
void move_base()
{
  if (M0 >= 0) analogWrite(M0_F, M0);
  else analogWrite(M0_B, M0 * -1);
  if (M1 >= 0) analogWrite(M1_F, M1);
  else analogWrite(M1_B, M1 * -1);
  if (M2 >= 0) analogWrite(M2_F, M2);
  else analogWrite(M2_B, M2 * -1);
}

//  Turn all motor speeds to 0.
void stop_base()
{
  digitalWrite(M0_F, 0);
  digitalWrite(M1_F, 0);
  digitalWrite(M2_F, 0);
  digitalWrite(M0_B, 0);
  digitalWrite(M1_B, 0);
  digitalWrite(M2_B, 0);
}

/*  Test the base movement along each of the axis   */
void test()
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
  double tests[10][4] = {
    {0, 1, 0, 150},
    {0, -1, 0, 150},
    {1, 0, 0, 150},
    { -1, 0, 0, 150},
    {0.71, 0.71, 0, 150},
    { -0.71, -0.71, 0, 150},
    { -0.71, 0.71, 0, 150},
    {0.71, -0.71, 0, 150},
    {0, 0, 1, 150},
    {0, 0, -1, 150}
  };

  for (int z = 0; z < 10; z++)
  {
    calc_base(tests[z]);
    move_base();
    delay(2000);
    stop_base();
    delay(1000);
  }
}
