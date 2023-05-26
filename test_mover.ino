#define M0_F 6     // Motor 0 Forwards (front)
#define M1_F 9     // Motor 1 Forwards (back left)
#define M2_F 11    // Motor 2 Forwards (back right)
#define M0_B 7     // Motor 0 Backwards
#define M1_B 8
#define M2_B 10

// Prototypes
void calc_base(const double *);
void move_base();
void stop_base();

// Global variables
double M0 = 0, M1 = 0, M2 = 0;

void setup() {
  // put your setup code here, to run once:
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

  Serial.begin(9600);
}

void loop()
{
  double tests[10][4] = {{0, 1, 0, 150}, 
                     {0, -1, 0, 150},
                     {1, 0, 0, 150},
                     {-1, 0, 0, 150},
                     {0.71, 0.71, 0, 150},
                     {-0.71, -0.71, 0, 150},
                     {-0.71, 0.71, 0, 150},
                     {0.71, -0.71, 0, 150},
                     {0, 0, 1, 150},
                     {0, 0, -1, 150}};
  
  for (int z = 0; z < 10; z++)
  {
    calc_base(tests[z]);
    move_base();
    delay(2000);
    stop_base();
    delay(1000);
  } 
}

void calc_base(double * movestr)
{
  M0 = 0; M1 = 0; M2 = 0;

  // PRE-CALCULATED -- SEE DOCS --
  // components:       x      y     w
  double m0c[] = { -0.67,     0, 0.33 };
  double m1c[] = {  0.33, -0.58, 0.33 };
  double m2c[] = {  0.33,  0.58, 0.33 };

  for (int i = 0; i <= 2; i++) {
    M0 += m0c[i] * movestr[i];
    M1 += m1c[i] * movestr[i];
    M2 += m2c[i] * movestr[i];
  }

  M0 *= movestr[3];
  M1 *= movestr[3];
  M2 *= movestr[3];
}

void move_base()
{
  if (M0 >= 0) analogWrite(M0_F, M0);
  else analogWrite(M0_B, M0 * -1);
  if (M1 >= 0) analogWrite(M1_F, M1);
  else analogWrite(M1_B, M1 * -1);
  if (M2 >= 0) analogWrite(M2_F, M2);
  else analogWrite(M2_B, M2 * -1);
}

void stop_base()
{
  digitalWrite(M0_F, 0);
  digitalWrite(M1_F, 0);
  digitalWrite(M2_F, 0);
  digitalWrite(M0_B, 0);
  digitalWrite(M1_B, 0);
  digitalWrite(M2_B, 0);
}
