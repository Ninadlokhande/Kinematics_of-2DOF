#include <Arduino.h>
#include <math.h>

// --- Motor pins and steps ---
#define dir1 2
#define step1 3
#define Maxsteplow 800
#define dir2 10
#define step2 11
#define Maxstephigh 1220
#define pulsedelay 1000

#define motor1MaxAngle 180
#define motor2MaxAngle 265

// --- Current motor positions ---
int currentAngle1 = 0;
int currentAngle2 = 0;

// --- Robot arm parameters ---
const float L1 = 0.10f; // meters
const float L2 = 0.11f; // meters
const float M1 = 0.5f;  // kg
const float M2 = 0.23f; // kg
const float G  = 9.81f; // m/s^2

// --- Workspace limits ---
const float xMin = fabs(L1 - L2);
const float xMax = L1 + L2;
const float yMin = 0; // minimum Y (depends on arm configuration)
const float yMax = L1 + L2;

// --- Structs ---
struct Point { float x; float y; };
struct Angles { float theta1; float theta2; };
struct JointState { float angularAcceleration1; float angularAcceleration2; };

// ------------------------- FUNCTION DECLARATIONS -------------------------
Point forwardKine(float theta1_deg, float theta2_deg);
Angles inverseKine(float x, float y);
void inversedynamics(float theta1_deg, float theta2_deg);
JointState forwardDynamics(float theta1_deg,float theta2_deg,float tau1,float tau2);
void moveStepperToAngle(int dirPin, int stepPin, int stepsPerRev, int maxAngle, int currentAngle, int targetAngle);
void moveMotorsToAngles(int targetAngle1, int targetAngle2);

// ------------------------- SETUP -------------------------
void setup() {
  Serial.begin(9600);
  while(!Serial) { ; }

  Serial.println("=== Robot Arm Control ===");
  Serial.println("Commands:");
  Serial.println("F theta1,theta2   -> Forward Kinematics (angles in degrees)");
  Serial.println("I x,y             -> Inverse Kinematics (position in cm)");

  Serial.print("Forward Kinematics valid ranges: Theta1 = 0 to "); Serial.print(motor1MaxAngle);
  Serial.print("°, Theta2 = 0 to "); Serial.print(motor2MaxAngle); Serial.println("°");

  Serial.print("Inverse Kinematics valid workspace: X = "); Serial.print(xMin*100,2); 
  Serial.print(" to "); Serial.print(xMax*100,2); 
  Serial.print(" cm, Y = "); Serial.print(yMin*100,2); 
  Serial.print(" to "); Serial.print(yMax*100,2); Serial.println(" cm");

  pinMode(dir1, OUTPUT);
  pinMode(step1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(step2, OUTPUT);
}

// ------------------------- LOOP -------------------------
void loop() {
  if(Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if(input.length() < 1) return;

    char command = toupper(input.charAt(0));  // 'F' or 'I'
    String data = input.substring(1);
    data.trim();

    int commaIndex = data.indexOf(',');
    if(commaIndex < 0) {
      Serial.println("Invalid format! Use comma separated values.");
      return;
    }

    float val1 = data.substring(0, commaIndex).toFloat();
    float val2 = data.substring(commaIndex + 1).toFloat();

    Angles targetAngles;

    if(command == 'F') {
      Serial.println(">>> Forward Kinematics (angles to position)");

      // Clamp angles to max allowed
      if(val1 < 0) val1 = 0; 
      if(val1 > motor1MaxAngle) val1 = motor1MaxAngle;
      if(val2 < 0) val2 = 0; 
      if(val2 > motor2MaxAngle) val2 = motor2MaxAngle;

      Serial.print("Using angles: Theta1="); Serial.print(val1);
      Serial.print(", Theta2="); Serial.println(val2);

      Point pos = forwardKine(val1, val2); // compute position
      targetAngles.theta1 = val1;
      targetAngles.theta2 = val2;
    }
    else if(command == 'I') {
      Serial.println(">>> Inverse Kinematics (position to angles)");

      // Convert cm to meters
      float x_m = val1 / 100.0;
      float y_m = val2 / 100.0;

      // Check workspace limits
      float distance = sqrt(x_m*x_m + y_m*y_m);
      if(distance > (L1 + L2) || distance < fabs(L1 - L2)) {
        Serial.println("Position is outside the robot's reachable workspace!");
        Serial.print("Min reach = "); Serial.print(fabs(L1-L2)*100); Serial.print(" cm, ");
        Serial.print("Max reach = "); Serial.print((L1+L2)*100); Serial.println(" cm");
        return;
      }

      targetAngles = inverseKine(x_m, y_m); // compute angles in degrees
      if(targetAngles.theta1 < 0 || targetAngles.theta2 < 0) return; // safety
    }
    else {
      Serial.println("Unknown command! Use 'F' or 'I'.");
      return;
    }

    // --- Move motors to target angles ---
    moveMotorsToAngles(targetAngles.theta1, targetAngles.theta2);

    // --- Dynamics ---
    inversedynamics(targetAngles.theta1, targetAngles.theta2);
    forwardDynamics(targetAngles.theta1, targetAngles.theta2, 0.5f, 0.2f);

    Serial.println("-----------------------------");
  }
}

// ------------------------- MOTOR MOVEMENT -------------------------
void moveMotorsToAngles(int targetAngle1, int targetAngle2) {
  moveStepperToAngle(dir1, step1, Maxsteplow, motor1MaxAngle, currentAngle1, targetAngle1);
  currentAngle1 = targetAngle1;

  moveStepperToAngle(dir2, step2, Maxstephigh, motor2MaxAngle, currentAngle2, targetAngle2);
  currentAngle2 = targetAngle2;

  Serial.print("Motors moved to angles: ");
  Serial.print("Motor1="); Serial.print(currentAngle1);
  Serial.print("°, Motor2="); Serial.println(currentAngle2);
}

void moveStepperToAngle(int dirPin, int stepPin, int stepsPerRev, int maxAngle, int currentAngle, int targetAngle) {
  if(targetAngle > maxAngle) targetAngle = maxAngle;
  if(targetAngle < 0) targetAngle = 0;

  int stepCurrent = map(currentAngle, 0, maxAngle, 0, stepsPerRev);
  int stepTarget = map(targetAngle, 0, maxAngle, 0, stepsPerRev);
  int stepsToMove = abs(stepTarget - stepCurrent);
  bool direction = (stepTarget > stepCurrent); // true=HIGH, false=LOW

  digitalWrite(dirPin, direction ? HIGH : LOW);

  for(int i=0;i<stepsToMove;i++){
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(pulsedelay);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(pulsedelay);
  }
}

// ------------------------- FORWARD KINEMATICS -------------------------
Point forwardKine(float theta1_deg, float theta2_deg) {
  Point result;
  float t1 = theta1_deg*PI/180.0f;
  float t2 = theta2_deg*PI/180.0f;

  result.x = L1*cosf(t1) + L2*cosf(t1+t2);
  result.y = L1*sinf(t1) + L2*sinf(t1+t2);

  Serial.println("Forward Kinematics Result:");
  Serial.print("X = "); Serial.print(result.x*100,4); Serial.println(" cm");
  Serial.print("Y = "); Serial.print(result.y*100,4); Serial.println(" cm");

  return result;
}

// ------------------------- INVERSE KINEMATICS -------------------------
Angles inverseKine(float x, float y) {
  Angles result;
  result.theta1 = -1; result.theta2 = -1;

  float d2 = x*x + y*y;
  float d = sqrtf(d2);

  if(d > (L1+L2) || d < fabsf(L1-L2)) {
    Serial.println("Target outside workspace.");
    return result;
  }

  float cos_t2 = (d2 - L1*L1 - L2*L2)/(2*L1*L2);
  cos_t2 = constrain(cos_t2,-1,1);
  float t2 = acosf(cos_t2);
  float phi = atan2f(y,x);
  float psi = atan2f(L2*sinf(t2), L1 + L2*cosf(t2));
  float t1 = phi - psi;

  result.theta1 = t1*180.0/PI;
  result.theta2 = t2*180.0/PI;

  Serial.println("Inverse Kinematics Result:");
  Serial.print("Theta1 = "); Serial.println(result.theta1,4);
  Serial.print("Theta2 = "); Serial.println(result.theta2,4);

  return result;
}

// ------------------------- INVERSE DYNAMICS -------------------------
void inversedynamics(float theta1_deg, float theta2_deg) {
  float t1 = theta1_deg*PI/180.0f;
  float t2 = theta2_deg*PI/180.0f;

  float tau1 = M1*G*(L1/2)*cosf(t1) + M2*G*(L1*cosf(t1) + L2/2*cosf(t1+t2));
  float tau2 = M2*G*(L2/2)*cosf(t1+t2);

  Serial.println("Inverse Dynamics (Gravity):");
  Serial.print("Tau1 = "); Serial.print(tau1,4); Serial.println(" Nm");
  Serial.print("Tau2 = "); Serial.print(tau2,4); Serial.println(" Nm");
}

// ------------------------- FORWARD DYNAMICS -------------------------
JointState forwardDynamics(float theta1_deg,float theta2_deg,float tau1,float tau2){
  float t1=theta1_deg*PI/180.0f;
  float t2=theta2_deg*PI/180.0f;

  float I1=M1*L1*L1/12;
  float I2=M2*L2*L2/12;
  float r1=L1/2,r2=L2/2;

  float m11=I1+I2+M1*r1*r1+M2*(L1*L1+r2*r2+2*L1*r2*cosf(t2));
  float m12=I2+M2*(r2*r2+L1*r2*cosf(t2));
  float m21=m12;
  float m22=I2+M2*r2*r2;

  float g1=M1*G*r1*cosf(t1)+M2*G*(L1*cosf(t1)+r2*cosf(t1+t2));
  float g2=M2*G*r2*cosf(t1+t2);

  float rhs1=tau1-g1;
  float rhs2=tau2-g2;
  float det=m11*m22-m12*m21;

  JointState result;
  if(fabsf(det)<1e-6){ 
    result.angularAcceleration1=0; 
    result.angularAcceleration2=0; 
    Serial.println("Singular matrix!"); 
    return result; 
  }

  result.angularAcceleration1=(m22*rhs1 - m12*rhs2)/det;
  result.angularAcceleration2=(-m21*rhs1 + m11*rhs2)/det;

  Serial.println("Forward Dynamics (Angular Acceleration):");
  Serial.print("Joint1 = "); Serial.print(result.angularAcceleration1,4); Serial.println(" rad/s^2");
  Serial.print("Joint2 = "); Serial.print(result.angularAcceleration2,4); Serial.println(" rad/s^2");

  return result;
}
