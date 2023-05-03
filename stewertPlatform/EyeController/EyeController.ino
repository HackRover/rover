#include <Servo.h>
#include <math.h>

double degreesToRadians(double x);
double radiansToDegrees(double x);

Servo topWest;
Servo topEast;
Servo leftWest;
Servo leftEast;
Servo rightWest;
Servo rightEast;

double testLine;
bool test = true;

long stewartRotationSpin;
long stewartPitch;
long stewartRoll;
long stewartYaw;

double platformBaseOffset[3] = {0, 0, 0};

double platformXRotation = degreesToRadians(10);
double platformYRotation = degreesToRadians(0);
double platformZRotation = degreesToRadians(0);

double prevPlatformXRotation = 0;
double prevPlatformYRotation = 0;
double prevPlatformZRotation = 0;

double lowVectorDistance = 3.22782990761707;
double lowVector[6][3] = {
    {2, 3, 0},
    {3.7, 0.3, 0},
    {1.6, -3.3, 0},
    {-1.6, -3.3, 0},
    {-3.7, 0.3, 0},
    {-2, 3, 0},
};

double highVectorDistance = 3.57708764;
double highVector[6][3] = {
    {0.4, 3.7, 0},
    {3.4, -1.5, 0},
    {3, -2.2, 0},
    {-3, -2.2, 0},
    {-3.4, -1.5, 0},
     {-0.4, 3.7, 0},
};

double translationVector[3] = {0, 0, 5.98632};
// 5.98632
double servoArmLength = 1.2;
double legLength = 6;

double servoArmXRotationOffset = 0;
// double servoArmZRotationOffset[6] = {33.69, 86.31, 153.69, 206.31, 273.69, 326.31};

double armLegJointPoint[3] = {1.225, 0.8, 0};
double servoArmRotationPoint[3] = {0, 0, 0};
double legPlatformRotationPoint[3] = {1.301, -1.056, 5.98632};

int stewertServos[6] = {0, 0, 0, 0, 0, 0};
double effectiveLegLength[6] = {0, 0, 0, 0, 0, 0};
double stewertServoRotation[6] = {0, 0, 0, 0, 0, 0};
double rotationOffset = 0;

double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));

double fullRotationMatrix[3][3];

double degreesToRadians(double x) {
  return(x * (3.14159265358979323846 / 180)); 
}

double radiansToDegrees(double x) {
  return(x * (180 / 3.14159265358979323846)); 
}

double servoArmZRotationOffset[6] = {degreesToRadians(90), degreesToRadians(-90), degreesToRadians(90), degreesToRadians(-90), degreesToRadians(90), degreesToRadians(-90)};


void updateFullRotationMatrix(double x, double y, double z) {
  fullRotationMatrix[0][0] = cos(y) * cos(z);
  fullRotationMatrix[0][1] = -1 * sin(z) * cos(x) + cos(z) * sin(y) * sin(x);
  fullRotationMatrix[0][2] = sin(z) * sin(x) + cos(z) * sin(y) * cos(x);
  fullRotationMatrix[1][0] = sin(z) * sin(y) * cos(z) - cos(x) * sin(z);
  fullRotationMatrix[1][1] = cos(z) * cos(x) + sin(y) * sin(z) * sin(x);
  fullRotationMatrix[1][2] = -1 * cos(z) * sin(x) + sin(y) * sin(z) * cos(x);
  fullRotationMatrix[2][0] = -1 * sin(y);
  fullRotationMatrix[2][1] = cos(y) * sin(x);
  fullRotationMatrix[2][2] = cos(y) * cos(x);
}

double funLegLength[6] = {0, 0, 0, 0, 0, 0};

double updateLegLength(double rotMat[][3], double lowVec[], double highVec[], double transVec[]) {
  double mat[3] = {(rotMat[0][0] + rotMat[0][1] + rotMat[0][2]) * highVec[0], (rotMat[1][0] + rotMat[1][1] + rotMat[1][2]) * highVec[1], (rotMat[2][0] + rotMat[2][1] + rotMat[2][2]) * highVec[2]};
  for (int i = 0; i < 3; i++) {
      mat[i] = mat[i] + transVec[i];
  }
  return(sqrt(pow(mat[0] - lowVec[0], 2) + pow(mat[1] - lowVec[1], 2) + pow(mat[2] - lowVec[2], 2)));
}

double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double s, double servoArmXRotationOffset, double servoArmLength, double ln) {
    double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));
    double L = pow(ln, 2) - (pow(s, 2) + pow(servoArmXRotationOffset, 2));
    double M = 2 * servoArmLength * (legPlatformJoint[2] - servoArmJoint[2]);
    double N = 2 * servoArmLength * (cos(servoArmZRotationOffset[servo] * (legPlatformJoint[0] - servoArmJoint[0])) + sin(servoArmZRotationOffset[servo] * (legPlatformJoint[1] - servoArmJoint[1])));
    Serial.println("69");
    Serial.println(servo);
    Serial.println(L);
    Serial.println(M);
    Serial.println(N);
    return(asin(L / sqrt(pow(M, 2) + pow(N, 2))) - atan(N / M));
}

void stewertUpdateValue() {
    updateFullRotationMatrix(platformXRotation, platformYRotation, platformZRotation);
    for (int i = 0; i < 6; i ++) {
      effectiveLegLength[i] = updateLegLength(fullRotationMatrix, lowVector[i], highVector[i], translationVector);
      stewertServoRotation[i] = stewertGetServoRotation(i, armLegJointPoint, servoArmRotationPoint, legPlatformRotationPoint, legLength, servoArmXRotationOffset, servoArmLength, effectiveLegLength[i]);
      //Serial.println(stewertServoRotation[i]);
    }
}

void stewertServoUpdate() {
  topWest.write((stewertServoRotation[0] + -0.15) * 180);
  topEast.write(180 - (stewertServoRotation[1] + 0.15) * 180 );
  leftWest.write((stewertServoRotation[2] + -0.15) * 180);
  leftEast.write(180 - (stewertServoRotation[3] + 0.15) * 180 );
  rightWest.write((stewertServoRotation[4] + -0.15) * 180);
  rightEast.write(180 - (stewertServoRotation[5] + 0.15) * 180 );
  
}

void getPosition(bool test) {
  if(test == true) {
    return;
  }

  return;
}

void setup() {

  Serial.begin(9600);

  topWest.attach(2);
  topEast.attach(3);
  leftWest.attach(4);
  leftEast.attach(5);
  rightWest.attach(6);
  rightEast.attach(7);

  stewertUpdateValue();
  stewertServoUpdate();

  testLine = stewertServoRotation[0];
  Serial.println("345");
  Serial.println(testLine);

  for (int i = 0; i < 6; i ++) {
    testLine = stewertServoRotation[i];
    Serial.println(i);
    Serial.println(effectiveLegLength[i]);
    Serial.println(testLine);
  }
  
}

void loop() {

  // stewertUpdateValue();
  // stewertServoUpdate();

}
