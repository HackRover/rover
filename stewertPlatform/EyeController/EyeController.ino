#include <Servo.h>
#include <math.h>

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

double platformXRotation = 45;
double platformYRotation = 45;
double platformZRotation = 45;

double prevPlatformXRotation = 0;
double prevPlatformYRotation = 0;
double prevPlatformZRotation = 0;

double lowVectorDistance = 3.22782990761707;
double lowVector[6][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};

double highVectorDistance = 3.57708764;
double highVector[6][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};

double translationVector[3] = {0, 0, 5.98632};

double servoArmLength = 1.2;
double legLength = 6;

double servoArmXRotationOffset = 0;
double servoArmZRotationOffset[6] = {0, 0, 0, 0, 0, 0};

double armLegJointPoint[3] = {1.225, 0.8, 0};
double servoArmRotationPoint[3] = {0, 0, 0};
double legPlatformRotationPoint[3] = {1.301, -1.056, 5.98632};

int stewertServos[6] = {0, 0, 0, 0, 0, 0};
double effectiveLegLength[6] = {0, 0, 0, 0, 0, 0};
double stewertServoRotation[6] = {0, 0, 0, 0, 0, 0};
double rotationOffset = 0;

double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));

double fullRotationMatrix[3][3];

void updateFullRotationMatrix(double x, double y, double z) {
    double rotationMatrix[3][3] = {
        {cos(y) * cos(z), -1 * sin(z) * cos(x) + cos(z) * sin(y) * sin(x), sin(z) * sin(x) + cos(z) * sin(y) * cos(x)},
        {sin(z) * sin(y) * cos(z) - cos(x) * sin(z), cos(z) * cos(x) + sin(y) * sin(z) * sin(x), -1 * cos(z) * sin(x) + sin(y) * sin(z) * cos(x)},
        { -1 * sin(y), cos(y) * sin(x), cos(y) * cos(x)}
    };
}

double funLegLength[6] = {0, 0, 0, 0, 0, 0};

double updateLegLength(double rotMat[][3], double lowVec[], double highVec[], double transVec[]) {
    double mat[3] = {rotMat[0][0] * rotMat[0][1] * rotMat[0][2] + highVec[0], rotMat[1][0] * rotMat[1][1] * rotMat[1][2] + highVec[1], rotMat[2][0] * rotMat[2][1] * rotMat[2][2] + highVec[2]};
    for (int i = 0; i < 3, i++;) {
        mat[i] = mat[i] + transVec[i];
    }
    return(sqrt(pow(mat[0] - lowVec[0], 2) + pow(mat[1] - lowVec[1], 2) + pow(mat[2] - lowVec[2], 2)));
}

double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double s, double servoArmXRotationOffset, double servoArmLength, double ln) {
    double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));
    double L = ln - (pow(s, 2) - pow(servoArmXRotationOffset, 2));
    double M = 2 * a * (legPlatformJoint[2] - servoArmJoint[2]);
    double N = 2 * a * (cos(servoArmZRotationOffset[servo]) * (legPlatformJoint[0] - servoArmJoint[0]) + sin(servoArmZRotationOffset[servo]) * (legPlatformJoint[1] - servoArmJoint[1]));
    return(asin(L / sqrt(pow(M, 2) + pow(N, 2)) - atan(N / M)));
}

void stewertUpdateValue() {
    

    updateFullRotationMatrix(platformXRotation, platformYRotation, platformZRotation);
    for (int i = 0; 6 < 6, i ++;) {
      effectiveLegLength[6] = updateLegLength(fullRotationMatrix, lowVector[i], highVector[i], translationVector);
      stewertServoRotation[i] = stewertGetServoRotation(i, servoArmRotationPoint, legPlatformRotationPoint, legPlatformRotationPoint, legLength, servoArmXRotationOffset, servoArmLength, effectiveLegLength[i]);
    }
}

void stewertServoUpdate() {
  topWest.write(stewertServoRotation[0] + rotationOffset);
  topEast.write(stewertServoRotation[1] + rotationOffset);
  leftWest.write(stewertServoRotation[2] + rotationOffset);
  leftEast.write(stewertServoRotation[3] + rotationOffset);
  rightWest.write(stewertServoRotation[4] + rotationOffset);
  rightEast.write(stewertServoRotation[5] + rotationOffset);
  
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
  Serial.println("1");
  Serial.println(testLine);

}

void loop() {

  // stewertUpdateValue();
  // stewertServoUpdate();

  // testLine = stewertServoRotation[0];
  // Serial.println(testLine);


  // platformXRotation ++;

}
