//Using math from https://cdn.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf
//Goal with controlling a stewert platform with a camera
//All calculations done in radians
//HackRover 2023
//Code done by: Raien Elliston
//Second#9713
#include <Servo.h>
#include <math.h>

//Declaring functions
double degreesToRadians(double x);
double radiansToDegrees(double x);
void updateFullRotationMatrix(double x, double y, double z);
double updateLegLength(double rotMat[][3], double lowVec[], double highVec[], double transVec[]);
double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double s, double servoArmXRotationOffset, double servoArmLength, double ln);
void stewertUpdateValue();
void stewertServoUpdate();
bool getPosition(bool test);

//Servo object declaration
Servo topWest;
Servo topEast;
Servo leftWest;
Servo leftEast;
Servo rightWest;
Servo rightEast;

//If testing variables. Keep updating true, only change test
bool test = true;
bool updating = true;
double testLine;

//Desired position and rotation of the platform
long stewartRotationSpin;
long stewartPitch;
long stewartRoll;
long stewartYaw;

//Desired position and rotation of the platform
double platformXRotation = degreesToRadians(10);
double platformYRotation = degreesToRadians(0);
double platformZRotation = degreesToRadians(0);

//Lower vector and distance from the center of the platform to servos to servo arm rotation point
double lowVectorDistance = 2.7;
double lowVector[6][3] = {
    {1.3, 2.3, 1.4},
    {2.7, 0, 1.4},
    {1.3, -2.3, 1.4},
    {-1.3, -2.3, 1.4},
    {-2.7, 0, 1.4},
    {-1.3, 2.3, 1.4},
};

//Higher vector and distance from the center of the platform to leg rotation point
double highVectorDistance = 2.154067;
double highVector[6][3] = {
    {0.4, 2.1, 4.468745},
    {2, -0.8, 4.468745},
    {1.7, -1.4, 4.468745},
    {-1.7, -1.4, 4.468745},
    {-2, -0.8, 4.468745},
    {-0.4, 2.1, 4.468745},
};

//Traslation vector; the offset of the high platform to the lower platform that stays constant
double translationVector[3] = {0, 0, 4};

//servo arm and leg length
//leg length needs remeasuring and to be fixed
double servoArmLength = 1.2;
double legLength = 3.14;

//vertical offset of the servo arms
double servoArmXRotationOffset = 0;

//rotational offset of the servo arms reletive to the servos itself
//iteration 1
double servoArmZRotationOffset[6] = {0.38051, 0.38051, 0.38051, 0.38051, 0.38051, 0.38051};

//postition of all rotation points of the leg/arm reletive to it's connected servo
double armLegJointPoint[3] = {1.2, 0.5, 0};
double servoArmRotationPoint[3] = {0, 0, 0};
double legPlatformRotationPoint[3] = {1.035103, 0.0871, 3.068747};

//variables storing the effective leg length of each leg on the stewert platform and the rotation of the servo
double effectiveLegLength[6] = {0, 0, 0, 0, 0, 0};
double stewertServoRotation[6] = {0, 0, 0, 0, 0, 0};

double rotationOffset = 0;

//declaring variables for the full rotation matrix
double fullRotationMatrix[3][3];

//turns degrees into radians
double degreesToRadians(double x) {
  return(x * (3.14159265358979323846 / 180)); 
}

//turns radians into degrees
double radiansToDegrees(double x) {
  return(x * (180 / 3.14159265358979323846)); 
}

//updates the full rotation matrix
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

//updates the effective leg length of each leg
double updateLegLength(double rotMat[][3], double lowVec[], double highVec[], double transVec[]) {
  double mat[3] = {(rotMat[0][0] * highVec[0]) + (rotMat[0][1] * highVec[1]) + (rotMat[0][2] * highVec[2]), (rotMat[1][0] * highVec[0]) + (rotMat[1][1] * highVec[1]) + (rotMat[1][2] * highVec[2]), (rotMat[2][0] * highVec[0]) + (rotMat[2][1] * highVec[1]) + (rotMat[2][2] * highVec[2])};
  for (int i = 0; i < 3; i++) {
      mat[i] = mat[i] + transVec[i];
  }
  return(sqrt(pow(mat[0] - lowVec[0], 2) + pow(mat[1] - lowVec[1], 2) + pow(mat[2] - lowVec[2], 2)));
}

//updates the rotation of the servo based on the effective leg length
double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double s, double servoArmXRotationOffset, double servoArmLength, double ln) {
    double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));
    double L = pow(ln, 2) - (pow(s, 2) + pow(servoArmXRotationOffset, 2));
    double M = 2 * servoArmLength * (legPlatformJoint[2] - servoArmJoint[2]);
    double N = 2 * servoArmLength * ((cos(servoArmZRotationOffset[servo]) * (legPlatformJoint[0] - servoArmJoint[0])) + (sin(servoArmZRotationOffset[servo]) * (legPlatformJoint[1] - servoArmJoint[1])));
    Serial.println("69");
    Serial.println(servo);
    Serial.println(L);
    Serial.println(M);
    Serial.println(N);
    return(asin(L / sqrt(pow(M, 2) + pow(N, 2))) - atan(N / M));
}

//updates the effective leg length and the rotation of the servo
void stewertUpdateValue() {
    updateFullRotationMatrix(platformXRotation, platformYRotation, platformZRotation);
    for (int i = 0; i < 6; i ++) {
      effectiveLegLength[i] = updateLegLength(fullRotationMatrix, lowVector[i], highVector[i], translationVector);
      stewertServoRotation[i] = stewertGetServoRotation(i, armLegJointPoint, servoArmRotationPoint, legPlatformRotationPoint, legLength, servoArmXRotationOffset, servoArmLength, effectiveLegLength[i]);
      //Serial.println(stewertServoRotation[i]);
    }
}

//updates the rotation being sent to the servos
//needs to be in degrees
//inverting the numbers isn't needed, just makes it look nicer
void stewertServoUpdate() {
  topWest.write( radiansToDegrees(stewertServoRotation[5]) * 1 + 90);
  topEast.write(radiansToDegrees(stewertServoRotation[0]) * 1 + 90 );
  leftWest.write(radiansToDegrees( stewertServoRotation[1]) * 1 + 90);
  leftEast.write(radiansToDegrees(stewertServoRotation[2]) * 1 + 90);
  rightWest.write(radiansToDegrees( stewertServoRotation[3]) * 1 + 90);
  rightEast.write(radiansToDegrees(stewertServoRotation[4]) * 1 + 90);
}

//gets the new position of the platform
bool getPosition(bool is_test) {
  //runs update once to insure consol isn't spammed
  if(updating == false) {
    return false;
  }
  if(is_test == true) {
    updating = false;
  }
  //put code to get new posistion here
  return true;
}

void setup() {

  //start serial for testing
  Serial.begin(9600);

  //attach servo objects to different pins on the board
  topWest.attach(2);
  topEast.attach(3);
  leftWest.attach(4);
  leftEast.attach(5);
  rightWest.attach(6);
  rightEast.attach(7);

  stewertUpdateValue();

  //prints test numbers
  for (int i = 0; i < 6; i ++) {
    testLine = radiansToDegrees(stewertServoRotation[i]);
    Serial.println(i);
    Serial.println(effectiveLegLength[i]);
    Serial.println(testLine);
  }
  
}

void loop() {

  //checks if new positions are avalible
  if(getPosition(test) == true) {
    //update the position
    stewertUpdateValue();
  }

  stewertServoUpdate();

}
