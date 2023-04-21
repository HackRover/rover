#include <cmath>

double platformBaseOffset;

double comLowVec = 3.22782990761707;
double lowVectors[6] = {comLowVec, comLowVec, comLowVec, comLowVec, comLowVec, comLowVec};

double comHiVec = 3.57708764;
double highVectors[6] = {comHiVec, comHiVec, comHiVec, comHiVec, comHiVec, comHiVec};

double servoArmLength;
double legLength;

double servoArmXRotationOffset;
double servoArmZRotationOffset[6] = {0, 0, 0, 0, 0, 0};

double armLegJointPoint[3] = {0, 0, 0};
double servoArmRotationPoint[3] = {0, 0, 0};
double legPlatformRotationPoint[3] = {0, 0, 0};

double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));

double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double l, double s, double servoArmXRotationOffset, double servoArmLength) {
    double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));
    double L = pow(l, 2) - (pow(s, 2) - pow(servoArmXRotationOffset, 2));
    double M = 2 * a * (legPlatformJoint[2] - servoArmJoint[2]);
    double N = 2 * a * (cos(servoArmZRotationOffset[servo]) * (legPlatformJoint[0] - servoArmJoint[0]) + sin(servoArmZRotationOffset[servo]) * (legPlatformJoint[1] - servoArmJoint[1]));
    return(asin(L / sqrt(pow(M, 2) + pow(N, 2)) - atan(N / M)));
}