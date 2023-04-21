//Using math from https://cdn.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf
// Need to implament equation 1 and 3 from it
#include <cmath>

double platformBaseOffset;

double platformXRotation;
double platformYRotation;
double platformZRotation;

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

double fullRotationMatrix[3][3];

void updateFullRotationMatrix(double x, double y, double z) {
    
    double rotationMatrix[3][3] = {
        {cos(y) * cos(z), -1 * sin(z) * cos(x) + cos(z) * sin(y) * sin(x), sin(z) * sin(x) + cos(z) * sin(y) * cos(x)},
        {sin(z) * sin(y) * cos(z) - cos(x) * sin(z), cos(z) * cos(x) + sin(y) * sin(z) * sin(x), -1 * cos(z) * sin(x) + sin(y) * sin(z) * cos(x)},
        { -1 * sin(y), cos(y) * sin(x), cos(y) * cos(x)}
    };
}

double stewertGetServoRotation(int servo, double armLegJoint[], double servoArmJoint[], double legPlatformJoint[], double s, double servoArmXRotationOffset, double servoArmLength) {
    double a = sqrt(pow(armLegJointPoint[0] - servoArmRotationPoint[0], 2) + pow(armLegJointPoint[1] - servoArmRotationPoint[1], 2) + pow(armLegJointPoint[2] - servoArmRotationPoint[2], 2));
    double ln = pow(armLegJoint[0] - servoArmJoint[0], 2) + pow(armLegJoint[1] - servoArmJoint[1], 2) + pow(armLegJoint[2] - servoArmJoint[2], 2);
    double L = ln - (pow(s, 2) - pow(servoArmXRotationOffset, 2));
    double M = 2 * a * (legPlatformJoint[2] - servoArmJoint[2]);
    double N = 2 * a * (cos(servoArmZRotationOffset[servo]) * (legPlatformJoint[0] - servoArmJoint[0]) + sin(servoArmZRotationOffset[servo]) * (legPlatformJoint[1] - servoArmJoint[1]));
    return(asin(L / sqrt(pow(M, 2) + pow(N, 2)) - atan(N / M)));
}
