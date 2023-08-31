/*Created by Justin Heinzig, 8/16/23.
This program serves as the controls for the Stewart platform.
See the associated notes (Stewart Platform Iteration 3) for a better explanation of the math.
Program maintains 3 quaternions (X, Y, and Z rotations), and 1 transformation vector.
Motion from the joystick changes values of the quaternionns and trarnsformation vector.
Button presses select different axes to edit. 
10 times a second the linkages and then servo angles are calculated and updated.
The servo and controller are constantly updating.
*/

#include <Wire.h>
#include <WiiChuck.h>
#include <Math.h>
#include <Servo.h>
/*For safety, assume that the servos must be arrayed counter clockwise 
from the directionn of point of the LiDAR. So servo 1 is the top left, 
servo 2 is left most, etc.*/
const int servoPins[] = {4, 5, 6, 7, 8, 9}; //Pins for the 6 servos
Servo servos[6];

const float linSpeed = 0.7; //Maximum linear speed in inches per second.
const float rotSpeed = 0.6; //Maximum rotational speed in inches per second.

const float realLinkLength = 3.5; //Linkage length in inches.
const float hornLength = 1.325; //Horn length in inches.
const int servoOrientAng[6] = {30, 270, 150, 30, 270, 150}; //All horns assumed to 'point' clockwise.

Accessory nunchuck;
const float deadZoneThreshold = 0.1; //Deadzone threshold for joystick.

//Quaternion and Vector structures
struct Quaternion {
  float n;
  float i;
  float j;
  float k;
  float w; //Actual angle.

  void print() const {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", i: "); Serial.print(i);
    Serial.print(", j: "); Serial.print(j);
    Serial.print(", k: "); Serial.print(k);
    Serial.print(", w: "); Serial.println(w);
  }
};
struct Vector {
  float x;
  float y;
  float z;

  void print() const {
    Serial.print("x: "); Serial.print(x);
    Serial.print(", y: "); Serial.print(y);
    Serial.print(", z: "); Serial.println(z);
  }
  Vector operator+(const Vector &other) const {
    Vector result;
    result.x = x + other.x;
    result.y = y + other.y;
    result.z = z + other.z;
    return result;
  }
  Vector operator-(const Vector &other) const {
    Vector result;
    result.x = x - other.x;
    result.y = y - other.y;
    result.z = z - other.z;
    return result;
  }
};

Quaternion quatX, quatY, quatZ;
//Vector transformVec; //Temporary translational movements.  //POSSIBLY REMOVE THIS
Vector transformVec; //Permanent transform state vector.
Vector transformVecClamp; //Clamping on transform vector.
float transformVecClampLower; //Bottom of z.
Vector rotationVec; //Permanent rotation state vector.
Vector rotationVecClamp; //Clamping on rotation vector.
const Vector baseVec[6]; //Vectors of linkage ends on base.
Vector platVec[6]; //Vectors of linkage ends on moving platform.
Vector linkageVec[6]; //Vectors describing linkages.
float servoAngles[6]; //Converted into degrees.
float xAdjSum = 0, yAdjSum = 0; //Global adjustment sums record controller inputs.
unsigned long lastOutputTime10ms = 0;  //Used for  timing  purposes.  
unsigned long lastOutputTime100ms = 0;
unsigned long lastOutputTime1000ms = 0; 

//Functions defined at the bottom of the code.
Quaternion quatCalcZ(Quaternion &example);
Quaternion quatCalcY(Quaternion &example);
Quaternion quatCalcX(Quaternion &example);
Vector conjugation(const Quaternion &quat, const Vector &vec);
float calcAngles(const Vector &linkage, const int &servoOrientAng);
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max);


void setup() {
    //Servo setup---------------------
    for (int i = 0; i < 6; i++) {
      servos[i].attach(servoPins[i]); // Attach each servo to its respective pin
    }
    
    //Nunchuck setup---------------------
    Serial.begin(9600);
	  nunchuck.begin();
	  nunchuck.type = NUNCHUCK;
    
    //Stewart platform geometry default setup. Convention is CC---------------------
    //Constant, base vectors. Base reference frame.
    baseVec[0] = {-1.3425, 2.3250, 0};
    baseVec[1] = {-2.685, 0, 0};
    baseVec[2] = {-1.3425, -2.3250, 0};
    baseVec[3] = {1.3425, -2.3250, 0};
    baseVec[4] = {2.685, 0, 0};
    baseVec[5] = {1.3425, 2.3250, 0};
    //Constant, platform vectors. Platform reference frame. 
    platVec[0] = {-1.64, 1.35, 0};
    platVec[1] = {-1.99, 0.74, 0};
    platVec[2] = {-0.36, -2.1, 0};
    platVec[3] = {0.36, -2.1, 0};
    platVec[4] = {1.64, 1.35, 0};
    platVec[5] = {1.99, 0.74, 0};
    //Setting servo angles in degrees. Default position. POSSIBLY CHHANGE THESEEE
    servos[0].write(180);
    servos[1].write(0);
    servos[2].write(180);
    servos[3].write(0);
    servos[4].write(180);
    servos[5].write(0);
    delay(3000);
    //Establishing rotation initial state.
    rotationVec.x = 0;
    rotationVec.y = 0;
    rotationVec.z = 0;
    transformVec.x = 0;  
    transformVec.y = 0; 
    transformVec.z = 3; //Vertical distance between platform and ground.
    //Set state boundaries here.
    rotationVecClamp.x = 1; //Clamp angles in radians.
    rotationVecClamp.y = 1;
    rotationVecClamp.z = 1;
    transformVecClamp.x = 1; //Clamp translation distance in inches.
    transformVecClamp.y = 1;
    transformVecClamp.z = 4.125; //Run the program to printout servos alone. This value represents maximum vertical extension, so all servos shouhld read 90 degrees.
    transformVecClampLower = 0.6; //This value isn't realistic. Set it so the lowest position equals roughly 0 degrees on servos.
}

void loop() {
    //Poll the nunchuck for data.
    nunchuck.readData(); 
    uint8_t posX = nunchuck.getJoyX();
    uint8_t posY = nunchuck.getJoyY();
    float xMultiplier = mapFloats(posX, 25, 226, -1, 1);
    float yMultiplier = mapFloats(posY, 32, 226, -1, 1);
    // Apply dead zone filter to joystick inputs
    if (abs(xMultiplier) < deadZoneThreshold) {
        xMultiplier = 0.0;
    }
    if (abs(yMultiplier) < deadZoneThreshold) {
        yMultiplier = 0.0;
    }
    //Serial.print("xMult:"); Serial.print(xMultiplier); Serial.print(" ");
    //Serial.print("yMult:"); Serial.println(yMultiplier);
    bool ZPressed = nunchuck.getButtonZ();
    bool CPressed = nunchuck.getButtonC();

    //Determines the max movement allowed per 100ms.
    float maxMovement = linSpeed/100;
    xAdjSum += (maxMovement * xMultiplier);
    yAdjSum += (maxMovement * yMultiplier);
    //Serial.print("xAdjSum:"); Serial.print(xAdjSum); Serial.print(" ");
    //Serial.print("yAdjSum:"); Serial.println(yAdjSum);
    
    if(ZPressed && (millis() - lastOutputTime10ms >= 10)){ //Apply adjustments to the Z axis.
      if(abs(transformVec.z + yAdjSum) <= abs(transformVecClamp.z) && abs(transformVec.z + yAdjSum) >= transformVecClampLower){
        transformVec.z += yAdjSum;
        //Serial.print("Transform_State_Z:"); Serial.print(transformVec.z); Serial.print(" ");          
        }
      if(abs(rotationVec.z + xAdjSum) <= abs(rotationVecClamp.z)){
        rotationVec.z += xAdjSum * (rotSpeed/linSpeed);
        //Serial.print("Rotation_State_Z:"); Serial.println(rotationVec.z);
        quatZ.w += xAdjSum * (rotSpeed/linSpeed);
      }
      xAdjSum = 0; yAdjSum = 0;
      lastOutputTime10ms = millis();
    }
    else if(CPressed && (millis() - lastOutputTime10ms >= 10)){ //Apply adjustments to the Y axis.
      if(abs(transformVec.y + yAdjSum) <= abs(transformVecClamp.y)){
        transformVec.y += yAdjSum;
        //Serial.print("Transform_State_Y:"); Serial.print(transformVec.y); Serial.print(" ");   
      }
      if(abs(rotationVec.y + xAdjSum) <= abs(rotationVecClamp.y)){
        rotationVec.y += xAdjSum * (rotSpeed/linSpeed);
        //Serial.print("Rotation_State_Y:"); Serial.println(rotationVec.y);
        quatY.w += xAdjSum * (rotSpeed/linSpeed);
      }
      xAdjSum = 0; yAdjSum = 0;
      lastOutputTime10ms = millis();
    }
    else if((millis() - lastOutputTime10ms >= 10)){
      if(abs(transformVec.x + xAdjSum) <= abs(transformVecClamp.x)){
        transformVec.x += xAdjSum;
        //Serial.print("Transform_State_X:"); Serial.print(transformVec.x); Serial.print(" ");  
      }
      if(abs(rotationVec.x + yAdjSum) <= abs(rotationVecClamp.x)){
        rotationVec.x += yAdjSum * (rotSpeed/linSpeed);
        //Serial.print("Rotation_State_X:"); Serial.println(rotationVec.x);          
        quatX.w += yAdjSum * (rotSpeed/linSpeed);
      }
      xAdjSum = 0; yAdjSum = 0;
      lastOutputTime10ms = millis();
    }
    

    if(millis() - lastOutputTime100ms >= 50){//Checks every 100 milliseconds.
      //Update the quaternions.
      quatX = quatCalcX(quatX);
      quatY = quatCalcY(quatY);
      quatZ = quatCalcZ(quatZ);
      
      //Conjugate and rotate the platform vectors.
      Vector intermedVec[6]; 
      for(int i = 0; i < 6; i++){
        intermedVec[i] = conjugation(quatX, platVec[i]); 
        intermedVec[i] = conjugation(quatY, intermedVec[i]);
        intermedVec[i] = conjugation(quatZ, intermedVec[i]);
      }
      
      //Perform the sum to determine linkage conditions.
      for(int i = 0; i < 6; i++){
        linkageVec[i] = transformVec + intermedVec[i] - baseVec[i];
        //Serial.print("Linkage "); Serial.print(i); Serial.print(": "); linkageVec[i].print();
      }
            
      //Calculate the servo angles from linkage data.
      for(int i = 0; i < 6; i++){
        servoAngles[i] = calcAngles(linkageVec[i], servoOrientAng[i]);
      }
      quatZ.w = 0; quatZ.i = 0; quatZ.j = 0; quatZ.k = 0; //Zeroing out the quaternions. 
      quatY.w = 0; quatY.i = 0; quatY.j = 0; quatY.k = 0;
      quatX.w = 0; quatX.i = 0; quatX.j = 0; quatX.k = 0;

      //Write the servo angles  to the servos.
      for(int i = 0; i < 6; i++){
        servos[i].write(servoAngles[i]);
      }
      lastOutputTime100ms = millis();
    }

    if(millis() - lastOutputTime1000ms >= 1000){
      for(int i = 0; i < 6; i++){
        Serial.print("Servo_Angle "); Serial.print(i); Serial.print(": "); Serial.println(servoAngles[i]);        
      }
      lastOutputTime1000ms = millis();      
    }
}

float mapFloats(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Accepts an unprocessed quaternion (angle only) and calculates it.
Quaternion quatCalcZ(Quaternion &example){ 
  Quaternion returnQuat;
  returnQuat.n = cos(example.w/2);
  returnQuat.k = sin(example.w/2);  
  return returnQuat;
}
Quaternion quatCalcY(Quaternion &example){  
  Quaternion returnQuat;
  returnQuat.n = cos(example.w/2);
  returnQuat.j = sin(example.w/2);  
  return returnQuat;
}
Quaternion quatCalcX(Quaternion &example){
  Quaternion returnQuat;
  returnQuat.n = cos(example.w/2);
  returnQuat.i = sin(example.w/2);  
  return returnQuat;
}

//Accepts a modifying quaternion and input vector and performs conjugation.
Vector conjugation(const Quaternion &quat, const Vector &vec) {  
  //Normalizing the quaternion.
  Quaternion nQuat = quat; //The normalized quaternion.
  float magnitude = sqrt(nQuat.n * nQuat.n + nQuat.i * nQuat.i + nQuat.j * nQuat.j + nQuat.k * nQuat.k);
  nQuat.n /= magnitude;
  nQuat.i /= magnitude;
  nQuat.j /= magnitude;
  nQuat.k /= magnitude;
  
  //Getting the conjugate quaternion. Already normalized.
  Quaternion cQuat = {nQuat.n, -nQuat.i, -nQuat.j, -nQuat.k};

  //Performing the premultiplication (rotation): R * p = nQuat * vec. Note that I imagine the vector p as a quaternion with n = 0, and only the i, j, and k components of the quaternion result are mapped to x, y, and z of the vector. 
  Quaternion rVec = {
    (0 - nQuat.i * vec.x - nQuat.j  * vec.y - nQuat.k * vec.z),
    (nQuat.n * vec.x + 0 + nQuat.j * vec.z -  nQuat.k * vec.y),
    (nQuat.n * vec.y - nQuat.i * vec.z + 0 + nQuat.k * vec.x),
    (nQuat.n * vec.z + nQuat.i * vec.y - nQuat.j  * vec.x + 0)
  };

  //Performing the postmultiplication (conjugation): rotatedVec (rVec) * R^-1 = rotatedVec * cQuat.
  Vector conjugatedVec = {
    (0 + rVec.i * cQuat.n + rVec.j * cQuat.k - rVec.k * cQuat.j),
    (0 - rVec.i * cQuat.k + rVec.j * cQuat.n + rVec.k * cQuat.i),
    (0 + rVec.i * cQuat.j - rVec.j * cQuat.i + rVec.k * cQuat.n)
  };

  return conjugatedVec;
}

//Accepts a linkage vector and servo orientation angle and calculates servo angles.
float calcAngles(const Vector &linkage, const int &servoOrientAng){
  //Calculate the constants.
  float e, f, g;
  e = 2 * hornLength * linkage.z;
  f = 2 * hornLength * ((cos(radians(servoOrientAng)) * linkage.x) + (sin(radians(servoOrientAng)) * linkage.y)); //Converting servo orientation angles from degrees to radians.
  //I just broke up g into multiple steps for easier reading and debugging.
  float equLinkLengthSquared = abs(linkage.x * linkage.x) + abs(linkage.y * linkage.y) + abs(linkage.z * linkage.z);
  float realLinkLengthSquared = abs(realLinkLength * realLinkLength);
  float hornLengthSquared = abs(hornLength * hornLength); 
  g = equLinkLengthSquared - (realLinkLengthSquared - hornLengthSquared);

  //Calculate the angles from the governing equation.
  float angInRad; 
  float constantVal = g/(sqrt(e * e + f * f));   
  float arcSine = asin(constantVal);
  float arcTan2 = atan2(f, e);
  angInRad = arcSine - arcTan2;   
  return degrees(angInRad);  //Converted back to degrees for publishing to servo.
}


