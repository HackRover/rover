/*Created by Justin Heinzig, 8/15/23.
This program serves as a simple demonstration of functionality for Nunchuck controllers.
It should help students learn the basics of programming with controllers,
and feedback from built in IDE tools.
*/

//Common library for integrating Nintendo or adjacent controllers. 
//Library depends on Wire.h (no need to call), therefore it's not compatible with Due.
//Found here: https://github.com/madhephaestus/WiiChuck; unfortunately inaccurate documentation.
#include <WiiChuck.h> 

//Create an object of type accessory named nunchuck.
Accessory nunchuck;
//Creating variables for filtering. History is required so created globally.
uint8_t posXF;
uint8_t posYF;

void setup() {
	Serial.begin(9600);
  //Start the object (named nunchuck), and make sure it's type is properly set.
	nunchuck.begin();
	nunchuck.type = NUNCHUCK;

  //Variables to filter.
}

void loop() {
  /*Nunchuck is powered by 3.3V and GND pins, and communicates with SDA and SCL pins.
  No additional setup is needed by the user, since WiiChuck.h handles it.
  Read data at the start of the loop; necessary command.*/
  nunchuck.readData();   

  //----------------------------------------------------
  /*Store data in a variety of variables (mapped and unmapped), for use later.
  Library author claims data is stored/can be access from array; but, it's faulty.*/
  //Values are premapped 0 - 255.
  uint8_t posX = nunchuck.getJoyX();
  uint8_t posY = nunchuck.getJoyY();

  uint8_t accelX = map(nunchuck.getAccelX(),0,1024,0,255);
  uint8_t accelY = map(nunchuck.getAccelY(),0,1024,0,255);
  uint8_t accelZ = map(nunchuck.getAccelZ(),0,1024,0,255);
  //uint8_t umAccelX = nunchuck.getAccelX(); //Demonstrable difference to mapped.
  //uint8_t umAccelY = nunchuck.getAccelY();
  //uint8_t umAccelZ = nunchuck.getAccelZ();

  uint8_t roll = map(nunchuck.getRollAngle(),0,1024,0,255);
  uint8_t pitch = map(nunchuck.getPitchAngle(),0,1024,0,255);
  //uint8_t umRoll = nunchuck.getRollAngle();
  //uint8_t umPitch = nunchuck.getPitchPitch();

  uint8_t ZButton = nunchuck.getButtonZ()?255:0;
  uint8_t CButton = nunchuck.getButtonC()?255:0;  


  //----------------------------------------------------
  //Some print statements for Monitoring.
  /*
  Serial.print("The X and Y stick positions are: ");
  Serial.print(posX); Serial.print(", ");
  Serial.println(posY);
  
  Serial.print("The X, Y, and Z accelerations are: ");
  Serial.print(accelX); Serial.print(", ");
  Serial.print(accelY); Serial.print(", ");
  Serial.println(accelZ); 

  Serial.print("The roll and pitch are: ");
  Serial.print(roll); Serial.print(", ");
  Serial.println(pitch);

  if(ZButton){
    Serial.println("Z is pressed.");    
  }
  if(CButton){
    Serial.println("C is pressed.");
  }
  */
  

  //----------------------------------------------------
  //Some basic plotting without modification. Note that the Serial plotter still uses Serial.print.
  //Serial.print("Baseline:"); Serial.print(0); Serial.print(" "); //Baseline helps to turn off autoscale.
  //Serial.print("X_Position:"); Serial.print(posX); Serial.print(" "); //Variable names cannot have spaces.
  //Serial.print("Y_Position:"); Serial.println(posY); //No " " delimeter needed for last value.

  //Serial.print("Baseline:"); Serial.print(0); Serial.print(" ");
  //Serial.print("Accel_X:"); Serial.print(accelX); Serial.print(" "); 
  //Serial.print("Accel_Y:"); Serial.print(accelY); Serial.print(" "); 
  //Serial.print("Accel_Z:"); Serial.println(accelZ);

  /*Important pitch and roll note. 
  For roll, 255 is equilibrium, 45 is upside down, 0-45 is tilt to the right, and 255-210 is tilt to the left.
  For pitch, 255 is equilibrium, 45 is upside down, 0-45 is tilt up, and 255-210 is tilt down.*/
  //Serial.print("Baseline:"); Serial.print(0); Serial.print(" ");
  //Serial.print("Pitch:"); Serial.print(pitch); Serial.print(" ");
  //Serial.print("Roll:"); Serial.println(roll);
  //delay(50); //Advised to use a short delay for good resolution.


  //----------------------------------------------------
  //Filtering and plotting example.
  //posXF = (0.6 * posXF) + (0.4 * posX); //Low pass filter from this video: https://www.youtube.com/watch?v=1e_ZB8p5n6s
  //posYF = (0.6 * posYF) + (0.4 * posY);
  //Serial.print("X_Filtered:"); Serial.print(posXF); Serial.print(" ");
  //Serial.print("Y_Filtered:"); Serial.println(posYF);

  //----------------------------------------------------
  //Custom mapping. The numbers in the map I got from the serial monitor; lower bounds appeared to be 25 or 32.
  float xMultiplier = mapFloats(posX, 25, 226, -1, 1);
  float yMultiplier = mapFloats(posY, 32, 226, -1, 1);
  Serial.print("X_Multiplier:"); Serial.print(xMultiplier); Serial.print(" ");
  Serial.print("Y_Multiplier:"); Serial.println(yMultiplier);
  
  delay(50);
}

float mapFloats(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}
