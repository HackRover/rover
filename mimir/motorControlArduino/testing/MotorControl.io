/*
 * IBT-2 Motor Control Board driven by Arduino.
 * Created By Houming Ge for HackRover
 * Speed and direction are controlled by a potentiometer attached to analog input 0.
 * One side pin of the potentiometer (either one) to ground; the other side pin to +5V
 * 
 * Connection to the IBT-2 board:
 * IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
 * IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
 * IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
 * IBT-2 pin 8 (GND) to Arduino GND
 * IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
 * 
 * If you change the Pin connect, Change the Those Value as it was responding to:
 *     Motor1_RPWM_Pin , Motor1_FRPM_Pin , Motor2_RPWM_Pin , Motor2_FRPM_Pin
 *     Motor1_RPWM_Digital , Motor1_FPWM_Digital , Motor2_RPWM_Digital , Motor2_FPWM_Digital
 * 
 * Those Pin Should Support the analogWrite function to be able to control the speed of the motor
 * 
 * If the DEBUG LED Pin changes, it needs to be supported digitally Only
 * 
 * 
 * Command type:
 *
 * status :
 * Getting the information for both motors for their speed and direction
 *
 *
 * speedXXXXX :
 * Setting a speed for one motor for the speed it needs to be resigned to
 * The examples for the speed type of command are:
 * speed0-100 - meaning that motor 0 set the speed to -100 % of the motor output in the reverse direction
 *
 * speed112 - meaning that motor 1 sets the speed to 12 % of the motor output in the forward direction
 *
 * Error Code:
 * 0 - Stand for the nothing that will be shown when the command is executed successfully
 * 1 - Serial Port has not connect 
 * 2 - Can not read which type of command from the serial read
 * 3 - Motor speed is not able to be read correctly
 * 
 * 
 * //TODO: adding the conver to product the error input from the user
 * speed1-1333 meaning that motor 2 should be signed to the speed of 1333 % however the MAX value is only limited to 100
 * This will cause the invalid command to be returned
 * //TODO:The  Cover should have a value sign it to help the work in future change motor in
 *
 */
#include <Arduino.h>

/**
 * For Future Coder:
 * method stander
 *  sName: Serial Name - meaning the method will call the Serial class
 *
 * //TODO: Adding a random move for the cyber people to use
 * // TODO: NEED to add the random move?
 */

#define Serial_Speed 9600

#define Motor1_RPWM_Digital false
#define Motor1_RPWM_Pin 5
#define Motor1_FPWM_Digital false
#define Motor1_FRPM_Pin 6

#define Motor2_RPWM_Digital false
#define Motor2_RPWM_Pin 9
#define Motor2_FPWM_Digital false
#define Motor2_FRPM_Pin 10

// Defult Build In LED On the MOST ARDUNO BOARD / ESP32
#define ledPin LED_BUILTIN

// Global Static Values
static int commandDelayMs = 10;
static int errorBitOutPut = 3;

// Global Values
bool bufferString = false;
String inputString = "";

// Gobal Motor Speed
int motor1S = 0;
int motor2S = 0;

void setup()
{

  // TODO: NEED CHECKING if the pinMode is needed or not
  // Document says that analogWrite does not need pinMode <-- Not Test
  // // Motor 1 setup ide
  // pinMode(Motor1_RPWM_Pin, OUTPUT);
  // pinMode(Motor1_FRPM, OUTPUT);

  // // Motor 2 setup ide
  // pinMode(Motor2_RPWM, OUTPUT);
  // pinMode(Motor2_FRPM, OUTPUT);

  // Defult LED pin setup ide
  pinMode(ledPin, OUTPUT);

  Serial.begin(Serial_Speed);

  //If the Serial is not being connected to the other system that supports the Serial
  // Puse the command here
  while (!Serial)
  {
    sErrorStatus("Serial have not been connect", 1);
  }

  // Once the Serial is connected to the other system that supports the Serial
  // loop the current status until the First command has been send from the UART
  while (Serial.available() <= 0)
  {
    sSpeedInfo();
    delay(200);
  }
}

// While loop that will not be stopped unless the Arduino Board is crash
void loop()
{
  // If the buffering is still in read or it was not finished, Puse the loop
  while (!bufferString)
  {
    sEventListerner();
  }

  // Base status information checking
  if (inputString.startsWith("status"))
  {
    sSpeedInfo();
  }
  // Getting speed from the command "speed"
  else if (inputString.startsWith("speed"))
  {
    inputString.remove(0, 5);
    inputString.remove(inputString.length());

    // Checking if the Motor speed and the select are in the digit format
    // It can check if the motor is in the negative value
    if (isStringDigit(inputString))
    {

      // Checking if the motor selected is the Motor ID: 0
      if (inputString.startsWith("0"))
      {
        inputString.remove(0, 1);
        unsigned long messageFromRBPI = strtoul(inputString.c_str(), NULL, 10);
        motor1S = messageFromRBPI;

        // Motor 1 control speed from the RBPI to the setting of the PWM signal
        // forward rotation
        analogWrite(Motor1_RPWM_Pin, 0);
        analogWrite(Motor1_FRPM_Pin, abs(motor1S));

        // reverse rotation
        if (motor1S < 0)
        {
          analogWrite(Motor1_FRPM_Pin, 0);
          analogWrite(Motor1_RPWM_Pin, abs(motor1S));
        }
      }
      // Checking if the motor selected is the Motor ID: 1
      else if (inputString.startsWith("1"))
      {
        inputString.remove(0, 1);
        unsigned long messageFromRBPI = strtoul(inputString.c_str(), NULL, 10);
        motor2S = messageFromRBPI;

        // Motor 2 control speed from the RBPI to the setting of the PWM signal
        // forward rotation
        analogWrite(Motor2_FRPM_Pin, 0);
        analogWrite(Motor2_RPWM_Pin, abs(motor2S));

        // reverse rotation
        if (motor2S < 0)
        {
          analogWrite(Motor2_RPWM_Pin, 0);
          analogWrite(Motor2_FRPM_Pin, abs(motor2S));
        }
      }
      // If they are not both Return the ERROR
      else
      {
        sErrorStatus("Speed Data Error:/n - Can not select the Motor ID:" + inputString, 3);
      }

      // Return the current status of both motor
      sSpeedInfo();
    }
    // If the motor information is not the digit after the word "speed" Return ERROR
    else
    {
      sErrorStatus("Speed Data Error: /n - Motor is not in the digit format", 2);
    }
  }
  // If the command is not else status or speed Return the ERROR invalid command
  else
  {
    sErrorStatus("Inviald Command", 2);
  }
  // reset the command
  bufferString = false;

  // command delay 10ms
  delay(commandDelayMs);
}

// TODO: Need to add the over led
/**
 * Status Checking with the Building board LED flash once
 * This will return the both motor current speed at the same time
 * They will return in this format:
 * Current Speed1: x%, 2: x%
 * Once the LEC flashes once It means the report has been sent back to the UART
 */
void sSpeedInfo()
{
  char buffer[50];
  sprintf(buffer, "Current Speed 1: %d%%, 2: %d%%%", motor1S, motor2S);
  sErrorStatus(buffer, 0);
}

/**
 * Listerner that Listern the UART from the Serial
 * Lister only lister the command that is sent from the Serial that In being in the uart.
 * And they need to be in binary
 * Once the serial receives the command, It will cover they back-to-string form
 * They will be added to the inputString
 * And rest the buffering to true
 * Just Listerner not judgment
 */
void sEventListerner()
{
  inputString = "";
  while (Serial.available() && !bufferString)
  {
    String data = Serial.readString();
    data.trim();
    bufferString = true;
    inputString = data;
  }
}

/**
 * Checking if the String is a pure digit or else
 * If they are not pure digits it will return false
 * else return true
 */
bool isStringDigit(String data)
{
  for (int i = 0; i < inputString.length(); i++)
  {
    if (i == 1 && inputString[i] == '-')
    {
      continue;
    }

    if (!isdigit(inputString[i]))
    {
      return false;
    }
  }
  return true;
}

// TODO: Need to add the over led
/**
 * Report the error to the serial with a message about the type of error.
 * If the serial can not able to be read the method
 * Will make the led flash in the order of the type of error that is responded to.
 */
void sErrorStatus(String message, int typeOfError)
{
  Serial.println(message);
  int binary = decimalToBinary(typeOfError);
  for (int k = 0; k < 3; k++)
  {
    digitalWrite(ledPin, HIGH);

    if (binary % 10 == 1)
      delay(400);
    else
      delay(100);

    binary /= 10;
    digitalWrite(ledPin, LOW);
    delay(400);
  }
  delay(1000);
}

/**
 * Convert the number in decimal to the binary
 * The cover may not be full 32bits It has been a limited size of the array
 * The size of the array has been set by the value errorBitOutPut
 * No Matter what the value will have a value of  1 before all the value inputs start
 * This will limit the num be 2^30
 *
 * Example:
 *  Input 3: Output 1011
 *
 */
int decimalToBinary(int num)
{
  int binary = 10;
  for (int i = errorBitOutPut - 1; i >= 0; i--)
  {
    int mask = (1 << i);
    if (num & mask)
    {
      binary += 1;
      binary *= 10;
    }
    else
    {
      binary *= 10;
    }
  }
  return binary;
}
