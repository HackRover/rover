/*
IBT-2 Motor Control Board driven by Arduino.
Created By Houming Ge for HackRover

Speed and direction controlled by a potentiometer attached to analog input 0.
One side pin of the potentiometer (either one) to ground; the other side pin to +5V

Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
IBT-2 pin 8 (GND) to Arduino GND
IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected

If change the Pin connect, Change the Those Value as it was respond to:
    Motor1_RPWM , Motor1_FRPM , Motor2_RPWM , Motor2_FRPM

Those Pin Should Support the analogWrite function to be able to control the speed of the mortor

If the DEBUG LED Pin change, it need to be support dig
*/

/**
 * For Furture Coder:
 * method stander 
 *  sName: Serial Name - meanign the method will call the Serial class
 */


// TODO: Adding a map conver to conver the 0-100 to 0-255 base on the rated of cover
// TODO: Adding a random move for the cyber people to use
#include <Arduino.h>
// #include <Thread.h>
// #include <ThreadController.h>

static int Motor1_RPWM = 5;
static int Motor1_FRPM = 6;

static int Motor2_RPWM = 9;
static int Motor2_FRPM = 10;

// Defult Build In LED On the MOST ARDUNO BOARD / ESP32
static uint8_t ledPin = LED_BUILTIN;

// Gobal Static Values
static int commandDelayMs = 10;
static int errorBitOutPut = 3;

// Gobal Values
bool bufferString = false;
String inputString = "";

int motor1S = 0;
int motor2S = 0;

void setup()
{

  // TODO: NEED CHECKING if the pinMode need or not
  // Docment say that analogWrite does not need pinMode <-- Not Test
  // // Motor 1 setup ide
  // pinMode(Motor1_RPWM, OUTPUT);
  // pinMode(Motor1_FRPM, OUTPUT);

  // // Motor 2 setup ide
  // pinMode(Motor2_RPWM, OUTPUT);
  // pinMode(Motor2_FRPM, OUTPUT);

  // Defult LED pin setup ide
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);

  // if the Serial is not being connect to the other system that support the Serial
  // Puse the command here
  while (!Serial)
  {
    sErrorStatus("Serial have not been connect", 0);
  }

  // Once the Serial is connect to the other system that support the Serial
  // loop the current status until First command have being send from the UART
  while (Serial.available() <= 0)
  {
    sSpeedInfo();
    delay(200);
  }
}

// While loop that will not be stop unless the Ardurino Board is crash
void loop()
{
  // If the bufferString is still in read or it was not finish, Puse the loop
  while (!bufferString)
  {
    sEventListerner();
  }

  // Base status information checking
  if (inputString.startsWith("status"))
  {
    sSpeedInfo();
  }
  // Gettng speed from the command "speed"
  else if (inputString.startsWith("speed"))
  {
    inputString.remove(0, 5);
    inputString.remove(inputString.length() - 1);

    // Checking if the Motor speed and the select is in the digit format
    // It can chekcing if the motor is in the negivte value
    if (isStringDigit(inputString))
    {

      // Chekcing if the motor select is the Motor ID: 0
      if (inputString.startsWith("0"))
      {
        inputString.remove(0, 1);
        unsigned long messageFromRBPI = strtoul(inputString.c_str(), NULL, 10);
        motor1S = messageFromRBPI;

        // Motor 1 contorl speed from the RBPI to the setting the PWM signal
        // forward rotation
        analogWrite(Motor1_RPWM, 0);
        analogWrite(Motor1_FRPM, abs(motor1S));

        // reverse rotation
        if (motor1S < 0)
        {
          analogWrite(Motor1_FRPM, 0);
          analogWrite(Motor1_RPWM, abs(motor1S));
        }
      }
      // Chekcing if the motor select is the Motor ID: 1
      else if (inputString.startsWith("1"))
      {
        inputString.remove(0, 1);
        unsigned long messageFromRBPI = strtoul(inputString.c_str(), NULL, 10);
        motor2S = messageFromRBPI;

        // Motor 2 contorl speed from the RBPI to the setting the PWM signal
        // forward rotation
        analogWrite(Motor2_FRPM, 0);
        analogWrite(Motor2_RPWM, abs(motor2S));

        // reverse rotation
        if (motor2S < 0)
        {
          analogWrite(Motor2_RPWM, 0);
          analogWrite(Motor2_FRPM, abs(motor2S));
        }
      }
      // If they are not both Return the ERROR
      else
      {
        sErrorStatus("Speed Data Error:/n Can not select the Motor ID:" + inputString, 3);
      }

      // Return the current status of both motor
      sSpeedInfo();
    }
    // If the motor inforamtion is not the digit after the word "speed" Return ERROR
    else
    {
      sErrorStatus("Speed Data Error: /n Motor is not in the digit format", 2);
    }
  }
  // If the command is not else status or speed Return the ERROR invald command
  else
  {
    sErrorStatus("Inviald Command", 2);
  }
  // reset the command
  bufferString = false;

  // command delay 10ms
  delay(commandDelayMs);
}

// TODO: Need adding the over led
/**
 * Status Chekcing with the Building board LED flash once
 * This will return the both motor current speed at the same time
 * They will return in this formart:
 * Current Speed1: x%, 2: x%
 * Once the LEC flash once It mean the report have send back on the UART
 */
void sSpeedInfo()
{
  char buffer[50];
  sprintf(buffer, "Current Speed 1: %d%%, 2: %d%%%", motor1S, motor2S);
  sErrorStatus(buffer, 1);
}

/**
 * Listerner that Listern the UART from the Serial
 * Lister only lister the command that is send fro mthe Serial that In beingfarmat in the uart.
 * And they need to be in binery
 * Once the serial recived the command, It will cover they back to string form
 * They will be add into the inputString
 * And rest the bufferString to true
 * Just Listerner not judgement
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
 * Checking if the String is pure digit or else
 * If they are not pure digit it will return false
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

// TODO: Need adding the over led
/**
 * Report the error to the serial with message about the type of error is.
 * If the srial can not able to be read the method
 * Will making the led to be flash in the order of the type of the error is respond to.
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
 * The conver maybe not be full 32bits It have been limited size of array
 * The size of the array is been set by the value errorBitOutPut
 * No Matter what the value will have a value 1 before all the value input start
 * This will be limit the num be 2^30
 * 
 * Example: 
 *  Input 3: output 1011
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