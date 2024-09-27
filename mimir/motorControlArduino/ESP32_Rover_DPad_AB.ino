/*
This code is forked from the example file 'Controller.ino' found in the bluepad32 library
The bluepad32 library can be found at: https://github.com/ricardoquesada/bluepad32
The bluepad32 library is licensed under the Apache 2.0 license
This code is also licensed under the Apache 2.0 license
*/


#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//'reverse' direction of left motor moves forward
const int MOTOR_LEFT_REVERSE_PWM = 12;
const int MOTOR_LEFT_FORWARD_PWM = 14;
//'forward' direction of right motor moves forward
const int MOTOR_RIGHT_REVERSE_PWM = 18;
const int MOTOR_RIGHT_FORWARD_PWM = 19;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    if (ctl->buttons() == 0x0001) { // A Button
      //turn on LED
      digitalWrite(2, HIGH); 

      //move forward
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  100);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    0);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 0);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   100);
    }
    else if (ctl->buttons() == 0x0002) { // B Button
      //turn on LED
      digitalWrite(2, HIGH);

      //move backward
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  0);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    100);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 100);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   0);
    }
    else if (ctl->dpad() == 0x01) { // d-pad forward
      //turn on LED
      digitalWrite(2, HIGH);

      //move forward
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  100);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    0);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 0);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   100);
    }
    else if (ctl->dpad() == 0x02) { // d-pad backward
      //turn on LED
      digitalWrite(2, HIGH);

      //move backward
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  0);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    100);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 100);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   0);
    }
    else if (ctl->dpad() == 0x04) { // d-pad right
      //turn on LED
      digitalWrite(2, HIGH);

      //rotate right
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  0);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    100);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 0);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   100);
    }
    else if (ctl->dpad() == 0x08) { // d-pad left
      //turn on LED
      digitalWrite(2, HIGH);

      //rotate left
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  100);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    0);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 100);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   0);
    }
    // else if (ctl->throttlle() == ... ) // right trigger value placeholder; can fall between 0 - 1023
    // else if (ctl->brake() == ... ) // left trigger value placeholder; can fall between 0 - 1023
    else { // nothing pressed
      //turn off LED
        digitalWrite(2, LOW);

        //stop moving
      analogWrite(MOTOR_LEFT_REVERSE_PWM,  0);
      analogWrite(MOTOR_LEFT_FORWARD_PWM,    0);
      analogWrite(MOTOR_RIGHT_REVERSE_PWM, 0);
      analogWrite(MOTOR_RIGHT_FORWARD_PWM,   0);
    }

    // tests gamepad connectivity using rumble function
    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(2, OUTPUT);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(150);
}
