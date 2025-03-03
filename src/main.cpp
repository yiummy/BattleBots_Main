#include <Arduino.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>

// Define motor driver pins
// left motor (I think)
// TODO: edit these pinouts to match the actual board
const int motor1_in1 = 15;
const int motor1_in2 = 2;
const int motor1_pow = 3;

// right motor
const int motor2_in1 = 0;
const int motor2_in2 = 4;
const int motor2_pow = 2874;

// weapon motor
const int motor3_in1 = 16;
const int motor3_in2 = 17;
const int motor3_pow = 345; // dummy pin, should be wired to nothing

// Define servo pins
const int servo1_pin = 10;
const int servo2_pin = 11;

// Create servo objects
Servo servo1;
Servo servo2;

// ps4 connection and weapon states
bool lastConnectionState = false;
bool drumOn = false;
bool flipperUp = false;
bool safetyEnabled = true;
bool lastSafetyState = true;
bool EStop = false;

int leftX;
int leftY;
bool squareVal;
bool crossVal;
unsigned long crossPressTime = 0;
unsigned long squarePressTime = 0;
int flipperAngle = 60; // degrees, how high the flipper goes (must be less than 90)

// motor values to send to drivers
int LMPow; // Pow is 0-255 for analogwrite duty cycle
int RMPow;
int LMState; // state is 1 for forward, -1 for backward, anything else for still
int RMState;
float scale;

// function definitions
void motorControl(int motorState, int pin1, int pin2, int speedpin, int motorSpeed);
void flipperWrite(bool flipperState);
void stopAll();

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);

  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);

  pinMode(motor3_in1, OUTPUT);
  pinMode(motor3_in2, OUTPUT);

  // Attach servos
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nBattleBot PS4 Controller Starting");

  Serial.println("Motor pins initialized");
  Serial.println("Initializing PS4 Controller...");

  PS4.begin("84:2F:57:1A:B8:54"); // TODO: change the mac address of the code for new contorller
  Serial.println("PS4 initialization complete");
  Serial.println("Waiting for controller connection...");
}

void loop() {

  if (PS4.isConnected()) {
    if (!lastConnectionState) {
      Serial.println("Controller Connected!");
      lastConnectionState = true;
    }

    // Emergency stop
    if (PS4.PSButton() || EStop) {
      EStop = true;
      Serial.println("EMERGENCY STOP ACTIVATED");
      safetyEnabled = true;
      stopAll();
      Serial.println("All motors stopped - Emergency Stop. Restart system to resume");
      delay(200);
      return;
    }

    // digitalWrite(STATUS_LED, HIGH); // option status LED?

    // TODO: TEST THE CONTROLLLLS
    leftY = PS4.RStickY();
    leftX = PS4.RStickX();
    crossVal = PS4.Cross();
    squareVal = PS4.Square();

    // toggling flipper and drum on/off
    if (crossVal & millis() - crossPressTime > 100) {
      flipperUp = !flipperUp;
      crossPressTime = millis();
    }
    if (squareVal & millis() - squarePressTime > 100) {
      drumOn = !drumOn;
      squarePressTime = millis();
    }

    if (abs(leftY) > 10 || abs(leftX) > 10) {
      Serial.printf("Controls - LY: %d, LX: %d, FlipperUp: ", leftY, leftX, flipperUp);
    }

    if (abs(leftY) < 10 && abs(leftX) < 10) {
      // case where stick is 0
      LMPow = 0;
      RMPow = 0;
      RMState = 0;
      LMState = 0;
    }
    else if (abs(leftY) < 10) {
      // case where stick is horizontal only
      if (leftX > 0) {
        LMState = 1;
        RMState = -1;
      } else {
        LMState = -1;
        RMState = 1;
      }
      LMPow = map(abs(leftX), 0, 128, 0, 255);
      RMPow = map(abs(leftX), 0, 128, 0, 255);
    } else if (abs(leftX) < 10) {
      // case where stick is vertical only
      if (leftY < 0) {
        LMState = -1;
        RMState = -1;
      } else {
        LMState = 1;
        RMState = 1;
      }
      LMPow = map(abs(leftY), 0, 128, 0, 255);
      RMPow = map(abs(leftY), 0, 128, 0, 255);
    } else {
      // case where stick is off both hor and vert axes
      if (leftY > 0) {
        LMState = 1;
        RMState = 1;
        scale = sqrt(pow(leftY, 2) + pow(leftX, 2)) / 128;
        if (leftX > 0) {
          LMPow = 255;
          RMPow = map(abs(leftY), 0, 128, 0, 255);
        } else {
          RMPow = 255;
          LMPow = map(abs(leftY), 0, 128, 0, 255);
        }
      } else {
        LMState = -1;
        RMState = -1;
        if (leftX > 0) {
          LMPow = 255;
          RMPow = map(abs(leftY), 0, 128, 0, 255);
        } else {
          RMPow = 255;
          LMPow = map(abs(leftY), 0, 128, 0, 255);
        }
      }
      RMPow = int(RMPow * scale);
      LMPow = int(LMPow * scale);
    }

    // Safety controls
    if (PS4.Circle()) {
      safetyEnabled = !safetyEnabled;
      if (safetyEnabled != lastSafetyState) {
        Serial.printf("Safety %s\n", safetyEnabled ? "ENABLED" : "DISABLED");
        lastSafetyState = safetyEnabled;
      }
      if (safetyEnabled) {
        stopAll();
        Serial.println("All motors stopped due to safety");
      }
      delay(200);
    }
    // write to motors if safe
    if (!safetyEnabled) {
      motorControl(LMState, motor1_in1, motor1_in2, motor1_pow, LMPow);   // left motor
      motorControl(RMState, motor2_in1, motor2_in2, motor2_pow, LMPow);   // right motor
      motorControl(int(drumOn), motor3_in1, motor3_in2, motor3_pow, 255); // drum motor
      flipperWrite(flipperUp);
    }
  } else {
    if (lastConnectionState) {
      Serial.println("Controller Disconnected!");
      lastConnectionState = false;
    }
    // digitalWrite(STATUS_LED, LOW);
    stopAll();
  }

  delay(20);
}

void motorControl(int motorState, int pin1, int pin2, int speedpin, int motorSpeed) {
  if (motorState == 1) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(speedpin, motorSpeed);
  } else if (motorState == -1) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(speedpin, motorSpeed);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(speedpin, 0);
  }
}

void flipperWrite(bool flipperState) {
  // if flipperState = true, send a raised value
  // if false, then send lowered value
  if (flipperState) {
    servo1.write(90 + flipperAngle);
    servo2.write(90 - flipperAngle);
  } else {
    servo1.write(90);
    servo2.write(90);
  }
}

void stopAll() {
  motorControl(0, motor1_in1, motor1_in2, motor1_pow, 0); // left motor
  motorControl(0, motor2_in1, motor2_in2, motor2_pow, 0); // right motor
  motorControl(0, motor3_in1, motor3_in2, motor3_pow, 0); // drum motor
  // flipperWrite(flipperUp); // don't need to send data to flipper servos
}
