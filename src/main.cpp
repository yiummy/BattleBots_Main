#include <Arduino.h>


#include <ESP32Servo.h>
#include <PS4Controller.h>

unsigned long lastTimeStamp = 0;

// Define motor driver pins
// left motor (I think)
const int motor1_in1 = 15;
const int motor1_in2 = 2;

// right motor 
const int motor2_in1 = 0;
const int motor2_in2 = 4;

// weapon motor 
const int motor3_in1 = 16;
const int motor3_in2 = 17;


// Define servo pins
const int servo1_pin = 10;
const int servo2_pin = 11;


// Create servo objects
Servo servo1;
Servo servo2;


void setup() {
    // Initialize motor control pins as outputs
    pinMode(motor1_in1, OUTPUT);
    pinMode(motor1_in2, OUTPUT);


    pinMode(motor2_in1, OUTPUT);
    pinMode(motor2_in2, OUTPUT);


    pinMode(motor3_in1, OUTPUT);
    pinMode(motor3_in2, OUTPUT);


    // Attach servos
    //servo1.attach(servo1_pin);
    //servo2.attach(servo2_pin);
   
    Serial.begin(115200);
    PS4.attach(notify);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
    PS4.begin();
    removePairedDevices(); // This helps to solve connection issues
    Serial.print("This device MAC is: ");
    printDeviceAddress();
    Serial.println("");
  }
 
  void loop() {
    // Example motor control (Forward)
    digitalWrite(motor1_in1, HIGH);  // Motor 1 Forward
    digitalWrite(motor1_in2, LOW);  // Motor 1 Forward
   
   
   
    delay(1000);
  }
 
  void removePairedDevices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
      esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
  }
 
  void printDeviceAddress() {
    const uint8_t* point = esp_bt_dev_get_address();
    for (int i = 0; i < 6; i++) {
      char str[3];
      sprintf(str, "%02x", (int)point[i]);
      Serial.print(str);
      if (i < 5) {
        Serial.print(":");
      }
    }
  }
 
  void onConnect() {
    Serial.println("Connected!");
  }
 
  void notify() {
  #if EVENTS
    boolean sqd = PS4.event.button_down.square,
            squ = PS4.event.button_up.square,
            trd = PS4.event.button_down.triangle,
            tru = PS4.event.button_up.triangle;
    if (sqd)
      Serial.println("SQUARE down");
    else if (squ)
      Serial.println("SQUARE up");
    else if (trd)
      Serial.println("TRIANGLE down");
    else if (tru)
      Serial.println("TRIANGLE up");
  #endif
 
  #if BUTTONS
    boolean sq = PS4.Square(),
            tr = PS4.Triangle();
    if (sq)
      Serial.print(" SQUARE pressed");
    if (tr)
      Serial.print(" TRIANGLE pressed");
    if (sq | tr)
      Serial.println();
  #endif
 
    //Only needed to print the message properly on serial monitor. Else we dont need it.
    if (millis() - lastTimeStamp > 50) {
  #if JOYSTICKS
      Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n",
                    PS4.LStickX(),
                    PS4.LStickY(),
                    PS4.RStickX(),
                    PS4.RStickY());
  #endif
  #if SENSORS
      Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n",
                    PS4.GyrX(),
                    PS4.GyrY(),
                    PS4.GyrZ(),
                    PS4.AccX(),
                    PS4.AccY(),
                    PS4.AccZ());
  #endif
      lastTimeStamp = millis();
    }
  }
 
  void onDisConnect() {
    Serial.println("Disconnected!");
  }



