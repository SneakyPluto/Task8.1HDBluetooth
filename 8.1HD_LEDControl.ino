/*
  LED Control

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  "19b10000-e8f2-537e-4f6c-d104768a1214" UUID is found. Once discovered and connected,
  it will remotely control the Bluetooth® Low Energy peripheral's LED, when the button is pressed or released.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.
  - Button with pull-up resistor connected to pin 2.

  You can use it with another board that is compatible with this library and the
  Peripherals -> LED example.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

// variables for button
const int buttonPin = 2;
int oldButtonState = LOW;

//Ultrasonic sensor setup
int trigPin = 11;  // Trigger
int echoPin = 12;  // Echo
long duration, cm, inches;

void setup() {
  Serial.begin(9600);

  while (!Serial)
    ;

  //Define inputs and outputs for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

// configure the button pin as input
// pinMode(buttonPin, INPUT);

// initialize the Bluetooth® Low Energy hardware
BLE.begin();

Serial.println("Bluetooth® Low Energy Central - LED control");

// start scanning for peripherals
BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

void loop() {

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "LED") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

//   while (peripheral.connected()) {
//     // while the peripheral is connected

//     // read the button pin
//     int buttonState = digitalRead(buttonPin);

//     if (oldButtonState != buttonState) {
//       // button changed
//       oldButtonState = buttonState;

//       if (buttonState) {
//         Serial.println("button pressed");

//         // button is pressed, write 0x01 to turn the LED on
//         ledCharacteristic.writeValue((byte)0x01);
//       } else {
//         Serial.println("button released");

//         // button is released, write 0x00 to turn the LED off
//         ledCharacteristic.writeValue((byte)0x00);
//       }
//     }
//   }
//   Serial.println("Peripheral disconnected");
// }

  while (peripheral.connected()) {
    // The ultrasonic sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);

    // Convert the time into a distance
    cm = (duration / 2) / 29.1;    // Divide by 29.1 or multiply by 0.0343
    inches = (duration / 2) / 74;  // Divide by 74 or multiply by 0.0135

    if (cm < 10) {
      Serial.println("Ultrasonic sesnor triggered !");
      ledCharacteristic.writeValue((byte)0x01);
    } else {
      Serial.println("Ultrasonic sensor in standby (reading)");

      // ultrasonic is reading values greater than 10cm, write 0x00 to turn the LED off
      ledCharacteristic.writeValue((byte)0x00);
    }
  }
  Serial.println("Peripheral disconnected");
}
