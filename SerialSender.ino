/**
 * This program sends serial data received via LabVIEW VISA Write to a main Arduino for processing
 * It uses Software Serial allowing LabVIEW to use hardware serial for publishing.
 */

#include <SoftwareSerial.h>
 
// Define the pins for the software serial port
// RX pin is 10, TX pin is 11
// The sender's TX pin (11) will be connected to the receiver's RX pin.
SoftwareSerial mySerial(10, 11);
 
void setup() {
  // Start the hardware serial port at 9600 bps.
  // This is the port connected to the computer via USB.
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Hardware Serial Initialized. Ready to forward to Software Serial.");
 
  // Start the software serial port at 9600 bps.
  mySerial.begin(9600);
}
 
void loop() {
  // Check if there is data available on the hardware serial port.
  if (Serial.available()) {
    // Read the incoming byte from the hardware serial.
    char data = Serial.read();
    // Write the byte to the software serial port.
    mySerial.write(data);
  }
}