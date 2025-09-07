/**
 * This program controls an EDF (Electric Ducted Fan) via an ESC and a thrust-vectoring servo.
 * It allows for user control via the Serial Monitor, provides automatic servo sweeping for testing,
 * reads/prints ESC telemetry data, and listens for external data on a hardware serial port.
 * Data is printed on a fixed interval (100ms).
 *
 * Hardware: Arduino Mega 2560
 * Servo PWM Output: Pin 46 (Timer5)
 * ESC PWM Output:   Pin 6 (Timer4)
 * ESC Telemetry Input: Serial1 (Pin 19 - RX1)
 * External Data Input: Serial2 (Pin 17 - RX2)
 */

// --- Global Variables ---
String labviewData = "";
// Holds the last COMPLETE data packet
String labviewDataBuffer = ""; // Builds the incoming data packet
char c;
//variable to read each byte of data from serial2 port.

// --- Servo Configuration ---
const int homePositionAngle = 0;
// The "parking" spot for the servo when motion is stopped.
const int maxSweepAngle = 80;
// This is the upper limit for the automatic sweep.
int servoPin = 46;
// PWM output pin for the servo.

// --- Servo State Variables ---
int angle = homePositionAngle;
// The servo's current angle, starting at the home position.
bool isMoving = false;
// Tracks if the servo is in automatic sweeping motion.
bool isReturning = false;
// Tracks if the servo is returning to its home position.
int direction = 1;
// Direction starts at 1 (moving up).

// --- Servo Timing Control Variables ---
unsigned long lastUpdateTime = 0;      // Stores the last time the servo was updated.
const int updateInterval = 20;         // Time between servo updates in ms (50Hz).

// --- ESC & Telemetry Configuration ---
uint16_t throttlePulse = 1000;
// Current PWM pulse width for the ESC in microseconds.
bool inPacket = false;
// Flag to track receipt of a telemetry packet.
uint8_t packet[32];                // Buffer to store incoming ESC telemetry packet data.
uint8_t index = 0;                 // Index for the telemetry packet buffer.
// --- Timing for Data Printing ---
unsigned long previousMillis = 0; // Stores the last time data was printed.
const long printInterval = 100;   // Interval at which to print data (in milliseconds).

// --- Function Prototypes ---
void setupESCPWM();
void setThrottlePulse(uint16_t pulse_us);
void setupServoTimer();
void setServoAngle(int servoAngle);
void printData();


//================================================================================
//  SETUP
//================================================================================
void setup() {
  // --- Initialize Serial Communication ---
  Serial.begin(115200);
// For user commands and telemetry output.
  Serial1.begin(115200); // For ESC telemetry input.
  Serial2.begin(9600);   // For external data.
  // --- Initialize Timers ---
  setupServoTimer(); // Configure Timer5 for Servo PWM.
  setupESCPWM();     // Configure Timer4 for ESC PWM.
  // --- Set Initial Positions ---
  setServoAngle(angle); // Command the servo to its initial home position.
  // --- Arm ESC ---
  setThrottlePulse(throttlePulse); // Send the 1000us arming signal.
  Serial.println("--- System Initialized ---");
  Serial.println("Commands:");
  Serial.println("  '1,1150' -> Start servo motion & set PWM to 1150us");
  Serial.println("  '0,1100' -> Stop servo motion & set PWM to 1100us");
  Serial.println("  '1150'   -> Set PWM to 1150us only");
  Serial.println("--------------------------");
  Serial.println("Arming ESC with 1000us signal. Waiting 3 seconds...");
  delay(3000); // This delay is acceptable as it only runs once in setup().
  Serial.println("System ready. Enter commands.");
}

//================================================================================
//  MAIN LOOP
//================================================================================
void loop() {
  // --- 1. Check for User Input ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
      String servoCmdStr = input.substring(0, commaIndex);
      String pwmCmdStr = input.substring(commaIndex + 1);
      int servoCommand = servoCmdStr.toInt();
      int pulse = pwmCmdStr.toInt();
      if (servoCommand == 1) {
        isMoving = true;
        isReturning = false;
        // Serial.println("Servo motion: STARTED.");
      } else if (servoCommand == 0) {
        isMoving = false;
        isReturning = true;
        // Serial.print("Servo motion: STOPPING... returning to ");
        // Serial.print(homePositionAngle);
        // Serial.println(" degrees.");
      }

      if (pulse >= 1000 && pulse <= 2000) {
        throttlePulse = pulse;
        setThrottlePulse(throttlePulse);
        // Serial.print("PWM pulse set to: ");
        // Serial.print(throttlePulse);
        // Serial.println(" us");
      } else {
        Serial.println("Invalid PWM value. Must be between 1000 and 2000.");
      }
    } else {
      int pulse = input.toInt();
      if (pulse >= 1000 && pulse <= 2000) {
        throttlePulse = pulse;
        setThrottlePulse(throttlePulse);
        // Serial.print("PWM pulse set to: ");
        // Serial.print(throttlePulse);
        // Serial.println(" us");
      } else {
        Serial.println("Invalid input. Enter a value or command like '1,1100'.");
      }
    }
  }

  // --- 2. Handle Servo Motion (with timing control) ---
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis(); // Reset the timer

    if (isMoving) {
      if (angle >= maxSweepAngle) {
        direction = -1; // Move down
      } else if (angle <= homePositionAngle) {
        direction = 1;  // Move up
      }
      angle = angle + direction;
      setServoAngle(angle);
    } else if (isReturning) {
      if (angle > homePositionAngle) {
        angle--;
        setServoAngle(angle);
      } else {
        isReturning = false;
        // Serial.print("Servo motion: STOPPED at ");
        // Serial.print(homePositionAngle);
        // Serial.println(" degrees.");
      }
    }
  }

  // --- 3. Process ESC Telemetry ---
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (!inPacket) {
      if (b == 0xDD) {
        inPacket = true;
        index = 0;
        packet[index++] = b;
      }
    } else {
      packet[index++] = b;
      if (index == 32) {
        inPacket = false;
      }
    }
  }

  // --- 4. Process LabVIEW Data ---
  while (Serial2.available() > 0) {
    c = Serial2.read();
    if (c == '\n') {
      // A complete line has been received.
      // Copy the complete line from the buffer for printing.
      labviewData = labviewDataBuffer;
      // Clear the buffer to get ready for the next line.
      labviewDataBuffer = "";
    } else {
      // Still receiving, add the character to the buffer.
      labviewDataBuffer += c;
    }
  }

  // --- 5. Print All Data Periodically ---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= printInterval) {
    previousMillis = currentMillis;
    // Save the last time you printed data
    printData();
  }

}

//================================================================================
//  HELPER FUNCTIONS
//================================================================================

/**
 * Prints all sensor and state data to the Serial Monitor.
 */
void printData() {
  // Print the last received LabVIEW data.
  labviewData.trim();
  Serial.print(labviewData);
  Serial.print(",");
  // Print servo angle (always available).
  int scaledAngle = round(angle / (float)maxSweepAngle * 100.0);
  Serial.print(scaledAngle);
  Serial.print(",");
  // Print current throttle pulse (always available).
  Serial.print(throttlePulse);
  Serial.print(",");

  // Check if ESC telemetry data is valid.
  if (packet[0] == 0xDD) {
    // --- Parse and Print ESC Telemetry ---
    uint16_t voltageRaw = ((uint16_t)packet[3] << 8) |
      packet[4];
    float voltage = voltageRaw / 10.0;
    uint16_t currentRaw = ((uint16_t)packet[5] << 8) | packet[6];
    float current = currentRaw;
    uint8_t throttleIn = packet[7];
    uint16_t rpmRaw = ((uint16_t)packet[8] << 8) | packet[9];
    float rpm = (float)rpmRaw*2.5;
    uint8_t mosTemp = packet[10];
    uint8_t motorTemp = packet[11];
    uint8_t throttleOut = packet[12];
    uint16_t state = ((uint16_t)packet[13] << 8) | packet[14];
    uint16_t mahUsed = ((uint16_t)packet[15] << 8) | packet[16];
    uint8_t uartThrottle = packet[17];
    uint8_t canThrottle = packet[18];
    float becVoltage = packet[19] / 10.0;

    Serial.print(voltage);
    Serial.print(",");
    Serial.print(current);
    Serial.print(",");
    Serial.print(throttleIn);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.print(mosTemp);
    Serial.print(",");
    Serial.print(motorTemp);
    Serial.print(",");
    Serial.print(throttleOut);
    Serial.print(",");
    Serial.print(state, HEX);
    Serial.print(",");
    Serial.print(mahUsed);
    Serial.print(",");
    Serial.print(uartThrottle);
    Serial.print(",");
    Serial.print(canThrottle);
    Serial.print(",");
    Serial.println(becVoltage);
  } else {
    // Print placeholders for all ESC values if data is not available.
    Serial.println("N/A, N/A, N/A, N/A, N/A, N/A, N/A, N/A, N/A, N/A, N/A, N/A");
  }
}

/**
 * Configures Timer5 for 50Hz servo PWM on pin 46.
 */
void setupServoTimer() {
  DDRL |= (1 << PL3);
  // Set pin 46 (PL3) as an output.
  TCCR5A = 0; TCCR5B = 0; TCNT5 = 0;
  // Reset timer registers.
  ICR5 = 39999;                        // Set TOP value for 50Hz PWM.
  TCCR5A = (1 << COM5A1) |
    (1 << WGM51); // Fast PWM mode 14, non-inverting.
  TCCR5B = (1 << WGM53) | (1 << WGM52) |
    (1 << CS51); // Set prescaler to 8 and start timer.
}

/**
 * Calculates and sets the servo pulse width based on angle.
 * servoAngle The desired angle, from 0 to 270 degrees.
 */
void setServoAngle(int servoAngle) {
  // 0 degrees -> 2500µs, 270 degrees -> 500µs, due to gear coupling
  long pulse_us = map(servoAngle, 0, 270, 2500, 500);
  // Convert pulse width (µs) to timer ticks (pulse_us * 2).
  OCR5A = pulse_us * 2;
}

/**
 * Configures Timer4 for 50Hz ESC PWM on pin 6.
 */
void setupESCPWM() {
  DDRH |= (1 << PH3);
  // Set pin 6 (PH3) as an output.
  TCCR4A = 0; TCCR4B = 0; TCNT4 = 0;
  // Reset timer registers.
  ICR4 = 39999;                        // Set TOP value for 50Hz PWM.
  TCCR4A = (1 << COM4A1) |
    (1 << WGM41); // Fast PWM mode 14, non-inverting.
  TCCR4B = (1 << WGM43) | (1 << WGM42) |
    (1 << CS41); // Set prescaler to 8 and start timer.
}

/**
 * Updates the ESC throttle pulse width.
 * pulse_us The desired throttle pulse in microseconds (typically 1000-2000).
 */
void setThrottlePulse(uint16_t pulse_us) {
  OCR4A = pulse_us * 2; // Convert µs to timer ticks.
}