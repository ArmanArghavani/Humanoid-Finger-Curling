#include <ESP32Servo.h>   // Servo library for ESP32
#include <PID_v1.h>       // Arduino PID library

// === Servo Pin Definitions ===
const int servoFlexPin = 18;  // GPIO pin for flexion (curling) servo
const int servoExtPin  = 19;  // GPIO pin for extension servo

Servo servoFlex;   // Flexion (curling) servo
Servo servoExt;    // Extension servo

// === PID Control Variables ===
double setpoint = 180;     // Target angle for curling
double inputVal = 0;       // Current servo angle
double outputVal = 0;      // PID-calculated angle command

// PID tuning parameters (adjust as needed)
double Kp = 1.2;
double Ki = 0.05;
double Kd = 0.01;

// Create PID controller
PID curlPID(&inputVal, &outputVal, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  // Attach servos to their pins
  servoFlex.attach(servoFlexPin);
  servoExt.attach(servoExtPin);

  // Initialize servos to extended position
  servoFlex.write(0);    // Curling tendon released
  servoExt.write(180);   // Extension tendon pulled
  delay(1000);

  // Set PID limits and timing
  curlPID.SetMode(AUTOMATIC);
  curlPID.SetOutputLimits(0, 180);
  curlPID.SetSampleTime(10);
}

void loop() {
  for (int i = 0; i < 10; i++) {
    Serial.print("Cycle ");
    Serial.println(i + 1);

    // === Extension Phase ===
    servoExt.write(180);   // Pull extension tendon
    servoFlex.write(0);    // Release curl tendon
    Serial.println("Extended");
    delay(3000);

    // === Curling Phase using PID ===
    servoExt.write(0);     // Release extension tendon
    inputVal = 0;          // Start at 0° (extended)
    setpoint = 180;        // Target curl position

    Serial.println("Curling...");
    while (abs(setpoint - inputVal) > 1) {
      curlPID.Compute();                    // Compute new output
      int angleCmd = constrain((int)outputVal, 0, 180);
      servoFlex.write(angleCmd);           // Move flexion servo
      inputVal = angleCmd;                 // Update current angle
      delay(20);                           // Smooth movement
    }

    Serial.println("Curled");
    delay(3000);  // Hold curled
  }

  // After 10 cycles, stop movement
  Serial.println("Done.");
  while (true);
}
