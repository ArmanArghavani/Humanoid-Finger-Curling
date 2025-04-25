#include <ESP32Servo.h>     // Include servo library for ESP32
#include <PID_v1.h>         // Include Arduino PID control library

// === Define Servo Pins ===
const int servoFlexPin = 18;  // Servo controlling flexion (curling)
const int servoExtPin  = 19;  // Servo controlling extension

Servo servoFlex;              // Servo object for flexion
Servo servoExt;               // Servo object for extension

// === PID Variables for Flexion ===
double flexSetpoint = 180;    // Desired angle for flexion (curl)
double flexInput = 0;         // Simulated current flexion angle
double flexOutput = 0;        // Output command from PID for flexion

double flexKp = 1.2, flexKi = 0.05, flexKd = 0.01;  // PID tuning constants
PID flexPID(&flexInput, &flexOutput, &flexSetpoint, flexKp, flexKi, flexKd, DIRECT); // Create PID object

// === PID Variables for Extension ===
double extSetpoint = 180;     // Desired angle for extension
double extInput = 0;          // Simulated current extension angle
double extOutput = 0;         // Output command from PID for extension

double extKp = 1.2, extKi = 0.05, extKd = 0.01;  // PID tuning constants
PID extPID(&extInput, &extOutput, &extSetpoint, extKp, extKi, extKd, DIRECT); // Create PID object

void setup() {
  Serial.begin(115200);          // Start serial communication for debugging

  servoFlex.attach(servoFlexPin);  // Attach flexion servo to specified pin
  servoExt.attach(servoExtPin);    // Attach extension servo to specified pin

  // Set initial positions (fully extended)
  servoFlex.write(0);    // Relax flexion
  servoExt.write(180);   // Pull extension
  delay(1000);           // Give time for servo to reach position

  // === Initialize PID Controllers ===
  flexPID.SetMode(AUTOMATIC);       // Enable automatic PID computation
  flexPID.SetOutputLimits(0, 180);  // Limit servo command range
  flexPID.SetSampleTime(10);        // Run PID every 10ms

  extPID.SetMode(AUTOMATIC);
  extPID.SetOutputLimits(0, 180);
  extPID.SetSampleTime(10);
}

void loop() {
  for (int i = 0; i < 10; i++) {  // Repeat 10 times
    Serial.printf("Cycle %d\n", i + 1);

    // === EXTEND PHASE using PID ===
    flexSetpoint = 0;      // Relax flexion tendon
    extSetpoint = 180;     // Pull extension tendon

    flexInput = 180;       // Simulated starting position: curled
    extInput = 0;          // Simulated starting position: retracted

    Serial.println("Extending...");
    while (abs(extSetpoint - extInput) > 1) {  // Run until extension target is reached
      extPID.Compute();         // Update extension PID output
      flexPID.Compute();        // Maintain relaxed flexion

      int extCmd = constrain(extOutput, 0, 180);  // Clamp output to servo-safe range
      int flexCmd = constrain(flexOutput, 0, 180);

      servoExt.write(extCmd);   // Send command to extension servo
      servoFlex.write(flexCmd); // Send command to flexion servo

      extInput = extCmd;        // Simulate servo movement (if no sensors)
      flexInput = flexCmd;

      delay(20);                // Small delay between PID updates
    }

    delay(3000);  // Hold finger extended for 3 seconds

    // === CURL PHASE using PID ===
    flexSetpoint = 180;   // Pull flexion tendon
    extSetpoint = 0;      // Relax extension tendon

    flexInput = 0;        // Simulated starting position: extended
    extInput = 180;

    Serial.println("Curling...");
    while (abs(flexSetpoint - flexInput) > 1) {  // Run until curl target is reached
      flexPID.Compute();         // Update flexion PID output
      extPID.Compute();          // Maintain relaxed extension

      int flexCmd = constrain(flexOutput, 0, 180);
      int extCmd = constrain(extOutput, 0, 180);

      servoFlex.write(flexCmd);  // Command flexion servo
      servoExt.write(extCmd);    // Command extension servo

      flexInput = flexCmd;       // Simulate actual movement
      extInput = extCmd;

      delay(20);
    }

    delay(3000);  // Hold curled for 3 seconds
  }

  Serial.println("Finished all cycles.");
  while (true);  // Halt program after finishing all cycles
}