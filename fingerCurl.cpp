#include <ESP32Servo.h>
#include <PID_v1.h>

// === Servo Pins ===
const int servoFlexPin = 18;  // Flexion tendon
const int servoExtPin  = 19;  // Extension tendon

Servo servoFlex;
Servo servoExt;

// === Flexion PID Variables ===
double flexSetpoint = 180;
double flexInput = 0;
double flexOutput = 0;

double flexKp = 1.2, flexKi = 0.05, flexKd = 0.01;
PID flexPID(&flexInput, &flexOutput, &flexSetpoint, flexKp, flexKi, flexKd, DIRECT);

// === Extension PID Variables ===
double extSetpoint = 180;
double extInput = 0;
double extOutput = 0;

double extKp = 1.2, extKi = 0.05, extKd = 0.01;
PID extPID(&extInput, &extOutput, &extSetpoint, extKp, extKi, extKd, DIRECT);

void setup() {
  Serial.begin(115200);

  servoFlex.attach(servoFlexPin);
  servoExt.attach(servoExtPin);

  // Start with finger extended
  servoFlex.write(0);
  servoExt.write(180);
  delay(1000);

  // Initialize PIDs
  flexPID.SetMode(AUTOMATIC);
  flexPID.SetOutputLimits(0, 180);
  flexPID.SetSampleTime(10);

  extPID.SetMode(AUTOMATIC);
  extPID.SetOutputLimits(0, 180);
  extPID.SetSampleTime(10);
}

void loop() {
  for (int i = 0; i < 10; i++) {
    Serial.printf("Cycle %d\n", i + 1);

    // === EXTEND PHASE with PID ===
    flexSetpoint = 0;     // Release flexion
    extSetpoint = 180;    // Pull extension

    flexInput = 180;      // Simulated: assume was curled
    extInput = 0;         // Simulated: assume was unextended

    Serial.println("Extending...");
    while (abs(extSetpoint - extInput) > 1) {
      extPID.Compute();
      flexPID.Compute();  // Still maintain flex release

      int extCmd = constrain(extOutput, 0, 180);
      int flexCmd = constrain(flexOutput, 0, 180);

      servoExt.write(extCmd);
      servoFlex.write(flexCmd);

      extInput = extCmd;
      flexInput = flexCmd;

      delay(20);
    }

    delay(3000);  // Hold extended

    // === CURL PHASE with PID ===
    flexSetpoint = 180;   // Pull flexion
    extSetpoint = 0;      // Release extension

    flexInput = 0;        // Simulated: assume was extended
    extInput = 180;

    Serial.println("Curling...");
    while (abs(flexSetpoint - flexInput) > 1) {
      flexPID.Compute();
      extPID.Compute();  // Still maintain extension release

      int flexCmd = constrain(flexOutput, 0, 180);
      int extCmd = constrain(extOutput, 0, 180);

      servoFlex.write(flexCmd);
      servoExt.write(extCmd);

      flexInput = flexCmd;
      extInput = extCmd;

      delay(20);
    }

    delay(3000);  // Hold curled
  }

  Serial.println("Finished all cycles.");
  while (true);  // Halt execution
}