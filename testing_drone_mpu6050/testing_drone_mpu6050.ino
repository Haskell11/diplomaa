/*  ESP32 integrated firmware
    - MPU6050 DMP (yaw/pitch/roll)
    - UDP streaming (START / STOP)
    - 4x ESC control via Servo.writeMicroseconds
    - PID on angles (always ON)
    - UDP command: SET <yaw> <pitch> <roll> <throttle>
      e.g. "SET 0 10 0 1300"  -> desired yaw=0°, pitch=10°, roll=0°, throttle=1300us
    WARNING: test with props off and on test stand first.
*/

// Libraries
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>   // Use ServoESP32 1.0.3 / 1.0.2 for ESP32 compatibility

// ----------------- WiFi / UDP ---------------------
const char* ssid = "iphoneMax11";
const char* password = "haskellq";
WiFiUDP udp;
const char* pcIP = "172.20.10.2"; // Python/PC IP (receive IMU)
const int pcPort = 3333;
const int localPort = 3333;

// ----------------- MPU6050 (DMP) -------------------
MPU6050 mpu(0x68);
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll (radians)

// Optionally use interrupt-based DMP (not required here, we'll poll dmpGetCurrentFIFOPacket)
volatile bool mpuFlag = false;
#define INTERRUPT_PIN 2
void IRAM_ATTR dmpReadyISR() { mpuFlag = true; }

// ----------------- Motors / Servos ----------------
Servo motor1, motor2, motor3, motor4;

// Default pins (change if needed)
const int MOTOR_PIN_1 = 13;
const int MOTOR_PIN_2 = 12;
const int MOTOR_PIN_3 = 14;
const int MOTOR_PIN_4 = 27;

// ESC PWM limits (microseconds)
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
int throttleSet = 1100; // default throttle setpoint (us) — can be changed by SET

// Safety flag: when true, motors are stopped (set to min)
bool emergencyStop = false;

// ----------------- Control variables ----------------
bool streamActive = false;      // START/STOP streaming yaw/pitch/roll
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 50; // ~20Hz

// Desired setpoints (degrees)
float yawDesired   = 0.0;
float pitchDesired = 0.0;
float rollDesired  = 0.0;

// PID parameters for each axis (tune these)
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevErr;
  float outLimit; // absolute limit for output (in 'control units')
};
PID pidPitch = { 4.0, 0.02, 0.5, 0.0, 0.0, 1.5 }; // example values - tune on stand
PID pidRoll  = { 4.0, 0.02, 0.5, 0.0, 0.0, 1.5 };
PID pidYaw   = { 2.0, 0.005, 0.3, 0.0, 0.0, 1.0 };

// How PID output maps to motor microseconds (scale)
const float MOTOR_SCALE_US = 180.0; // 1.0 PID unit -> 180 us on ESC (tune)

// Timekeeping for PID loop
uint32_t prevMicros = 0;
const uint32_t PID_PERIOD_US = 8000; // 8 ms ~ 125 Hz update

// Motor PWMs (microseconds)
int PWM1=THROTTLE_MIN, PWM2=THROTTLE_MIN, PWM3=THROTTLE_MIN, PWM4=THROTTLE_MIN;

// ---------- Function prototypes ----------
void armMotors();
void stopAllMotors();
int constrainPWM(int pwm);
float runPID(PID &pid, float setpoint, float measurement, float dt);
void applyMotorMix(float uPitch, float uRoll, float uYaw, int throttleUs);
void processUdpCommand(const String &cmd);

// ----------------- Setup --------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // MPU init
  Serial.println(F("Init MPU6050..."));
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 not responding!"));
    while (1) { delay(1000); } // halt
  }

  uint8_t devStatus = mpu.dmpInitialize();
  Serial.print(F("dmpInitialize status: "));
  Serial.println(devStatus);

  // Optionally set offsets if known (comment out to use auto-calibration)
  // mpu.setXAccelOffset(...); etc.

  // Calibrate (optional) - can be time consuming
  Serial.println(F("Calibrating (accel+gyro) ..."));
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP ready"));
  } else {
    Serial.println(F("DMP init failed"));
    while (1) { delay(1000); }
  }

  // attach interrupt if you want (we'll still poll), optional:
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpReadyISR, RISING);

  // Motors attach
  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);
  motor3.attach(MOTOR_PIN_3);
  motor4.attach(MOTOR_PIN_4);

  // Start with motors disarmed (min)
  stopAllMotors();

  // WiFi
  Serial.print("Connecting WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  udp.begin(localPort);
  Serial.println("UDP started on port " + String(localPort));

  prevMicros = micros();
  Serial.println("Setup done");
}

// ----------------- Loop ---------------------------
void loop() {
  // UDP command processing
  int packetSizeUdp = udp.parsePacket();
  if (packetSizeUdp) {
    char buf[128];
    int len = udp.read(buf, sizeof(buf)-1);
    if (len > 0) buf[len] = 0;
    String cmd = String(buf);
    cmd.trim();
    Serial.println("UDP recv: " + cmd);
    processUdpCommand(cmd);
  }

  // Read DMP when available
  if (!dmpReady) return;

  // Polling DMP: guard with packet fetch
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw  = ypr[0] * 180.0 / M_PI;
    float pitch= ypr[1] * 180.0 / M_PI;
    float roll = ypr[2] * 180.0 / M_PI;

    // Safety: if angles too big, cut throttle
    if (abs(pitch) > 60.0 || abs(roll) > 60.0 || abs(yaw) > 120.0) {
      emergencyStop = true;
      Serial.println("Emergency stop: angle limit exceeded");
    }

    // PID update at fixed rate (PID_PERIOD_US)
    uint32_t now = micros();
    uint32_t dt_us = now - prevMicros;
    if (dt_us >= PID_PERIOD_US) {
      float dt = dt_us * 1e-6f; // seconds

      // compute PID outputs (units roughly "control units")
      float uPitch = runPID(pidPitch, pitchDesired, pitch, dt);
      float uRoll  = runPID(pidRoll,  rollDesired,  roll,  dt);
      float uYaw   = runPID(pidYaw,   yawDesired,   yaw,   dt);

      // apply motor mixing -> compute PWM1..4
      applyMotorMix(uPitch, uRoll, uYaw, throttleSet);

      // write to motors (unless emergency)
      if (!emergencyStop) {
        motor1.writeMicroseconds(PWM1);
        motor2.writeMicroseconds(PWM2);
        motor3.writeMicroseconds(PWM3);
        motor4.writeMicroseconds(PWM4);
      } else {
        stopAllMotors();
      }

      prevMicros = now;
    }

    // Send telemetry over UDP if streaming enabled
    if (streamActive && (millis() - lastSend >= SEND_INTERVAL_MS)) {
      char msg[80];
      snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f", yaw, pitch, roll);
      udp.beginPacket(pcIP, pcPort);
      udp.print(msg);
      udp.endPacket();
      lastSend = millis();
    }
  }
}

// ----------------- Functions ----------------------

// Arm motors: gradual ramp to starting throttle
void armMotors() {
  Serial.println("Arming motors...");
  // Set to MIN, wait, then soft ramp to throttleSet
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  delay(1000);

  // soft ramp to throttleSet (safe)
  int steps = 20;
  int start = THROTTLE_MIN;
  for (int i=1;i<=steps;i++){
    int t = start + (throttleSet - start) * i / steps;
    motor1.writeMicroseconds(t);
    motor2.writeMicroseconds(t);
    motor3.writeMicroseconds(t);
    motor4.writeMicroseconds(t);
    delay(60);
  }
  Serial.println("Armed");
}

// sets motors to MIN
void stopAllMotors() {
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  PWM1 = PWM2 = PWM3 = PWM4 = THROTTLE_MIN;
}

// constrain PWM
int constrainPWM(int pwm) {
  if (pwm < THROTTLE_MIN) return THROTTLE_MIN;
  if (pwm > THROTTLE_MAX) return THROTTLE_MAX;
  return pwm;
}

// Simple PID implementation (angle controller)
float runPID(PID &pid, float setpoint, float measurement, float dt) {
  float err = setpoint - measurement; // degrees
  pid.integral += err * dt;
  float derivative = 0;
  if (dt > 0) derivative = (err - pid.prevErr) / dt;
  pid.prevErr = err;

  float out = pid.Kp * err + pid.Ki * pid.integral + pid.Kd * derivative;

  // limit output to outLimit
  if (out > pid.outLimit) out = pid.outLimit;
  if (out < -pid.outLimit) out = -pid.outLimit;
  return out;
}

// Apply mixing to create individual motor PWMs
// Mixing assumes X configuration with motors numbered:
// 1: front-right, 2: rear-right, 3: rear-left, 4: front-left
// Adjust mixing signs if your motor numbering is different.
void applyMotorMix(float uPitch, float uRoll, float uYaw, int throttleUs) {
  // u* are in control units -> convert to microseconds
  int dPitch = int(uPitch * MOTOR_SCALE_US);
  int dRoll  = int(uRoll  * MOTOR_SCALE_US);
  int dYaw   = int(uYaw   * MOTOR_SCALE_US);

  // Common mixing (adjust sign convention if needed)
  // Using mapping similar to your original code:
  // THRST1 = 0.25*uh + 0.25*uP + 0.25*uR - 0.25*uY
  // but here we convert deltas directly on microseconds:
  // compute each motor pwm: base throttle + combination
  // Note: choose signs to produce expected pitch/roll yaw direction on your stand
  PWM1 = throttleUs + ( dPitch + dRoll - dYaw ); // motor 1
  PWM2 = throttleUs + ( dPitch - dRoll + dYaw ); // motor 2
  PWM3 = throttleUs + ( -dPitch - dRoll - dYaw );// motor 3
  PWM4 = throttleUs + ( -dPitch + dRoll + dYaw );// motor 4

  PWM1 = constrainPWM(PWM1);
  PWM2 = constrainPWM(PWM2);
  PWM3 = constrainPWM(PWM3);
  PWM4 = constrainPWM(PWM4);
}

// Process UDP command strings
// Supported:
//   START        -> enable streaming
//   STOP         -> disable streaming
//   ARM          -> arm motors (soft ramp)
//   DISARM       -> stop motors
//   SET y p r t  -> set yaw pitch roll throttle  (degrees, degrees, degrees, microseconds)
// Example: "SET 0 10 0 1300"
void processUdpCommand(const String &cmd) {
  if (cmd.equalsIgnoreCase("START")) {
    streamActive = true;
    Serial.println("STREAM START");
    return;
  }
  if (cmd.equalsIgnoreCase("STOP")) {
    streamActive = false;
    Serial.println("STREAM STOP");
    return;
  }
  if (cmd.equalsIgnoreCase("ARM")) {
    emergencyStop = false;
    armMotors();
    return;
  }
  if (cmd.equalsIgnoreCase("DISARM")) {
    emergencyStop = true;
    stopAllMotors();
    Serial.println("DISARMED");
    return;
  }

  // parse SET
  if (cmd.startsWith("SET")) {
    // tokenize
    // expected: SET yaw pitch roll throttle
    float y=0,p=0,r=0;
    int t=throttleSet;
    // using sscanf
    int parsed = sscanf(cmd.c_str(), "SET %f %f %f %d", &y, &p, &r, &t);
    if (parsed >= 3) {
      yawDesired = y;
      pitchDesired = p;
      rollDesired = r;
      if (parsed == 4) {
        // clamp throttle
        if (t < THROTTLE_MIN) t = THROTTLE_MIN;
        if (t > THROTTLE_MAX) t = THROTTLE_MAX;
        throttleSet = t;
      }
      Serial.printf("SET -> yaw=%.2f pitch=%.2f roll=%.2f throttle=%d\n", yawDesired, pitchDesired, rollDesired, throttleSet);
    } else {
      Serial.println("SET parsing failed. Use: SET yaw pitch roll throttle");
    }
    return;
  }

  // Optional: parse PID tuning command (e.g. "PID Pp Pi Pd")
  if (cmd.startsWith("PIDPITCH")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDPITCH %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidPitch.Kp=kp; pidPitch.Ki=ki; pidPitch.Kd=kd; Serial.println("PIDPITCH updated"); }
    return;
  }
  if (cmd.startsWith("PIDROLL")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDROLL %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidRoll.Kp=kp; pidRoll.Ki=ki; pidRoll.Kd=kd; Serial.println("PIDROLL updated"); }
    return;
  }
  if (cmd.startsWith("PIDYAW")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDYAW %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidYaw.Kp=kp; pidYaw.Ki=ki; pidYaw.Kd=kd; Serial.println("PIDYAW updated"); }
    return;
  }

  // If command not recognized:
  Serial.println("Unknown command");
}
