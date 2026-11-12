// Seaglider depth-hold controller (PID on depth, cascaded to plunger position).
//
// Built on the SeaGlide starter firmware (RoboNation, https://seaglide.org/firmware)
// for the same hardware: a continuous-rotation servo drives the plunger and
// counterweight, and a limit switch marks the fully-dived end of travel.
// A pressure sensor on I2C supplies depth.
//
// How it differs from a plain "depth error -> servo speed" loop:
//   A continuous-rotation servo sets plunger SPEED, but buoyancy depends on plunger
//   POSITION. Feeding a PID straight into servo speed puts two integrators between
//   the controller and the depth, which tends to oscillate. So this sketch splits it:
//     outer loop: PID on depth  -> desired plunger position (a buoyancy setting)
//     inner loop: proportional  -> servo speed that moves the plunger to that position
//   The plunger position is estimated by timing the servo (no position sensor) and is
//   re-zeroed every time the dive limit switch is hit.
//
// Sensor: a BMP280 is NOT waterproof. Set SENSOR to SENSOR_MS5837 (Blue Robotics
// Bar30/Bar02 library) for any run that goes underwater.
#include <Wire.h>
#include <Servo.h>

#define SENSOR_BMP280 0
#define SENSOR_MS5837 1
#define SENSOR SENSOR_BMP280          // <- change to SENSOR_MS5837 before going underwater

#if SENSOR == SENSOR_MS5837
  #include <MS5837.h>
  MS5837 pressureSensor;
#else
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 pressureSensor;
#endif

Servo myservo;

// ---------------- Pins (match the build) ----------------
const byte SERVO_PIN = 10;
const byte DIVE_STOP = 11;            // LOW when the plunger is at the fully-dived end

// ---------------- Servo ----------------
// 0 = dive (take in water), 180 = rise (push out). 90 = stopped; trim if yours creeps.
const int   SERVO_STOP        = 90;
const int   SERVO_FULL        = 90;   // deflection from stop at full speed (0 or 180)
const int   SERVO_RANGE       = 60;   // max deflection used by the controller
const int   SERVO_MIN_MOVE    = 8;    // smallest deflection that actually turns your servo (measure)

// ---------------- Plunger position estimate ----------------
// Time for the plunger to go end to end at FULL deflection. Measure this on the
// bench. The estimate assumes speed is proportional to deflection; check that too,
// since many servos saturate well before full deflection.
const float TRAVEL_MS         = 15500.0;
const float NEUTRAL_MS        = 7750.0;   // plunger position that gives neutral buoyancy; the
                                          // integral term trims the error in this guess
const float POS_DEADBAND_MS   = 150.0;    // stop the plunger when within this of the target
const float POS_KP            = 0.06;     // servo degrees per ms of position error (60 deg at 1 s)
const float END_MARGIN_MS     = 300.0;    // keep the commanded target this far from the ends

// ---------------- Depth control ----------------
const float TARGET_DEPTH_M    = 3.0;      // set per test
const float MAX_DEPTH_M       = 5.0;      // above this, rise at full range. Keep below hull/sensor rating.
const float KP_MS_PER_M       = 1500.0;   // plunger ms per meter of depth error (tune)
const float KI_MS_PER_MS      = 60.0;     // plunger ms per (meter * second) of error (tune)
const float KD_MS_PER_MPS     = 9000.0;   // plunger ms per m/s of vertical speed (tune)
const float I_LIMIT_MS        = 3000.0;   // clamp on the integral term
const float DEPTH_DEADBAND_M  = 0.10;     // treat error inside this as zero so it doesn't hunt

// ---------------- Sensor handling ----------------
const float FLUID_DENSITY     = 997.0;    // kg/m^3: fresh water ~997, seawater ~1025
const float GRAVITY           = 9.80665;
const float MAX_JUMP_M        = 1.0;      // a single reading farther than this from the filter is a spike
const byte  MAX_REJECTS       = 5;        // ...unless it persists this many loops in a row, then believe it
const float DEPTH_ALPHA       = 0.3;      // depth low-pass: lower = smoother, slower
const float VEL_ALPHA         = 0.3;      // vertical-speed low-pass
const unsigned long SENSOR_TIMEOUT_MS = 1000;  // no good reading for this long -> fail-safe rise

// ---------------- Timing ----------------
const unsigned long LOOP_MS   = 100;
const unsigned long HOMING_TIMEOUT_MS = 20000;

// ---------------- State ----------------
float surfacePa   = 0;
float depthM      = 0;
float velMps      = 0;                // + = descending
float iTermMs     = 0;
float plungerMs   = 0;                // 0 = fully dived, TRAVEL_MS = fully risen
byte  rejects     = 0;
unsigned long lastLoop = 0;
unsigned long lastGoodRead = 0;
bool  failSafe    = false;

// Returns true and sets pa on a good reading.
bool readPressurePa(float &pa) {
#if SENSOR == SENSOR_MS5837
  pressureSensor.read();
  pa = pressureSensor.pressure() * 100.0;      // library returns mbar
#else
  pa = pressureSensor.readPressure();
#endif
  return !isnan(pa) && pa > 30000.0 && pa < 200000.0;   // plausible absolute range
}

void halt(const char *msg) {
  myservo.write(SERVO_STOP);
  Serial.println(msg);
  while (1);
}

void setup() {
  Serial.begin(115200);
  pinMode(DIVE_STOP, INPUT_PULLUP);

#if SENSOR == SENSOR_MS5837
  Wire.begin();
  if (!pressureSensor.init()) halt("Pressure sensor not found");
  pressureSensor.setModel(MS5837::MS5837_30BA);   // use MS5837_02BA for the Bar02
  pressureSensor.setFluidDensity(FLUID_DENSITY);
#else
  if (!pressureSensor.begin(0x76)) halt("Pressure sensor not found");
#endif

  // Zero depth at the surface: average good readings.
  float sum = 0;
  int n = 0;
  for (int i = 0; i < 30 && n < 20; i++) {
    float pa;
    if (readPressurePa(pa)) { sum += pa; n++; }
    delay(50);
  }
  if (n < 20) halt("Could not get a stable surface pressure");
  surfacePa = sum / n;

  // Home the plunger to the dive limit switch.
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_STOP - SERVO_RANGE);
  unsigned long t0 = millis();
  while (digitalRead(DIVE_STOP) == HIGH) {
    if (millis() - t0 > HOMING_TIMEOUT_MS) halt("Homing timed out: check limit switch and servo");
  }
  myservo.write(SERVO_STOP);
  plungerMs = 0;

  Serial.println("time_ms,pressure_pa,depth_m,vel_mps,error_m,target_ms,plunger_ms,servo_cmd,flags");
  lastLoop = lastGoodRead = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  // ---- Sensor: validate, reject spikes, low-pass, differentiate ----
  float pa;
  bool ok = readPressurePa(pa);
  if (ok) {
    lastGoodRead = now;
    float raw = (pa - surfacePa) / (FLUID_DENSITY * GRAVITY);
    if (fabs(raw - depthM) < MAX_JUMP_M || rejects >= MAX_REJECTS) {
      if (rejects >= MAX_REJECTS) depthM = raw;              // persistent jump is real: re-seed
      float prev = depthM;
      depthM = DEPTH_ALPHA * raw + (1.0 - DEPTH_ALPHA) * depthM;
      velMps = VEL_ALPHA * ((depthM - prev) / dt) + (1.0 - VEL_ALPHA) * velMps;
      rejects = 0;
    } else {
      rejects++;
    }
  }
  failSafe = (now - lastGoodRead) > SENSOR_TIMEOUT_MS;

  // ---- Outer loop: PID on depth -> desired plunger position ----
  float error = TARGET_DEPTH_M - depthM;           // + = need to go deeper
  if (fabs(error) < DEPTH_DEADBAND_M) error = 0;

  float pTerm = KP_MS_PER_M * error;
  float dTerm = -KD_MS_PER_MPS * velMps;           // derivative on measurement (no kick on setpoint change)
  float targetMs = NEUTRAL_MS - (pTerm + iTermMs + dTerm);   // deeper -> toward 0 (dive end)

  float lo = END_MARGIN_MS, hi = TRAVEL_MS - END_MARGIN_MS;
  bool saturated = (targetMs < lo && error > 0) || (targetMs > hi && error < 0);
  if (!saturated && ok) {                          // conditional integration = anti-windup
    iTermMs = constrain(iTermMs + KI_MS_PER_MS * error * dt, -I_LIMIT_MS, I_LIMIT_MS);
  }
  targetMs = constrain(targetMs, lo, hi);

  // ---- Safety overrides: too deep or no sensor -> rise ----
  byte flags = 0;
  if (depthM > MAX_DEPTH_M) { targetMs = hi; flags |= 1; }
  if (failSafe)             { targetMs = hi; flags |= 2; }

  // ---- Inner loop: plunger position -> servo speed ----
  float posErr = targetMs - plungerMs;             // + = needs to move toward rise
  int cmd = SERVO_STOP;
  if (fabs(posErr) > POS_DEADBAND_MS) {
    int defl = (int)constrain(fabs(posErr) * POS_KP, (float)SERVO_MIN_MOVE, (float)SERVO_RANGE);
    if (flags) defl = SERVO_RANGE;                 // safety rise at full controller speed
    cmd = SERVO_STOP + (posErr > 0 ? defl : -defl);
  }

  // Hard limits: dive switch is real; the rise end is only estimated.
  if (cmd < SERVO_STOP && digitalRead(DIVE_STOP) == LOW) { cmd = SERVO_STOP; plungerMs = 0; }
  if (cmd > SERVO_STOP && plungerMs >= TRAVEL_MS)          cmd = SERVO_STOP;

  myservo.write(cmd);

  // Dead-reckon plunger position (+ = toward rise).
  plungerMs = constrain(plungerMs + ((float)(cmd - SERVO_STOP) / SERVO_FULL) * dt * 1000.0, 0.0, TRAVEL_MS);
  if (digitalRead(DIVE_STOP) == LOW) plungerMs = 0;   // re-zero whenever the switch is pressed

  // ---- Log for plotting ----
  Serial.print(now);          Serial.print(',');
  Serial.print(ok ? pa : NAN); Serial.print(',');
  Serial.print(depthM, 3);    Serial.print(',');
  Serial.print(velMps, 3);    Serial.print(',');
  Serial.print(error, 3);     Serial.print(',');
  Serial.print(targetMs, 0);  Serial.print(',');
  Serial.print(plungerMs, 0); Serial.print(',');
  Serial.print(cmd);          Serial.print(',');
  Serial.println(flags);
}
