#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_PIN 5
#define IN2 6
#define IN1 7

// --- encoder position (updated in ISR) ---
volatile int posi = 0;

float pwr = 0.0f;

// --- PID state ---
float eprev = 0;
float eintegral = 0;
const float I_MAX = 500.0f;  // tune this (units: error-seconds)
// start small then increase till Ess goes away w/o overshoot

// --- timing ---
const uint32_t PID_PERIOD_US = 1000;    // 1 kHz PID
const uint32_t LOG_PERIOD_US = 10000;   // 100 Hz logging
const uint32_t STEP_PERIOD_MS = 1000;   // wait 1s after reaching target before stepping again

uint32_t lastPidUs = 0;
uint32_t lastLogUs = 0;
uint32_t lastStepMs = 0;

// --- settle detection ---
const int   POS_TOL = 5;            // counts (your current band)
const int   VEL_TOL = 1;            // counts per PID tick (1ms). Tune.
const uint16_t SETTLE_MS = 1000;     // must be stable for 200ms
uint16_t settleCount = 0;           // counts PID ticks in a row
int lastPosForSettle = 0;
bool settled;

// --- target management ---
int targetDeg = 0;     // target in degrees (0, 90, 180, ...)
int target = 0;        // target in motor position units (0..239)

// 240 counts per revolution assumed by your modulo
const float CONVERSION = 240.0f / 360.0f;  // degrees -> motor pos (0.6666...)

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // initialize target
  targetDeg = 0;
  target = convert_to_motor_pos(targetDeg, CONVERSION);
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ---- Read position atomically once per loop ----
  int pos;
  noInterrupts();
  pos = posi;
  interrupts();

  // ---- Target stepping logic ----
  // Only step to the next 90° after you've arrived and waited STEP_PERIOD_MS.
  static bool wasSettled = false;

  if (settled) {
    if (!wasSettled) {
      // just became settled: start "post-settle" timer
      lastStepMs = nowMs;
      wasSettled = true;
    }

    if ((uint32_t)(nowMs - lastStepMs) >= STEP_PERIOD_MS) {
      // step target
      if (targetDeg == 0) targetDeg = 135;
      else targetDeg = 135;

      target = convert_to_motor_pos(targetDeg, CONVERSION);

      // reset PID + settle detector for the next move
      eintegral = 0;
      eprev = 0;
      settleCount = 0;
      wasSettled = false;
    }
  } else {
    wasSettled = false;
  }

  // ---- PID at fixed rate ----
  if ((uint32_t)(nowUs - lastPidUs) >= PID_PERIOD_US) {
    lastPidUs += PID_PERIOD_US;              // keeps cadence stable
    const float deltaT = PID_PERIOD_US / 1.0e6f;

    // PID constants (make these globals if you plan to tune live)
    const float kp = 15.0f;
    const float kd = 0.35f;
    const float ki = 10.0f;

    // error
    const int e = pos - target;

    // derivative
    const float dedt = (e - eprev) / deltaT;

    // integral
    eintegral += e * deltaT;
    if (eintegral > I_MAX) eintegral = I_MAX;
    else if (eintegral < -I_MAX) eintegral = -I_MAX;

    // control signal
    const float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    pwr = fabs(u);
    if (pwr > 255) pwr = 255;

    // motor direction
    const int dir = (u < 0) ? -1 : 1;

    // signal the motor
    setMotor(dir, (int)pwr, PWM_PIN, IN1, IN2);

    // store previous error
    eprev = e;

    // "velocity" estimate: how much position changed since last PID tick
    int dpos = pos - lastPosForSettle;
    lastPosForSettle = pos;

    bool inPosBand = (abs(e) <= POS_TOL);
    bool slowEnough = (abs(dpos) <= VEL_TOL);

    if (inPosBand && slowEnough) {
      if (settleCount < 60000) settleCount++;   // prevent overflow
    } else {
      settleCount = 0;
    }

    settled = (settleCount >= SETTLE_MS);  // because 1 tick = 1 ms at 1kHz
  }

  // ---- Logging at fixed rate (won't stall PID) ----
  if ((uint32_t)(nowUs - lastLogUs) >= LOG_PERIOD_US) {
    lastLogUs += LOG_PERIOD_US;

    // Serial.print("MIN:");
    // Serial.print(-180); // To freeze the lower limit
    // Serial.print(",");
    // Serial.print("MAX:");
    // Serial.print(180); // To freeze the upper limit
    // Serial.print(",");

    // Serial.print("Target:"); // comment if sending to python
    Serial.print(targetDeg);
    Serial.print(" ");
    // Serial.print("Actual:"); // comment if sending to python
    Serial.print(int(pos / CONVERSION));
    Serial.print(" ");
    Serial.println(int(pwr));

    //}
  }
}

void setMotor(int dir, int pwmVal, int pin, int in1, int in2) {
  analogWrite(pin, pwmVal);

  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) posi++;
  else       posi--;
}


// desired = degrees, conversion = counts/degree
int convert_to_motor_pos(float desired, float conversion) {
  int p = (int)lround(desired * conversion);
  //p %= 240;
  //if (p < 0) p += 240;     // keep it 0..239 even if negative
  return p;
}