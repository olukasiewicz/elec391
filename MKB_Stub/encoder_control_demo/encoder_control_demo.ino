#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_PIN 5
#define IN2 6
#define IN1 7

// --- encoder position and filtering (updated in ISR) ---
volatile int posi = 0; // updated in ISR
float pos_f = 0.20; // filtered position (counts)
bool pos_f_init = false; 
const float ALPHA = 0.50; // weighting of filter: lower value is smoother but more lag

float pwr = 0.0f;

// --- PID state ---
float eprev = 0;
float eintegral = 0;
const float I_MAX = 500.0f;  // tune this (units: error-seconds)

// --- Derivative filter state ---
float dpos_f = 0.0f;            // filtered velocity estimate (counts/s)
const float DERIV_TAU = 0.0221f;  // calculated in MATLAB

// --- timing ---
const uint32_t PID_PERIOD_US = 1000;    // 1 kHz PID
const uint32_t LOG_PERIOD_US = 1000;     // 5 kHz logging
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
bool settled = false;
float lastPosForDeriv = 0.0f;
float lastPosForSettleTick = 0.0f;

// --- target management ---
int targetDeg = 0;     // target in degrees (0, 90, 180, ...)
int target = 0;        // target in motor position units (0..239) 
const float CONVERSION = 240.0f / 360.0f; // 240 counts per revolution

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

  // ---- read position once per loop ----
  int pos;
  noInterrupts();
  pos = posi;
  interrupts();

  // ---- EWMA filter on position ----
  if (!pos_f_init) {
    pos_f = float(pos);
    pos_f_init = true;
  }
  pos_f += ALPHA * ((float)pos - pos_f);

  // ---- Target stepping logic ----
  // Only step to the next position after you've arrived and waited STEP_PERIOD_MS.
  static bool wasSettled = false;

  if (settled) {
    if (!wasSettled) {
      lastStepMs = nowMs;
      wasSettled = true;
    }

    if ((uint32_t)(nowMs - lastStepMs) >= STEP_PERIOD_MS) {
      // step target
      if (targetDeg == 0) targetDeg = 135;
      else if (targetDeg == 135) targetDeg = 45;
      else if (targetDeg == 45) targetDeg = 0;
      else targetDeg = 0;

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

    // PID constants
    const float kp = 15.0f;
    const float ki = 23.1f;
    const float kd = 0.34f;

    // --- Error (counts) ---
    const float e = (float)(target - pos_f);

    // --- Derivative on measurement (counts/s), with 1st-order LPF ---
    const float dpos_raw = (pos_f - lastPosForDeriv) / deltaT;
    lastPosForDeriv = pos_f;

    const float alpha = DERIV_TAU / (DERIV_TAU + deltaT);
    dpos_f = alpha * dpos_f + (1.0f - alpha) * dpos_raw;

    // --- PID (unsaturated) ---
    const float u_unsat = kp * e + ki * eintegral - kd * dpos_f;

    // --- Saturate command to PWM limits ---
    float u = u_unsat;
    if (u > 255.0f)       u = 255.0f;
    else if (u < -255.0f) u = -255.0f;

    // If we're saturated stop integrator.
    // Otherwise, integrate normally.
    const bool saturated_high = (u_unsat > 255.0f) && (e > 0.0f);
    const bool saturated_low  = (u_unsat < -255.0f)  && (e < 0.0f);
    if (!(saturated_high || saturated_low)) {
      eintegral += e * deltaT;
    }

    // --- Drive motor ---
    pwr = fabs(u);
    const int dir = (u < 0.0f) ? -1 : 1;
    setMotor(dir, (int)pwr, PWM_PIN, IN1, IN2);

    // --- Settle detection (counts per tick) ---
    const float dpos_tick_f = pos_f - lastPosForSettleTick;
    lastPosForSettleTick = pos_f;

    const bool inPosBand  = (fabs(e) <= (float)POS_TOL);
    const bool slowEnough = (fabs(dpos_tick_f) <= (float)VEL_TOL);

    if (inPosBand && slowEnough) {
      if (settleCount < 60000) settleCount++;
    } else {
      settleCount = 0;
    }
    settled = (settleCount >= SETTLE_MS);
  }

  // ---- logging at fixed rate ----
  if ((uint32_t)(nowUs - lastLogUs) >= LOG_PERIOD_US) {
    lastLogUs += LOG_PERIOD_US;

    // Serial.print("MIN:");
    // Serial.print(-180); // To freeze the lower limit
    // Serial.print(",");
    // Serial.print("MAX:");
    // Serial.print(180); // To freeze the upper limit
    // Serial.print(",");

    // Serial.print("Target:"); // comment if sending to python
    // Serial.print(targetDeg);
    // Serial.print(" ");
    // // Serial.print("Actual:"); // comment if sending to python
    // Serial.print(int(posf / CONVERSION));
    // Serial.print(" ");
    // Serial.println(int(pwr));
  }
}


void setMotor(int dir, int pwmVal, int pin, int in1, int in2) {
  analogWrite(pin, pwmVal);

  if (dir == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == 1) {
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
  return p;
}