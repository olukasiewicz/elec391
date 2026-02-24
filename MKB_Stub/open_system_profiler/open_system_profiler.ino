// ===================== Your Setup =====================
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_PIN 5
#define IN2 6
#define IN1 7

// --- encoder position (updated in ISR) ---
volatile int posi = 0;

unsigned long startTime;
int pwmValue = 0;  // 0-255


void setup() {
  Serial.begin(115200);
  
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.print("time,");
  Serial.print("pwmValue,");
  Serial.println("position");

  startTime = millis();
}

void loop() {

  unsigned long t = millis() - startTime;

  if (t > 650) {
    pwmValue = 0;
  }
  else if (t > 500){
    pwmValue = 255;
  }

  if (t > 500) {
    setMotor(-1, pwmValue, PWM_PIN, IN1, IN2);   // step at 0.5s
  }

  Serial.print(t);
  Serial.print(",");
  Serial.print(pwmValue);
  Serial.print(",");
  Serial.println(posi);

  delay(1); // 200 Hz sample rate
}

// -------------------- Motor helper --------------------
void setMotor(int dir, int pwmVal, int pin, int in1, int in2) {
  pwmVal = constrain(pwmVal, 0, 255);
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

// -------------------- Encoder ISR --------------------
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) posi++;
  else       posi--;
}