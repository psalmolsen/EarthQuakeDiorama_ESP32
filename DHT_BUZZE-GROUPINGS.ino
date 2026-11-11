#include <Wire.h>
#include <math.h>

// ================= PINS =================
#define BUZZER_PIN 4
#define RELAY_PIN 15
#define BUZZER_ACTIVE_LOW false   // set true if the buzzer sounds when it should be silent
#define RELAY_ACTIVE_LOW true     // flip if the relay is backwards
#define LED_ACTIVE_LOW false      // set true if the LEDs light when they should be off
#define SDA_PIN 21
#define SCL_PIN 22

// 4 LEDs, one pin each
const int LED_PINS[] = {33, 27, 17, 19};
const int LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

// ================= SETTINGS =================
const float MIN_TRIGGER = 0.02;     // minimum RMS trigger (g). Raise = LESS sensitive
const float NOISE_MULT = 4.0;       // trigger = measured noise x this
const float RELEASE_RATIO = 0.5;
const int RELEASE_COUNT = 5;        // calm windows needed to clear the alarm
const int WINDOW_MS = 100;
const int FROZEN_LIMIT = 15;        // identical windows in a row = sensor frozen
const unsigned long HEALTH_MS = 500; // how often to check the chip
const unsigned long LED_PULSE_MS = 150; // LED on/off time while alarming (lower = faster)
const bool LED_ALTERNATE = false;   // false = all LEDs blink together, true = alternate in pairs

// ================= MPU6050 =================
const int MPU_ADDR = 0x68;
const float ACCEL_SCALE = 16384.0;  // ±2 g
const int MAX_SAMPLES = 150;

// ================= VARIABLES =================
float TRIGGER_ON = 0.02;
float JOLT_ON = 0.06;
float noiseRMS = 0, noiseJolt = 0;
float vibrationRMS = 0, vibrationJolt = 0;
bool vibrating = false;
int calmCount = 0;
int frozenCount = 0;
unsigned long lastHealth = 0;
unsigned long resetCount = 0;

// ================= OUTPUTS =================
void setBuzzer(bool on) {
  if (BUZZER_ACTIVE_LOW) digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
  else                   digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void setRelay(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

void setLedPin(int index, bool on) {
  if (LED_ACTIVE_LOW) digitalWrite(LED_PINS[index], on ? LOW : HIGH);
  else                digitalWrite(LED_PINS[index], on ? HIGH : LOW);
}

// LEDs pulse (emergency blink) while vibrating, all OFF otherwise
void updateLed() {
  if (!vibrating) {
    for (int i = 0; i < LED_COUNT; i++) setLedPin(i, false);
    return;
  }

  bool phase = ((millis() / LED_PULSE_MS) % 2) == 0;
  for (int i = 0; i < LED_COUNT; i++) {
    bool on = phase;
    if (LED_ALTERNATE && (i % 2 == 1)) on = !phase;   // every second LED is opposite
    setLedPin(i, on);
  }
}

void updateOutputs() {
  if (vibrating) {          // vibration: buzzer ON, relay OFF (LEDs pulse in updateLed)
    setBuzzer(true);
    setRelay(false);
  } else {                  // level NONE: buzzer OFF, relay ON
    setBuzzer(false);
    setRelay(true);
  }
  updateLed();
}

// ================= MPU6050 =================
void writeReg(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

int readReg(byte reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, 1, true) != 1) return -1;
  return Wire.read();
}

// Full (re)initialization of the sensor and the I2C bus
bool initMPU() {
  Wire.end();
  delay(20);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  writeReg(0x6B, 0x80);   // reset
  delay(300);
  writeReg(0x6B, 0x01);   // wake up, gyro clock
  delay(300);
  writeReg(0x6C, 0x00);   // all axes on
  writeReg(0x1C, 0x00);   // ±2 g
  writeReg(0x1A, 0x02);   // low-pass filter ~94 Hz
  delay(100);
  return true;
}

// Checks whether the chip fell asleep or stopped answering. Reinitializes if so.
void checkSensorHealth(const char* reason) {
  int pwr = readReg(0x6B);
  bool bad = (pwr < 0) || (pwr & 0x40);   // no answer, or sleep bit set

  if (bad || reason != nullptr) {
    resetCount++;
    Serial.print(">>> SENSOR RESET (");
    Serial.print(reason ? reason : (pwr < 0 ? "no answer" : "chip was asleep"));
    Serial.print(") - reinitializing. Total resets: ");
    Serial.println(resetCount);

    if (!initMPU()) {
      Serial.println(">>> MPU6050 NOT RESPONDING - check wires / power");
    }
    frozenCount = 0;
  }
}

// Reads X and Y only (the Z axis on this module is stuck, so it is ignored)
bool readXY(float &x, float &y) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  x = (int16_t)((b[0] << 8) | b[1]) / ACCEL_SCALE;
  y = (int16_t)((b[2] << 8) | b[3]) / ACCEL_SCALE;
  return true;
}

// RMS wobble around the window average + biggest jolt between samples
bool measureVibration(int duration) {
  static float sx[MAX_SAMPLES], sy[MAX_SAMPLES];
  int n = 0;
  unsigned long start = millis();

  while (millis() - start < (unsigned long)duration && n < MAX_SAMPLES) {
    float x, y;
    if (readXY(x, y)) { sx[n] = x; sy[n] = y; n++; }
    updateLed();            // keep the LEDs pulsing while sampling
    delay(1);
  }
  if (n < 10) return false;

  float mx = 0, my = 0;
  for (int i = 0; i < n; i++) { mx += sx[i]; my += sy[i]; }
  mx /= n; my /= n;

  float sum = 0, jolt = 0;
  for (int i = 0; i < n; i++) {
    float dx = sx[i] - mx, dy = sy[i] - my;
    sum += dx * dx + dy * dy;
    if (i > 0) {
      float jx = sx[i] - sx[i - 1], jy = sy[i] - sy[i - 1];
      float j = sqrt(jx * jx + jy * jy);
      if (j > jolt) jolt = j;
    }
  }
  vibrationRMS = sqrt(sum / n);
  vibrationJolt = jolt;
  return true;
}

void updateJoltThreshold() {
  JOLT_ON = max(TRIGGER_ON * 3.0f, noiseJolt * 2.0f);
}

void calibrateNoise() {
  Serial.println();
  Serial.println("CALIBRATING - keep the sensor STILL for 4 seconds");
  delay(2000);

  float hiRMS = 0, hiJolt = 0;
  for (int i = 0; i < 20; i++) {
    if (measureVibration(WINDOW_MS)) {
      if (vibrationRMS > hiRMS) hiRMS = vibrationRMS;
      if (vibrationJolt > hiJolt) hiJolt = vibrationJolt;
    }
    delay(10);
  }
  noiseRMS = hiRMS;
  noiseJolt = hiJolt;
  TRIGGER_ON = max(MIN_TRIGGER, noiseRMS * NOISE_MULT);
  updateJoltThreshold();

  Serial.println("CALIBRATION DONE");
  Serial.print("Noise RMS: ");   Serial.print(noiseRMS, 5);
  Serial.print(" g | Noise jolt: "); Serial.println(noiseJolt, 5);
  Serial.print("Trigger RMS: "); Serial.print(TRIGGER_ON, 4);
  Serial.print(" | Trigger jolt: "); Serial.println(JOLT_ON, 4);
  Serial.println();
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      TRIGGER_ON -= 0.005; if (TRIGGER_ON < 0.005) TRIGGER_ON = 0.005;
      updateJoltThreshold();
      Serial.print("More sensitive. Trigger = "); Serial.println(TRIGGER_ON, 4);
    } else if (c == '-') {
      TRIGGER_ON += 0.005; if (TRIGGER_ON > 1.0) TRIGGER_ON = 1.0;
      updateJoltThreshold();
      Serial.print("Less sensitive. Trigger = "); Serial.println(TRIGGER_ON, 4);
    } else if (c == 'c' || c == 'C') {
      calibrateNoise();
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  for (int i = 0; i < LED_COUNT; i++) pinMode(LED_PINS[i], OUTPUT);
  vibrating = false;
  updateOutputs();

  delay(1500);

  if (!initMPU()) {
    Serial.println("ERROR: MPU6050 NOT FOUND!");
    while (true) delay(1000);
  }

  Serial.println();
  Serial.println("=== EARTHQUAKE / VIBRATION MONITOR ===");
  calibrateNoise();
  Serial.println("READY.  [+] more sensitive  [-] less sensitive  [C] recalibrate");
  lastHealth = millis();
}

// ================= LOOP =================
void loop() {
  processSerialCommands();

  // Watchdog 1: is the chip still awake and answering?
  if (millis() - lastHealth >= HEALTH_MS) {
    lastHealth = millis();
    checkSensorHealth(nullptr);
  }

  if (!measureVibration(WINDOW_MS)) {
    Serial.println("SENSOR READ ERROR - trying to recover");
    checkSensorHealth("read error");
    return;
  }

  // Watchdog 2: readings exactly identical for too long = sensor frozen
  if (vibrationRMS == 0.0f && vibrationJolt == 0.0f) {
    frozenCount++;
    if (frozenCount >= FROZEN_LIMIT) {
      checkSensorHealth("frozen readings");
    }
  } else {
    frozenCount = 0;
  }

  float rmsOff = TRIGGER_ON * RELEASE_RATIO;
  float joltOff = JOLT_ON * RELEASE_RATIO;

  bool detected = (vibrationRMS >= TRIGGER_ON) || (vibrationJolt >= JOLT_ON);
  bool stopped  = (vibrationRMS <= rmsOff) && (vibrationJolt <= joltOff);

  if (!vibrating) {
    if (detected) {
      vibrating = true;
      calmCount = 0;
      Serial.println("!!! VIBRATION DETECTED !!!");
    }
  } else {
    if (stopped) {
      if (++calmCount >= RELEASE_COUNT) {
        vibrating = false;
        calmCount = 0;
        Serial.println("VIBRATION CLEARED");
      }
    } else {
      calmCount = 0;
    }
  }

  updateOutputs();

  Serial.print("RMS: ");  Serial.print(vibrationRMS, 4);
  Serial.print(" (trig "); Serial.print(TRIGGER_ON, 3);
  Serial.print(") | JOLT: "); Serial.print(vibrationJolt, 4);
  Serial.print(" (trig "); Serial.print(JOLT_ON, 3);
  Serial.print(") | Level: ");
  if (vibrating) {
    Serial.println("VIBRATION | buzzer ON, 4 LEDs PULSING, relay OFF");
  } else {
    Serial.println("NONE | buzzer OFF, LEDs OFF, relay ON");
  }
}