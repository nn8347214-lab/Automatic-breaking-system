# Automatic-breaking-system
Automatic Braking System using Arduino – Designed and implemented an obstacle-detection braking system with Arduino UNO, ultrasonic sensor, relay module, gear motor, and buzzer. Programmed distance-based control logic to cut motor power and trigger alerts, enhancing safety and automation in prototype vehicles.
// ---------- Pins ----------
const uint8_t PIN_TRIG   = 9;
const uint8_t PIN_ECHO   = 8;
const uint8_t PIN_RELAY  = 7;
const uint8_t PIN_BUZZER = 6;

// ---------- Config ----------
const bool RELAY_ACTIVE_LOW = true; // set true if IN=LOW energizes relay (common on many modules)
const unsigned BRAKE_DIST_CM    = 25; // start braking at/inside this distance
const unsigned CLEAR_HYST_CM    = 8;  // release brake when distance >= BRAKE_DIST_CM + this
const uint8_t  SAMPLES          = 7;  // median of N samples (odd number)
const unsigned MEAS_INTERVAL_MS = 60; // spacing between measures
const unsigned RELAY_MIN_MS     = 400; // min on/off time to reduce chatter
const unsigned MAX_SENSE_CM     = 300; // cap readings

// ---------- State ----------
bool brakeOn = false;
unsigned long lastRelayChange = 0;

// ---------- Helpers ----------
inline void setRelay(bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_RELAY, level);
}

unsigned long pulseInSafe(uint8_t pin, uint8_t state, unsigned long timeout) {
  // Wrapper to avoid blocking too long
  return pulseIn(pin, state, timeout);
}

unsigned measureDistanceCm() {
  // Trigger 10us pulse
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  // Echo pulse width (timeout ~ 30 ms = ~5 m)
  unsigned long dur = pulseInSafe(PIN_ECHO, HIGH, 30000UL);
  if (dur == 0) return MAX_SENSE_CM; // timeout → treat as “far”
  // Sound speed approx 343 m/s → distance cm = (dur in µs)/58
  unsigned dist = (unsigned)(dur / 58UL);
  if (dist > MAX_SENSE_CM) dist = MAX_SENSE_CM;
  return dist;
}

unsigned medianDistanceCm(uint8_t n) {
  // Small insertion sort for tiny arrays
  unsigned vals[15];                 // supports up to 15 samples
  if (n > 15) n = 15;
  for (uint8_t i=0; i<n; ++i) {
    vals[i] = measureDistanceCm();
    delay(MEAS_INTERVAL_MS);
  }
  for (uint8_t i=1; i<n; ++i) {      // sort
    unsigned key = vals[i]; int j = i-1;
    while (j>=0 && vals[j] > key) { vals[j+1] = vals[j]; j--; }
    vals[j+1] = key;
  }
  return vals[n/2];
}

void shortBeep() {
  tone(PIN_BUZZER, 3000, 80); // short chirp
}

void brakeTone(bool on) {
  if (on) {
    tone(PIN_BUZZER, 1200);   // continuous while braking
  } else {
    noTone(PIN_BUZZER);
  }
}

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  // Safe defaults: brake ON at boot for a moment
  setRelay(true); brakeOn = true; brakeTone(true);
  delay(500);
  // Release brake to start normal operation
  setRelay(false); brakeOn = false; brakeTone(false);
}

void loop() {
  unsigned d = medianDistanceCm(SAMPLES);

  // Pre-warning beep if near but not braking
  if (!brakeOn && d <= (BRAKE_DIST_CM + 10) && d > BRAKE_DIST_CM) {
    shortBeep();
  }

  unsigned long now = millis();
  bool canToggle = (now - lastRelayChange) >= RELAY_MIN_MS;

  if (!brakeOn && d <= BRAKE_DIST_CM && canToggle) {
    // Engage brake
    setRelay(true); brakeOn = true; brakeTone(true);
    lastRelayChange = now;
  } else if (brakeOn && d >= (BRAKE_DIST_CM + CLEAR_HYST_CM) && canToggle) {
    // Release brake
    setRelay(false); brakeOn = false; brakeTone(false);
    lastRelayChange = now;
  }

  // Small loop delay
  delay(10);
}
