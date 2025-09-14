// ==================== CONFIG ====================
const uint8_t hallPins[4] = {2, 3, 4, 5};   // 4 Hall sensors
volatile uint32_t wheelCount[4] = {0, 0, 0, 0};
volatile uint32_t lastPulseTime[4] = {0, 0, 0, 0};  // debounce timers

const float wheelRadius = 0.035;     // 3.5 cm
const uint16_t pulsesPerRev = 20;    // depends on your encoder/magnet setup
const float wheelBase = 0.15;        // 15 cm between left & right wheels
const float distancePerPulse = (2.0 * PI * wheelRadius) / pulsesPerRev;

const uint32_t debounce_us = 5000;   // 5 ms debounce (tune if needed)

// ==================== ODOMETRY ====================
struct Pose {
  float x;
  float y;
  float theta;
};
Pose robotPose = {0.0, 0.0, 0.0};
Pose kalmanPose = {0.0, 0.0, 0.0};

uint32_t lastCount[4] = {0, 0, 0, 0};

// ==================== KALMAN FILTER ====================
float P_x = 1, P_y = 1, P_theta = 1;
const float Q = 0.01;  // process noise
const float R = 0.05;  // measurement noise

float kalmanUpdate(float estimate, float measurement, float &P) {
  float P_pred = P + Q;
  float K = P_pred / (P_pred + R);
  float x_new = estimate + K * (measurement - estimate);
  P = (1 - K) * P_pred;
  return x_new;
}

// ==================== ISR HANDLERS ====================
void handlePulse(uint8_t idx) {
  uint32_t now = micros();
  if (now - lastPulseTime[idx] > debounce_us) {
    wheelCount[idx]++;
    lastPulseTime[idx] = now;
  }
}

void wheel0ISR() { handlePulse(0); }
void wheel1ISR() { handlePulse(1); }
void wheel2ISR() { handlePulse(2); }
void wheel3ISR() { handlePulse(3); }

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  for (int i = 0; i < 4; i++) {
    pinMode(hallPins[i], INPUT_PULLUP);
  }

  // Attach interrupts (FALLING edge = magnet arrival)
  attachInterrupt(digitalPinToInterrupt(hallPins[0]), wheel0ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallPins[1]), wheel1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallPins[2]), wheel2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallPins[3]), wheel3ISR, FALLING);

  Serial.println("Interrupt-driven odometry with Kalman filter");
}

// ==================== LOOP ====================
void loop() {
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();

  if (now - lastUpdate >= 100) { // 10 Hz update
    float dt = (now - lastUpdate) / 1000.0; // seconds
    lastUpdate = now;

    // Compute wheel distances
    float dLeft = ((wheelCount[0] - lastCount[0]) + (wheelCount[1] - lastCount[1])) / 2.0 * distancePerPulse;
    float dRight = ((wheelCount[2] - lastCount[2]) + (wheelCount[3] - lastCount[3])) / 2.0 * distancePerPulse;

    // Save counts
    for (int i = 0; i < 4; i++) lastCount[i] = wheelCount[i];

    // Update odometry
    float d = (dLeft + dRight) / 2.0;
    float dTheta = (dRight - dLeft) / wheelBase;

    robotPose.x += d * cos(robotPose.theta);
    robotPose.y += d * sin(robotPose.theta);
    robotPose.theta += dTheta;

    // Normalize theta
    if (robotPose.theta > PI) robotPose.theta -= 2 * PI;
    if (robotPose.theta < -PI) robotPose.theta += 2 * PI;

    // Kalman smoothing
    kalmanPose.x = kalmanUpdate(kalmanPose.x, robotPose.x, P_x);
    kalmanPose.y = kalmanUpdate(kalmanPose.y, robotPose.y, P_y);
    kalmanPose.theta = kalmanUpdate(kalmanPose.theta, robotPose.theta, P_theta);

    // Velocities
    float v = d / dt;      // m/s
    float w = dTheta / dt; // rad/s

    // Print
    Serial.print("Pose: ");
    Serial.print(kalmanPose.x, 3); Serial.print(", ");
    Serial.print(kalmanPose.y, 3); Serial.print(", ");
    Serial.print(kalmanPose.theta, 3); Serial.print(" | ");
    Serial.print("Vel: ");
    Serial.print(v, 3); Serial.print(", ");
    Serial.println(w, 3);
  }
}
