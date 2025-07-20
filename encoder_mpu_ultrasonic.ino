#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050 setup
Adafruit_MPU6050 mpu;

// Filter parameters
float alpha = 0.96;

// Orientation angles
float pitch_accel, roll_accel;
float pitch_gyro = 0, roll_gyro = 0, yaw_gyro = 0;
float pitch_filter = 0, roll_filter = 0;

// Calibration offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

unsigned long prevTime;

// Ultrasonic sensor pins
#define TRIG_PIN 18
#define ECHO_PIN 19

// Rotary encoder pins
#define CLK 5   // Clock pin
#define DT 4    // Data pin
#define SW 27   // Switch pin

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastSerialOutput = 0;
const unsigned long SENSOR_INTERVAL = 50;    // Read sensors every 50ms (20Hz)
const unsigned long OUTPUT_INTERVAL = 1000;  // Output data every 1000ms (1Hz)

// Rotary encoder variables
volatile int encoder_counter = 0;
volatile bool encoder_changed = false;
int lastCLK = HIGH;
int lastDT = HIGH;
unsigned long lastEncoderTime = 0;

// Button handling
unsigned long lastButtonPress = 0;
bool buttonPressed = false;

// Software-based encoder checking (no timer needed)
void checkEncoder() {
  int currentCLK = digitalRead(CLK);
  int currentDT = digitalRead(DT);
  unsigned long currentTime = millis();
  
  // Check if CLK state changed
  if (currentCLK != lastCLK) {
    // Debounce - ignore changes within 5ms
    if (currentTime - lastEncoderTime > 5) {
      // Check for falling edge (HIGH to LOW)
      if (lastCLK == HIGH && currentCLK == LOW) {
        // Determine direction based on DT state
        if (currentDT == HIGH) {
          encoder_counter++;  // Clockwise
        } else {
          encoder_counter--;  // Counterclockwise
        }
        encoder_changed = true;
      }
      lastEncoderTime = currentTime;
    }
  }
  
  // Update states
  lastCLK = currentCLK;
  lastDT = currentDT;
}

// === Calibration Function ===
void calibrateSensors() {
  const int samples = 200;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  Serial.println("Calibrating MPU6050... Keep sensor still!");

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;

    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;

    // Check encoder during calibration to not miss clicks
    checkEncoder();
    delay(10);
  }

  gyroX_offset = sum_gx / samples;
  gyroY_offset = sum_gy / samples;
  gyroZ_offset = sum_gz / samples;

  accelX_offset = sum_ax / samples;
  accelY_offset = sum_ay / samples;
  accelZ_offset = (sum_az / samples) - 9.81;

  Serial.println("Calibration complete!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Ultrasonic sensor initialization
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Rotary encoder initialization
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);

  // Initialize encoder states
  lastCLK = digitalRead(CLK);
  lastDT = digitalRead(DT);

  delay(500);
  calibrateSensors();
  prevTime = millis();

  Serial.println("All sensors initialized successfully!");
  Serial.println("Sensor Reading: 20Hz | Encoder Check: ~1000Hz | Data Output: 1Hz");
  Serial.println("================================================");
}

void readSensors() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // === MPU6050 Reading ===
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration
  float corrected_ax = a.acceleration.x - accelX_offset;
  float corrected_ay = a.acceleration.y - accelY_offset;
  float corrected_az = a.acceleration.z - accelZ_offset;

  float corrected_gx = g.gyro.x - gyroX_offset;
  float corrected_gy = g.gyro.y - gyroY_offset;
  float corrected_gz = g.gyro.z - gyroZ_offset;

  // Calculate accelerometer angles
  pitch_accel = atan2(corrected_ay, corrected_az) * 180 / PI;
  roll_accel = atan2(-corrected_ax, sqrt(corrected_ay * corrected_ay + corrected_az * corrected_az)) * 180 / PI;

  // Calculate gyroscope angles (integrated)
  pitch_gyro += corrected_gx * dt * 180 / PI;
  roll_gyro  += corrected_gy * dt * 180 / PI;
  yaw_gyro   += corrected_gz * dt * 180 / PI;

  // Apply complementary filter
  pitch_filter = alpha * (pitch_filter + corrected_gx * dt * 180 / PI) + (1 - alpha) * pitch_accel;
  roll_filter  = alpha * (roll_filter + corrected_gy * dt * 180 / PI) + (1 - alpha) * roll_accel;
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.0343 / 2;
  
  return (duration == 0) ? -1 : distance;
}

void checkButton() {
  if (digitalRead(SW) == LOW) {
    if (millis() - lastButtonPress > 300) {
      buttonPressed = true;
      lastButtonPress = millis();
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check encoder continuously for best responsiveness
  checkEncoder();
  
  // Read sensors every 50ms (20Hz)
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    readSensors();
    lastSensorRead = currentTime;
  }
  
  // Check button state
  checkButton();
  
  // Output data every 1000ms (1Hz)
  if (currentTime - lastSerialOutput >= OUTPUT_INTERVAL) {
    
    // Get current distance reading
    float distance = readUltrasonic();
    
    // Get current encoder state
    int current_encoder_count = encoder_counter;
    bool encoder_activity = encoder_changed;
    encoder_changed = false; // Reset flag
    
    // === Serial Output ===
    Serial.println("=== SENSOR READINGS ===");
    
    // MPU6050 data
    Serial.printf("Pitch[accel]: %.2f°, Roll[accel]: %.2f°\n", pitch_accel, roll_accel);
    Serial.printf("Pitch[gyro]: %.2f°, Roll[gyro]: %.2f°, Yaw[gyro]: %.2f°\n", pitch_gyro, roll_gyro, yaw_gyro);
    Serial.printf("Pitch[filter]: %.2f°, Roll[filter]: %.2f°\n", pitch_filter, roll_filter);

    // Ultrasonic data
    if (distance < 0) {
      Serial.println("Distance: No obstacle detected");
    } else {
      Serial.printf("Distance: %.1f cm\n", distance);
    }

    // Rotary encoder data
    Serial.printf("Encoder Position: %d", current_encoder_count);
    if (encoder_activity) {
      Serial.print(" [CHANGED]");
    }
    Serial.println();
    
    if (buttonPressed) {
      Serial.println("*** BUTTON PRESSED! ***");
      buttonPressed = false;
    }

    Serial.println("========================");
    Serial.println();
    
    lastSerialOutput = currentTime;
  }
  
  // Very small delay to prevent watchdog issues while maintaining responsiveness
  delayMicroseconds(100);
}
