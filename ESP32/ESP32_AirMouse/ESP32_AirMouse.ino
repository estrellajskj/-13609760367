/******接线说明********/
/*左键按钮————GPIO11
  右键按钮————GPIO12
  MPU6050 SCL————GPIO8（PB8）
  MPU6050 SDA————GPIO9（PB9）*/
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <math.h>

#define LEFTBUTTON 11  //左键
#define RIGHTBUTTON 12  //右键
#define I2C_SCL 8
#define I2C_SDA 9
#define SPEED 5.0f

constexpr uint32_t SAMPLE_PERIOD_MS = 10;
constexpr uint16_t CALIBRATION_SAMPLES = 120;
constexpr float FILTER_ALPHA = 0.25f;
constexpr float GYRO_DEADBAND = 0.025f;  // rad/s

Adafruit_MPU6050 mpu;
BleMouse bleMouse;

bool sleepMPU = true;
bool leftButtonWasPressed = false;
bool rightButtonWasPressed = false;
float gyroZOffset = 0.0f;
float gyroXOffset = 0.0f;
float filteredMouseX = 0.0f;
float filteredMouseY = 0.0f;
float fractionalMouseX = 0.0f;
float fractionalMouseY = 0.0f;
uint32_t previousSampleMs = 0;

void calibrateGyro() {
  sensors_event_t a, g, temp;
  float zSum = 0.0f;
  float xSum = 0.0f;

  Serial.println("Keep the mouse still: calibrating gyro...");
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    mpu.getEvent(&a, &g, &temp);
    zSum += g.gyro.z;
    xSum += g.gyro.x;
    delay(5);
  }

  gyroZOffset = zSum / CALIBRATION_SAMPLES;
  gyroXOffset = xSum / CALIBRATION_SAMPLES;
  filteredMouseX = 0.0f;
  filteredMouseY = 0.0f;
  fractionalMouseX = 0.0f;
  fractionalMouseY = 0.0f;
  previousSampleMs = millis();
  Serial.println("Gyro calibration complete.");
}

int8_t takeWholeCounts(float &fractionalCounts) {
  int16_t wholeCounts = static_cast<int16_t>(fractionalCounts);
  if (wholeCounts > 127) wholeCounts = 127;
  if (wholeCounts < -127) wholeCounts = -127;
  fractionalCounts -= wholeCounts;
  return static_cast<int8_t>(wholeCounts);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);

  bleMouse.begin();

  Wire.begin(I2C_SDA, I2C_SCL);

  delay(1000);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Sleep until Bluetooth connects to avoid disturbing BLE startup.
  mpu.enableSleep(sleepMPU);
}

void loop() {
  if (!bleMouse.isConnected()) {
    if (!sleepMPU) {
      mpu.enableSleep(true);
      sleepMPU = true;
    }
    return;
  }

  if (sleepMPU) {
    delay(3000);
    Serial.println("MPU6050 awakened!");
    sleepMPU = false;
    mpu.enableSleep(false);
    delay(500);
    calibrateGyro();
  }

  const uint32_t now = millis();
  const uint32_t elapsedMs = now - previousSampleMs;
  if (elapsedMs < SAMPLE_PERIOD_MS) {
    delay(SAMPLE_PERIOD_MS - elapsedMs);
    return;
  }
  previousSampleMs = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gyroZ = g.gyro.z - gyroZOffset;
  float gyroX = g.gyro.x - gyroXOffset;
  filteredMouseX += FILTER_ALPHA * (gyroZ - filteredMouseX);
  filteredMouseY += FILTER_ALPHA * (gyroX - filteredMouseY);

  if (fabsf(filteredMouseX) < GYRO_DEADBAND) filteredMouseX = 0.0f;
  if (fabsf(filteredMouseY) < GYRO_DEADBAND) filteredMouseY = 0.0f;

  // Gyro values are rad/s. SPEED preserves the existing sensitivity at 100 Hz;
  // fractional counts are carried forward instead of being lost to integer truncation.
  const float sampleScale = SPEED * (elapsedMs / 10.0f);
  fractionalMouseX += -filteredMouseX * sampleScale;
  fractionalMouseY += -filteredMouseY * sampleScale;
  const int8_t mouseX = takeWholeCounts(fractionalMouseX);
  const int8_t mouseY = takeWholeCounts(fractionalMouseY);

  if (mouseX != 0 || mouseY != 0) {
    bleMouse.move(mouseX, mouseY);
  }

  const bool leftButtonPressed = !digitalRead(LEFTBUTTON);
  if (leftButtonPressed && !leftButtonWasPressed) {
    Serial.println("Left click");
    bleMouse.click(MOUSE_LEFT);
  }
  leftButtonWasPressed = leftButtonPressed;

  const bool rightButtonPressed = !digitalRead(RIGHTBUTTON);
  if (rightButtonPressed && !rightButtonWasPressed) {
    Serial.println("Right click");
    bleMouse.click(MOUSE_RIGHT);
  }
  rightButtonWasPressed = rightButtonPressed;
}
