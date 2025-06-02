/*
  Project: MPU6050 3D Angle Visualization on LED Matrices
  This code obtains raw data from an MPU6050 inertial measurement unit (IMU)
  using the I2C protocol for communication with an STM32 Nucleo-L053R8 board.
  The raw data is processed using the Madgwick sensor fusion algorithm to
  calculate orientation angles (e.g., roll, pitch, yaw).

  These angles are then visualized on two LD1088BS 8x8 LED matrices,
  driven by MAX7219 display driver ICs. Communication with the MAX7219 drivers
  is handled via the SPI protocol from the STM32. The LED matrices aim to
  represent the device's orientation on a conceptual 3D coordinate plane.
*/

#include "I2Cdev.h"
#include "MPU6050.h"

/* MPU6050 raw data */
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

bool blinkState;

/* Madgwick Filter Variables */
#define MADGWICK_BETA_DEFAULT 0.1f // Filter gain beta
volatile float beta = MADGWICK_BETA_DEFAULT; // Algorithm gain
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternion of sensor frame relative to auxiliary frame

// Variables for calculating dT
unsigned long lastUpdateTime = 0;
float dt = 0.0f; // Delta time in seconds

// MPU6050 Sensitivity Scale Factors
const float ACCEL_SENSITIVITY = 16384.0f; // Accel: +/- 2g (16384 LSB/g)
const float GYRO_SENSITIVITY = 131.0f;    // Gyro: +/- 250 dps (131 LSB/dps)

//Euler Angles
float roll, pitch, yaw;

void setup() {
  /* --Start I2C interface-- */
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(38400); // Initializate Serial wo work well at 8MHz/16MHz

  /* Initialize device and check connection */
  Serial.println("Initializing MPU...");
  imu.initialize();

  Serial.println("Testing MPU6050 connection...");
  if (imu.testConnection() ==  false) {
    Serial.println("MPU6050 connection failed");
    while(true); // Halt
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Update sensor offset values */
  Serial.println("Updating internal sensor offsets...\n");
  imu.setXAccelOffset(0); //Set accelerometer offset for axis X
  imu.setYAccelOffset(0); //Set accelerometer offset for axis Y
  imu.setZAccelOffset(0); //Set accelerometer offset for axis Z
  imu.setXGyroOffset(0);  //Set gyro offset for axis X
  imu.setYGyroOffset(0);  //Set gyro offset for axis Y
  imu.setZGyroOffset(0);  //Set gyro offset for axis Z
  Serial.println("MPU6050 configured.\n");

  /* Configure board LED pin for output */
  pinMode(LED_BUILTIN, OUTPUT);

  lastUpdateTime = micros(); // Initialize lastUpdateTime for dT calculation
}

void loop() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastUpdateTime) / 1000000.0f; // Calculate delta time in seconds
  lastUpdateTime = currentTime;

  /* Read raw accel/gyro data from the module */
  imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Convert raw sensor data to physical units
  // Accelerometer: g's (gravity)
  float ax = (float)ax_raw / ACCEL_SENSITIVITY;
  float ay = (float)ay_raw / ACCEL_SENSITIVITY;
  float az = (float)az_raw / ACCEL_SENSITIVITY;

  // Gyroscope: radians per second
  // M_PI is defined in math.h
  float gx = ((float)gx_raw / GYRO_SENSITIVITY) * (M_PI / 180.0f);
  float gy = ((float)gy_raw / GYRO_SENSITIVITY) * (M_PI / 180.0f);
  float gz = ((float)gz_raw / GYRO_SENSITIVITY) * (M_PI / 180.0f);

  // Update the Madgwick filter
  Madgwick6D(gx, gy, gz, ax, ay, az, dt);

  // Get Euler angles
  roll = getRoll();
  pitch = getPitch();
  yaw = getYaw();

  /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);

  delay(50);
}

// Function to convert quaternion to Euler angles (Roll, Pitch, Yaw)
// Roll (x-axis rotation)
float getRoll() {
    return atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / M_PI;
}
// Pitch (y-axis rotation)
float getPitch() {
    return -asin(2.0f * (q1 * q3 - q0 * q2)) * 180.0f / M_PI;
}
// Yaw (z-axis rotation) - Note: Yaw is unreliable without a magnetometer
float getYaw() {
    return atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / M_PI;
}


void Madgwick6D(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply corrective step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

