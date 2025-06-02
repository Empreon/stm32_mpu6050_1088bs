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
}

void loop() {
  /* Read raw accel/gyro data from the module */
  imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
}
