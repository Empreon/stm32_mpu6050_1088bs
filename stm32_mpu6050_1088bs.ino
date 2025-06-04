#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // Using DMP v2.0

MPU6050 mpu;

/* User Defined Variables from original code (some may be less relevant with DMP) */
// #define USE_MPU6050_I2C // Implied by this example
// #define GYRO_250DPS     // DMP configures this internally or uses defaults
// #define ACCEL_2G        // DMP configures this internally or uses defaults

// Filter parameters (DMP has its own internal filtering, and we add a timed Euler update)
// float B_madgwick = 0.04; // Not used with DMP
// float B_accel = 0.14;    // Not used, DMP handles/internal DLPF
// float B_gyro = 0.1;      // Not used, DMP handles/internal DLPF

// IMU calibration parameters (DMP uses its own internal calibration via mpu.CalibrateAccel/Gyro)
// If you have PRE-CALCULATED RAW OFFSETS (not scaled like before), you can set them.
// For now, we rely on the auto-calibration in setup.
// int16_t rawAccelOffsetX = 0; // Example: -7000; (These are RAW sensor values)
// int16_t rawAccelOffsetY = 0; // Example: -1200;
// int16_t rawAccelOffsetZ = 0; // Example:  1200;
// int16_t rawGyroOffsetX = 0;  // Example:  220;
// int16_t rawGyroOffsetY = 0;  // Example:   76;
// int16_t rawGyroOffsetZ = 0;  // Example:  -85;


/* PINS */
#ifndef LED_BUILTIN
#define LED_BUILTIN 13 // Define LED_BUILTIN if not already defined (e.g. for ESP32)
#endif
// IMPORTANT: Connect MPU6050 INT pin to this Arduino Pin
const int INTERRUPT_PIN = 2; // For Uno/Nano, this is Interrupt 0
                              // For ESP32, you can use any GPIO, e.g., const int INTERRUPT_PIN = 15;


/* Global Variables*/
//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter_main; // Renamed to avoid conflict if any other print_counter exists

//IMU / DMP:
Quaternion q;        // [w, x, y, z] quaternion container from DMP
VectorFloat gravity; // [x, y, z] gravity vector from DMP
float ypr[3];        // [yaw, pitch, roll] Raw radians from DMP processing

// Final Euler angles in degrees
float roll_IMU, pitch_IMU, yaw_IMU;

// DMP control/status
bool dmpReady = false;    // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];   // FIFO storage buffer

// For less frequent Euler calculation
unsigned long last_euler_calc_time = 0;
const unsigned long EULER_CALC_INTERVAL_US = 10000; // 100 Hz (10000 us = 0.01s)

// Interrupt flag
volatile bool mpuInterrupt = false; // true if MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/* SETUP */
void setup() {
  Serial.begin(500000); // Your desired baud rate
  while (!Serial && millis() < 2000); // Wait for Serial port to connect (with timeout)

  // Initialize I2C
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Stable for DMP.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  pinMode(LED_BUILTIN, OUTPUT); // For visual feedback

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT); // Setup interrupt pin

  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    Serial.println(F("MPU6050 connection failed. Halting."));
    while (1); // Halt
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    // Set any known RAW offsets BEFORE calibration if you have them
    // Otherwise, calibration will try to find them.
    // mpu.setXAccelOffset(rawAccelOffsetX);
    // mpu.setYAccelOffset(rawAccelOffsetY);
    // mpu.setZAccelOffset(rawAccelOffsetZ);
    // mpu.setXGyroOffset(rawGyroOffsetX);
    // mpu.setYGyroOffset(rawGyroOffsetY);
    // mpu.setZGyroOffset(rawGyroOffsetZ);

    Serial.println(F("Performing MPU6050 calibration. Keep it steady..."));
    // These default parameters are good. For finer tuning, see library examples.
    // Accel calibration: 6 = 1g sensitivity, Gyro calibration: 6 = 250 dps sensitivity
    mpu.CalibrateAccel(6); 
    mpu.CalibrateGyro(6);
    
    Serial.println(F("Calibration done. Active offsets:"));
    mpu.PrintActiveOffsets(); // Display the offsets found by calibration

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus(); // Read initial interrupt status

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));

    // Initialize timers
    prev_time = micros();
    current_time = micros();
    print_counter_main = micros();
    last_euler_calc_time = micros();

  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while(1); // Halt
  }
}

/* LOOP */
void loop() {
  if (!dmpReady) return; // If DMP failed, don't do anything

  // Update time
  // dt will represent time since last DMP packet processed, or last loop iteration if no packet
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0f;

  unsigned long t_dmp_processing_start = 0, t_dmp_processing_end = 0;
  bool new_dmp_data = false;

  // Wait for MPU interrupt or extra packet(s) available
  // mpu.dmpGetCurrentFIFOPacket will check the interrupt flag MPUInterrupt
  // and also directly check fifoCount. It resets MPUInterrupt if a packet is read.
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet from FIFO
    new_dmp_data = true;
    t_dmp_processing_start = micros();

    // Get Quaternion data from DMP FIFO
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Calculate Euler angles only at the EULER_CALC_INTERVAL_US
    if (current_time - last_euler_calc_time >= EULER_CALC_INTERVAL_US) {
      last_euler_calc_time = current_time; // Reset the timer for Euler calc

      mpu.dmpGetGravity(&gravity, &q);        // Calculate gravity vector from quaternion
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Calculate Yaw, Pitch, Roll in radians

      // Convert to degrees and store in our global variables
      // The ypr array order from dmpGetYawPitchRoll is [yaw, pitch, roll]
      yaw_IMU   = ypr[0] * 180.0f / M_PI;
      pitch_IMU = ypr[1] * 180.0f / M_PI;
      roll_IMU  = ypr[2] * 180.0f / M_PI;
    }
    t_dmp_processing_end = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED on new data
  }


  // Print data at 1Hz (your original style)
  if (current_time - print_counter_main > 1000000) { // Print every 1 second
    print_counter_main = current_time;
    Serial.print("dt (s): "); Serial.println(dt, 7); // dt is time since last loop or DMP packet
    if (new_dmp_data) {
        Serial.print("DMP FIFO Read + Quat (us): "); // This is the core processing time now
        Serial.println(t_dmp_processing_end - t_dmp_processing_start); 
        // Note: If Euler angles were calculated in this specific DMP packet read, that time is included.
    } else {
        Serial.println("No new DMP data this iteration.");
    }
    printRollPitchYaw();
  }

  // Your LED matrix control code would go here.
  // It should use yaw_IMU, pitch_IMU, roll_IMU.
  // Since these are updated at 100Hz (due to EULER_CALC_INTERVAL_US),
  // your LED matrix display will also effectively update at that rate if called every loop.
  // displayOnLedMatrices(roll_IMU, pitch_IMU, yaw_IMU); // Example call

  // No explicit loopRate() function needed.
  // The loop is effectively paced by the DMP interrupt rate for data processing
  // and by your EULER_CALC_INTERVAL_US for Euler angle updates.
  // If the LED matrix code is very fast, the loop will mostly idle waiting for interrupts.
}


/* FUNCTIONS */

// Removed IMUinit(), getIMUdata(), Madgwick6DOF(), calculate_IMU_error(), invSqrt()
// as their functionalities are replaced or handled by the DMP library.

void printRollPitchYaw() {
  // This function now prints the latest calculated Euler angles
  // It's called from the 1-second timed print block in loop()
  Serial.print(F("Yaw:"));
  Serial.print(yaw_IMU);
  Serial.print(F(" Pitch:"));
  Serial.print(pitch_IMU);
  Serial.print(F(" Roll:"));
  Serial.println(roll_IMU);
}

// You can add back printGyroData() and printAccelData() if you want to read
// raw (but DMP-corrected) accel/gyro data. You'd use:
// mpu.getAcceleration(&ax, &ay, &az);
// mpu.getRotation(&gx, &gy, &gz);
// And then scale them. However, the primary output now is YPR via DMP.