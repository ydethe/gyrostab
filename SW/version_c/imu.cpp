#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include "imu.h"


IMU::IMU()
{
  Wire.begin();
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  m_accelgyro.initialize();
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(m_accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
}

void IMU::read() {
  m_accelgyro.getMotion6(&m_ax, &m_ay, &m_az, &m_gx, &m_gy, &m_gz);

}

vec3 IMU::getAccel() {
  vec3 acc;
  acc.x = m_ax/15190.;
  acc.y = m_ay/15190.;
  acc.z = m_az/15190.;

  return acc;
  
}

vec3 IMU::getGyro() {
  vec3 gyr;
  gyr.x = m_gx/15190.;
  gyr.y = m_gy/15190.;
  gyr.z = m_gz/15190.;

  return gyr;
  
}



