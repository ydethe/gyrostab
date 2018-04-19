#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"

typedef struct {
  float x;
  float y;
  float z;
} vec3;


class IMU {
  private:
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
    MPU6050 m_accelgyro;
    int16_t m_ax, m_ay, m_az, m_gx, m_gy, m_gz;
    
  public:
    IMU();

    void read();

    vec3 getAccel();

    vec3 getGyro();
    
};

