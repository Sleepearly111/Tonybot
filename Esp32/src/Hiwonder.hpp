#ifndef __HIWONDER_H
#define __HIWONDER_H

#include "stdint.h"
#include "IMU/iic_sensor_task.h"

class IMU{
  public:
    void begin();
    void get_angle(float* roll , float* pitch);
    float get_yaw();  // 0-360 deg, gyro-only, drifts ~1-3 deg/min
};

#endif //__HIWONDER_H
