#ifndef __HIWONDER_H
#define __HIWONDER_H

#include <Ticker.h>
#include "IMU/iic_sensor_task.h"

class IMU{
  public:
    void begin();
    void get_angle(float* roll , float* pitch);
};

#endif //__HIWONDER_H
