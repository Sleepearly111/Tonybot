#include "TumbleGuard.h"

// 定义姿态变量
float radianX;
float radianY;

// 防跌倒/自动起立 核心函数
void tumble(void)
{
  static uint32_t Time;
  static uint8_t step = 0;
  static uint8_t count1 = 0;
  static uint8_t count2 = 0;

  if (Time > millis())
    return;

  switch (step)
  {
    case 0:
      imu.get_angle(&radianX, &radianY);

      // 前倒判断
      if (radianX < 60 && radianX > -30)
      {
        count1++;
        Time = millis() + 50;
        if (count1 > 50)
        {
          count1 = 0;
          step = 1;
          Time = millis() + 1000;
        }
      }
      // 后倒判断
      else if (radianX > 120 || radianX < -140)
      {
        count2++;
        Time = millis() + 50;
        if (count2 > 50)
        {
          count2 = 0;
          step = 2;
          Time = millis() + 1000;
        }
      }
      // 正常姿态
      else
      {
        count1 = 0;
        count2 = 0;
      }
      break;

    case 1:
      Controller.runActionGroup(102, 1);
      Time = millis() + 7000;
      step = 0;
      break;

    case 2:
      Controller.runActionGroup(101, 1);
      Time = millis() + 7000;
      step = 0;
      break;

    default:
      step = 0;
      break;
  }
}