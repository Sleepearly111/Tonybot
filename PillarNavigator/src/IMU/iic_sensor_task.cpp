/**
 * @file imu_task.cpp
 * @author CuZn
 * 
 * @brief Qmi8658 task
 * 
 * @version 1.0
 * @date 2024-12-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "iic_sensor_task.h"
#include "../../Hiwonder.hpp"

// extern SemaphoreHandle_t sp_IIC1;
// extern SemaphoreHandle_t sp_IIC2;

SensorQMI8658 qmi;
Madgwick filter;

static void task_process_handler(void *arg)
{
    
    static IMUdata gyr_offset;
    IMUdata acc;
    IMUdata gyr;
    static uint32_t lase_tick = 0;

    qmi.configAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        SensorQMI8658::ACC_RANGE_2G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
        * */
        SensorQMI8658::ACC_ODR_1000Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        * */
        SensorQMI8658::LPF_MODE_0,
        // selfTest enable
        true);

    qmi.configGyroscope(
        /*
        * GYR_RANGE_16DPS
        * GYR_RANGE_32DPS
        * GYR_RANGE_64DPS
        * GYR_RANGE_128DPS
        * GYR_RANGE_256DPS
        * GYR_RANGE_512DPS
        * GYR_RANGE_1024DPS
        * */
        SensorQMI8658::GYR_RANGE_256DPS,
        /*
         * GYR_ODR_7174_4Hz
         * GYR_ODR_3587_2Hz
         * GYR_ODR_1793_6Hz
         * GYR_ODR_896_8Hz
         * GYR_ODR_448_4Hz
         * GYR_ODR_224_2Hz
         * GYR_ODR_112_1Hz
         * GYR_ODR_56_05Hz
         * GYR_ODR_28_025H
         * */
        SensorQMI8658::GYR_ODR_896_8Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        * */
        SensorQMI8658::LPF_MODE_3,
        // selfTest enable
        true);

    /* In 6DOF mode (accelerometer and gyroscope are both enabled),
     *
     *the output data rate is derived from the nature frequency of gyroscope
     */
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    /* 打印注册的配置信息 */
    qmi.dumpCtrlRegister(); 

    /* 滤波设置 */
    filter.begin(25);
    
    qmi.getGyroscope(gyr_offset.x, gyr_offset.y, gyr_offset.z);
    for(int i = 0 ; i < 10 ; i++)
    {
      qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
      gyr_offset.x = (gyr_offset.x + gyr.x)/2;
      gyr_offset.y = (gyr_offset.y + gyr.y)/2;
      gyr_offset.z = (gyr_offset.z + gyr.z)/2;
      delay(40);
    }

    while(true)
    {
      lase_tick = millis();
      // xSemaphoreTake(sp_IIC1 , portMAX_DELAY);
      /* 获取IMU数据 */
      if(qmi.getDataReady())
      {
          qmi.getAccelerometer(acc.x, acc.y, acc.z);
          qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
          /* 更新imu数据，获取六轴数据信息 */
          filter.updateIMU(gyr.x-gyr_offset.x, gyr.y-gyr_offset.y, gyr.z-gyr_offset.z, acc.x, acc.y, acc.z);
      }
      // xSemaphoreGive( sp_IIC1 );
      delay(40);
    }
}

void register_iic_sensor_task()
{
    xTaskCreatePinnedToCore(task_process_handler,"IICTask",3 * 1024,(void *)NULL,3,NULL,0);
}
