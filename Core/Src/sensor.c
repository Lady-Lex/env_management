/**
******************************************************************************
* @file    BSP/Src/sensors.c 
* @author  MCD Application Team
* @brief   This example code shows how to use the QSPI Driver
******************************************************************************
* @attention
*
* Copyright (c) 2017 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/** @addtogroup STM32L4xx_HAL_Examples
* @{
*/

/** @addtogroup BSP
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint32_t get_Timestamp(void) {
    return HAL_GetTick();  // 以毫秒为单位获取当前时间
}

// 获取压力数据
SensorData get_Pressure(void) {
    SensorData data;
    data.value[0] = BSP_PSENSOR_ReadPressure();  // 读取压力值
    data.dim = 1;  // 1D 数据
    data.timestamp = get_Timestamp();
    return data;
}

// 获取温度数据
SensorData get_Temperature(void) {
    SensorData data;
    data.value[0] = BSP_TSENSOR_ReadTemp();  // 读取温度值
    data.dim = 1;
    data.timestamp = get_Timestamp();
    return data;
}

// 获取湿度数据
SensorData get_Humidity(void) {
    SensorData data;
    data.value[0] = BSP_HSENSOR_ReadHumidity();  // 读取湿度值
    data.dim = 1;
    data.timestamp = get_Timestamp();
    return data;
}

// 获取加速度数据（三轴）
SensorData get_Acceleration(void) {
    SensorData data;
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    data.value[0] = pDataXYZ[0] / 1000.0f;
    data.value[1] = pDataXYZ[1] / 1000.0f;
    data.value[2] = pDataXYZ[2] / 1000.0f;
    data.dim = 3;  // 三轴数据
    data.timestamp = get_Timestamp();
    return data;
}

SensorData get_Gyroscope(void) {
    SensorData data;
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    data.value[0] = pGyroDataXYZ[0] / 1000.0f;
    data.value[1] = pGyroDataXYZ[1] / 1000.0f;
    data.value[2] = pGyroDataXYZ[2] / 1000.0f;
    data.dim = 3;
    data.timestamp = get_Timestamp();
    return data;
}

SensorData get_Magnetometer(void) {
    SensorData data;
    BSP_MAGNETO_GetXYZ(pDataXYZ);
    data.value[0] = pDataXYZ[0];
    data.value[1] = pDataXYZ[1];
    data.value[2] = pDataXYZ[2];
    data.dim = 3;
    data.timestamp = get_Timestamp();
    return data;
}
/**
* @}
*/ 

/**
* @}
*/ 