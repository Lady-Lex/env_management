#ifndef SENSOR_H
#define SENSOR_H

#ifndef SENSOR_THRESHOLDS_H
#define SENSOR_THRESHOLDS_H

// **HTS221 温度传感器**
#define TEMPERATURE_UPPER_THRESHOLD  27.0f  // 过热温度 (°C)
#define TEMPERATURE_LOWER_THRESHOLD  18.0f  // 过冷温度 (°C)

// **HTS221 湿度传感器**
#define HUMIDITY_UPPER_THRESHOLD     70.0f  // 过湿 (%RH)
#define HUMIDITY_LOWER_THRESHOLD     30.0f  // 过干 (%RH)

// **LPS22HB 压力传感器**
#define PRESSURE_UPPER_THRESHOLD     1020.0f  // 过高压力 (hPa)
#define PRESSURE_LOWER_THRESHOLD     980.0f   // 低气流压力 (hPa)

// **LSM6DSL 加速度计**
#define ACCELERATION_THRESHOLD       1.0f  // 异常振动 (g)

// **LSM6DSL 陀螺仪**
#define GYROSCOPE_THRESHOLD          5.0f  // 旋转过快 (°/s)

// **LIS3MDL 磁力计**
#define MAGNETIC_FIELD_THRESHOLD     50.0f  // 过强磁场 (Gauss)

#endif // SENSOR_THRESHOLDS_H

#include <stdint.h>
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_qspi.h"

#define MAX_SENSOR_DIM 3  // 最大维度，例如 3 轴加速度


#define TIMER_INTERVAL_MS 500  // 定时器中断间隔 500ms
// 各传感器触发周期 (单位: ms)
#define TEMP_TRIGGER_PERIOD     1000   // 1s 触发
#define HUMIDITY_TRIGGER_PERIOD 1000   // 1s 触发
#define PRESSURE_TRIGGER_PERIOD 1500   // 1.5s 触发
#define ACCEL_TRIGGER_PERIOD    500    // 0.5s 触发
#define GYRO_TRIGGER_PERIOD     500    // 0.5s 触发
#define MAG_TRIGGER_PERIOD      1000   // 1s 触发


// 传感器数据结构体
typedef struct {
    float value[MAX_SENSOR_DIM];  // 允许 1D 或 3D 数据
    uint8_t dim;  // 数据维度（1 = 单值，3 = 三轴）
    uint32_t timestamp;  // 读取时间戳
} SensorData;

// 异常数据变量
extern volatile SensorData temp_alarm_data;
extern volatile SensorData humidity_alarm_data;
extern volatile SensorData pressure_alarm_data;
extern volatile SensorData accel_alarm_data;
extern volatile SensorData gyro_alarm_data;
extern volatile SensorData mag_alarm_data;

// 时间戳获取函数
uint32_t get_Timestamp(void);

// 传感器数据获取函数
SensorData get_Pressure(void);
SensorData get_Temperature(void);
SensorData get_Humidity(void);
SensorData get_Acceleration(void);
SensorData get_Gyroscope(void);
SensorData get_Magnetometer(void);

#endif // SENSOR_H
