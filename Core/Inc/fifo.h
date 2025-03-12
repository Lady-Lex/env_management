#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>

#define FIFO_SIZE 32  // FIFO 深度
#define MAX_SENSOR_DIM 3  // 最大数据维度，例如 3 轴加速度

// 传感器数据结构
typedef struct {
    float value[MAX_SENSOR_DIM];  // 允许存储 1D 或 3D 数据
    uint8_t dim;  // 数据维度（1 代表单值，3 代表三轴）
    uint32_t timestamp;  // 读取时间
} SensorData;

// FIFO 结构体
typedef struct {
    SensorData buffer[FIFO_SIZE];  // 存储数据的数组
    int head;
    int tail;
    int count;
} FIFO_Buffer;

// FIFO 操作函数
void FIFO_Init(FIFO_Buffer *fifo);
bool FIFO_Push(FIFO_Buffer *fifo, SensorData data);
bool FIFO_Pop(FIFO_Buffer *fifo, SensorData *data);
bool FIFO_IsEmpty(FIFO_Buffer *fifo);
int FIFO_GetUsage(FIFO_Buffer *fifo);

#endif // FIFO_H
