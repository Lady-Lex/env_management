#include "fifo.h"
#include <string.h>  // 用于 memset

// 初始化 FIFO
void FIFO_Init(FIFO_Buffer *fifo) {
    memset(fifo, 0, sizeof(FIFO_Buffer)); // 清空结构体
}

// 向 FIFO 添加数据（入队）
bool FIFO_Push(FIFO_Buffer *fifo, SensorData data) {
    if (fifo->count >= FIFO_SIZE) {
        return false;  // FIFO 满
    }
    fifo->buffer[fifo->tail] = data;
    fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
    fifo->count++;
    return true;
}

// 从 FIFO 读取数据（出队）
bool FIFO_Pop(FIFO_Buffer *fifo, SensorData *data) {
    if (fifo->count == 0) {
        return false;  // FIFO 为空
    }
    *data = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) % FIFO_SIZE;
    fifo->count--;
    return true;
}

// 获取 FIFO 当前占用率
int FIFO_GetUsage(FIFO_Buffer *fifo) {
    return fifo->count;
}
