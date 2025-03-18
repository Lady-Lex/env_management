#include "log.h"
#include <stdio.h>
#include "main.h" // 确保包含 HAL_GetTick() 支持

// **性能监测变量**
volatile uint32_t total_latency = 0;           // 累积处理时延
volatile uint32_t critical_event_count = 0;    // 关键事件触发次数
volatile uint32_t missed_updates = 0;          // 丢失的传感器更新
volatile uint32_t total_sensor_reads = 0;      // 传感器读取总次数

// **记录传感器事件**
// void log_sensor_event(const char *sensor_name, float value, uint32_t timestamp) {
//     printf("[LOG] Sensor: %s | Value: %.2f | Timestamp: %lu ms\n", sensor_name, value, timestamp);
// }

// **记录响应延迟**
void log_response_delay(uint32_t trigger_time, const char *event) {
    uint32_t now = HAL_GetTick();
    uint32_t delay = now - trigger_time;
    total_latency += delay;
    critical_event_count++;

    printf("[%-11s] %s Response Delay: %lu ms\n", "PERFORMANCE", event, delay);
}

// **记录丢失的传感器数据**
void log_missed_percentage(void) {
    float missed_percentage = ((float)missed_updates / total_sensor_reads) * 100.0;
    printf("[%-11s] Missed Updates: %u / %u (%.2f%%)\n", "PERFORMANCE", missed_updates, total_sensor_reads, missed_percentage);
}

// **计算并打印关键事件平均处理延迟**
void log_average_latency(void) {
    if (critical_event_count == 0) return;
    uint32_t avg_latency = total_latency / critical_event_count;
    printf("[%-11s] Average Latency: %lu ms\n", "PERFORMANCE", avg_latency);
}


// **重置性能监测指标**
void reset_performance_metrics(void) {
    total_latency = 0;
    critical_event_count = 0;
    missed_updates = 0;
    total_sensor_reads = 0;
}
