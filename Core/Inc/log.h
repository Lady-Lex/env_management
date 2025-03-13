#ifndef LOG_H
#define LOG_H

#include <stdint.h>

// **性能监测变量**
extern volatile uint32_t total_latency;      // 总事件处理时间
extern volatile uint32_t critical_event_count; // 关键事件触发次数
extern volatile uint32_t missed_updates;    // 丢失的传感器更新次数
extern volatile uint32_t total_sensor_reads; // 传感器读取总数

// **函数声明**
void log_sensor_event(const char *sensor_name, float value, uint32_t timestamp);
void log_response_delay(uint32_t trigger_time, const char *event);
void log_missed_percentage(void);
void log_average_latency(void);
void reset_performance_metrics(void);

#endif // LOG_H
