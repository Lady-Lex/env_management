#include "main.h"  // 包含 GPIO 控制等相关内容


// 记录最近一次触发动作的时间
volatile uint32_t last_temp_action = 0;
volatile uint32_t last_humidity_action = 0;
volatile uint32_t last_vibration_action = 0;


void process_control_actions(uint8_t control_type) {
    uint32_t now = HAL_GetTick();  // 获取当前时间
    uint32_t response_delay = 0;   // 记录响应延迟

    switch (control_type) {
        case 0:  // **温度控制**
            if (now - last_temp_action > 5000) {  // 避免短时间内重复触发
                response_delay = now - last_temp_action;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // 假设冷却系统连接到 GPIOB_PIN_0
                printf("[%-11s] Temperature exceeded threshold! Cooling activated.\n", "CONTROL");
                last_temp_action = now;
            } else {
                printf("[%-11s] Cooling system working, skipping action.\n", "CONTROL");
            }
            break;

        case 1:  // **湿度控制**
            if (now - last_humidity_action > 5000) {
                response_delay = now - last_humidity_action;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // 假设加湿器连接到 GPIOB_PIN_1
                printf("[%-11s] Humidity out of range! Humidifier activated.\n", "CONTROL");
                last_humidity_action = now;
            } else {
                printf("[%-11s] Humidifier working, skipping action.\n", "CONTROL");
            }
            break;

        case 2:  // **振动控制**
            if (now - last_vibration_action > 1000) {
                response_delay = now - last_vibration_action;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // 警报系统连接到 GPIOB_PIN_2
                printf("[%-11s] High vibration detected! Alarm triggered.\n", "CONTROL");
                last_vibration_action = now;
            } else {
                printf("[%-11s] Vibration-alarm activated recently, skipping action.\n", "CONTROL");
            }
            break;

        default:
            printf("[%-11s] Unknown control type: %d\n", "CONTROL", control_type);
            return;
    }
}




// **日志记录**
void log_sensor_data(void) {
    printf("Logging sensor data...\n");
}
