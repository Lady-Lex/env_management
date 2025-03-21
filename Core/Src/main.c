/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    RANDOM_SELECTION,   // 随机选择
    FULL_BUFFER_SELECTION, // 选择数据最多的 FIFO
    PREDICTIVE_SELECTION  // 基于频率和占用率的预测选择
} FIFO_Selection_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef hDiscoUart;
#ifdef __GNUC__
/* With GCC/RAISONANCE, small msg_info (option LD Linker->Libraries->Small msg_info
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FIFO_Buffer pressure_fifo;
FIFO_Buffer temperature_fifo;
FIFO_Buffer humidity_fifo;
FIFO_Buffer accel_fifo;
FIFO_Buffer gyro_fifo;
FIFO_Buffer mag_fifo;

volatile uint8_t temp_alarm = 0;
volatile uint8_t humidity_alarm = 0;
volatile uint8_t pressure_alarm = 0;
volatile uint8_t accel_alarm = 0;
volatile uint8_t gyro_alarm = 0;
volatile uint8_t mag_alarm = 0;

volatile uint32_t temp_trigger_time = 0;
volatile uint32_t humidity_trigger_time = 0;
volatile uint32_t pressure_trigger_time = 0;
volatile uint32_t accel_trigger_time = 0;
volatile uint32_t gyro_trigger_time = 0;
volatile uint32_t mag_trigger_time = 0;

volatile SensorData temp_alarm_data;
volatile SensorData humidity_alarm_data;
volatile SensorData pressure_alarm_data;
volatile SensorData accel_alarm_data;
volatile SensorData gyro_alarm_data;
volatile SensorData mag_alarm_data;

volatile uint8_t temp_counter = 0;
volatile uint8_t humidity_counter = 0;
volatile uint8_t pressure_counter = 0;
volatile uint8_t accel_counter = 0;
volatile uint8_t gyro_counter = 0;
volatile uint8_t mag_counter = 0;


FIFO_Buffer *fifo_list[] = { &pressure_fifo, &temperature_fifo, &humidity_fifo, 
                        &accel_fifo, &gyro_fifo, &mag_fifo };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void alarm_response(void);
int FIFO_Select(FIFO_Selection_Mode mode);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  BSP_PSENSOR_Init();
  BSP_TSENSOR_Init();
  BSP_HSENSOR_Init();
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  BSP_MAGNETO_Init();

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1024, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

  BSP_LED_Init(LED2); 
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1; 
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
  
  printf("Press User button to put LED2 ON \n");
  while(BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET);
  while(BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET);
  BSP_LED_On(LED2);  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    printf("----------------------------------------------------------------\n");
    alarm_response();

    SensorData data;

    // 随机从 FIFO 读取数据
    int selected_fifo = FIFO_Select(3);
    switch (selected_fifo) {
        case 0:
            if (FIFO_Pop(&pressure_fifo, &data)) {
                printf("[%-11s] Pressure: %.2f hPa | Timestamp: %lu ms\n", "FIFO POP", data.value[0], data.timestamp);
                fflush(stdout);
            }
            break;
        case 1:
            if (FIFO_Pop(&temperature_fifo, &data)) {
                printf("[%-11s] Temperature: %.2f °C | Timestamp: %lu ms\n", "FIFO POP", data.value[0], data.timestamp);
                fflush(stdout);
            }
            break;
        case 2:
            if (FIFO_Pop(&humidity_fifo, &data)) {
                printf("[%-11s] Humidity: %.2f%% RH | Timestamp: %lu ms\n", "FIFO POP", data.value[0], data.timestamp);
                fflush(stdout);
            }
            break;
        case 3:
            if (FIFO_Pop(&accel_fifo, &data)) {
                printf("[%-11s] Acceleration: X=%.2f g, Y=%.2f g, Z=%.2f g | Timestamp: %lu ms\n", 
                      "FIFO POP", data.value[0], data.value[1], data.value[2], data.timestamp);
                fflush(stdout);
            }
            break;
        case 4:
            if (FIFO_Pop(&gyro_fifo, &data)) {
                printf("[%-11s] Gyroscope: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s | Timestamp: %lu ms\n", 
                      "FIFO POP", data.value[0], data.value[1], data.value[2], data.timestamp);
                fflush(stdout);
            }
            break;
        case 5:
            if (FIFO_Pop(&mag_fifo, &data)) {
                printf("[%-11s] Magnetometer: X=%.2f Gauss, Y=%.2f Gauss, Z=%.2f Gauss | Timestamp: %lu ms\n", 
                      "FIFO POP", data.value[0], data.value[1], data.value[2], data.timestamp);
                fflush(stdout);
            }
            break;
    }



    log_missed_percentage();
    log_average_latency();

    HAL_Delay(500); 
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // LED 触发，表示 RTC 运行

    SensorData data;

    // **增加计数器**
    temp_counter++;
    humidity_counter++;
    pressure_counter++;
    accel_counter++;
    gyro_counter++;
    mag_counter++;

    // **温度数据**
    if (temp_counter * TIMER_INTERVAL_MS >= TEMP_TRIGGER_PERIOD) {
        temp_counter = 0;  // 计数清零
        data = get_Temperature();
        if (data.value[0] >= TEMPERATURE_UPPER_THRESHOLD) {
            temp_alarm = 1;
            temp_alarm_data = data;
            temp_trigger_time = HAL_GetTick();
        } else if (data.value[0] <= TEMPERATURE_LOWER_THRESHOLD) {
            temp_alarm = 2;
            temp_alarm_data = data;
            temp_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&temperature_fifo, data);
        }
    }

    // **湿度数据**
    if (humidity_counter * TIMER_INTERVAL_MS >= HUMIDITY_TRIGGER_PERIOD) {
        humidity_counter = 0;
        data = get_Humidity();
        if (data.value[0] >= HUMIDITY_UPPER_THRESHOLD) {
            humidity_alarm = 1;
            humidity_alarm_data = data;
            humidity_trigger_time = HAL_GetTick();
        } else if (data.value[0] <= HUMIDITY_LOWER_THRESHOLD) {
            humidity_alarm = 2;
            humidity_alarm_data = data;
            humidity_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&humidity_fifo, data);
        }
    }

    // **压力数据**
    if (pressure_counter * TIMER_INTERVAL_MS >= PRESSURE_TRIGGER_PERIOD) {
        pressure_counter = 0;
        data = get_Pressure();
        if (data.value[0] >= PRESSURE_UPPER_THRESHOLD) {
            pressure_alarm = 1;
            pressure_alarm_data = data;
            pressure_trigger_time = HAL_GetTick();
        } else if (data.value[0] <= PRESSURE_LOWER_THRESHOLD) {
            pressure_alarm = 2;
            pressure_alarm_data = data;
            pressure_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&pressure_fifo, data);
        }
    }

    // **加速度数据**
    if (accel_counter * TIMER_INTERVAL_MS >= ACCEL_TRIGGER_PERIOD) {
        accel_counter = 0;
        data = get_Acceleration();
        if (data.value[0] >= ACCELERATION_THRESHOLD || data.value[1] >= ACCELERATION_THRESHOLD || data.value[2] >= ACCELERATION_THRESHOLD) {
            accel_alarm = 1;
            accel_alarm_data = data;
            accel_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&accel_fifo, data);
        }
    }

    // **陀螺仪数据**
    if (gyro_counter * TIMER_INTERVAL_MS >= GYRO_TRIGGER_PERIOD) {
        gyro_counter = 0;
        data = get_Gyroscope();
        if (data.value[0] >= GYROSCOPE_THRESHOLD || data.value[1] >= GYROSCOPE_THRESHOLD || data.value[2] >= GYROSCOPE_THRESHOLD) {
            gyro_alarm = 1;
            gyro_alarm_data = data;
            gyro_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&gyro_fifo, data);
        }
    }

    // **磁力计数据**
    if (mag_counter * TIMER_INTERVAL_MS >= MAG_TRIGGER_PERIOD) {
        mag_counter = 0;
        data = get_Magnetometer();
        if (data.value[0] >= MAGNETIC_FIELD_THRESHOLD || data.value[1] >= MAGNETIC_FIELD_THRESHOLD || data.value[2] >= MAGNETIC_FIELD_THRESHOLD) {
            mag_alarm = 1;
            mag_alarm_data = data;
            mag_trigger_time = HAL_GetTick();
        } else {
            FIFO_Push(&mag_fifo, data);
        }
    }
}


void alarm_response(void) {
    if (temp_alarm == 1) {
        printf("[%-11s] Temperature too HIGH: %.2f °C! Cooling activated.\n", "ALERT", temp_alarm_data.value[0]);
        log_response_delay(temp_trigger_time, "HIGH Temperature");
        process_control_actions(0);
        temp_alarm = 0;  // 清除报警
    } else if (temp_alarm == 2) {
        printf("[%-11s] Temperature too LOW: %.2f °C! No action required.\n", "ALERT", temp_alarm_data.value[0]);
        log_response_delay(temp_trigger_time, "LOW Temperature");
        temp_alarm = 0;
    }

    if (humidity_alarm == 1) {
        printf("[%-11s] Humidity too HIGH: %.2f%% RH! No action required.\n", "ALERT", humidity_alarm_data.value[0]);
        log_response_delay(humidity_trigger_time, "HIGH Humidity");
        humidity_alarm = 0;
    } else if (humidity_alarm == 2) {
        printf("[%-11s] Humidity too LOW: %.2f%% RH! Humidifier activated.\n", "ALERT", humidity_alarm_data.value[0]);
        log_response_delay(humidity_trigger_time, "LOW Humidity");
        process_control_actions(1);
        humidity_alarm = 0;
    }

    if (pressure_alarm == 1) {
        printf("[%-11s] Pressure too HIGH: %.2f hPa!\n", "ALERT", pressure_alarm_data.value[0]);
        log_response_delay(pressure_trigger_time, "HIGH Pressure");
        pressure_alarm = 0;
    } else if (pressure_alarm == 2) {
        printf("[%-11s] Pressure too LOW: %.2f hPa!\n", "ALERT", pressure_alarm_data.value[0]);
        log_response_delay(pressure_trigger_time, "LOW Pressure");
        pressure_alarm = 0;
    }

    if (accel_alarm == 1) {
        printf("[%-11s] High Vibration Detected: X=%.2f g, Y=%.2f g, Z=%.2f g! Alarm triggered.\n", 
               "ALERT", accel_alarm_data.value[0], accel_alarm_data.value[1], accel_alarm_data.value[2]);
        log_response_delay(accel_trigger_time, "High Vibration");
        process_control_actions(2);
        accel_alarm = 0;
    }

    if (gyro_alarm == 1) {
        printf("[%-11s] Unstable Rotation Detected: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s! Alarm triggered.\n", 
               "ALERT", gyro_alarm_data.value[0], gyro_alarm_data.value[1], gyro_alarm_data.value[2]);
        log_response_delay(gyro_trigger_time, "Unstable Rotation");
        process_control_actions(2);
        gyro_alarm = 0;
    }

    if (mag_alarm == 1) {
        printf("[%-11s] Strong Magnetic Field Detected: X=%.2f Gauss, Y=%.2f Gauss, Z=%.2f Gauss!\n", 
               "ALERT", mag_alarm_data.value[0], mag_alarm_data.value[1], mag_alarm_data.value[2]);
        log_response_delay(mag_trigger_time, "Strong Magnetic Field");
        mag_alarm = 0;
    }
}


int FIFO_Select(FIFO_Selection_Mode mode) {
    int selected_fifo = 0;
    int max_size = 0;
    
    switch (mode) {
        case RANDOM_SELECTION:
            selected_fifo = rand() % 6;  // 生成 0~5 的随机数
            break;

        case FULL_BUFFER_SELECTION:
            for (int i = 0; i < 6; i++) {
                int current_size = FIFO_GetUsage(fifo_list[i]); // 获取当前 FIFO 数据量
                if (current_size > max_size) {
                    max_size = current_size;
                    selected_fifo = i;
                }
            }
            break;

        case PREDICTIVE_SELECTION:
            // 这里可以根据传感器采样频率、FIFO 占用率等进行优化选择
            for (int i = 0; i < 6; i++) {
                int current_size = FIFO_GetUsage(fifo_list[i]);
                float occupancy = (float)current_size / FIFO_GetUsage(fifo_list[i]);  // 计算占用率
                if (occupancy > 0.75) {  // 如果 FIFO 使用率超过 75%，优先选择
                    selected_fifo = i;
                    break;
                }
            }
            selected_fifo = rand() % 6;
            break;

        default:
            selected_fifo = 0;  // 默认返回第一个 FIFO
            break;
    }

    return selected_fifo;
}

/**
  * @brief Retargets the C library msg_info function to the USART.
  * @param None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the serial port and Loop until the end of transmission */
  while (HAL_OK != HAL_UART_Transmit(&hDiscoUart, (uint8_t *) &ch, 1, 30000))
  {
    ;
  }
  return ch;
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. readwrite a character to the USART2 and Loop until the end of transmission */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
