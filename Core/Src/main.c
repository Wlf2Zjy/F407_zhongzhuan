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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart3;  // 上位机
extern UART_HandleTypeDef huart5;  // F103

//uint8_t rx3, rx5;  // 分别用于两个串口的单字节缓冲
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if (huart->Instance == USART3)
//  {
//    // 收到上位机数据 -> 转发给F103
//    HAL_UART_Transmit(&huart5, &rx3, 1, HAL_MAX_DELAY);
//    // 再次打开中断接收
//    HAL_UART_Receive_IT(&huart3, &rx3, 1);
//  }
//  else if (huart->Instance == UART5)
//  {
//    // 收到F103数据 -> 转发给上位机
//    HAL_UART_Transmit(&huart3, &rx5, 1, HAL_MAX_DELAY);
//    // 再次打开中断接收
//    HAL_UART_Receive_IT(&huart5, &rx5, 1);
//  }
//}

// 与F103保持一致的帧定义
#define FRAME_HEADER_1 0xFE
#define FRAME_HEADER_2 0xFE
#define FRAME_END      0xFA
#define FRAME_MAX_LEN  256

typedef enum {
    RX_STATE_WAIT_HEADER1 = 0,
    RX_STATE_WAIT_HEADER2,
    RX_STATE_WAIT_LENGTH,
    RX_STATE_WAIT_ID,
    RX_STATE_WAIT_CMD,
    RX_STATE_WAIT_CONTENT,
	RX_STATE_WAIT_CRC_HIGH,    // 新增：CRC 高字节
    RX_STATE_WAIT_CRC_LOW,     // 新增：CRC 低字节
    RX_STATE_WAIT_END
} RxState;

// 抽象一个“端口”结构，左右两路各来一份
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rxByte;
    volatile RxState state;
    uint8_t  buf[FRAME_MAX_LEN];
    uint16_t idx;
    uint8_t  rx_len;           
    volatile uint8_t frameReady;
    uint8_t  lastByte;         //记录上一个字节，用于检测 FE FE
} BridgePort;

static BridgePort portPC   = { .huart = &huart3, .state = RX_STATE_WAIT_HEADER1 };
static BridgePort portF103 = { .huart = &huart5, .state = RX_STATE_WAIT_HEADER1 };

// 复位函数
static inline void reset_port(BridgePort *p)
{
    p->state = RX_STATE_WAIT_HEADER1;
    p->idx = 0;
    p->rx_len = 0;
    p->frameReady = 0;
}


// 把某端口的已收完整帧原样转发到另一端
static inline void forward_full_frame(BridgePort *from, BridgePort *to)
{
    // 总帧长 = 2(头) + 1(长度) + rx_len
    uint16_t total = 3 + from->rx_len;
    if (total > FRAME_MAX_LEN) total = FRAME_MAX_LEN; // 兜底

    HAL_UART_Transmit(to->huart, from->buf, total, HAL_MAX_DELAY);
    from->frameReady = 0;
}

// 自同步帧状态机
static inline void rx_state_machine(BridgePort *p)
{
    //自同步检测逻辑：在任意状态，如果连续检测到 FE FE，则立即重同步
    if (p->lastByte == FRAME_HEADER_1 && p->rxByte == FRAME_HEADER_2) {
        p->idx = 0;
        p->buf[p->idx++] = FRAME_HEADER_1;
        p->buf[p->idx++] = FRAME_HEADER_2;
        p->state = RX_STATE_WAIT_LENGTH;
        p->lastByte = p->rxByte;
        return; // 直接重同步，不走后面的状态机逻辑
    }

    switch (p->state)
    {
    case RX_STATE_WAIT_HEADER1:
        if (p->rxByte == FRAME_HEADER_1) {
            p->idx = 0;
            p->buf[p->idx++] = p->rxByte;
            p->state = RX_STATE_WAIT_HEADER2;
        }
        break;

    case RX_STATE_WAIT_HEADER2:
        if (p->rxByte == FRAME_HEADER_2) {
            p->buf[p->idx++] = p->rxByte;
            p->state = RX_STATE_WAIT_LENGTH;
        } else if (p->rxByte == FRAME_HEADER_1) {
            // 如果连续两个FE（可能上次丢失一个），保持同步
            p->idx = 1;
            p->buf[0] = FRAME_HEADER_1;
            p->state = RX_STATE_WAIT_HEADER2;
        } else {
            p->state = RX_STATE_WAIT_HEADER1;
            p->idx = 0;
        }
        break;

    case RX_STATE_WAIT_LENGTH:
        p->rx_len = p->rxByte;
        p->buf[p->idx++] = p->rxByte;
        if (p->rx_len >= 3 && (3 + p->rx_len) <= FRAME_MAX_LEN)
            p->state = RX_STATE_WAIT_ID;
        else
            reset_port(p);
        break;

    case RX_STATE_WAIT_ID:
        p->buf[p->idx++] = p->rxByte;
        p->state = RX_STATE_WAIT_CMD;
        break;

    case RX_STATE_WAIT_CMD:
    p->buf[p->idx++] = p->rxByte;

    if (p->rx_len > 5) {     // 有内容
        p->state = RX_STATE_WAIT_CONTENT;
    } else {                 // 无内容（LEN = 5）
        p->state = RX_STATE_WAIT_CRC_HIGH;
    }
    break;

case RX_STATE_WAIT_CONTENT:
    if (p->idx < FRAME_MAX_LEN) {
        p->buf[p->idx++] = p->rxByte;

        // 正确：内容长度 = LEN - 5
        uint16_t payload_len = (uint16_t)p->rx_len - 5; 
        // 已收到的内容字节数 = idx - 5
        uint16_t received = (uint16_t)p->idx - 5;

        // 如果 payload_len==0 的情况在进入到此分支前应已被避开（LEN==5）
        if (received >= payload_len) {
            p->state = RX_STATE_WAIT_CRC_HIGH;
        }
    } else {
        reset_port(p);
    }
    break;

case RX_STATE_WAIT_CRC_HIGH:
    p->buf[p->idx++] = p->rxByte;
    p->state = RX_STATE_WAIT_CRC_LOW;
    break;

case RX_STATE_WAIT_CRC_LOW:
    p->buf[p->idx++] = p->rxByte;
    p->state = RX_STATE_WAIT_END;
    break;

case RX_STATE_WAIT_END:
    if (p->rxByte == FRAME_END) {
        p->buf[p->idx++] = p->rxByte;
        p->frameReady = 1;
    }
    p->state = RX_STATE_WAIT_HEADER1;
    break;
    }

    // 更新 lastByte
    p->lastByte = p->rxByte;
}

// HAL 回调：两路端口共用同一套状态机
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BridgePort *p = NULL;

    if (huart->Instance == USART3) {
        p = &portPC;
        p->rxByte = portPC.rxByte;
    } else if (huart->Instance == UART5) {
        p = &portF103;
        p->rxByte = portF103.rxByte;
    } else {
        // 不是我们的桥接串口
        return;
    }

    rx_state_machine(p);

    // 继续收下一个字节
    HAL_UART_Receive_IT(p->huart, &p->rxByte, 1);
}
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
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 启动接收中断
  HAL_UART_Receive_IT(portPC.huart,   &portPC.rxByte,   1);
  HAL_UART_Receive_IT(portF103.huart, &portF103.rxByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		        // 当上位机发来完整帧，转发给F103
        // PC -> F103
        if (portPC.frameReady) {
            forward_full_frame(&portPC, &portF103);
        }
        // F103 -> PC
        if (portF103.frameReady) {
            forward_full_frame(&portF103, &portPC);
				}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
