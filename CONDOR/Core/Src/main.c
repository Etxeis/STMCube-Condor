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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    PULSE_IDLE = 0,
    PULSE_WAIT_RISING
} PulseState;

typedef struct {
    uint8_t channel;
    uint32_t timestamp;
} Event;

// Maquinas de estado que definen el estado en el que se encuentra el pulso
typedef struct {
    uint32_t t_start;
    uint32_t t_end;
    uint32_t width;
} PulseEvent;

typedef struct {
    PulseState state;
    uint32_t t_start;
} PulseFSM;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_CLK_HZ 85000000UL
#define PULSE_TIMEOUT_US 1UL   // ejemplo: 50 microsegundos
#define PULSE_TIMEOUT_TICKS (TIM_CLK_HZ / 1000000UL * PULSE_TIMEOUT_US)


#define PULSE_BUF_LEN 64

#define PUTCHAR_PROTOTYPE 		int __io_putchar(int ch)

#define IC_BUF_LEN         16
#define MAX_EVENTS         128

#define COINCIDENCE_WINDOW 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim2_ch4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Maquina de estados para procesar los tiempos */
volatile PulseFSM pulse_fsm_ch1 = {
    .state = PULSE_IDLE,
    .t_start = 0
};

volatile PulseEvent pulse_buffer[PULSE_BUF_LEN];
volatile uint32_t pulse_index = 0; 

/* Buffers DMA por canal */
uint32_t ic_ch1_buf[IC_BUF_LEN];
uint32_t ic_ch2_buf[IC_BUF_LEN];
uint32_t ic_ch3_buf[IC_BUF_LEN];
uint32_t ic_ch4_buf[IC_BUF_LEN];

// Flags para los canales
volatile uint8_t ic_ch1_ready = 0;
volatile uint8_t ic_ch2_ready = 0;
volatile uint8_t ic_ch3_ready = 0;
volatile uint8_t ic_ch4_ready = 0;

/* Eventos */
volatile Event event_buffer[MAX_EVENTS];
volatile uint32_t event_index = 0;

/* Extensión 64 bits */
volatile uint32_t timer_high = 0;

volatile Event last_event = {0, 0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
//static inline void push_event(uint8_t ch, uint32_t ts_low);
void Print_IC_Buffer(uint8_t ch, uint32_t *buf, uint32_t len);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, ic_ch1_buf, IC_BUF_LEN);
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, ic_ch2_buf, IC_BUF_LEN);
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, ic_ch3_buf, IC_BUF_LEN);
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, ic_ch4_buf, IC_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Hello - Test\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (pulse_fsm_ch1.state == PULSE_WAIT_RISING)
      {
          uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
          uint32_t dt = now - pulse_fsm_ch1.t_start;

          // Codigo del Timeout para resetear el estado si es que no se decta un flanco de subida en un tiempo defonido (1 microsegundos)
          if (dt > PULSE_TIMEOUT_TICKS)
          {
              pulse_fsm_ch1.state = PULSE_IDLE;
          }
          
      }

      // Detecta el flanco de Bajada y de Subida (CH1)
      if (ic_ch1_ready)
      {
          ic_ch1_ready = 0;
          Process_Falling_Buffer(ic_ch1_buf, IC_BUF_LEN);
          Process_Rising_Buffer(ic_ch2_buf, IC_BUF_LEN);
          //Print_IC_Buffer(1, ic_ch1_buf, IC_BUF_LEN);
      }
      //if (ic_ch2_ready)
      //{
      //    ic_ch2_ready = 0;
      //    Process_Falling_Buffer(ic_ch1_buf, IC_BUF_LEN);
      //    Process_Rising_Buffer(ic_ch2_buf, IC_BUF_LEN);
      //    //Print_IC_Buffer(2, ic_ch2_buf, IC_BUF_LEN);
      //}
      //if (ic_ch3_ready)
      //{
      //    ic_ch3_ready = 0;
      //    Process_Falling_Buffer(ic_ch3_buf, IC_BUF_LEN);
      //    Process_Rising_Buffer(ic_ch4_buf, IC_BUF_LEN);
      //    //Print_IC_Buffer(3, ic_ch3_buf, IC_BUF_LEN);
      //}
      //if (ic_ch4_ready)
      //{
      //    ic_ch4_ready = 0;
      //    Process_Falling_Buffer(ic_ch3_buf, IC_BUF_LEN);
      //    Process_Rising_Buffer(ic_ch4_buf, IC_BUF_LEN);
      //    //Print_IC_Buffer(4, ic_ch4_buf, IC_BUF_LEN);
      //}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// En esta función hacemos el callback del DMA se ejecute cuando el buffer esté completo
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) || 
            (htim->hdma[TIM_DMA_ID_CC1] != NULL && htim->hdma[TIM_DMA_ID_CC1]->State == HAL_DMA_STATE_READY))
    {
        ic_ch1_ready = 1;
    }
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) || 
            (htim->hdma[TIM_DMA_ID_CC2] != NULL && htim->hdma[TIM_DMA_ID_CC2]->State == HAL_DMA_STATE_READY))
    {
        ic_ch2_ready = 1;
    }
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) || 
            (htim->hdma[TIM_DMA_ID_CC3] != NULL && htim->hdma[TIM_DMA_ID_CC3]->State == HAL_DMA_STATE_READY))
    {
        ic_ch3_ready = 1;
    }
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) || 
            (htim->hdma[TIM_DMA_ID_CC4] != NULL && htim->hdma[TIM_DMA_ID_CC4]->State == HAL_DMA_STATE_READY))
    {
        ic_ch4_ready = 1;
    }
}

// Función para imprimir los buffers
void Print_IC_Buffer(uint8_t ch, uint32_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        printf("EVENTO: CH%u | t=%lu\r\n", ch, buf[i]);
    }
}

void Process_Falling_Buffer(uint32_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t t = buf[i]; // ticks crudos del timer

        // Cambios en la maquina de estados (ocurre una bajada por lo que se queda esperando flanco de subida una vez que ocurre una bajada)
        if (pulse_fsm_ch1.state == PULSE_IDLE)
        {
            pulse_fsm_ch1.t_start = t;
            pulse_fsm_ch1.state = PULSE_WAIT_RISING;
        }
        else
        {
            // Bajada inesperada, entonces descartar
            pulse_fsm_ch1.state = PULSE_IDLE;
        }
    }
}

void Process_Rising_Buffer(uint32_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t t = buf[i];

        // Detecta el cambio al flanco de subida por lo que saca el ancho del pulso
        if (pulse_fsm_ch1.state == PULSE_WAIT_RISING)
        {
            uint32_t width = t - pulse_fsm_ch1.t_start;

            // Implementación del Timeout
            if (width <= PULSE_TIMEOUT_TICKS)
            {
                // Se almacenan los valores de el ancho de pulso y los timestamps de subida y bajada de flancos
                pulse_buffer[pulse_index].t_start = pulse_fsm_ch1.t_start;
                pulse_buffer[pulse_index].t_end   = t;
                pulse_buffer[pulse_index].width   = width;

                pulse_index++;

                // Esta logica comienza una vez que el buffer se llena, entonces imprime los datos del buffer almacenados
                if (pulse_index >= PULSE_BUF_LEN)
                {
                    printf("---- PULSOS ----\r\n");
                    for (uint32_t j = 0; j < PULSE_BUF_LEN; j++)
                    {
                        printf("START=%lu | END=%lu | WIDTH=%lu ticks\r\n", pulse_buffer[j].t_start, pulse_buffer[j].t_end, pulse_buffer[j].width);
                    }
                    pulse_index = 0;
                }
            }

            pulse_fsm_ch1.state = PULSE_IDLE;
        }
    }
}


/* Overflow del TIM2 → extiende a 64 bits */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        timer_high++;
}

/* Redirección printf a UART */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
