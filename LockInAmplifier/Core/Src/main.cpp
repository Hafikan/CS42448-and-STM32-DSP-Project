/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "CS42448.hpp"
#include "arm_math.h"
using namespace Codec_Controller;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 1024
#define BLOCK_SIZE 156
#define NUM_TAPS 156

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
Codec_Typedef cs42448;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */
void MX_CODEC_Init(void){
	cs42448.CHIP_ADDRESS = 0x90;
	cs42448.i2c_port = hi2c2;
	cs42448.Timeout = 500;
	cs42448.Trials = 3;
	cs42448.Codec_Reset_Pin = CODEC_RST_Pin;
	cs42448.Codec_Reset_Pin_Port = CODEC_RST_GPIO_Port;

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
static volatile int16_t codec_buffer[BUFFER_SIZE];
static volatile int16_t codec_buffer_r[BUFFER_SIZE/2];
static volatile int16_t codec_buffer_l[BUFFER_SIZE/2];
static volatile int16_t filter_out_buffer_r[BUFFER_SIZE];
static volatile int16_t filter_out_buffer_l[BUFFER_SIZE];
float32_t filter_in[BUFFER_SIZE];
float32_t filter_out[BUFFER_SIZE];
float32_t *filter_in_ptr = &filter_in[0];
float32_t *filter_out_ptr = &filter_out[0];
float32_t filter_in_2[BUFFER_SIZE];
float32_t filter_out_2[BUFFER_SIZE];
float32_t *filter_in_ptr_2 = &filter_in_2[0];
float32_t *filter_out_ptr_2 = &filter_out_2[0];
__HAL_RCC_FMC_CLK_ENABLE();
FMAC_

static float32_t fir_state[BLOCK_SIZE+NUM_TAPS-1];
const float32_t fir_coeffs[NUM_TAPS] = {
		  0.005172297278269435,
		  0.000691085441073851,
		  0.0007364934661995266,
		  0.0007833330616343696,
		  0.0008314309095881036,
		  0.0008808637704817325,
		  0.0009316054923466316,
		  0.0009836740399602583,
		  0.0010363242611205886,
		  0.0010915080786413332,
		  0.001146186788449287,
		  0.001202676511894471,
		  0.001260670295338245,
		  0.0013191856575960022,
		  0.0013783759848459713,
		  0.0014386953268898611,
		  0.001500174140205052,
		  0.0015623618017705772,
		  0.0016257098567251918,
		  0.0016895218967297539,
		  0.0017540993381709555,
		  0.001819696844582189,
		  0.0018859449512150937,
		  0.0019527127722429394,
		  0.0020201734194449364,
		  0.0020880287178480336,
		  0.002156021361230318,
		  0.0022244012480791006,
		  0.002292908551085153,
		  0.0023613631437434503,
		  0.0024300616718126934,
		  0.00249898064675117,
		  0.0025677443003562536,
		  0.0026367716407843375,
		  0.0027056680356674625,
		  0.002774438396436514,
		  0.0028432981460767624,
		  0.0029119651334827625,
		  0.002979608620910208,
		  0.0030462601965383727,
		  0.00311250553991895,
		  0.0031767976858819375,
		  0.003240818829501474,
		  0.0033044573035579117,
		  0.0033683652561028025,
		  0.003432511201503763,
		  0.003495411788126375,
		  0.003553584320643537,
		  0.0036052604649049577,
		  0.003671888205717174,
		  0.003724257235436093,
		  0.003779411123785605,
		  0.0038327290395655106,
		  0.003884417269345659,
		  0.003934482762558805,
		  0.0039828991347179805,
		  0.004030229139848226,
		  0.004073568994229324,
		  0.004119066886621473,
		  0.004159523352986089,
		  0.004198087205446811,
		  0.004235939370221528,
		  0.0042720361991397705,
		  0.004305569187191348,
		  0.004336534659601636,
		  0.004365447845882646,
		  0.004391866401525129,
		  0.004416855667835182,
		  0.004439469669356571,
		  0.004459605013668342,
		  0.004477598472958103,
		  0.004493190222417955,
		  0.004506177981610728,
		  0.004516867360583609,
		  0.004525397400883775,
		  0.004531380572232312,
		  0.004535116440140057,
		  0.004536449693795746,
		  0.004535116440140057,
		  0.004531380572232312,
		  0.004525397400883775,
		  0.004516867360583609,
		  0.004506177981610728,
		  0.004493190222417955,
		  0.004477598472958103,
		  0.004459605013668342,
		  0.004439469669356571,
		  0.004416855667835182,
		  0.004391866401525129,
		  0.004365447845882646,
		  0.004336534659601636,
		  0.004305569187191348,
		  0.0042720361991397705,
		  0.004235939370221528,
		  0.004198087205446811,
		  0.004159523352986089,
		  0.004119066886621473,
		  0.004073568994229324,
		  0.004030229139848226,
		  0.0039828991347179805,
		  0.003934482762558805,
		  0.003884417269345659,
		  0.0038327290395655106,
		  0.003779411123785605,
		  0.003724257235436093,
		  0.003671888205717174,
		  0.0036052604649049577,
		  0.003553584320643537,
		  0.003495411788126375,
		  0.003432511201503763,
		  0.0033683652561028025,
		  0.0033044573035579117,
		  0.003240818829501474,
		  0.0031767976858819375,
		  0.00311250553991895,
		  0.0030462601965383727,
		  0.002979608620910208,
		  0.0029119651334827625,
		  0.0028432981460767624,
		  0.002774438396436514,
		  0.0027056680356674625,
		  0.0026367716407843375,
		  0.0025677443003562536,
		  0.00249898064675117,
		  0.0024300616718126934,
		  0.0023613631437434503,
		  0.002292908551085153,
		  0.0022244012480791006,
		  0.002156021361230318,
		  0.0020880287178480336,
		  0.0020201734194449364,
		  0.0019527127722429394,
		  0.0018859449512150937,
		  0.001819696844582189,
		  0.0017540993381709555,
		  0.0016895218967297539,
		  0.0016257098567251918,
		  0.0015623618017705772,
		  0.001500174140205052,
		  0.0014386953268898611,
		  0.0013783759848459713,
		  0.0013191856575960022,
		  0.001260670295338245,
		  0.001202676511894471,
		  0.001146186788449287,
		  0.0010915080786413332,
		  0.0010363242611205886,
		  0.0009836740399602583,
		  0.0009316054923466316,
		  0.0008808637704817325,
		  0.0008314309095881036,
		  0.0007833330616343696,
		  0.0007364934661995266,
		  0.000691085441073851,
		  0.005172297278269435
};
uint32_t block_size = BLOCK_SIZE;
uint32_t num_blocks = BUFFER_SIZE / BLOCK_SIZE;
arm_fir_instance_f32 lpf;
unsigned long t1;
unsigned long t2;
unsigned long diff;
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

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  MX_CODEC_Init();
  arm_fir_init_f32(&lpf, NUM_TAPS, (float32_t *)&fir_coeffs[0], &fir_state[0], block_size);


  PowerDownEnable(&cs42448);
  Codec_IsReady(&cs42448);
  SetI2SInterface(&cs42448);
  HandleRegisters(&cs42448,0x04);// Check I2S Interface
  SetADCMode(&cs42448);
  HandleRegisters(&cs42448,0x05);// Check ADC Mode
  PowerDownDisable(&cs42448);
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)codec_buffer, BUFFER_SIZE);

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2sClockSelection = RCC_I2SCLKSOURCE_SYSCLK;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_SYSCLK;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CODEC_RST_GPIO_Port, CODEC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CODEC_RST_Pin */
  GPIO_InitStruct.Pin = CODEC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CODEC_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){


	for(int i=0;i<BUFFER_SIZE;i++){
		filter_in[i] = codec_buffer[i];
		if(i % 2 == 0){
			codec_buffer_r[i/2] = codec_buffer[i];

		}
		else{
			codec_buffer_l[i/2] = codec_buffer[i];
		}

	}
	t1 = DWT->CYCCNT;
	for(int i =0;i<num_blocks;i++){
		arm_fir_f32(&lpf,filter_in_ptr+(i*block_size), filter_out_ptr+(i*block_size),block_size);
	}
	t2 = DWT->CYCCNT;
	diff = t2 - t1;
	for(int i=0;i<BUFFER_SIZE;i++){
		filter_in_2[i] = filter_in[i];

	}
	for(int i =0;i<num_blocks;i++){
		arm_fir_f32(&lpf,filter_in_ptr_2+(i*block_size), filter_out_ptr_2+(i*block_size),block_size);
	}

	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)codec_buffer, BUFFER_SIZE);

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
