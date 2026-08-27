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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "arm_const_structs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// refer to ADC buffer size
#define CHUNK_SIZE 1024
#define TOTAL_BUFFER_SIZE (CHUNK_SIZE * 2)

// refer to FFT buffer size
#define FFT_FULL_BUFFER_SIZE CHUNK_SIZE

// 512
#define FFT_NUM_BINS (FFT_FULL_BUFFER_SIZE / 2)

// 511
#define FFT_NUM_COMPLEX_BINS (FFT_FULL_BUFFER_SIZE / 2 - 1)

#define PI 3.14159265358979f
#define SAMPLING_FREQ 16000.0f

#define HANN_GAIN 0.5f
#define FFT_SCALE (2.0f / (FFT_FULL_BUFFER_SIZE * HANN_GAIN))
#define EPSILON 1e-12f

#define MIN_DB (-50.0f)
#define MAX_DB (0.0f)

#define LCD_HEIGHT 128

// refer to FFT visualization
#define NUM_BARS 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// adc samples buffer
uint16_t adc_buffer[TOTAL_BUFFER_SIZE];

// 0 - nothing ready, 1 - first half ready, 2 - second half ready
volatile uint8_t buffer_ready_flag = 0;

// input buffer passed to CMSIS FFT
float32_t fft_input_buffer[FFT_FULL_BUFFER_SIZE];

// output buffer from CMSIS FFT
float32_t fft_output_buffer[FFT_FULL_BUFFER_SIZE];

// magnitude of FFT buffer
float32_t fft_magnitude_buffer[FFT_NUM_BINS];

// CMSIS FFT instance
arm_rfft_fast_instance_f32 fft_instance;

// window coefficients
float32_t window_buffer[FFT_FULL_BUFFER_SIZE];

// bars for visualization
float bars[NUM_BARS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void process_data(uint16_t* data, uint16_t length)
{
    // start measuring signal processing part
    HAL_GPIO_WritePin(ANALYZER_PIN_2_GPIO_Port, ANALYZER_PIN_2_Pin, GPIO_PIN_SET);

    // convert from uint16_t to float
    for (size_t i = 0; i < length; i++)
    {
        fft_input_buffer[i] = (float32_t) data[i];
    }

    float32_t mean;

    // calculate mean
    arm_mean_f32(fft_input_buffer, length, &mean);

    // remove DC offset
    arm_offset_f32(fft_input_buffer, -mean, fft_input_buffer, length);

    // normalize signal to approx -1.0 - +1.0
    for (size_t i = 0; i < length; ++i)
    {
        fft_input_buffer[i] /= 2048.f;
    }

    // apply window function
    arm_mult_f32(fft_input_buffer, window_buffer, fft_input_buffer, length);

    // do the FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);

    // magnitude
    // CMSIS-DSP arm_rfft_fast_f32 output format:
    // fft_output_buffer[0] = Re(DC)
    // fft_output_buffer[1] = Re(Nyquist)
    // fft_output_buffer[2] = Re(bin 1)
    // fft_output_buffer[3] = Im(bin 1)
    // fft_output_buffer[4] = Re(bin 2)
    // fft_output_buffer[5] = Im(bin 2)
    // ...
    // we ignore Nyquist bin

    // DC
    fft_magnitude_buffer[0] = fabsf(fft_output_buffer[0]);

    // bins 1.. N/2 - 1
    for (size_t bin = 1; bin <= FFT_NUM_COMPLEX_BINS; ++bin)
    {
        const float32_t real = fft_output_buffer[2 * bin];
        const float32_t imag = fft_output_buffer[(2 * bin) + 1];
        fft_magnitude_buffer[bin] = sqrtf(real * real + imag * imag);
    }

    // normalize magnitude

    // for real input and one-sided spectrum: 
    // - DC: amplitude = magnitude / (N * window_gain)
    // - ordinary bins: amplitude = (2 * magnitude) / (N * window_gain)
    // for Hann - window_gain = 0.5. therefore:
    // - DC: amplitude = magnitude / (N * 0.5)
    // - ordinary bins: amplitude = (2 * magnitude) / (N * 0.5)

    // DC
    fft_magnitude_buffer[0] /= FFT_FULL_BUFFER_SIZE * HANN_GAIN;

    // ordinary bins
    for (size_t bin = 1; bin <= FFT_NUM_COMPLEX_BINS; ++bin)
    {
      fft_magnitude_buffer[bin] *= FFT_SCALE;
    }

    // magnitude -> db
    for (size_t bin = 0; bin < FFT_NUM_BINS; ++bin)
    {
        fft_magnitude_buffer[bin] = 20.0f * log10f(fmaxf(fft_magnitude_buffer[bin], EPSILON));
    }

    // // debug: print min and max FFT magnitude in dB
    // float32_t min_db = fft_magnitude_buffer[0];
    // float32_t max_db = fft_magnitude_buffer[0];

    // for (size_t bin = 1; bin < FFT_NUM_BINS; ++bin)
    // {
    //     if (fft_magnitude_buffer[bin] < min_db)
    //     {
    //         min_db = fft_magnitude_buffer[bin];
    //     }

    //     if (fft_magnitude_buffer[bin] > max_db)
    //     {
    //         max_db = fft_magnitude_buffer[bin];
    //     }
    // }

    // printf("min=%d,max=%d\n", (int)lrintf(min_db), (int)lrintf(max_db));

    // peak detector
    float peak_db = 0.0f;
    uint32_t peak_bin = 0U;
    
    // skip bin 0 because DC should not be interesting after DC removal
    arm_max_f32(&fft_magnitude_buffer[1], FFT_NUM_BINS - 1, &peak_db, &peak_bin);

    // arm_max_f32() searched starting at bin 1 - therefore restore the actual fft bin index
    peak_bin++;

    const float peak_hz = ((float32_t) peak_bin * SAMPLING_FREQ) / FFT_FULL_BUFFER_SIZE;

    // linear grouping into bars
    // we group bins 0 ... 511 into NUM_BARS groups

    for (size_t bar = 0; bar < NUM_BARS; ++bar)
    {
        const size_t first_bin = (bar * FFT_NUM_BINS) / NUM_BARS;
        const size_t last_bin = ((bar + 1) * FFT_NUM_BINS) / NUM_BARS;

        float bar_db = MIN_DB;

        for (size_t bin = first_bin; bin < last_bin; ++bin)
        {
            if (fft_magnitude_buffer[bin] > bar_db)
            {
                bar_db = fft_magnitude_buffer[bin];
            }
        }

        // clamp to display range
        if (bar_db < MIN_DB)
        {
            bar_db = MIN_DB;
        }

        if (bar_db > MAX_DB)
        {
            bar_db = MAX_DB;
        }

        // map dB to LCD height
        const float32_t normalized = (bar_db - MIN_DB) / (MAX_DB - MIN_DB);
        const uint16_t bar_height = (uint16_t) (normalized * LCD_HEIGHT);
        bars[bar] = bar_height;
    }

    // stop measuring signal processing part
    HAL_GPIO_WritePin(ANALYZER_PIN_2_GPIO_Port, ANALYZER_PIN_2_Pin, GPIO_PIN_RESET);
  
    // start measuring visualization time
    HAL_GPIO_WritePin(ANALYZER_PIN_3_GPIO_Port, ANALYZER_PIN_3_Pin, GPIO_PIN_SET);
    
    // visualize bars
    for (size_t bar = 0; bar < NUM_BARS; ++bar)
    {
        printf("%d,", (int) bars[bar]);
    }
    printf("\n");

    // stop measuring visualization time
    HAL_GPIO_WritePin(ANALYZER_PIN_3_GPIO_Port, ANALYZER_PIN_3_Pin, GPIO_PIN_RESET);
    
}

void generate_hanning_window(float32_t* p_dst, uint16_t length)
{
    for (uint32_t i = 0; i < length; ++i)
    {
        p_dst[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t) length));
    }
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, TOTAL_BUFFER_SIZE);

  // init FFT
  arm_rfft_fast_init_f32(&fft_instance, FFT_FULL_BUFFER_SIZE);

  generate_hanning_window(window_buffer, FFT_FULL_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (buffer_ready_flag == 1)
    {
      // process first half of the buffer
      process_data(&adc_buffer[0], CHUNK_SIZE);
      
      // clean the flag
      buffer_ready_flag = 0;
    }
    else if (buffer_ready_flag == 2)
    {
      // process second half of the buffer
      process_data(&adc_buffer[CHUNK_SIZE], CHUNK_SIZE);

      // clean the flag
      buffer_ready_flag = 0;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) 
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
    return len;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_GPIO_WritePin(ANALYZER_PIN_1_GPIO_Port, ANALYZER_PIN_1_Pin, GPIO_PIN_SET);
    if (hadc->Instance == ADC1)
    {
        buffer_ready_flag = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_GPIO_WritePin(ANALYZER_PIN_1_GPIO_Port, ANALYZER_PIN_1_Pin, GPIO_PIN_RESET);
    if (hadc->Instance == ADC1)
    {
        buffer_ready_flag = 2;
    }
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
