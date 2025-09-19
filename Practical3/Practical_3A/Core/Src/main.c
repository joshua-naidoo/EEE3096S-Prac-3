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
#include <stdint.h>
#include "stm32f4xx.h"
#include <stdio.h>  // Add this for printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Task 1
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/
// Define global variables for Mandelbrot benchmarking
uint64_t checksum = 0;   // checksum returned from Mandelbrot functions
uint32_t start_time = 0;   // start time in ms (HAL_GetTick)
uint32_t end_time = 0;   // end time in ms (HAL_GetTick)
uint32_t execution_time = 0;   // execution time (end_time - start_time)
const int image_sizes[] = {128, 160, 192, 224, 256};   // array of square image dimensions

uint32_t first_time = 0;   // first time in ms (HAL_GetTick)
uint32_t last_time = 0;   // last time in ms (HAL_GetTick)
uint32_t runtime = 0;   // runtime (first_time - last_time)

uint64_t checksums_fixed[5] = {0};   // store fixed-point checksums
uint64_t checksums_double[5] = {0};   // store double checksums
uint64_t checksums_float[5] = {0};   // store float checksums
uint32_t times_float[5] = {0};   // store float execution times 
uint32_t times_fixed[5] = {0};   // store fixed-point execution times
uint32_t times_double[5] = {0};   // store double execution times


/* USER CODE END PV */

/* USER CODE END PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);


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
  /* USER CODE BEGIN 2 */

  // Task 3
  // Iterate through all five square images (i.e. width = height) of dimensions (128, 160, 192, 224, 256) for testing
  for (int i = 0; i < 5; i++) {

	  // Fixed-point arithmetic:

	  //TODO: Turn on LED 0 to signify the start of the operation
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  //TODO: Record the start time
	  start_time = HAL_GetTick();

	  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
	  checksum = calculate_mandelbrot_fixed_point_arithmetic(image_sizes[i], image_sizes[i], MAX_ITER);

	  //TODO: Record the end time
	  end_time = HAL_GetTick();

	  //TODO: Calculate the execution time
	  execution_time = end_time - start_time;

	  // Store checksums and execution times for fixed-point arithmetic
	  checksums_fixed[i] = checksum;

	  times_fixed[i] = execution_time;


	  //TODO: Turn on LED 1 to signify the end of the operation
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	  //TODO: Hold the LEDs on for a 1s delay
	  HAL_Delay(1000);

	  //TODO: Turn off the LEDs
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);


  }

  /*for (int i = 0; i < 5; i++) {
	  // Repeated Task 3 for float-precision arithmetic:

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  start_time = HAL_GetTick();
	  checksum = calculate_mandelbrot_float(image_sizes[i], image_sizes[i], MAX_ITER);
	  end_time = HAL_GetTick();
	  execution_time = end_time - start_time;

	  // Store checksums and execution times for float-precision arithmetic
	  checksums_float[i] = checksum;
	  times_float[i] = execution_time;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);


  }*/

  /*for (int i = 0; i < 5; i++) {
	  // Repeated Task 3 for double-precision arithmetic:

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  start_time = HAL_GetTick();
	  checksum = calculate_mandelbrot_double(image_sizes[i], image_sizes[i], MAX_ITER);
	  end_time = HAL_GetTick();
	  execution_time = end_time - start_time;

	  // Store checksums and execution times for double-precision arithmetic
	  checksums_double[i] = checksum;
	  times_double[i] = execution_time;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);


  }*/
  runtime = HAL_GetTick();


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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic

// Task 2
// Fixed-point Mandelbrot function
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;

    // Larger scale factor for higher precision (16.16 fixed-point)
    const int32_t SCALE = 1 << 16;
    const int64_t FOUR_SCALED = 4LL * SCALE;

    // Map pixel coordinates to [-2.5, 1] x [-1, 1]
    const int32_t x_scale = (int32_t)((3.5 * SCALE) / width);
    const int32_t y_scale = (int32_t)((2.0 * SCALE) / height);
    const int32_t x_offset = (int32_t)(-2.5 * SCALE);
    const int32_t y_offset = (int32_t)(-1.0 * SCALE);

    for (int y = 0; y < height; y++) {
        int32_t y0 = y * y_scale + y_offset;

        for (int x = 0; x < width; x++) {
            int32_t x0 = x * x_scale + x_offset;

            int32_t xi = 0;
            int32_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                // Promote to 64-bit before multiply
                int64_t xi_sq = ((int64_t)xi * xi) / SCALE;
                int64_t yi_sq = ((int64_t)yi * yi) / SCALE;

                // Escape condition
                if (xi_sq + yi_sq > FOUR_SCALED)
                    break;

                int64_t temp = xi_sq - yi_sq + x0;
                yi = (int32_t)(((int64_t)2 * xi * yi) / SCALE + y0);
                xi = (int32_t)(temp);

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;
}

// Task 4
//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;

    // Pre-calculate scaling factors to avoid repeated calculations
    const double x_scale = 3.5 / (double)width;
    const double y_scale = 2.0 / (double)height;

    for (int y = 0; y < height; y++) {
        double y0 = (double)y * y_scale - 1.0;

        for (int x = 0; x < width; x++) {
            double x0 = (double)x * x_scale - 2.5;

            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations) {
                double xi_sq = xi * xi;
                double yi_sq = yi * yi;

                // Early termination check
                if (xi_sq + yi_sq > 4.0) {
                    break;
                }

                double temp = xi_sq - yi_sq + x0;
                yi = 2.0 * xi * yi + y0;
                xi = temp;

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    const float x_scale = 3.5f / (float)width;
    const float y_scale = 2.0f / (float)height;

    for (int y = 0; y < height; y++) {
        float y0 = (float)y * y_scale - 1.0f;
        for (int x = 0; x < width; x++) {
            float x0 = (float)x * x_scale - 2.5f;
            float xi = 0.0f;
            float yi = 0.0f;
            int iteration = 0;
            while (iteration < max_iterations && (xi * xi + yi * yi) <= 4.0f) {
                float temp = xi * xi - yi * yi + x0;
                yi = 2.0f * xi * yi + y0;
                xi = temp;
                iteration++;
            }
            mandelbrot_sum += iteration;
        }
    }
    return mandelbrot_sum;
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
