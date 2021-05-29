/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define R/W addresses for temp sensor
#define HTS221_READ_ADDRESS 0xbf
#define HTS221_WRITE_ADDRESS 0xbe
#define LPS22HB_READ_ADDRESS 0xbb
#define LPS22HB_WRITE_ADDRESS 0xba

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// Global calibration variable declarations
uint8_t H0_rH_Value;
uint8_t H1_rH_Value;
int16_t H0_T0_OUT_Value;
int16_t H1_T0_OUT_Value;

uint8_t T0_degC_Value;
uint8_t T1_degC_Value;
uint8_t T0_degC;
uint8_t T1_degC;
int16_t T0_OUT_Value;
int16_t T1_OUT_Value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

static void HTS221_pwr_en(void);
static void HTS221_get_cal_data(void);
static void HTS221_get_sensor_data(void);
static void LPS22HB_get_sensor_data(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void HTS221_pwr_en(void)
{
	// Configure control register 1 (CTRL_REG1, 0x20) bit 7 to enable one-shot
    uint8_t ctrlReg1 = 0x20;
    uint8_t ctrlData[] = {ctrlReg1, (1 << 7)};

    // Send the target register to the device
    HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, ctrlData, sizeof(ctrlData), 1000);

}

static void HTS221_get_cal_data(void)
{
    /*****************************************************************************************************************/
    // Request humidity and temperature calibration data stored in registers 0x30 to 0x3F
    // Reference pg. 26 of data sheet (https://www.st.com/resource/en/datasheet/hts221.pdf)
    //   for register names and definitions
    // Reference tech note TN1218 on calibration procedures
    // https://www.st.com/resource/en/technical_note/dm00208001-interpreting-humidity-and-temperature-readings-in-the-hts221-digital-humidity-sensor-stmicroelectronics.pdf

    /*****************************************************************************************************************/
    // Humidity calibration values

    // Register H0_rh_x2, address 0x30. Divide register value by 2 for calibration value
    uint8_t H0_rH_Address = 0x30;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &H0_rH_Address, sizeof(H0_rH_Address), 1000);
	H0_rH_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&H0_rH_Value, sizeof(H0_rH_Value), 1000);
	H0_rH_Value = H0_rH_Value / 2;

	// Register H1_rh_x2, address 0x31. Divide register value by 2 for calibration value
	uint8_t H1_rH_Address = 0x31;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &H1_rH_Address, sizeof(H1_rH_Address), 1000);
	H1_rH_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&H1_rH_Value, sizeof(H1_rH_Value), 1000);
	H1_rH_Value = H1_rH_Value / 2;

	// Register H0_T0_OUT, addresses 0x36 and 0x37
	uint8_t H0_T0_OUT_Address = 0x36 | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &H0_T0_OUT_Address, sizeof(H0_T0_OUT_Address), 1000);
	H0_T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&H0_T0_OUT_Value, sizeof(H0_T0_OUT_Value), 1000);

	// Register H1_T0_OUT, addresses 0x3A and 0x3B
	uint8_t H1_T0_OUT_Address = 0x3A | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &H1_T0_OUT_Address, sizeof(H1_T0_OUT_Address), 1000);
	H1_T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&H1_T0_OUT_Value, sizeof(H1_T0_OUT_Value), 1000);

    /*****************************************************************************************************************/
    // Temperature calibration values

    // Register T0_degC_x8, address 0x32. Divide register value by 8 for calibration value
    uint8_t T0_degC_Address = 0x32;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T0_degC_Address, sizeof(T0_degC_Address), 1000);
	T0_degC_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T0_degC_Value, sizeof(T0_degC_Value), 1000);
	T0_degC_Value = T0_degC_Value / 8;

    // Register T1_degC_x8, address 0x33. Divide register value by 8 for calibration value
    uint8_t T1_degC_Address = 0x33;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T1_degC_Address, sizeof(T1_degC_Address), 1000);
	T1_degC_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T1_degC_Value, sizeof(T1_degC_Value), 1000);
	T1_degC_Value = T1_degC_Value / 8;

    // Register T1/T0 msb, address 0x35. Mask bits (0 & 1), (2 & 3) to get values of T0_degC & T1_degC
    uint8_t T1_T0_msb_Address = 0x35;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T1_T0_msb_Address, sizeof(T1_T0_msb_Address), 1000);
	uint8_t T1_T0_msb_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T1_T0_msb_Value, sizeof(T1_T0_msb_Value), 1000);
	T0_degC = (T1_T0_msb_Value && (0b0011));
    T1_degC = (T1_T0_msb_Value && (0b1100));

    // Register T0_OUT, addresses 0x3C and 0x3D
	uint8_t T0_OUT_Address = 0x3C | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T0_OUT_Address, sizeof(T0_OUT_Address), 1000);
	T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T0_OUT_Value, sizeof(T0_OUT_Value), 1000);

    // Register T1_OUT, addresses 0x3C and 0x3D
	uint8_t T1_OUT_Address = 0x3C | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T1_OUT_Address, sizeof(T1_OUT_Address), 1000);
	T1_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T1_OUT_Value, sizeof(T1_OUT_Value), 1000);

}

static void HTS221_get_sensor_data(void)
{
    // Large char buffer for strings sent over the console
    char buffer[100] = {0};

	// Configure control register 2 (CTRL_REG2, 0x21) bit 0 to enable one-shot
    uint8_t ctrlReg2 = 0x21;
    uint8_t ctrlData[] = {ctrlReg2, (1 << 0)};

    // Send the target register to the device
    HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, ctrlData, sizeof(ctrlData), 1000);

    // Define status register (STATUS_REG2, 0x27) bit 0 to monitor for new sample available
    uint8_t statusReg = 0x27;
    uint8_t sampleReady = 0;

    // Loiter for a bit to allow time for conversion to complete and be made available
    uint8_t count = 0;
    while (count < 10)  // arbitrary "long enough" delay value
    {
        // Send the address of the status register
        HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &statusReg, sizeof(statusReg), 1000);

        // Read back the value of the status register
        HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&sampleReady, sizeof(sampleReady), 1000);

        // If the new sample is ready, break out of while-loop...
        if (sampleReady & 0x01)
        {
            break;
        }

        // Else wait for a bit, increment the counter, and keep looping
        HAL_Delay(100);
        count++;
    }

    // Read the values of the humidity register H_OUT, address 0x28 and 0x29
	uint8_t H_OUT_Address = 0x28 | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &H_OUT_Address, sizeof(H_OUT_Address), 1000);
	int16_t H_OUT_Value = 0xbeef; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&H_OUT_Value, sizeof(H_OUT_Value), 1000);

	// Calculate and print value of humidity in %rH.
	int16_t humidityValue = (((H1_rH_Value - H0_rH_Value) * (H_OUT_Value - H0_T0_OUT_Value))/(H1_T0_OUT_Value - H0_T0_OUT_Value)) + (H0_rH_Value);
	snprintf(buffer, sizeof(buffer), "\tHumidity: %d%%rH\n", humidityValue);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 1000);


	// Read the values of the temperature register T_OUT, address 0x2A and 0x2B
	uint8_t T_OUT_Address = 0x28 | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HTS221_WRITE_ADDRESS, &T_OUT_Address, sizeof(T_OUT_Address), 1000);
	int16_t T_OUT_Value = 0xbeef; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HTS221_READ_ADDRESS, (uint8_t *)&T_OUT_Value, sizeof(T_OUT_Value), 1000);

    // Calculate and print value of temperature in degC.
	int16_t temperatureValue = (((T1_degC_Value - T0_degC_Value) * (T_OUT_Value - T0_OUT_Value))/(T1_OUT_Value - T0_OUT_Value)) + (T0_degC_Value);
	snprintf(buffer, sizeof(buffer), "\tTemperature: %ddegC\n", temperatureValue);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 1000);

}

static void LPS22HB_get_sensor_data(void)
{
	/*****************************************************************************************************************/
	// To read from the on-board pressure sensor, we will configure the control registers to one-shot mode and then
	//   read the pressure value in three byte-sized chunks, shifting them into a 32-bit word that is then converted
	//   to units of hPa, as described in the module datasheet, https://www.st.com/resource/en/datasheet/dm00140895.pdf
	/*****************************************************************************************************************/

	// Large char buffer for strings sent over the console
	char buffer[100] = {0};

	// Register CTRL_REG2, address 0x11. Configures sensor module for one-shot mode
	uint8_t CTRL_REG2_Address = 0x11;
	HAL_I2C_Master_Transmit(&hi2c2, LPS22HB_WRITE_ADDRESS, &CTRL_REG2_Address, sizeof(CTRL_REG2_Address), 1000);

	// Register PRESS_OUT_XL, address 0x28. Pressure output value (LSB)
	uint8_t PRESS_OUT_XL_Address = 0x28;
	HAL_I2C_Master_Transmit(&hi2c2, LPS22HB_WRITE_ADDRESS, &PRESS_OUT_XL_Address, sizeof(PRESS_OUT_XL_Address), 1000);
	uint8_t PRESS_OUT_XL_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, LPS22HB_READ_ADDRESS, (uint8_t *)&PRESS_OUT_XL_Value, sizeof(PRESS_OUT_XL_Value), 1000);

	// Register PRESS_OUT_L, address 0x29. Pressure output value (mid part)
	uint8_t PRESS_OUT_L_Address = 0x29;
	HAL_I2C_Master_Transmit(&hi2c2, LPS22HB_WRITE_ADDRESS, &PRESS_OUT_L_Address, sizeof(PRESS_OUT_L_Address), 1000);
	uint8_t PRESS_OUT_L_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, LPS22HB_READ_ADDRESS, (uint8_t *)&PRESS_OUT_L_Value, sizeof(PRESS_OUT_L_Value), 1000);

	// Register PRESS_OUT_XL, address 0x2A. Pressure output value (MSB)
	uint8_t PRESS_OUT_H_Address = 0x2A;
	HAL_I2C_Master_Transmit(&hi2c2, LPS22HB_WRITE_ADDRESS, &PRESS_OUT_H_Address, sizeof(PRESS_OUT_H_Address), 1000);
	uint8_t PRESS_OUT_H_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, LPS22HB_READ_ADDRESS, (uint8_t *)&PRESS_OUT_H_Value, sizeof(PRESS_OUT_H_Value), 1000);

	// Variable to hold raw pressure output from sensor. Shift each PRESS_OUT_x variable in as directed in datasheet
	uint32_t PRESS_OUT_Value = (PRESS_OUT_XL_Value << 0);
	PRESS_OUT_Value = PRESS_OUT_Value | (PRESS_OUT_L_Value << 8);
	PRESS_OUT_Value = PRESS_OUT_Value | (PRESS_OUT_H_Value << 16);

	// Divide by 4096 as per datasheet to get units hPa and print results to console
	uint32_t pressureValue = PRESS_OUT_Value / 4096;
	snprintf(buffer, sizeof(buffer), "\tPressure: %luhPa\n", pressureValue);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 1000);

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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  // Enable the HTS221 PD bit in CR1 to allow collection of samples
  HTS221_pwr_en();

  // Get the calibration data stored in the HTS221 non-volatile memory
  HTS221_get_cal_data();

  // Header info for CLI
  char* cliHeader = "\nsimpleCLI Interface v0.4\n------------------------------\n";
  HAL_UART_Transmit(&huart1, (uint8_t*) cliHeader, strlen(cliHeader), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Define strings to structure prompt around
	  char* cliPrompt = "Options:\n\t1: HTS221 read\n\t2: LPS22HB read\n\t3: RTC read (not implemented)\r\n$> ";
	  char* cliResponse = "Invalid input!\r\n";

	  // Issue prompt
	  HAL_UART_Transmit(&huart1, (uint8_t*) cliPrompt, strlen(cliPrompt), 1000);

	  // Get the user selection and echo it on the terminal
	  char cliInput;
	  HAL_UART_Receive(&huart1, (uint8_t*) &cliInput, 1, HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart1, (uint8_t*) &cliInput, 1, 1000);

	  // Evaluate input
	  switch (cliInput)
	  {
		case '1':
			cliResponse = "\r\nHTS221 read request:\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) cliResponse, strlen(cliResponse), 1000);
			HTS221_get_sensor_data();
			break;

		case '2':
			cliResponse = "\r\nLPS22HB read request\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) cliResponse, strlen(cliResponse), 1000);
			LPS22HB_get_sensor_data();
			break;

		case '3':
			cliResponse = "\r\nRTC read *not implemented*\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) cliResponse, strlen(cliResponse), 1000);
			// TBD()
			break;

		default:
			HAL_UART_Transmit(&huart1, (uint8_t*) cliResponse, strlen(cliResponse), 1000);
			break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c2.Init.Timing = 0x10909CEC;
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
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
