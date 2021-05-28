// Global calibration variable declrations
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

static void get_cal_HST221(void)
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
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &H0_rH_Address, sizeof(H0_rH_Address), 1000);
	H0_rH_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&H0_rH_Value, sizeof(H0_rH_Value), 1000);
	H0_rH_Value = H0_rH_Value / 2;

	// Register H1_rh_x2, address 0x31. Divide register value by 2 for calibration value
	uint8_t H1_rH_Address = 0x31;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &H1_rH_Address, sizeof(H1_rH_Address), 1000);
	H1_rH_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&H1_rH_Value, sizeof(H1_rH_Value), 1000);
	H1_rH_Value = H1_rH_Value / 2;

	// Register H0_T0_OUT, addresses 0x36 and 0x37
	uint8_t H0_T0_OUT_Address = 0x36 | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &H0_T0_OUT_Address, sizeof(H0_T0_OUT_Address), 1000);
	H0_T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&H0_T0_OUT_Value, sizeof(H0_T0_OUT_Value), 1000);

	// Register H1_T0_OUT, addresses 0x3A and 0x3B
	uint8_t H1_T0_OUT_Address = 0x3A | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &H1_T0_OUT_Address, sizeof(H1_T0_OUT_Address), 1000);
	H1_T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&H1_T0_OUT_Value, sizeof(H1_T0_OUT_Value), 1000);

    /*****************************************************************************************************************/
    // Temperature calibration values

    // Register T0_degC_x8, address 0x32. Divide register value by 8 for calibration value
    uint8_t T0_degC_Address = 0x32;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T0_degC_Address, sizeof(T0_degC_Address), 1000);
	T0_degC_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T0_degC_Value, sizeof(T0_degC_Value), 1000);
	T0_degC_Value = T0_degC_Value / 8;

    // Register T1_degC_x8, address 0x33. Divide register value by 8 for calibration value
    uint8_t T1_degC_Address = 0x33;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T1_degC_Address, sizeof(T1_degC_Address), 1000);
	T1_degC_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T1_degC_Value, sizeof(T1_degC_Value), 1000);
	T1_degC_Value = T1_degC_Value / 8;

    // Register T1/T0 msb, address 0x35. Mask bits (0 & 1), (2 & 3) to get values of T0_degC & T1_degC
    uint8_t T1_T0_msb_Address = 0x35;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T1_T0_msb_Address, sizeof(T1_T0_msb_Address), 1000);
	uint8_t T1_T0_msb_Value = 0xff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T1_T0_msb_Value, sizeof(T1_T0_msb_Value), 1000);
	T0_degC = (T1_T0_msb_Value && (0b0011));
    T1_degC = (T1_T0_msb_Value && (0b1100));

    // Register T0_OUT, addresses 0x3C and 0x3D
	uint8_t T0_OUT_Address = 0x3C | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T0_OUT_Address, sizeof(T0_OUT_Address), 1000);
	T0_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T0_OUT_Value, sizeof(T0_OUT_Value), 1000);

    // Register T1_OUT, addresses 0x3C and 0x3D
	uint8_t T1_OUT_Address = 0x3C | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T1_OUT_Address, sizeof(T1_OUT_Address), 1000);
	T1_OUT_Value = 0xffff; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T1_OUT_Value, sizeof(T1_OUT_Value), 1000);

}


static void get_data_HST221(void)
{
    // Large-ish char buffer for strings sent over the console
    char buffer[100] = {0};

	// Configure control register 2 (CTRL_REG2, 0x21) bit 0 to enable one-shot
    uint8_t ctrlReg2 = 0x21;
    uint8_t ctrlData[] = {ctrlReg2, (1 << 0)};

    // Send the target register to the device
    HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, ctrlData, sizeof(ctrlData), 1000);

    // Define status register (STATUS_REG2, 0x27) bit 0 to monitor for new sample available
    uint8_t statusReg = 0x27;
    uint8_t sampleReady = 0;

    // Loiter for a bit to allow time for conversion to complete and be made available
    uint8_t count = 0;
    while (count < 10)  // arbitrary "long enough" delay value
    {
        // Send the address of the status register
        HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &statusReg, sizeof(statusReg), 1000);

        // Read back the value of the status register
        HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&sampleReady, sizeof(sampleReady), 1000);

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
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &H_OUT_Address, sizeof(H_OUT_Address), 1000);
	int16_t H_OUT_Value = 0xbeef; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&H_OUT_Value, sizeof(H_OUT_Value), 1000);

	// Calculate and print value of humidity in %rH.
	int16_t humidityValue = (((H1_rH_Value - H0_rH_Value) * (H_OUT_Value - H0_T0_OUT_Value))/(H1_T0_OUT_Value - H0_T0_OUT_Value)) + (H0_rH_Value);
	snprintf(buffer, sizeof(buffer), "\tHumidity: %d%%rH\n", humidityValue);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 1000);


	// Read the values of the temperature register T_OUT, address 0x2A and 0x2B
	uint8_t T_OUT_Address = 0x28 | 0x80;
	HAL_I2C_Master_Transmit(&hi2c2, HST221_WRITE_ADDRESS, &T_OUT_Address, sizeof(T_OUT_Address), 1000);
	int16_t T_OUT_Value = 0xbeef; // Junk default value
	HAL_I2C_Master_Receive(&hi2c2, HST221_READ_ADDRESS, (uint8_t *)&T_OUT_Value, sizeof(T_OUT_Value), 1000);

    // Calculate and print value of temperature in degC.
	int16_t temperatureValue = (((T1_degC_Value - T0_degC_Value) * (T_OUT_Value - T0_OUT_Value))/(T1_OUT_Value - T0_OUT_Value)) + (T0_degC_Value);
	snprintf(buffer, sizeof(buffer), "\tTemperature: %ddegC\n", temperatureValue);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 1000);
    
}
