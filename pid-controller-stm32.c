/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 **This notice applies to any and all portions of this file
 *that are not between comment pairs USER CODE BEGIN and
 *USER CODE END. Other portions of this file, whether 
 *inserted by the user or by software development tools
 *are owned by their respective copyright owners.
 *
 *COPYRIGHT(c) 2018 STMicroelectronics
 *
 *Redistribution and use in source and binary forms, with or without modification,
 *are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. Neither the name of STMicroelectronics nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/*USER CODE BEGIN Includes */

/*USER CODE END Includes */

/*Private variables ---------------------------------------------------------*/

/*USER CODE BEGIN PV */
/*Private variables ---------------------------------------------------------*/

/*USER CODE END PV */

/*Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/*USER CODE BEGIN PFP */
/*Private function prototypes -----------------------------------------------*/

/*USER CODE END PFP */

/*USER CODE BEGIN 0 */
#define mpu6050address 0xD0
#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define dt 0.001
#define M_PI 3.14159265359
float setpoint = -3.1;

uint8_t i2cbuf[6] = { 0 };
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temp;

float kp = 0;
float ki = 0;
float kd = 0;

float xaccel, yaccel, zaccel;
float pitch, roll;
float pitchAcc, rollAcc;
float gyro_x, gyro_y, gyro_z;
float PID = 0;
float errorroll = 0;
float errorsum = 0;
float gyroangle;
float prevangle = 0;
float Gyro_lpf = 0;
float iterm = 0;
float pterm = 0;
float dterm = 0;

void moveforward()
{
	HAL_GPIO_WritePin(Motor_A_In_1_GPIO_Port, Motor_A_In_1_Pin, 1);
	HAL_GPIO_WritePin(Motor_A_In_2_GPIO_Port, Motor_A_In_2_Pin, 0);

	HAL_GPIO_WritePin(Motor_B_In_1_GPIO_Port, Motor_B_In_1_Pin, 0);
	HAL_GPIO_WritePin(Motor_B_In_2_GPIO_Port, Motor_B_In_2_Pin, 1);
}

void movebackward()
{

	HAL_GPIO_WritePin(Motor_A_In_1_GPIO_Port, Motor_A_In_1_Pin, 0);
	HAL_GPIO_WritePin(Motor_A_In_2_GPIO_Port, Motor_A_In_2_Pin, 1);

	HAL_GPIO_WritePin(Motor_B_In_1_GPIO_Port, Motor_B_In_1_Pin, 1);
	HAL_GPIO_WritePin(Motor_B_In_2_GPIO_Port, Motor_B_In_2_Pin, 0);
}

/*USER CODE END 0 */

/**
 *@brief  The application entry point.
 *
 *@retval None
 */
int main(void)
{
	/*USER CODE BEGIN 1 */

	/*USER CODE END 1 */

	/*MCU Configuration----------------------------------------------------------*/

	/*Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*USER CODE BEGIN Init */

	/*USER CODE END Init */

	/*Configure the system clock */
	SystemClock_Config();

	/*USER CODE BEGIN SysInit */

	/*USER CODE END SysInit */

	/*Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/*USER CODE BEGIN 2 */
	//AFIO->MAPR = (AFIO->MAPR &(!AFIO_MAPR_SWJ_CFG_Msk)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_I2C1_REMAP;	// to fix i2c remap problem on this micro
	// reset the MPU

	i2cbuf[0] = 0x6B;
	i2cbuf[1] = 0x03;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 2, 100);

	HAL_Delay(5);

	//activate_MPU6050
	i2cbuf[0] = 0x6B;
	i2cbuf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 2, 100);

	//set DLPF_CFG FOR MPU 6050
	i2cbuf[0] = 0x1A;
	i2cbuf[1] = 0x02;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 2, 100);

	//configure gyro(250dps full scale)
	i2cbuf[0] = 0x1B;
	i2cbuf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 2, 100);
	// Configure accelerometer(+/- 2g)
	i2cbuf[0] = 0x1C;
	i2cbuf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 2, 100);
	// Finish setup MPU-6050 register

	kp = 40;
	kd = 2.1;
	ki = 300;

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	/*USER CODE END 2 */

	/*Infinite loop */
	/*USER CODE BEGIN WHILE */
	while (1)
	{

		/*USER CODE END WHILE */

		/*USER CODE BEGIN 3 */
	}
	/*USER CODE END 3 */

}

/**
 *@brief System Clock Configuration
 *@retval None
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks 
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time 
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/*SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/*USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	i2cbuf[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, mpu6050address, i2cbuf, 6, 100);

	ax = -(i2cbuf[0] << 8 | i2cbuf[1]);
	ay = -(i2cbuf[2] << 8 | i2cbuf[3]);
	az = (i2cbuf[4] << 8 | i2cbuf[5]);

	xaccel = ax / ACCELEROMETER_SENSITIVITY;
	yaccel = ay / ACCELEROMETER_SENSITIVITY;
	zaccel = az / ACCELEROMETER_SENSITIVITY;

	i2cbuf[0] = 0x43;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050address, i2cbuf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, mpu6050address, i2cbuf, 6, 100);

	gx = i2cbuf[0] << 8 | i2cbuf[1];
	gy = i2cbuf[2] << 8 | i2cbuf[3];
	gz = i2cbuf[4] << 8 | i2cbuf[5];

	gyro_x = gx / GYROSCOPE_SENSITIVITY;
	gyro_y = gy / GYROSCOPE_SENSITIVITY;
	gyro_z = gz / GYROSCOPE_SENSITIVITY;

	gyroangle = gyro_y * dt;
	//roll  = gyro_y *dt ;

	int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
	if (forceMagnitudeApprox > 16384 && forceMagnitudeApprox < 65536)
	{

		rollAcc = atan2f(ax, az) *(180 / M_PI);
		//		 		      roll =(gyroangle + prevangle) *0.98 + rollAcc *0.02;
		roll = (gyroangle + prevangle) *0.9934 + rollAcc *0.0066;
	}

	// without checking the roll just apply the out put of pid to pwm

	errorroll = roll - setpoint;	// calculating the error of angle
	errorsum = errorsum + errorroll;

	Gyro_lpf = (gyro_y + gyro_y *9) / 10;

	//PID =  kp*errorroll +  kd*gyro_y + ki*(errorsum)*dt;
	pterm = kp * errorroll;
	dterm = kd * gyro_y;
	iterm = ki *(errorsum) *dt;
	PID = pterm + iterm + dterm;
	//----------------------------

	if (PID > 999)
	{
		iterm = 0;
		PID = pterm + dterm;
	}
	else if (iterm > 300)
	{
		iterm = 300;
		PID = pterm + dterm + 300;
	}

	if (PID > 0)
	{
		movebackward();
		htim3.Instance->CCR1 = PID;
		htim3.Instance->CCR2 = PID;
		prevangle = roll;
	}
	if (PID < 0)
	{
		moveforward();

		htim3.Instance->CCR1 = fabs(PID);
		htim3.Instance->CCR2 = fabs(PID);
		prevangle = roll;
	}
}

/*USER CODE END 4 */

/**
 *@brief  This function is executed in case of error occurrence.
 *@param  file: The file name as string.
 *@param  line: The line in file as a number.
 *@retval None
 */
void _Error_Handler(char *file, int line)
{
	/*USER CODE BEGIN Error_Handler_Debug */
	/*User can add his own implementation to report the HAL error return state */
	while (1) {}
	/*USER CODE END Error_Handler_Debug */
}

# ifdef USE_FULL_ASSERT
/**
 *@brief  Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 *@param  file: pointer to the source file name
 *@param  line: assert_param error line source number
 *@retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/*USER CODE BEGIN 6 */
	/*User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/*USER CODE END 6 */
}#
endif /*USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************(C) COPYRIGHT STMicroelectronics *****END OF FILE****/