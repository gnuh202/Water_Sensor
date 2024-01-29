/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define wait_time		53		//10S
#define TIMEOUT			3000	//ms

#define PH_V1			1063
#define PH_1			10.1
#define PH_V2			1804
#define PH_2			5.39
#define PH_OFFSET		0


#define STX				0x02
#define ETX				0x03
#define ACK				0x06
#define NAK				0x15

#define Device_ID		0x01
#define RK50004			0x0A	//Dissolved Oxygen

#define Frame_OK		1
#define Frame_Error		0

#define ADS1115_ADDRESS 0x48
#define READ_CONVERSION 0x00
#define CONFIG_REGISTER 0x01
#define A0 0x04
#define A1 0x05
#define A2 0x06
#define A3 0x07
#define GAIN_6114 0x00
#define GAIN_4096 0x01
#define GAIN_2048 0x02
#define MODE_CONTINOUS 0
#define MODE_SINGLE_SHOT 1
#define LSB_REGISTER 0x83

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void SysTick_Interrupt();
void SysTick_Init(uint32_t time);
void Delay_ms(uint32_t mS);

void USART1_RXNE_CallBack();
void USART2_RXNE_CallBack();
void USART3_RXNE_CallBack();

void Lora_Send(uint8_t dat);
void Lora_Buffer_Write(uint8_t dat);

void ACK_Send();
void NAK_Send();
void Send_Data();
void Frame_Send(USART_TypeDef *USARTx , uint8_t *buff , uint8_t length);
void Frame_Process(uint8_t datalength);

void Led1_On()		{LL_GPIO_SetOutputPin(LED_1_GPIO_Port, LED_1_Pin);}				// Power
void Led1_Off()		{LL_GPIO_ResetOutputPin(LED_1_GPIO_Port, LED_1_Pin);}
void Led2_On()		{LL_GPIO_SetOutputPin(LED_2_GPIO_Port, LED_2_Pin);}				// Relay
void Led2_Off()		{LL_GPIO_ResetOutputPin(LED_2_GPIO_Port, LED_2_Pin);}
void Led3_On()		{LL_GPIO_SetOutputPin(LED_3_GPIO_Port, LED_3_Pin);}				// Feedback
void Led3_Off()		{LL_GPIO_ResetOutputPin(LED_3_GPIO_Port, LED_3_Pin);}
void Relay_On()		{LL_GPIO_SetOutputPin(Relay_GPIO_Port, Relay_Pin);}
void Relay_Off()	{LL_GPIO_ResetOutputPin(Relay_GPIO_Port, Relay_Pin);}

void FeedBack();
void Buffer_Clear(uint8_t *data, uint8_t datleng);
void float2ascii(uint8_t *data ,float num , uint8_t length2);
uint16_t CRC_Cal(unsigned char *buffer, unsigned int length);
void Modbus_configframe(USART_TypeDef *USARTx,uint8_t slaveAddr, uint8_t FC, uint16_t startAddr, uint16_t numRegisters);
uint8_t ModBus_Read(USART_TypeDef *USARTx,uint32_t RX_Buff,uint8_t *RX_Index,uint8_t slaveAddr, uint8_t FC, uint16_t startAddr, uint16_t numRegisters);

float pH_Cal(float val);

uint8_t ADS1115_Init(uint8_t adc_in, uint8_t gain, uint8_t mode);
float Read_Voltage(uint8_t adc_in, uint8_t gain, uint8_t mode);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RX3_Buffer[20],TX3_Buffer[99];	//LORA
uint8_t RX3_Temp = 0,TX3_Temp = 0;

//uint8_t RX2_Buffer[50];
//uint8_t RX2_Temp = 10;

uint8_t RX2_Buffer[50];			// DO
uint8_t RX2_Temp = 0;

char data[5][6] = {"id","relay","ph","do","temp"};
char value[5][10] = {"7","-","-","-","-"};

uint8_t fail = 0;
uint32_t Led_count;
uint32_t silent_interval;
uint32_t Send_Interval = 1000*wait_time;
uint32_t timeout;

float pH_Temp;
float pH_Voltage;
float ph_Val;
//float a_ph,b_ph;

uint8_t ADS_Buffer[10];

uint8_t Relay_status;

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(LL_GPIO_IsInputPinSet(GPIOB, FeedBack_Pin))
  {
	  value[1][0] = 0x30;
	  Led3_Off();
  }
  else
  {
	  value[1][0] = 0x31;
	  Led3_On();
  }
  SysTick_Init(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(Send_Interval >= wait_time*1000)
	{
	  Send_Data();
	  while(timeout < 1000);
	  Frame_Send(USART3, TX3_Buffer, TX3_Temp);
	  timeout = 0;
	  Send_Interval = 0;
	}


	if(Relay_status)
	{
	  Relay_status = 0;
	  Frame_Process(2);
	  while(timeout < 1000);
	  Frame_Send(USART3, TX3_Buffer, TX3_Temp);
	  //timeout = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */
  LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE END USART2_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */
  LL_USART_EnableIT_RXNE(USART3);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Relay_GPIO_Port, Relay_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Relay_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Relay_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE1);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(FeedBack_GPIO_Port, FeedBack_Pin, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SysTick_Init(uint32_t time)
{
  SysTick->VAL = 0;
  SysTick->LOAD = 40*time-1;
  silent_interval = 0;
  SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk);
}

void Delay_ms(uint32_t mS)
{
  uint32_t time = mS;
  SysTick_Init(1000);
  while(time != 0)
     while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == SysTick_CTRL_COUNTFLAG_Msk)
	time--;

}
void SysTick_Interrupt()
{
	silent_interval++;
	Send_Interval++;
	Led_count++;
	timeout++;
	if(Led_count < 500)
		LL_GPIO_SetOutputPin(LED_1_GPIO_Port, LED_1_Pin);
	else
		LL_GPIO_ResetOutputPin(LED_1_GPIO_Port, LED_1_Pin);
	if(Led_count >= 5500)
		Led_count = 0;
}


void Lora_Send(uint8_t dat)
{
	  LL_USART_TransmitData8(USART3,dat);
	  while(!LL_USART_IsActiveFlag_TC(USART3));
}

void Lora_Buffer_Write(uint8_t dat)
{
	TX3_Buffer[TX3_Temp] = dat;
	if(TX3_Temp == 0xFF)	TX3_Temp = 0;
	else					TX3_Temp++;
}
void ACK_Send()
{
	Lora_Send(STX);
	Lora_Send(ACK);
	Lora_Send(ETX);
}
void NAK_Send()
{
	Lora_Send(STX);
	Lora_Send(NAK);
	Lora_Send(ETX);
}
void Frame_Send(USART_TypeDef *USARTx , uint8_t *buff , uint8_t length)
{
	for(uint8_t i = 0 ; i < length ; i++)
	{
		LL_USART_TransmitData8(USARTx,buff[i]);
		while(!LL_USART_IsActiveFlag_TC(USARTx));
	}
}

void Frame_Process(uint8_t datalength)
{
	uint8_t nchar,n,m,i,CheckSum;

	TX3_Temp = 0;
	Lora_Buffer_Write(STX);
	TX3_Temp = 2;
	Lora_Buffer_Write('{');
	for(nchar = 0 ; nchar < datalength ;nchar ++)
	{
	    Lora_Buffer_Write('"');
	    for(n = 0 ; n < 6 ; n++)
	    {
	    	if(data[nchar][n] == 0)	break;
	    	Lora_Buffer_Write(data[nchar][n]);
	    }
	    Lora_Buffer_Write('"');
	    Lora_Buffer_Write(':');
	    Lora_Buffer_Write('"');
	    for(m = 0 ; m < 5 ; m++)
	    {
	    	if(value[nchar][m] == 0)	break;
	    	Lora_Buffer_Write(value[nchar][m]);
	    }
	    Lora_Buffer_Write('"');
	    if(nchar < datalength - 1)
	    	Lora_Buffer_Write(',');
	    else
	    	Lora_Buffer_Write('}');
	    if(nchar == 1)	Relay_status = 0;
	}
	CheckSum = TX3_Temp - 2;
	TX3_Buffer[1] = CheckSum;
	CheckSum = 0;
	for(i = 1 ; i < TX3_Temp; i++)
	   CheckSum += TX3_Buffer[i];
	Lora_Buffer_Write(CheckSum);
	Lora_Buffer_Write(ETX);
}

void FeedBack()
{
  if(LL_GPIO_IsInputPinSet(GPIOB, FeedBack_Pin))
	{
		value[1][0] = 0x30;
		Led3_Off();
	}
	else
	{
		value[1][0] = 0x31;
		Led3_On();
	}
	Relay_status = 1;
}

void float2ascii(uint8_t *data ,float num , uint8_t length2)	// length.length2
{
	uint8_t i = 0;
	uint32_t qwe = 1;
	int32_t numvalue;
	uint8_t length = 1;
	*data = 0;
	data += 9;
	if(length2 != 0)	length2++;
	for(i = 0 ; i < 9 ; i++)
	{
		*data = 0;
		data--;
	}
	if(num<0)
	{
		*data = '-';
		num = -num;
		data++;
	}
	numvalue = (uint32_t)num;
	for(i = 0 ; i < 10 ; i++)
	{	numvalue = numvalue/10;
		if(numvalue == 0)	break;
		else
			length++;
	}

	data += length + length2 - 1;


	for(i = 1 ; i < length2 ; i++)
		qwe *= 10;
	numvalue = (uint32_t)(num*qwe);

	for(i = 1;i <= length + length2;i++)
	{
		if(i == length2)
			*data = '.';
		else
		{
			*data = numvalue%10 + 0x30;
			numvalue = numvalue/10;
		}
		data--;
	}
}

void Buffer_Clear(uint8_t *data, uint8_t datleng)
{
	for(uint8_t i = 0 ; i < datleng ; i++)
		*data = 0;
}
uint16_t CRC_Cal(unsigned char *buffer, unsigned int length)
{
     uint16_t i, j, temp_bit, temp_int, crc;
     crc = 0xFFFF;
     for (i=0; i < length; i++)
     {
         temp_int = (char) *buffer++;
         crc ^= temp_int;
         for(j=0; j<8; j++)
         {
             temp_bit = crc & 0x0001;
             crc >>= 1;
             if (temp_bit != 0) crc ^= 0xA001;
         }
     }
     return crc;
}

void Modbus_configframe(USART_TypeDef *USARTx,uint8_t slaveAddr, uint8_t FC, uint16_t startAddr, uint16_t numRegisters)
{
  uint8_t txBuffer[8];

  txBuffer[0] = slaveAddr;
  txBuffer[1] = FC;
  txBuffer[2] = startAddr >> 8;
  txBuffer[3] = startAddr & 0xFF;
  txBuffer[4] = numRegisters >> 8;
  txBuffer[5] = numRegisters & 0xFF;

  // CRC
  uint16_t crc = CRC_Cal(txBuffer, 6);
  txBuffer[6] = crc & 0xFF;
  txBuffer[7] = crc >> 8;
  Frame_Send(USARTx, txBuffer, 8);
}

uint8_t ModBus_Read(USART_TypeDef *USARTx,uint32_t RX_Buff,uint8_t *RX_Index,uint8_t slaveAddr, uint8_t FC, uint16_t startAddr, uint16_t numRegisters)
{
	uint8_t repeat = 3;
	  while(repeat > 0)
	  {
		  *RX_Index = 0;
		  Modbus_configframe(USARTx, slaveAddr, FC, startAddr, numRegisters);
		  silent_interval = 0;
		  while(silent_interval < 500)
			  if(*RX_Index != 0) break;

		  if(*RX_Index == 0)
		  {		//Timeout  -> Send again
			  repeat--;
			  continue;
		  }
		  else
		  {					//Ok
			  silent_interval = 0;
			  while(1)				// wait for received frame sucessfully
				  if(silent_interval >= 5) break;
			  if(CRC_Cal(RX_Buff, *RX_Index) == 0x0000)
				  return Frame_OK;
			  else
				  return Frame_Error;
		  }
	  }
	  return Frame_Error;
}

void Send_Data()
{
	uint32_t DO_Temp;
	uint32_t Nhietdo_Temp;

	float DO_val;
	float Nhietdo_val;

	pH_Temp = 0;
	for(uint8_t i = 0 ; i < 10 ; i ++)
	{
		  pH_Voltage = Read_Voltage(A2, GAIN_4096, MODE_SINGLE_SHOT);
		  silent_interval = 0;
		  while(silent_interval < 5);
		  pH_Temp +=  pH_Voltage;
	}
	pH_Voltage = pH_Temp/10;
	ph_Val = pH_Cal(pH_Voltage);

	float2ascii(value[2],ph_Val,2);
	//float2ascii(value[3],orp_Val,0);

	if(ModBus_Read(USART2, RX2_Buffer, &RX2_Temp, RK50004, 3, 0, 6) == Frame_OK)
	{
		  if(RX2_Buffer[0] == RK50004 && RX2_Buffer[1] == 0x03 && RX2_Buffer[2] == 0x0C)
		  {
			  DO_Temp = (RX2_Buffer[3] << 24) | (RX2_Buffer[4] << 16) | (RX2_Buffer[5] << 8) | RX2_Buffer[6];
			  DO_val = *(float*)&DO_Temp;
			  float2ascii(value[3], DO_val,  2);
			  Nhietdo_Temp = (RX2_Buffer[11] << 24) | (RX2_Buffer[12] << 16) | (RX2_Buffer[13] << 8) | RX2_Buffer[14];
			  Nhietdo_val = *(float*)&Nhietdo_Temp;
			  float2ascii(value[4], Nhietdo_val,  2);
		  }
		  Buffer_Clear(RX2_Buffer,50);
	}
	Frame_Process(5);
	//Frame_Send(USART3, TX3_Buffer, TX3_Temp);

}
void USART3_RXNE_CallBack()
{
	if(LL_USART_IsActiveFlag_RXNE(USART3))
	{
		RX3_Buffer[RX3_Temp++] = LL_USART_ReceiveData8(USART3);
		if(RX3_Buffer[0] != STX)	RX3_Temp = 0;
		else
		{
			if((RX3_Temp > 2)&&(RX3_Buffer[2] == ETX))
			{
				ACK_Send();
				if(RX3_Buffer[1] == '0')
				{
					Relay_Off();
					Led2_Off();
				}
				if(RX3_Buffer[1] == '1')
				{
					Relay_On();
					Led2_On();
				}
				RX3_Temp = 0;
				RX3_Buffer[0] = 0;
				RX3_Buffer[1] = 0;
				RX3_Buffer[2] = 0;
			}
		}
		LL_USART_ClearFlag_RXNE(USART3);
	}

	if(RX3_Temp > 19)	RX3_Temp = 0;
	LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	timeout = 0;

}
void USART2_RXNE_CallBack()
{
	if(LL_USART_IsActiveFlag_RXNE(USART2))
	{
		RX2_Buffer[RX2_Temp++] = LL_USART_ReceiveData8(USART2);
		silent_interval = 0;
		if(RX2_Temp > 50)	RX2_Temp = 0;
		LL_USART_ClearFlag_RXNE(USART2);
	}
	LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	/*
    RX2_Buffer[RX2_Temp++] = LL_USART_ReceiveData8(USART2);
    LL_USART_ClearFlag_RXNE(USART2);
    RX2_Temp++;
    silent_interval = 0;
    if(RX2_Temp > 50)	RX2_Temp = 0;
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);*/
}

float pH_Cal(float val)
{
	return PH_OFFSET + PH_1 + (PH_2 - PH_1)*(val - PH_V1)/(PH_V2 - PH_V1);
}

uint8_t ADS1115_Init(uint8_t adc_in, uint8_t gain, uint8_t mode)
{
    return 0x80|mode|(adc_in << 4)|(gain << 1);
}

float Read_Voltage(uint8_t adc_in, uint8_t gain, uint8_t mode)
{
	float factor;
	uint16_t ADS_Value;
	ADS_Buffer[0] = CONFIG_REGISTER;
	ADS_Buffer[1] = ADS1115_Init(adc_in, gain, mode);
	ADS_Buffer[2] = LSB_REGISTER;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADS_Buffer, 3, 100);
	ADS_Buffer[0] = READ_CONVERSION;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADS_Buffer, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADS_Buffer, 2, 100);
	ADS_Value = ADS_Buffer[0] << 8 | ADS_Buffer[1];
	if (ADS_Value < 0)	ADS_Value = 0;
	if (gain == 0x00)	factor = 6114.0/32767.0;
	else if (gain == 0x01)	factor = 4096.0/32767.0;
	else	factor = 2048.0/32767.0;
	return 	factor * (float)ADS_Value;
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
	  Led1_On();
	  Led2_On();
	  Led3_On();
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
