/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "eeprom.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SW_ON 1
#define SW_OFF 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId HTTPHandle;
/* USER CODE BEGIN PV */
uint16_t VirtAddVarTab[12]={0x000C,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,0x000A,0x000B};
struct StateSwitch Switch;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void create_XML_ToSend(char*c,struct StateSwitch Switch);
void http_server_serve(struct netconn *conn);
void SetSwitch(char* buf,struct StateSwitch *pSwitch);
void set_GPIO(struct StateSwitch *pSwitch);
void IP_PARSER(char* buf,uint16_t* IP,uint16_t* Mask,uint16_t* Gateway);
void Write_NewIPConfig(uint16_t* IP,uint16_t* Mask,uint16_t* Gateway);
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
  HAL_FLASH_Unlock();
  EE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of HTTP */
  osThreadDef(HTTP, StartDefaultTask, osPriorityNormal, 0, 2048);
  HTTPHandle = osThreadCreate(osThread(HTTP), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R8_Pin|R4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R3_Pin|R2_Pin|R7_Pin|R6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R1_Pin|R5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R8_Pin R4_Pin */
  GPIO_InitStruct.Pin = R8_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin R3_Pin LD3_Pin R2_Pin 
                           R7_Pin R6_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|R3_Pin|LD3_Pin|R2_Pin 
                          |R7_Pin|R6_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R5_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* Callback (interrupt on GPIO) */
	if (GPIO_Pin==USER_Btn_Pin)
	{
	/* Mass Erase on sector 5,6,7 in order to recover factory configuration */
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors     = 3;
	HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	/* Reboot the board */
	NVIC_SystemReset();
	}
}
void http_server_serve(struct netconn *conn)
{
	 err_t recv_err=-1;
	 struct fs_file file;
	 char* buf;
	 u16_t buflen;struct netbuf *inbuf;
	 char http_index_html[384];
	 const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: application/xml\r\n\r\n";
	 uint16_t IP[4]={0,0,0,0};
	 uint16_t Mask[4]={0,0,0,0};
	 uint16_t Gateway[4]={0,0,0,0};

	 recv_err = netconn_recv(conn, &inbuf);
	 if (recv_err == ERR_OK)
	 {
		 netbuf_data(inbuf,(void**) &buf, &buflen);
		 if ((buflen >=5) && (strncmp(buf, "GET /", 5) == 0))
	     	 {
			 HAL_UART_Transmit(&huart3, (uint8_t *)"received...\n\r", 50, 1000);

			 if (strncmp((char const *)buf,"GET / HTTP",10)==0)
	 	     {
				 /* HTML Page asked by a client*/
				 fs_open(&file, "/page.html");
				 netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
				 fs_close(&file);
	 	      }


			 if (strncmp((char const *)buf,"GET /Switch?",12)==0)
			 {
				 /* Switch state asked by a client*/
				 create_XML_ToSend(http_index_html,Switch);
				 /* Send XML */
				 netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
				 netconn_write(conn, http_index_html, sizeof(http_index_html)-1, NETCONN_NOCOPY);
			 }


			 if (strncmp((char const *)buf,"GET /StateChanged",17)==0)
			 {
				 /* Client want to set switch State*/
				 SetSwitch(buf,&Switch);
				 create_XML_ToSend(http_index_html,Switch);
				 /* Send XML */
				 netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
				 netconn_write(conn, http_index_html, sizeof(http_index_html)-1, NETCONN_NOCOPY);
			 }

			 if (strncmp((char const *)buf,"GET /ChangeIP",13)==0)
			 {

				 /* Client want to set new IP configuration */
				 IP_PARSER(buf,IP,Mask,Gateway);
				 /* Write config on EEPROM */
				 Write_NewIPConfig(IP,Mask,Gateway);
				 /* Reboot the board */
				 NVIC_SystemReset();

			 }

	    }

	 netconn_close(conn);
	 netbuf_delete(inbuf);
	 }
}

void create_XML_ToSend(char*c,struct StateSwitch Switch)
{
	sprintf(c,"<COMPONENT><Switch><ID>SW1</ID><State>%i</State></Switch><Switch><ID>SW2</ID><State>%i</State></Switch><Switch><ID>SW3</ID><State>%i</State></Switch><Switch><ID>SW4</ID><State>%i</State></Switch><Switch><ID>SW5</ID><State>%i</State></Switch><Switch><ID>SW6</ID><State>%i</State></Switch><Switch><ID>SW7</ID><State>%i</State></Switch><Switch><ID>SW8</ID><State>%i</State></Switch></COMPONENT>",Switch.SW1_state,Switch.SW2_state,Switch.SW3_state,Switch.SW4_state,Switch.SW5_state,Switch.SW6_state,Switch.SW7_state,Switch.SW8_state);
}

void SetSwitch(char* buf,struct StateSwitch *pSwitch)
{
	char payload_state[8]="";
	memcpy(payload_state,&buf[18],8);
	/* payload example : 10100001 */
	/* convert string to int for each value of the payload*/
	pSwitch->SW1_state=(int)(payload_state[0]-48);
	pSwitch->SW2_state=(int)(payload_state[1]-48);
	pSwitch->SW3_state=(int)(payload_state[2]-48);
	pSwitch->SW4_state=(int)(payload_state[3]-48);
	pSwitch->SW5_state=(int)(payload_state[4]-48);
	pSwitch->SW6_state=(int)(payload_state[5]-48);
	pSwitch->SW7_state=(int)(payload_state[6]-48);
	pSwitch->SW8_state=(int)(payload_state[7]-48);
	set_GPIO(pSwitch);
}

void set_GPIO(struct StateSwitch *pSwitch)
{
	/* Set HIGH/low GPIO output connected to LD2 */
	HAL_GPIO_WritePin(GPIOC,R1_Pin,(int)!(pSwitch->SW1_state));
	HAL_GPIO_WritePin(GPIOB,R2_Pin,(int)!(pSwitch->SW2_state));
	HAL_GPIO_WritePin(GPIOB,R3_Pin,(int)!(pSwitch->SW3_state));
	HAL_GPIO_WritePin(GPIOA,R4_Pin,(int)!(pSwitch->SW4_state));
	HAL_GPIO_WritePin(GPIOC,R5_Pin,(int)!(pSwitch->SW5_state));
	HAL_GPIO_WritePin(GPIOB,R6_Pin,(int)!(pSwitch->SW6_state));
	HAL_GPIO_WritePin(GPIOB,R7_Pin,(int)!(pSwitch->SW7_state));
	HAL_GPIO_WritePin(GPIOA,R8_Pin,(int)!(pSwitch->SW8_state));
	/* Can add at least 7 more GPIO to control remotly */


}

void IP_PARSER(char* buf,uint16_t* IP,uint16_t* Mask,uint16_t* Gateway)
{
	char* saveptr=NULL;
	char payload_IP[15]="";
	char payload_Mask[15]="";
	char payload_Gateway[15]="";
	memcpy(payload_IP,&buf[14],15);
	memcpy(payload_Mask,&buf[30],15);
	memcpy(payload_Gateway,&buf[46],15);

// loop through the string to extract all other tokens

    IP[0]=(uint8_t)atoi(strtok_r(payload_IP,".",&saveptr));
    IP[1]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    IP[2]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    IP[3]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));

    Mask[0]=(uint8_t)atoi(strtok_r(payload_Mask,".",&saveptr));
    Mask[1]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    Mask[2]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    Mask[3]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));

    Gateway[0]=(uint8_t)atoi(strtok_r(payload_Gateway,".",&saveptr));
    Gateway[1]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    Gateway[2]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));
    Gateway[3]=(uint8_t)atoi(strtok_r(NULL,".",&saveptr));


}

void Write_NewIPConfig(uint16_t* IP,uint16_t* Mask,uint16_t* Gateway)
{
	for (int i=0;i<sizeof(IP);i++)
	{
		EE_WriteVariable(VirtAddVarTab[i],IP[i]);
	}
	for (int i=0;i<sizeof(IP);i++)
	{
		EE_WriteVariable(VirtAddVarTab[i+4],Mask[i]);
	}
	for (int i=0;i<sizeof(IP);i++)
	{
		EE_WriteVariable(VirtAddVarTab[i+8],Gateway[i]);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	struct netconn *conn, *newconn;
			err_t err, accept_err;
			/* set all switch to state 0*/
			memset(&Switch,0,sizeof(Switch));


			  /* Create a new TCP connection handle */
			  conn = netconn_new(NETCONN_TCP);

			  if (conn!= NULL)
			  {
			    /* Bind to port 80 (HTTP) with default IP address */
			    err = netconn_bind(conn, NULL, 80);

			    if (err == ERR_OK)
			    {
			    	HAL_UART_Transmit(&huart3, (uint8_t *)"listening...\n\r", 16, 1000);
			      /* Put the connection into LISTEN state */
			      netconn_listen(conn);


			      while(1)
			      {
			    	  /* Grab new connection. */
			    	   accept_err = netconn_accept(conn, &newconn);

			    	   /* Process the new connection. */
			    	   if (accept_err == ERR_OK)
			    	   {

			    		   HAL_UART_Transmit(&huart3, (uint8_t *)"accepted...\n\r", 16, 1000);

			    		   /* serve connection */
			    		   http_server_serve(newconn);

			    		   netconn_delete(newconn);




			    }
			  }
			    }
			  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
