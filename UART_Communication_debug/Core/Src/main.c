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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
    uint8_t magic[4];         // Magic number
    uint8_t payload_id;       // ID del payload
    uint8_t packet_type;      // Tipo di pacchetto
    uint16_t cargo_length;    // Lunghezza del cargo
    uint16_t packet_counter;  // Contatore del pacchetto
} TC_Header;

typedef struct {
    uint8_t magic[4];      	// Magic number 0xA5A5A5A5
    uint8_t payload_id;    	// Payload ID
    uint8_t packet_type;   	// Package type
    uint16_t cargo_length; // Package length
    uint16_t packet_counter;// Package counter
    uint32_t coarse_time;  	// Time (seconds)
    uint8_t fine_time[3];  	// Time (fraction of second)
} TM_Header;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAGIC_NUM_1 0xA5
#define MAGIC_NUM_2 0xA5
#define MAGIC_NUM_3 0xA5
#define MAGIC_NUM_4 0xA5
#define PAYLOAD_ID  0xFF
#define PACKET_TYPE_TM_HK   0x01  // Housekeeping
#define PACKET_TYPE_TM_SC   0x02  // Science
#define PACKET_TYPE_TM_AN   0x08  // Ancillary
#define PACKET_TYPE_TM_ACK  0x03  // Acknowledge
#define PACKET_TYPE_TM_NAK  0x04  // Not Acknowledge
#define PACKET_TYPE_TC_GENERIC  0x00
#define PACKET_TYPE_TC_TIME     0x0E
#define PACKET_TYPE_TC_SYN      0x0F


#define CRC_POLY 0x1021        // Polynomial CRC: X^16 + X^12 + X^5 + 1
#define CRC_INIT 0xFFFF
#define TC_HEADER_SIZE 10
#define TM_HEADER_SIZE 17
#define MAX_TC_CARGO_SIZE 256	//arbitrary set
#define MAX_TM_CARGO_SIZE 256   //1103

#define EPOCH_OFFSET 1630454400  // 1 January 2022 in UNIX seconds
#define SYNC_TIMEOUT_MS 6000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t received_MET;
uint32_t sync_tick = 0;

typedef struct {
    TC_Header header;
    uint8_t cargo[MAX_TC_CARGO_SIZE];
    uint16_t crc;
} TC_Packet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint16_t calculate_crc(uint8_t *data, size_t length){
    uint16_t crc = CRC_INIT;
    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLY;
            }else{
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFF; // Ritorna un valore a 16 bit
}
//function to get time
void get_cuc_time(uint32_t *coarse, uint8_t *fine) {
    if (sync_tick == 0) {
        // Se non è ancora sincronizzato, usa il tick corrente
        *coarse = (HAL_GetTick() / 1000) + EPOCH_OFFSET;
    } else {
        // Calcola il tempo corretto
        uint32_t elapsed_time = (HAL_GetTick() - sync_tick) / 1000;
        *coarse = received_MET + elapsed_time;
        *coarse += 3600;     //Italy time
    }

    uint32_t millis = (HAL_GetTick() - sync_tick) % 1000;  // Millisecondi rimanenti
    uint32_t fine_time = millis * 16777;  // Normalizzazione su 24 bit

    fine[0] = (fine_time >> 16) & 0xFF;  // Byte più significativo
    fine[1] = (fine_time >> 8) & 0xFF;   // Byte centrale
    fine[2] = fine_time & 0xFF;          // Byte meno significativo
}

void send_packet(UART_HandleTypeDef* huart) {
    static uint16_t packet_counter = 0;
    char cargo[] = "Hello from STM32";
    uint16_t cargo_length = strlen(cargo) /*+ 20*/;
    if(cargo_length>MAX_TM_CARGO_SIZE){
    	return;
    }

    static uint8_t buffer[TM_HEADER_SIZE + MAX_TM_CARGO_SIZE + 2];

    TM_Header header;
    header.magic[0] = MAGIC_NUM_1;
	header.magic[1] = MAGIC_NUM_2;
	header.magic[2] = MAGIC_NUM_3;
	header.magic[3] = MAGIC_NUM_4;

	header.payload_id = PAYLOAD_ID;
	header.packet_type = PACKET_TYPE_TM_SC;
	header.cargo_length = cargo_length;
	header.packet_counter = packet_counter++;

    get_cuc_time(&header.coarse_time, header.fine_time);


    memcpy(buffer, &header, TM_HEADER_SIZE);

    //strcpy((char*)&buffer[17], cargo);
    memcpy(buffer + TM_HEADER_SIZE, &cargo, cargo_length);

    //uint32_t timestamp = HAL_GetTick() / 1000;
    //snprintf((char*)&buffer[17 + strlen(cargo)], sizeof(buffer) - (17 + strlen(cargo)), " %u", (unsigned int)timestamp);

    uint16_t crc = calculate_crc(buffer, TM_HEADER_SIZE + cargo_length);
    memcpy(buffer + TM_HEADER_SIZE + cargo_length, &crc, sizeof(uint16_t));
    printf("Sending packet: ");
	for (int i = 0; i < TM_HEADER_SIZE+cargo_length+2; i++) {
		printf(" %02X", buffer[i]);
	}
	printf("\n");

    HAL_UART_Transmit_DMA(huart, buffer, TM_HEADER_SIZE+cargo_length+2);
}

void send_response(UART_HandleTypeDef* huart, bool acknowledge, uint16_t counter){
    static uint8_t response_buffer[TM_HEADER_SIZE + 2];

    TM_Header header;
    header.magic[0] = MAGIC_NUM_1;
	header.magic[1] = MAGIC_NUM_2;
	header.magic[2] = MAGIC_NUM_3;
	header.magic[3] = MAGIC_NUM_4;

	header.payload_id = PAYLOAD_ID;
	if(acknowledge){
		header.packet_type = PACKET_TYPE_TM_ACK;
	}
	else{
		header.packet_type = PACKET_TYPE_TM_NAK;
	}
	header.cargo_length = 0;
	header.packet_counter = counter;

    get_cuc_time(&header.coarse_time, header.fine_time);


    memcpy(response_buffer, &header, TM_HEADER_SIZE);


    uint16_t crc = calculate_crc(response_buffer, TM_HEADER_SIZE);
    memcpy(response_buffer + TM_HEADER_SIZE, &crc, sizeof(uint16_t));
    printf("Sending response: ");
	for (int i = 0; i < TM_HEADER_SIZE+2; i++) {
		printf(" %02X", response_buffer[i]);
	}
	printf("\n");
    HAL_UART_Transmit_DMA(huart, response_buffer, TM_HEADER_SIZE+2);
}
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TC_Packet received_packet;
uint8_t rx_buffer[TC_HEADER_SIZE];  				// Buffer to receive TC header
uint8_t rx_payload_buffer[MAX_TC_CARGO_SIZE + 2];  	// TC Payload + CRC
uint8_t receiving_stage = 0;  						// 0: header, 1: payload+CRC
uint16_t expected_cargo_length = 0;					// cargo leangth



//uint8_t tx_buffer[50];     			// Buffer to send data (50 byte) //old
volatile bool send_tm = false;  					// Flag to send data
volatile bool tx_complete = true;  	// Flag per monitorare lo stato della trasmissione //volatile: slower execution
volatile bool sending_response = false;
volatile bool sync = false;
bool positive_response = true;
uint16_t pkg_counter_response=0;
int delay = 1000;


int _write(int file, char* ptr, int len){
	int DataIdx;
	for(DataIdx=0; DataIdx < len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Wait until the DMA has completed the previous transmission
	  	  if (tx_complete)
	  	  {
	  		  if(sending_response){
	  			  tx_complete = false;
	  			  sending_response = false;
	  			  send_response(&huart1, positive_response, pkg_counter_response);
	  			  if(sync) {
	  				  uint32_t start_time = HAL_GetTick();
	  				  while(sync){
	  					  __WFI();  // Wait in low-power mode
	  					  if((HAL_GetTick() - start_time) > SYNC_TIMEOUT_MS){
	  						  sync = false;
	  					  }
	  				  }
	  			  }
	  		  }
	  		  else if(send_tm){
	  			  tx_complete = false;  // Reset flag
	  			  //HAL_UART_Transmit_DMA(&huart1, tx_buffer, strlen((char *)tx_buffer)); //old
	  			  send_packet(&huart1);
	  			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  		  }
	  	  }
	  	  HAL_Delay(delay);  // Delay for periodic data sending
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_ODD;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // CHeck if it's the channel USART1
    {
        tx_complete = 1;  // Set the transmission as complete
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (receiving_stage == 0)  // Receiving header
        {
        	printf("Received packet header: ");
			for (int i = 0; i < TC_HEADER_SIZE; i++) {
				printf(" %02X", rx_buffer[i]);
			}
			printf("\n");
            memcpy(&received_packet.header, rx_buffer, TC_HEADER_SIZE);
            expected_cargo_length = received_packet.header.cargo_length;
            if(received_packet.header.magic[0]!=MAGIC_NUM_1 || received_packet.header.magic[1]!=MAGIC_NUM_2 ||
               received_packet.header.magic[2]!=MAGIC_NUM_3 || received_packet.header.magic[3]!=MAGIC_NUM_4)
            {
            	receiving_stage = 0;  // Reset
            	HAL_UART_Receive_IT(&huart1, rx_buffer, TC_HEADER_SIZE);
            	return;
            }
            if (expected_cargo_length > MAX_TC_CARGO_SIZE)  // Check
            {
                receiving_stage = 0;  // Reset
                HAL_UART_Receive_IT(&huart1, rx_buffer, TC_HEADER_SIZE);
                return;
            }
            receiving_stage = 1;
            HAL_UART_Receive_IT(&huart1, rx_payload_buffer, expected_cargo_length + 2);
        }
        else if (receiving_stage == 1)  // Receiving payload + CRC
        {
        	printf("Received second half packet: ");
			for (int i = 0; i < expected_cargo_length+2; i++) {
				printf(" %02X", rx_payload_buffer[i]);
			}
			printf("\n");

            memcpy(received_packet.cargo, rx_payload_buffer, expected_cargo_length);
            memcpy(&received_packet.crc, rx_payload_buffer + expected_cargo_length, 2);


            // Check CRC
            uint16_t calculated_crc = calculate_crc((uint8_t*)&received_packet, TC_HEADER_SIZE + expected_cargo_length);
            pkg_counter_response=received_packet.header.packet_counter;
            positive_response = false;
            if (calculated_crc == received_packet.crc)
            {// Check packet type
            	if(received_packet.header.packet_type == PACKET_TYPE_TC_GENERIC)
            	{// Check command
					if (strncmp((char*)received_packet.cargo, "Request TM", expected_cargo_length) == 0)
					{
						positive_response = true;
						send_tm = true;
					}
					else if (strncmp((char*)received_packet.cargo, "Stop TM", expected_cargo_length) == 0)
					{
						positive_response = true;
						send_tm = false;
					}
            	}
            	else if(received_packet.header.packet_type == PACKET_TYPE_TC_TIME)
				{
            		positive_response = true;
            		memcpy(&received_MET, received_packet.cargo, 4);
            		sync = true;
            	}
            	else if(received_packet.header.packet_type == PACKET_TYPE_TC_SYN)
            	{
            		positive_response = true;
            		sync_tick = HAL_GetTick();
            		sync = false;
            	}
            }
            sending_response = true;

            // Restart with header
            receiving_stage = 0;
            HAL_UART_Receive_IT(&huart1, rx_buffer, TC_HEADER_SIZE);
        }
    }
}

/*PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}*/
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
