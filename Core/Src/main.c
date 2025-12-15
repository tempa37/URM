/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mbcrc.h"
#include <stdbool.h>
#include <math.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buf_size_rx             11
#define buf_size_tx             10
//#define ID                      0x01
#define MODBUS_ILLEGAL_FUNCTION 0x01
#define ILLEGAL_DATA_ADDRESS    0x02
#define ILLEGAL_DATA_VALUE      0x03
#define I2C_DEV_ADDR            0x40

#define RELE_NUM            8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */
uint8_t receive_buf[buf_size_rx] = {0};
uint8_t transmit_buf[buf_size_tx] = {0};
uint8_t state = 0;
uint8_t buf_of_control = 0;
uint8_t a,b,state11 = 0xff, global_error = 0x00;
//uint8_t state22 = 0xFF;
uint8_t investigator = 0x00;
uint8_t readout = 0;
extern bool FLAG_OK = true;
uint8_t gID = 0;
uint32_t cntr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */
void RDO_command_1(uint8_t command_num);
void WDO_command_5(uint8_t command_num);
void WMDO_command_15(uint8_t command_num);
void ERROR_checksum_handler ();
void ERROR_handler(uint8_t command_num);
void check_addr();
void single_DO();
void double_DO();
void reading_DO();
void revolver_on();
void revolver_off();
void shoot(uint8_t shift, uint8_t meaning);
void BDU_pin_activate(uint8_t number, uint8_t on_off);
void zeroing_the_buffer(void);
uint8_t ID_parsing(void);
//uint8_t change_i2c(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t gSwitchToReceiveCount = 0;
void SwitchToReceive() {
  ++gSwitchToReceiveCount;
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT);
}

uint8_t ID_parsing(void)
{
  uint8_t ID_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) + 
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) << 1) +
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) << 2);
  return ID_1;
}
void zeroing_the_buffer(void)
{
  memset(receive_buf, 0, 11);
  SwitchToReceive();
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT);
}

uint8_t change_i2c(void)
{
  uint8_t out = 0;
  uint8_t port[RELE_NUM];
  
  port[0] = (GPIOB->ODR & GPIO_ODR_0) ? 1 : 0;
  port[1] = (GPIOA->ODR & GPIO_ODR_7) ? 1 : 0;
  port[2] = (GPIOB->ODR & GPIO_ODR_1) ? 1 : 0;
  port[3] = (GPIOA->ODR & GPIO_ODR_8) ? 1 : 0;
  port[4] = (GPIOA->ODR & GPIO_ODR_4) ? 1 : 0;
  port[5] = (GPIOA->ODR & GPIO_ODR_3) ? 1 : 0;
  port[6] = (GPIOA->ODR & GPIO_ODR_5) ? 1 : 0;
  port[7] = (GPIOA->ODR & GPIO_ODR_6) ? 1 : 0;
  
  for (int i = 0; i < RELE_NUM; i++)
    out |= (port[i] << i);

  return out;
}

extern TickType_t gLastTickCount;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if  (huart -> Instance  ==  USART1){ 
      if(receive_buf[0] == gID){
        switch (receive_buf[1]){
             case 0x03:                                                         //read Digital output
               gLastTickCount = xTaskGetTickCount();
              RDO_command_1(receive_buf[1]);                            
              break;
             case 0x0F:                                                         //write Multiple Digital outputs
               gLastTickCount = xTaskGetTickCount();
              WMDO_command_15(receive_buf[1]);                            
              break;
             default:                                                           //errors handler
              ERROR_handler(MODBUS_ILLEGAL_FUNCTION);                           //MODBUS ILLEGAL FUNCTION//
              break;
         }       
       }
      else{ 
        ERROR_checksum_handler();
      }
      }
}

//Handler of command 0x01
void RDO_command_1(uint8_t command_num){
    uint16_t checksum = 0;
    checksum = mbcrc(receive_buf, 6);                                           //make CRC data
    if(receive_buf[6] == (uint8_t)((checksum >> 8) & 0xFF) || 
       receive_buf[7] == (uint8_t)(checksum & 0xFF)){
    reading_DO();
    }
    else{
      ERROR_checksum_handler();
}
}

//Handler of command 0x0F
void WMDO_command_15(uint8_t command_num)
{
    uint16_t checksum = 0;
    
    if (receive_buf[6] == 0x01){
    checksum = mbcrc(receive_buf, 8);                                           //make CRC data
    if(receive_buf[8] == (uint8_t)((checksum >> 8) & 0xFF) && 
       receive_buf[9] == (uint8_t)(checksum & 0xFF)){
    single_DO();
    }
    else{
      ERROR_checksum_handler();
    }
    }
    if (receive_buf[6] == 0x02){
    checksum = mbcrc(receive_buf, 9);                                          //make CRC data
    if(receive_buf[9] == (uint8_t)((checksum >> 8) & 0xFF) || 
       receive_buf[10] == (uint8_t)(checksum & 0xFF)){
    double_DO();
    }
    else{
      ERROR_checksum_handler();
    }
    }
}

//When a checksum error occurs, the slave device is silent in response
void ERROR_checksum_handler()
{
  SwitchToReceive();
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
}

//This function handles the error by taking its number
void ERROR_handler(uint8_t command_num)
{
    uint16_t checksum = 0;
    uint8_t transmit_buf[5] = {0};
    
    transmit_buf[0] = receive_buf[0];
    transmit_buf[1] = receive_buf[1] + 0x80;
    transmit_buf[2] = command_num;                                              //The received function code cannot be processed.
    checksum = mbcrc(transmit_buf, 3);                                          //make CRC data
    transmit_buf[3] = (uint8_t) ((checksum >> 8) & 0xFF);                       //write CRC Hi bit
    transmit_buf[4] = (uint8_t) (checksum & 0xFF);                              //write CRC Lo bit
    HAL_UART_Transmit(&huart1, transmit_buf,  5, 10);
    
    SwitchToReceive();
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
}

#define TUMBLER_STATES_NUM 256
uint16_t gTumblerBuff = 0;

uint8_t gOsBuff = 0;

void reading_DO()
{
    uint16_t checksum = 0;
    
    if (receive_buf[5] > 0x02){
      ERROR_handler(ILLEGAL_DATA_VALUE);  //ILLEGAL DATA VALUE 
    }
  
    //check_addr();
    int cCrcIdx = 5;
    if (receive_buf[5] == 0x02) {
      cCrcIdx = 7;
    }
    transmit_buf[0] = gID;
    transmit_buf[1] = receive_buf[1];
    transmit_buf[2] = 0x02 * receive_buf[5];
    transmit_buf[3] = gOsBuff; //Состояние ОС реле
    transmit_buf[4] = global_error;
    if (receive_buf[5] == 0x02) {
      transmit_buf[5] = gTumblerBuff; //Состояние тумблеров
      transmit_buf[6] = state;
    }
    checksum = mbcrc(transmit_buf, cCrcIdx);                    //make CRC data
    transmit_buf[cCrcIdx] = (uint8_t) ((checksum >> 8) & 0xFF); //write CRC Hi byte
    transmit_buf[cCrcIdx + 1] = (uint8_t) (checksum & 0xFF);    //write CRC Lo byte
    while(cntr<600){
    cntr++;
    }
    cntr=0;
    HAL_UART_Transmit(&huart1, transmit_buf,  cCrcIdx + 2, 10);
    
    SwitchToReceive();
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
}

bool gReset = false;
bool gCheckingTumbler = false;

void ResetOutput() {
  gReset = true;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
}

void single_DO()
{
  uint16_t checksum = 0;

  if (receive_buf[2] > 0x00 || receive_buf[3] > 0x00){
    ERROR_handler(ILLEGAL_DATA_ADDRESS);                                        //ILLEGAL_DATA_ADDRESS (Error 0x02)
  }
  if (receive_buf[4] > 0x00 || receive_buf[5] > 0x08){
    ERROR_handler(ILLEGAL_DATA_VALUE);                                          //ILLEGAL DATA VALUE 
  }
  if ((!gReset) && (!gCheckingTumbler)) {
    if (receive_buf[7] & 0x01){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
          }
          else{
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x02){     
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x04){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x08){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x10){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x20){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x40){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x80){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
          }
  }
  
  transmit_buf[0] = gID;
  transmit_buf[1] = receive_buf[1];
  transmit_buf[2] = receive_buf[2];
  transmit_buf[3] = receive_buf[3];
  transmit_buf[4] = 0x00;
  transmit_buf[5] = 0x08;
  checksum = mbcrc(transmit_buf, 6);                                            //make CRC data
  transmit_buf[6] = (uint8_t) ((checksum >> 8) & 0xFF);                         //write CRC Hi bit
  transmit_buf[7] = (uint8_t) (checksum & 0xFF);                                //write CRC Lo bit
  HAL_UART_Transmit(&huart1, transmit_buf,  8, 10);
  
  SwitchToReceive();
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
}

uint8_t gInp = 0;
void double_DO()
{
  uint16_t checksum = 0;

  if (receive_buf[2] > 0x00 || receive_buf[3] > 0x00){
    ERROR_handler(ILLEGAL_DATA_ADDRESS);                                        //ILLEGAL_DATA_ADDRESS (Error 0x02)
  }
  if (receive_buf[4] > 0x00 || receive_buf[5] > 0x10){
    ERROR_handler(ILLEGAL_DATA_VALUE);                                          //ILLEGAL DATA VALUE 
  }
  FLAG_OK = true;
  
  gInp = receive_buf[7];
  if ((!gReset) && (!gCheckingTumbler)) {
    if (receive_buf[7] & 0x01){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
          }
          else{
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x02){     
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x04){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x08){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x10){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x20){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x40){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          }
    if (receive_buf[7] & 0x80){
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
          }
          else{
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
          }
  }
  
  if (receive_buf[8] & 0x01){
        state = 0x01;
        }
        else{
          state = state & (0 << 0);
        }
  if (receive_buf[8] & 0x02){     
        state = state + (1 << 0x01);
        }
        else{
        state = state + (0 << 0x01);
        }
  if (receive_buf[8] & 0x04){
        state = state + (1 << 0x02);
        }
        else{
        state = state + (0 << 0x02);
        }
  if (receive_buf[8] & 0x08){
        state = state + (1 << 0x03);
        }
        else{
        state = state + (0 << 0x03);
        }
  if (receive_buf[8] & 0x10){
        state = state + (1 << 0x04);
        }
        else{
        state = state + (0 << 0x04);
        }
  if (receive_buf[8] & 0x20){
        state = state + (1 << 0x05);
        }
        else{
        state = state + (0 << 0x05);
        }
  if (receive_buf[8] & 0x40){
        state = state + (1 << 0x06);
        }
        else{
        state = state + (0 << 0x06);
        }
  if (receive_buf[8] & 0x80){
        state = state + (1 << 0x07);
        }
        else{
        state = state + (0 << 0x07);
        }
  if (state != 0xff) {
    gInp = receive_buf[7];
  }
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &state, 1, 10);

while(cntr<400){
cntr++;
}
cntr=0;
  transmit_buf[0] = gID;
  transmit_buf[1] = receive_buf[1];
  transmit_buf[2] = receive_buf[2];
  transmit_buf[3] = receive_buf[3];
  transmit_buf[4] = 0x00;
  transmit_buf[5] = 0x10;
  checksum = mbcrc(transmit_buf, 6);                                            //make CRC data
  transmit_buf[6] = (uint8_t) ((checksum >> 8) & 0xFF);                         //write CRC Hi bit
  transmit_buf[7] = (uint8_t) (checksum & 0xFF);                                //write CRC Lo bit

  HAL_UART_Transmit(&huart1, transmit_buf,  8, 10);
  
  SwitchToReceive();
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
}

void check_addr()
{
  if (receive_buf[3] == 0x00){
    if(receive_buf[5] > 0x08){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x01){
    if(receive_buf[5] > 0x07){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x02){
    if(receive_buf[5] > 0x06){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x03){
    if(receive_buf[5] > 0x05){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x04){
    if(receive_buf[5] > 0x04){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x05){
    if(receive_buf[5] > 0x03){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x06){
    if(receive_buf[5] > 0x02){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] == 0x07){
    if(receive_buf[5] > 0x01){
      ERROR_handler(ILLEGAL_DATA_ADDRESS);                                      //ILLEGAL DATA ADDRESS 
    }
  }
  if (receive_buf[3] > 0x07){
    ERROR_handler(ILLEGAL_DATA_ADDRESS);                                        //ILLEGAL DATA ADDRESS 
  }
}

void CheckTumblerSetting() {
  uint8_t cRx;
  uint16_t i = 0;
  uint16_t cTumblerBuff = 0;
  //Reset outputs
  ResetOutput();
  gCheckingTumbler = true;
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, (uint8_t*)&i, 1, 20);
  while (i < TUMBLER_STATES_NUM) {
    cRx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
    if (cRx == GPIO_PIN_RESET) {
      cTumblerBuff = i;
      break;
    }
    ++i;
    if (i < TUMBLER_STATES_NUM) {
      HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, (uint8_t*)&i, 1, 20);
    } else {
      cTumblerBuff = 0x8000;
      break;
    }
  }
  gTumblerBuff = cTumblerBuff;
  uint8_t cInit = 0xff;
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, (uint8_t*)&cInit, 1, 20);
  for (int i = 0; i < 1000; ++i);
  gCheckingTumbler = false;
  gReset = false;
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
  //HAL_InitTick(TICK_INT_PRIORITY);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &state11, 1, 100);
  CheckTumblerSetting();
  gID = ID_parsing();
  //HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &state22, 1, 100);
  SwitchToReceive();
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
  //HAL_UART_Receive_DMA(&huart1, recive_buf, buf_size_rx);
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 96);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 96);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

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
    osDelay(50);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00606092;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void SetOsAddr(int8_t iAddr) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 , (iAddr & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , (iAddr & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (iAddr & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  bool cIsError = false;
  /* Infinite loop */
  for(;;)
  {
    uint8_t cRx, cOs = 0;
    int i = 0;
    SetOsAddr(i);
    osDelay(2);
    while (i < RELE_NUM) {
      cRx = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
      if (cRx == GPIO_PIN_RESET) {
        cOs |= (1 << i);
      }
      ++i;
      if (i < RELE_NUM) {
        SetOsAddr(i);
        osDelay(2);
      } else {
        break;
      }
    }
    gOsBuff = cOs;
    if (global_error != 0) {
      cIsError = true;
      CheckTumblerSetting();
//      osDelay(10);
    } else if (cIsError) {
      cIsError = false;
//      CheckTumblerSetting();
//      osDelay(10);
    }
//    receive_buf[0] = 0;
//    receive_buf[1] = 0;
//    receive_buf[2] = 0;
//    receive_buf[3] = 0;
//    receive_buf[4] = 0;
//    receive_buf[5] = 0;
//    receive_buf[6] = 0;
//    receive_buf[7] = 0;
//    receive_buf[8] = 0;
//    receive_buf[9] = 0;
//    receive_buf[10] = 0;
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,  DMA_IT_HT); 
 
    if(FLAG_OK != true){
      for(int i = 0; i<500; i++){};
      HAL_UART_AbortReceive(&huart1);
      zeroing_the_buffer();
    }
    
    osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
uint32_t gErrorCount = 0;
TickType_t gLastTickCount;

/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    TickType_t cTickCount = xTaskGetTickCount();
    if ((cTickCount > 5000) && ((cTickCount - gLastTickCount) > 1000)) {
      ResetOutput(); //Сброс реле
      gReset = false;
    }
    if (!gCheckingTumbler) {
      global_error = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
      if (global_error != 0) {
        ++gErrorCount;
      }
    }
  /*  investigator = investigator + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) << 0x00);
    investigator = investigator + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) << 0x01);
    investigator = investigator + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) << 0x02);
    if (investigator > 0x07){
    investigator = 0x00;
    }
    HAL_I2C_Master_Receive(&hi2c1, I2C_DEV_ADDR, &readout, 1, 10);
    a = readout & (investigator - 1);
    b = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    if(a != b){
      osDelay(1);
    }*/
    osDelay(2);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
