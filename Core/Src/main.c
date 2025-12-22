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
#define buf_size_tx             14
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

IWDG_HandleTypeDef hiwdg;

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
extern bool FLAG_OK = true;
uint16_t gID = 0;
uint32_t cntr = 0;
uint8_t gInp = 0;
uint16_t gTumblerBuff = 0;
uint8_t gOsBuff = 0;

/* Конфигурация UART */
typedef struct
{
  uint16_t baud_x100;   // 1152 -> 115200
  uint16_t wordlen;     // 8 or 9
  uint16_t parity;      // 0-none,1-odd,2-even
  uint16_t stop;        // 1 or 2
} UartCfgFlash;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
static UartCfgFlash gUartCfg;
static UartCfgFlash def_uart;

UartCfgFlash def_uart = {
    .baud_x100   = 1152u,
    .wordlen = UART_WORDLENGTH_9B,
    .stop   = UART_STOPBITS_1,
    .parity     = UART_PARITY_EVEN
};


UartCfgFlash gUartCfg = {
    .baud_x100   = 1152u,
    .wordlen = UART_WORDLENGTH_9B,
    .stop   = UART_STOPBITS_1,
    .parity     = UART_PARITY_EVEN
};



bool gReset = false;
bool gCheckingTumbler = false;

#define FLASH_PAGE_SIZE_F030   0x400u 
#define FLASH_CFG_PAGE_ADDR    0x08007C00u //разрешаем запись только с этого адреса и далее
#define UPDATE_FLAG            0x08007C04u
#define UPDATE_FLAG2           0x08007C08u

#define BAUD_RATE              0x08007C0Cu
#define WORD_LENGHT            0x08007C10u
#define STOP_BITS              0x08007C14u
#define PARITY                 0x08007C18u
#define ID_MODBUS              0x08007C1Cu

#define TUMBLER_STATES_NUM     256

#define VECTORS_SRAM_BASE      (0x20000000UL)
#define MODBUS_TIMEOUT_ADDRESS 0x08007C00
#define MODBUS_TIMEOUT_MAX     1000u

#define PASS                   0x3131  //12593 in dec

#define FLASH_BACKUP_LEN       100u //при перезаписи флага копируем в буффер первые 100 байт

volatile uint16_t modbus_timeout = 20;
volatile uint16_t new_paket = 0;

volatile uint8_t gCrcErrCnt = 0;
#pragma section = ".intvec"   // IAR: даёт доступ к началу/концу секции
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */
void RDO_command_1(uint8_t command_num);   //modbus 0x01
void WMDO_command_15(uint8_t command_num); //modbus 0x0F

void ERROR_checksum_handler();  //modbus error
void ERROR_handler(uint8_t command_num); //modbus error

void single_DO();  //запсь одного регистра
void double_DO();  //запись регистров modbus
void reading_DO(); //чтение регистра modbus

void UART1_FullRestartRx(void); //Поднимает uart когда тот упал
void zeroing_the_buffer(void);  //очищает RX буффер, мб избыточна

static void UartCfg_LoadFromFlash(UartCfgFlash *cfg); //загружает настройки из флеша 
static void UartCfg_SetDefaults(UartCfgFlash *cfg);   //ставит дефолтные настройки
static bool UartCfg_Validate(const UartCfgFlash *cfg); //проверяет настройки из флеша
static void UartCfg_Apply(const UartCfgFlash *cfg);   //Применяет настройки uart


uint8_t ID_parsing(void); //читает UART ID с перемычек
void func_06(void);   //запись регистра времени ответа modbus
void OS_update(void); //помечает флагом что началось обновление
void SetOsAddr(int8_t iAddr); //считывает обратную связь
void CheckTumblerSetting(void); //заполняет gTumblerBuff (состояние тумблеров)

HAL_StatusTypeDef Flash_WriteU16_Preserve100(uint32_t addr, uint16_t value); //записывает флаги
HAL_StatusTypeDef Flash_ReadU16(uint32_t addr, uint16_t *out);               //читает флаги  
static void VectorTable_CopyToSRAM_AndRemap(void);                           //ремап таблицы прерываний
void SwitchToReceive();         //переключение на прием
void ResetOutput();             //сброс состояний пинов
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t gSwitchToReceiveCount = 0;


static void VectorTable_CopyToSRAM_AndRemap(void)
{
    uint32_t *src = (uint32_t *)__section_begin(".intvec");
    uint32_t *end = (uint32_t *)__section_end(".intvec");
    uint32_t words = (uint32_t)(end - src);

    uint32_t *dst = (uint32_t *)VECTORS_SRAM_BASE;

    __disable_irq();

    for (uint32_t i = 0; i < words; i++) {
        dst[i] = src[i];
    }

    __DSB();
    __ISB();

    /* Enable SYSCFG clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Remap SRAM at 0x00000000: MEM_MODE = 0b11 */
    SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) |
                    (SYSCFG_CFGR1_MEM_MODE_0 | SYSCFG_CFGR1_MEM_MODE_1);

    __DSB();
    __ISB();

    __enable_irq();
}



void SwitchToReceive() {
++gSwitchToReceiveCount;
__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
__HAL_UART_CLEAR_OREFLAG(&huart1);
__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

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
}


extern TickType_t gLastTickCount;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if  (huart -> Instance  ==  USART1)
  { 
    new_paket = 1;
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
  // при CRC ошибке slave молчит; но если ошибок много — перезапускаем приём
  if (++gCrcErrCnt >= 10u)
  {
    gCrcErrCnt = 0u;

    // на всякий случай прибить текущий "пакет"
    new_paket = 0u;

    // полный рестарт UART+DMA+флагов и повторный запуск ReceiveToIdle
    UART1_FullRestartRx();

    // UART1_FullRestartRx() уже делает memset(receive_buf,0) и стартует RX,
    // поэтому тут дополнительно SwitchToReceive() обычно не нужен
  }
  else
  {
    SwitchToReceive();
  }
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
}


/*
 *  reading_DO
 * ---------
 * Запись регистра 
 * 
 */
void reading_DO()
{
    uint16_t checksum = 0;
    
    if (receive_buf[5] > 0x03)
      {
        ERROR_handler(ILLEGAL_DATA_VALUE);  //ILLEGAL DATA VALUE 
      }
  

    int cCrcIdx = 5;
    if (receive_buf[5] == 0x02) 
      {
        cCrcIdx = 7;
      }
    transmit_buf[0] = gID;
    transmit_buf[1] = receive_buf[1];
    transmit_buf[2] = 0x02 * receive_buf[5];
    transmit_buf[3] = gOsBuff; 
    transmit_buf[4] = global_error;
    if (receive_buf[5] >= 0x02)  
      {
        transmit_buf[5] = gTumblerBuff; 
        transmit_buf[6] = state;
      }
    if (receive_buf[5] == 0x03)  
      {
        transmit_buf[7] = (uint8_t)(modbus_timeout >> 8);   
        transmit_buf[8] = (uint8_t)(modbus_timeout & 0xFF); // Lo
        cCrcIdx = 9;
      }
    checksum = mbcrc(transmit_buf, cCrcIdx);                    //make CRC data
    transmit_buf[cCrcIdx] = (uint8_t) ((checksum >> 8) & 0xFF); //write CRC Hi byte
    transmit_buf[cCrcIdx + 1] = (uint8_t) (checksum & 0xFF);    //write CRC Lo byte
    while(cntr<600)
      {
        cntr++;
      }
    cntr=0;
    HAL_UART_Transmit(&huart1, transmit_buf,  cCrcIdx + 2, 10);
    
    SwitchToReceive(); 
}


/*
 *  func_06
 * ---------
 * Запись регистра 0x03 "таймаут ответа по modbus"
 * 
 */
void func_06(void)
{
  
  uint16_t crc = mbcrc(receive_buf, 6);

  // CRC check (в твоём проекте CRC кладётся как Hi затем Lo)
  if ((receive_buf[6] != (uint8_t)((crc >> 8) & 0xFF)) ||
      (receive_buf[7] != (uint8_t)(crc & 0xFF)))
  {
    ERROR_checksum_handler();   // по Modbus: при CRC ошибке slave молчит
    return;
  }

  uint16_t regAddr  = (uint16_t)((receive_buf[2] << 8) | receive_buf[3]);
  uint16_t regValue = (uint16_t)((receive_buf[4] << 8) | receive_buf[5]);

  // принимаем только "регистр 3"
  if (regAddr != 0x02)
  {
    ERROR_handler(ILLEGAL_DATA_ADDRESS);
    return;
  }

  // (опционально) валидация значения
  if ((regValue > MODBUS_TIMEOUT_MAX))
  {
    ERROR_handler(ILLEGAL_DATA_VALUE);
    return;
  }

  // запись значения
  modbus_timeout = regValue;
  Flash_WriteU16_Preserve100(MODBUS_TIMEOUT_ADDRESS, modbus_timeout);

  // ответ — эхо запроса
  transmit_buf[0] = gID;              // можно receive_buf[0], но gID логичнее
  transmit_buf[1] = 0x06;
  transmit_buf[2] = receive_buf[2];
  transmit_buf[3] = receive_buf[3];
  transmit_buf[4] = receive_buf[4];
  transmit_buf[5] = receive_buf[5];

  crc = mbcrc(transmit_buf, 6);
  transmit_buf[6] = (uint8_t)((crc >> 8) & 0xFF);
  transmit_buf[7] = (uint8_t)(crc & 0xFF);

  HAL_UART_Transmit(&huart1, transmit_buf, 8, 10);
  SwitchToReceive();
  
}



void ResetOutput() {
  gReset = true;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
}



/*
 *  double_DO
 * ---------
 * Запись регистров
 * 
 */
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


/*
 *  double_DO
 * ---------
 * Запись регистров
 * 
 */

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
  
  
    state = receive_buf[8];
  
    if (state != 0xff) {
    gInp = receive_buf[7];
  }
  
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &state, 1, 10);
  
  
  
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

}

/*
 * Flash_ReadU16
 * ---------
 * Читает переменную по адресу
 * 
 */
void CheckTumblerSetting(void) {
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
  VectorTable_CopyToSRAM_AndRemap();
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
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &state11, 1, 100);
  CheckTumblerSetting();
  gID = ID_parsing();
  UartCfg_LoadFromFlash(&gUartCfg);
  SwitchToReceive();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  Flash_ReadU16(FLASH_CFG_PAGE_ADDR, (uint16_t*)&modbus_timeout);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 3749;
  hiwdg.Init.Reload = 3749;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

/*
 * UART1_FullRestartRx
 * ---------
 * жесткий рестарт UART при зависании
 * 
 */
void UART1_FullRestartRx(void)
{
  // 1) Защититься от гонок с IRQ UART/DMA
  __disable_irq();
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);   // подправь под свой IRQ DMA

  // 2) Остановить текущие операции UART/DMA
  // Abort всех направлений на всякий случай (если в момент рестарта идёт ответ)
  (void)HAL_UART_Abort(&huart1);

  // Остановить DMA-запросы со стороны UART
  (void)HAL_UART_DMAStop(&huart1);

  // Дополнительно прибить сами DMA-каналы (важно, если указатель DMA "уехал")
  if (huart1.hdmarx) (void)HAL_DMA_Abort(huart1.hdmarx);
  if (huart1.hdmatx) (void)HAL_DMA_Abort(huart1.hdmatx);

  // 3) Вычистить флаги ошибок/IDLE и опустошить RDR (хвост мусора)
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);

  // На многих STM32 чтение RDR помогает гарантированно убрать "залипшие" байты
  volatile uint32_t tmp;
#if defined(USART_RDR_RDR)
  tmp = huart1.Instance->RDR;
#elif defined(USART_DR_DR)
  tmp = huart1.Instance->DR;
#else
  tmp = huart1.Instance->RDR; // если попадёшь на другую серию — поправишь по заголовкам
#endif
  (void)tmp;

  // 4) Сбросить сам USART в состояние "как после ресета"
  // Такой подход прямо рекомендуют: FORCE_RESET/RELEASE_RESET перед реинициализацией. [web:9]
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();

  // 5) Полная переинициализация UART (включая RS485 DE)
  (void)HAL_UART_DeInit(&huart1);
  MX_USART1_UART_Init();

  // (Опционально) на некоторых сериях есть "DMA Disable on Reception Error" (DDRE):
  // если включено — при ошибке приёма DMA может отключаться.
#if defined(USART_CR3_DDRE)
  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DDRE);
#endif

  // 6) Очистить буфер и снова запустить ReceiveToIdle DMA
  memset(receive_buf, 0, sizeof(receive_buf));

  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_buf, buf_size_rx);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  // 7) Вернуть IRQ обратно
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  FLAG_OK = true;
  __enable_irq();
}



/*
 * Flash_ReadU16
 * ---------
 * Читает переменную по адресу
 * 
 */
HAL_StatusTypeDef Flash_ReadU16(uint32_t addr, uint16_t *out)
{
  if (out == NULL) return HAL_ERROR;
  if ((addr & 0x1u) != 0u) return HAL_ERROR;          // halfword alignment

  uint16_t v = *(volatile const uint16_t*)addr;       // memory-mapped чтение
  if (v == 0xFFFFu) {                                 // стёртая флеш 
    v = 10u;
  }
  *out = v;
  return HAL_OK;
}


/*
 * Flash_WriteU16_Preserve100
 * ---------
 * Записывает переменную по адресу, сохраняя первые 100 байт
 * 
 */
HAL_StatusTypeDef Flash_WriteU16_Preserve100(uint32_t addr, uint16_t value)
{
  if ((addr & 0x1u) != 0u) return HAL_ERROR; // halfword alignment


  if ((addr < FLASH_CFG_PAGE_ADDR) || (addr >= (FLASH_CFG_PAGE_ADDR + FLASH_PAGE_SIZE_F030)))
    return HAL_ERROR;

  // Для F030: страница 1 KB
  uint32_t page_start = addr & ~(FLASH_PAGE_SIZE_F030 - 1u);
  uint32_t off        = addr - page_start;

  // перезаписываем только первые 100 байт страницы
  if ((off + 2u) > FLASH_BACKUP_LEN) return HAL_ERROR;

  uint8_t buf[FLASH_BACKUP_LEN];
  memcpy(buf, (const void*)page_start, FLASH_BACKUP_LEN);

  // Записали новое значение в копию первых 100 байт
  buf[off + 0u] = (uint8_t)(value & 0xFFu);
  buf[off + 1u] = (uint8_t)((value >> 8) & 0xFFu);

  HAL_StatusTypeDef st;
  uint32_t page_error = 0;

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase   = FLASH_TYPEERASE_PAGES;     // постраничное стирание 
  erase.PageAddress = page_start;                // адрес начала страницы 
  erase.NbPages     = 1;                         // одна страница 

  st = HAL_FLASHEx_Erase(&erase, &page_error);
  if (st != HAL_OK) {
    HAL_FLASH_Lock();
    return st;
  }

  // Восстановить первые 100 байт 
  for (uint32_t i = 0; i < FLASH_BACKUP_LEN; i += 2u) {
    uint16_t hw = (uint16_t)(buf[i] | ((uint16_t)buf[i + 1u] << 8));
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_start + i, hw); 
    if (st != HAL_OK) break;
  }

  HAL_FLASH_Lock();
  return st;
}


/*
 * OS_update
 * ---------
 * Помечает во флеше необходимость обновления прошивки и инициирует
 * программный перезапуск контроллера.
 */
void OS_update(void)
{
    //  Отправляем Modbus ответ об успешном выполнении функции 0x2B
    uint8_t resp[5] = {0};
    uint16_t crc;
    resp[0] = gID;       // Slave address
    resp[1] = 0x2B;       // Echo function code
    crc     = mbcrc(resp, 2);
    resp[2] = 0x00;
    resp[3] = (uint8_t)(crc >> 8);
    resp[4] = (uint8_t)(crc & 0xFF);

    // Индикация передачи и само отправление
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, resp, sizeof(resp), 100);
    SwitchToReceive();
    //HAL_UART_DMAResume(&huart1);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    
    
    HAL_IWDG_Refresh(&hiwdg);
    

    //  Помечаем флаг обновления в конце флеша
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, UPDATE_FLAG, 0x1111);
    HAL_FLASH_Lock();

    HAL_DeInit();
    HAL_NVIC_SystemReset();
}



static bool UartCfg_Validate(const UartCfgFlash *cfg)
{
  // baud_x100: 12..11520 => 1200..1_152_000 (подстрой под свои требования)
  if (cfg->baud_x100 < 12u || cfg->baud_x100 > 11520u) return false;

  if (!(cfg->wordlen == 8u || cfg->wordlen == 9u)) return false;
  if (!(cfg->parity == 0u || cfg->parity == 1u || cfg->parity == 2u)) return false;
  if (!(cfg->stop == 1u || cfg->stop == 2u)) return false;

  // Важно: в HAL “9B + parity” фактически даёт 8 data bits + parity (типичный Modbus RTU).
  // Поэтому для Modbus обычно cfg->wordlen=9 и parity!=0.
  if (cfg->wordlen == 9u && cfg->parity == 0u) return false;

  return true;
}


static void UartCfg_SetDefaults(UartCfgFlash *cfg)
{
  cfg->baud_x100 = def_uart.baud_x100 * 100;
  cfg->wordlen   = def_uart.wordlen;
  cfg->parity    = def_uart.parity;
  cfg->stop      = def_uart.stop;
}

static void UartCfg_LoadFromFlash(UartCfgFlash *cfg)
{

  (void)Flash_ReadU16(BAUD_RATE,   &cfg->baud_x100);
  (void)Flash_ReadU16(WORD_LENGHT, &cfg->wordlen);
  (void)Flash_ReadU16(PARITY,      &cfg->parity);
  (void)Flash_ReadU16(STOP_BITS,   &cfg->stop);
  if(gID == 0x00)
  {
    (void)Flash_ReadU16(ID_MODBUS,   &gID);
    if(gID == 0xFF)
    {
      gID = 0x01;
    }
  }


  if (!UartCfg_Validate(cfg))
  {
    UartCfg_SetDefaults(cfg);
  }
}



static void UartCfg_Apply(const UartCfgFlash *cfg)
{
  // 1) Прибить текущие операции UART/DMA (как минимум RX)
  (void)HAL_UART_Abort(&huart1);
  (void)HAL_UART_DMAStop(&huart1);
  if (huart1.hdmarx) (void)HAL_DMA_Abort(huart1.hdmarx);
  if (huart1.hdmatx) (void)HAL_DMA_Abort(huart1.hdmatx);

  // 2) Дождаться окончания передачи (на всякий случай)
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {;}

  // 3) Очистить ошибки
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);

  // 4) Переинициализация UART с новыми параметрами
  (void)HAL_UART_DeInit(&huart1);

  huart1.Init.BaudRate   = (uint32_t)cfg->baud_x100 * 100u;
  huart1.Init.WordLength = (cfg->wordlen == 9u) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

  switch (cfg->parity)
  {
    case 1u: huart1.Init.Parity = UART_PARITY_ODD;  break;
    case 2u: huart1.Init.Parity = UART_PARITY_EVEN; break;
    default: huart1.Init.Parity = UART_PARITY_NONE; break;
  }

  huart1.Init.StopBits   = (cfg->stop == 2u) ? UART_STOPBITS_2 : UART_STOPBITS_1;
  huart1.Init.Mode       = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  // У тебя USART в RS485 режиме:
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  // 5) Поднять RX обратно
  SwitchToReceive();
}




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
    } else if (cIsError) {
      cIsError = false;

    }

 
    if(FLAG_OK != true){
      for(int i = 0; i<500; i++){};
      HAL_UART_AbortReceive(&huart1);
      UART1_FullRestartRx();
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
  TickType_t cLastCheckTime = 0;
  /* Infinite loop */
  for(;;)
  {
    TickType_t cTickCount = xTaskGetTickCount();
    if ((cTickCount > 5000) && ((cTickCount - gLastTickCount) > 1000)) {
      ResetOutput(); //����� ����
      gReset = false;
    }
    if (!gCheckingTumbler) {
      global_error = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
      if (global_error != 0) {
        ++gErrorCount;
      }
    }

    if ((cTickCount - cLastCheckTime) >= 3000) {
      cLastCheckTime = cTickCount;
      //CheckTumblerSetting();
    }
    
    osDelay(2);
    HAL_IWDG_Refresh(&hiwdg);
    
    
    
    if(new_paket)
    {
         osDelay(modbus_timeout);    
          //HAL_Delay(modbus_timeout);
                    
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
             case 0x06:
                func_06();
          
               break;
               
             case 0x2B:
                  OS_update();
               break;
             default:                                                           //errors handler
              ERROR_handler(MODBUS_ILLEGAL_FUNCTION);                           //MODBUS ILLEGAL FUNCTION//
              break;
             
         }       
       }
      else{ 
        ERROR_checksum_handler();
      }
     new_paket = 0;
    }
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
  if (htim->Instance == TIM1)
  {
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
