/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "interface.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "interface_cmds.h"
#include "iwdg.h"

/* USER CODE END Includes */

#define ENABLE_DEBUG_BASEBAND_TASK 0

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for interfaceTask */
osThreadId_t interfaceTaskHandle;
const osThreadAttr_t interfaceTask_attributes = {
  .name = "interfaceTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for basebandTask */
osThreadId_t basebandTaskHandle;
const osThreadAttr_t basebandTask_attributes = {
  .name = "basebandTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for debugBasebandTask */
osThreadId_t debugBasebandTaskHandle;
const osThreadAttr_t debugBasebandTask_attributes = {
  .name = "dbgBaseband",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void interface_resp(enum cmd_t cmd, uint8_t resp);
static void interface_send_blocking(uint8_t *buf, uint16_t len);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartInterfaceTask(void *argument);
void StartBasebandTask(void *argument);
void StartDebugBasebandTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN 0 */
static void interface_resp(enum cmd_t cmd, uint8_t resp)
{
  uint8_t tmp[3] = { (uint8_t)cmd, 3, resp };
  interface_send_blocking(tmp, 3U);
}

static void interface_send_blocking(uint8_t *buf, uint16_t len)
{
  for (;;)
  {
    HAL_StatusTypeDef status = interface_transmit_command(buf, len);
    if (status == HAL_OK)
    {
      return;
    }
    if (status == HAL_BUSY || status == HAL_ERROR)
    {
      osDelay(1);
      continue;
    }
    return;
  }
}
/* USER CODE END 0 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* add timers, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  interfaceTaskHandle = osThreadNew(StartInterfaceTask, NULL, &interfaceTask_attributes);
  basebandTaskHandle = osThreadNew(StartBasebandTask, NULL, &basebandTask_attributes);
#if ENABLE_DEBUG_BASEBAND_TASK
  debugBasebandTaskHandle = osThreadNew(StartDebugBasebandTask, NULL, &debugBasebandTask_attributes);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    LL_IWDG_ReloadCounter(IWDG);
    osDelay(100);
    HAL_GPIO_TogglePin(SVC_LED_GPIO_Port,SVC_LED_Pin);

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInterfaceTask */
/**
* @brief Function implementing the interfaceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInterfaceTask */
void StartInterfaceTask(void *argument)
{
  /* USER CODE BEGIN StartInterfaceTask */
  (void)argument;

  for(;;)
  {
    interface_poll_pending();
    HAL_GPIO_TogglePin(TX_LED_GPIO_Port,TX_LED_Pin);

    if(interface_comm == COMM_RDY)
    {
      interface_abort_receive_it();
      interface_receive_it((uint8_t*)rxb, 1U);
      interface_comm = COMM_IDLE;
      // HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, GPIO_PIN_SET);
      // osDelay(20);
      // HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, GPIO_PIN_RESET);

      uint32_t freq = 0;
      uint8_t ident[128] = {0};
      uint8_t resp[10] = {0};

      switch(rxb[0])
      {
      case CMD_PING:
        resp[0] = CMD_PING;
        resp[1] = 6;
        memcpy(&resp[2], (uint8_t*)&dev_err, sizeof(uint32_t));
        interface_send_blocking(resp, 6U);
        break;

      case CMD_SET_RX_FREQ:
        memcpy((uint8_t*)&freq, (uint8_t*)&rxb[2], sizeof(uint32_t));
        if(freq >= 420000000U && freq <= 450000000U)
        {
          trx_writecmd(STR_IDLE);
          osDelay(10);
          trx_data.rx_frequency = freq;
          uint32_t freq_word = (uint32_t)roundf((float)freq / 5000000.0f * ((uint32_t)1 << 16));
          trx_writereg(0x2F0C, (freq_word >> 16) & 0xFF);
          trx_writereg(0x2F0D, (freq_word >> 8) & 0xFF);
          trx_writereg(0x2F0E, freq_word & 0xFF);
          interface_resp(CMD_SET_RX_FREQ, ERR_OK);
        }
        else
        {
          interface_resp(CMD_SET_RX_FREQ, ERR_RANGE);
        }
        break;

      case CMD_SET_TX_FREQ:
        memcpy((uint8_t*)&freq, (uint8_t*)&rxb[2], sizeof(uint32_t));
        if(freq >= 420000000U && freq <= 450000000U)
        {
          trx_writecmd(STR_IDLE);
          osDelay(10);
          trx_data.tx_frequency = freq;
          uint32_t freq_word = (uint32_t)roundf((float)freq / 5000000.0f * ((uint32_t)1 << 16));
          trx_writereg(0x2F0C, (freq_word >> 16) & 0xFF);
          trx_writereg(0x2F0D, (freq_word >> 8) & 0xFF);
          trx_writereg(0x2F0E, freq_word & 0xFF);
          interface_resp(CMD_SET_TX_FREQ, ERR_OK);
        }
        else
        {
          interface_resp(CMD_SET_TX_FREQ, ERR_RANGE);
        }
        break;

      case CMD_SET_TX_POWER:
        if(rxb[2] * 0.25f >= -16.0f && rxb[2] * 0.25f <= 14.0f)
        {
          tx_dbm = rxb[2] * 0.25f;
          trx_data.pwr = (uint8_t)floorf(((float)rxb[2] * 0.25f + 18.0f) * 2.0f - 1.0f);
          interface_resp(CMD_SET_TX_POWER, ERR_OK);
        }
        else
        {
          interface_resp(CMD_SET_TX_POWER, ERR_RANGE);
        }
        break;

      case CMD_SET_FREQ_CORR:
        trx_data.fcorr = *((int16_t*)&rxb[2]);
        interface_resp(CMD_SET_FREQ_CORR, ERR_OK);
        break;

      case CMD_SET_AFC:
        if(rxb[2])
        {
          trx_writereg(0x2F01, 0x22);
        }
        else
        {
          trx_writereg(0x2F01, 0x02);
        }
        interface_resp(CMD_SET_AFC, ERR_OK);
        break;

      case CMD_SET_TX_START:
        if(trx_state != TRX_TX && dev_err == ERR_OK)
        {
          trx_state = TRX_TX;
          config_rf(MODE_TX, trx_data);
          osDelay(10);
          trx_writecmd(STR_STX);

          HAL_TIM_Base_Stop_IT(&htim10);
          interface_abort_receive_it();

          memset((uint8_t*)tx_bsb_buff, 0, BSB_RUNUP);
          tx_bsb_total_cnt = BSB_RUNUP;

          uint8_t header[2] = {0x2F | 0x40, 0x7E};
          set_CS(0);
          HAL_SPI_Transmit(&hspi1, header, 2, 10);
          interface_receive_it((uint8_t*)rxb, 1U);
          HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
          HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 1);
          HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
        }
        else
        {
          interface_resp(CMD_SET_TX_START, dev_err);
        }
        break;

      case CMD_SET_RX:
        if(rxb[2])
        {
          if(trx_state != TRX_RX && dev_err == ERR_OK)
          {
            config_rf(MODE_RX, trx_data);
            osDelay(10);
            trx_writecmd(STR_SRX);
            trx_state = TRX_RX;
            uint8_t header[2] = {0x2F | 0xC0, 0x7D};
            set_CS(0);
            HAL_SPI_Transmit(&hspi1, header, 2, 10);
            FIX_TIMER_TRIGGER(&htim11);
            TIM11->CNT = 0;
            HAL_TIM_Base_Start_IT(&htim11);
            HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);
            HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
          }
          else
          {
            interface_resp(CMD_SET_RX, dev_err);
          }
        }
        else
        {
          HAL_TIM_Base_Stop_IT(&htim11);
          set_CS(1);
          trx_state = TRX_IDLE;
          HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
          interface_resp(CMD_SET_RX, ERR_OK);
        }
        break;

      case CMD_GET_IDENT:
        sprintf((char*)&ident[2], IDENT_STR);
        ident[0] = 0x80;
        ident[1] = strlen((char*)IDENT_STR) + 2;
        interface_send_blocking(ident, ident[1]);
        break;

      case CMD_GET_CAPS:
        interface_resp(CMD_GET_CAPS, 0x02);
        break;

      case CMD_GET_RX_FREQ:
        resp[0] = CMD_GET_RX_FREQ;
        resp[1] = sizeof(uint32_t) + 2;
        memcpy(&resp[2], (uint8_t*)&trx_data.rx_frequency, sizeof(uint32_t));
        interface_send_blocking(resp, resp[1]);
        break;

      case CMD_GET_TX_FREQ:
        resp[0] = CMD_GET_TX_FREQ;
        resp[1] = sizeof(uint32_t) + 2;
        memcpy(&resp[2], (uint8_t*)&trx_data.tx_frequency, sizeof(uint32_t));
        interface_send_blocking(resp, resp[1]);
        break;

      default:
        break;
      }
    }
    else if(interface_comm == COMM_TOT || interface_comm == COMM_OVF)
    {
      HAL_TIM_Base_Stop_IT(&htim10);
      interface_abort_receive_it();
      rx_bc = 0;
      interface_receive_it((uint8_t*)rxb, 1U);
      interface_comm = COMM_IDLE;
    }
    else
    {
      osDelay(10);
    }
    osDelay(10);
  }
  /* USER CODE END StartInterfaceTask */
}

/* USER CODE BEGIN Header_StartBasebandTask */
/**
* @brief Function implementing the basebandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBasebandTask */
void StartBasebandTask(void *argument)
{
  /* USER CODE BEGIN StartBasebandTask */
  (void)argument;

  for(;;)
    HAL_GPIO_TogglePin(RX_LED_GPIO_Port,RX_LED_Pin);
  {
    if(bsb_tx_pend == 1U)
    {
      HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)&tx_bsb_sample, 1);
      tx_bsb_sample = tx_bsb_buff[tx_bsb_cnt % BSB_BUFLEN];
      tx_bsb_cnt++;

      if(tx_bsb_cnt >= tx_bsb_total_cnt)
      {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        set_CS(1);
        trx_state = TRX_IDLE;
        config_rf(MODE_RX, trx_data);
        osDelay(10);
        trx_writecmd(STR_SRX);
        tx_bsb_cnt = 0;
        tx_bsb_total_cnt = 0;
        tx_bsb_sample = 0;
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
      }

      bsb_tx_pend = 0;
    }

    if(bsb_rx_pend == 1U)
    {
      uint8_t tmp = 0xFF;
      HAL_SPI_TransmitReceive(&hspi1, &tmp, (uint8_t*)&rx_bsb_sample, 1, 2);
      for (;;)
      {
        HAL_StatusTypeDef status = interface_transmit_baseband((uint8_t*)&rx_bsb_sample, 1U);
        if (status == HAL_OK)
        {
          break;
        }
        if (status == HAL_BUSY || status == HAL_ERROR)
        {
          osThreadYield();
          continue;
        }
        break;
      }
      bsb_rx_pend = 0;
    }

    if(bsb_tx_pend == 0U && bsb_rx_pend == 0U)
    {
      osThreadYield();
    }
    osDelay(20);
  }
  /* USER CODE END StartBasebandTask */
}

void StartDebugBasebandTask(void *argument)
{
#if ENABLE_DEBUG_BASEBAND_TASK
  (void)argument;
  static const int8_t debug_frame[] = {
    0x55, -0x55, 0x2A, -0x2A, 0x40, -0x40, 0x10, -0x10,
    0x7F, -0x7F, 0x30, -0x30, 0x00, 0x11, -0x11, 0x22
  };

  for(;;)
  {
    HAL_GPIO_TogglePin(SVC_LED_GPIO_Port,SVC_LED_Pin);

    for (size_t i = 0; i < sizeof(debug_frame); ++i)
    {
      uint8_t sample = (uint8_t)debug_frame[i];
      for (uint32_t attempt = 0; attempt < 100U; ++attempt)
      {
        HAL_StatusTypeDef status = interface_transmit_baseband(&sample, 1U);
        if (status == HAL_OK)
        {
          break;
        }
        if (status == HAL_BUSY)
        {
          osDelay(5);
          continue;
        }
        break;
      }
    }
    osDelay(1000);
  }
#else
  (void)argument;
  osThreadExit();
#endif
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
