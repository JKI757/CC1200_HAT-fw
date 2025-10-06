#include "interface.h"

#include <string.h>

#include "cc1200.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#include "cmsis_os.h"

volatile uint8_t rxb[100] = {0};
volatile uint8_t rx_bc = 0;
volatile int8_t tx_bsb_buff[BSB_BUFLEN] = {0};
volatile uint32_t tx_bsb_total_cnt = 0;
uint32_t tx_bsb_cnt = 0;
int8_t tx_bsb_sample = 0;
int8_t rx_bsb_sample = 0;
volatile uint8_t bsb_tx_pend = 0;
volatile uint8_t bsb_rx_pend = 0;
volatile enum interface_comm_t interface_comm = COMM_IDLE;

static interface_transport_t active_transport = INTERFACE_TRANSPORT_NONE;
static uint8_t transport_locked = 0U;
static uint8_t usb_pending_buffer[200];
static uint8_t usb_tx_buffer[CDC_PORT_COUNT][APP_TX_DATA_SIZE];
static uint32_t usb_pending_len = 0U;
static uint8_t usb_dtr_mask = 0U;

static HAL_StatusTypeDef uart_tx_it(uint8_t *buffer, uint16_t size);
static HAL_StatusTypeDef uart_rx_it(uint8_t *buffer, uint16_t size);
static HAL_StatusTypeDef uart_abort_rx(void);
static HAL_StatusTypeDef usb_tx_port(uint8_t port, uint8_t *buffer, uint16_t size);
static uint8_t interface_usb_port_ready(uint8_t port);

static void interface_reset_state(void);
static void interface_start_timeout(void);
static void interface_activate_transport(interface_transport_t transport);
static void interface_process_command_byte(uint8_t byte);
static void interface_process_baseband_byte(uint8_t byte);
static void interface_queue_usb_pending(const uint8_t *buf, uint32_t len);
static void interface_consume_usb_pending(void);

void interface_init(interface_transport_t transport)
{
  interface_reset_state();
  active_transport = INTERFACE_TRANSPORT_NONE;
  transport_locked = 0U;

  if (transport != INTERFACE_TRANSPORT_NONE)
  {
    interface_activate_transport(transport);
    transport_locked = 1U;
  }

  /* Always arm UART so we can auto-detect activity */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rxb, 1U);
}

HAL_StatusTypeDef interface_receive_it(uint8_t *buffer, uint16_t size)
{
  if (active_transport == INTERFACE_TRANSPORT_USB_CDC)
  {
    (void)buffer;
    (void)size;
    return HAL_OK;
  }

  return uart_rx_it(buffer, size);
}

HAL_StatusTypeDef interface_transmit_command(uint8_t *buffer, uint16_t size)
{
  if (active_transport == INTERFACE_TRANSPORT_USB_CDC)
  {
    return usb_tx_port(0U, buffer, size);
  }

  return uart_tx_it(buffer, size);
}

HAL_StatusTypeDef interface_transmit_baseband(uint8_t *buffer, uint16_t size)
{
  if (active_transport == INTERFACE_TRANSPORT_USB_CDC && interface_usb_port_ready(1U))
  {
    return usb_tx_port(1U, buffer, size);
  }
  return HAL_BUSY;
}

HAL_StatusTypeDef interface_abort_receive_it(void)
{
  if (active_transport == INTERFACE_TRANSPORT_USB_CDC)
  {
    return HAL_OK;
  }

  return uart_abort_rx();
}

void interface_handle_period_elapsed(TIM_HandleTypeDef *htim)
{
  TIM_TypeDef *tim = htim->Instance;

  if (tim == TIM10)
  {
    HAL_TIM_Base_Stop_IT(&htim10);

    if (interface_comm == COMM_IDLE)
    {
      interface_comm = COMM_TOT;
    }
  }
  else if (tim == TIM11)
  {
    bsb_rx_pend = 1U;
  }
}

void interface_handle_gpio_exti(uint16_t pin)
{
  if (pin == TRX_TRIG_Pin)
  {
    bsb_tx_pend = 1U;
  }
}


void interface_usb_receive_callback(uint8_t port, uint8_t *buf, uint32_t len)
{
  interface_activate_transport(INTERFACE_TRANSPORT_USB_CDC);

  if (len == 0U)
  {
    return;
  }

  if (port == 0U)
  {
    for (uint32_t i = 0; i < len; ++i)
    {
      interface_process_command_byte(buf[i]);

      if (trx_state != TRX_TX && interface_comm != COMM_IDLE)
      {
        if ((i + 1U) < len)
        {
          interface_queue_usb_pending(&buf[i + 1U], len - (i + 1U));
        }
        break;
      }
    }
  }
  else
  {
    for (uint32_t i = 0; i < len; ++i)
    {
      interface_process_baseband_byte(buf[i]);
    }
  }
}


void interface_usb_line_state_changed(uint8_t port, uint16_t state)
{
  if (port < CDC_PORT_COUNT)
  {
    if ((state & 0x0001U) != 0U)
    {
      usb_dtr_mask |= (1U << port);
      interface_activate_transport(INTERFACE_TRANSPORT_USB_CDC);
    }
    else
    {
      usb_dtr_mask &= (uint8_t)~(1U << port);
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef uart_tx_it(uint8_t *buffer, uint16_t size)
{
  return HAL_UART_Transmit_IT(&huart1, buffer, size);
}

static HAL_StatusTypeDef uart_rx_it(uint8_t *buffer, uint16_t size)
{
  return HAL_UART_Receive_IT(&huart1, buffer, size);
}

static HAL_StatusTypeDef uart_abort_rx(void)
{
  return HAL_UART_AbortReceive_IT(&huart1);
}

static HAL_StatusTypeDef usb_tx_port(uint8_t port, uint8_t *buffer, uint16_t size)
{
  if (port >= CDC_PORT_COUNT || size > sizeof(usb_tx_buffer[0]))
  {
    return HAL_ERROR;
  }

  memcpy(usb_tx_buffer[port], buffer, size);

  if (USBD_CDC_SetTxBufferPort(&hUsbDeviceFS, port, usb_tx_buffer[port], size) != USBD_OK)
  {
    return HAL_ERROR;
  }

  uint8_t status = USBD_CDC_TransmitPacketPort(&hUsbDeviceFS, port);
  if (status == USBD_OK)
  {
    return HAL_OK;
  }
  if (status == USBD_BUSY)
  {
    return HAL_BUSY;
  }
  return HAL_ERROR;
}

static uint8_t interface_usb_port_ready(uint8_t port)
{
  if (port >= CDC_PORT_COUNT)
  {
    return 0U;
  }
  return (usb_dtr_mask & (1U << port)) != 0U;
}

static void interface_reset_state(void)
{
  memset((uint8_t *)rxb, 0, sizeof(rxb));
  rx_bc = 0U;
  tx_bsb_total_cnt = 0U;
  tx_bsb_cnt = 0U;
  tx_bsb_sample = 0;
  rx_bsb_sample = 0;
  bsb_tx_pend = 0U;
  bsb_rx_pend = 0U;
  interface_comm = COMM_IDLE;
  usb_pending_len = 0U;
}

static void interface_start_timeout(void)
{
  TIM10->CNT = 0U;
  FIX_TIMER_TRIGGER(&htim10);
  HAL_TIM_Base_Start_IT(&htim10);
}

static void interface_activate_transport(interface_transport_t transport)
{
  if (transport == active_transport)
  {
    return;
  }

  if (transport_locked && transport != active_transport)
  {
    return;
  }

  switch (transport)
  {
  case INTERFACE_TRANSPORT_UART:
    active_transport = INTERFACE_TRANSPORT_UART;
    transport_locked = 1U;
    break;

  case INTERFACE_TRANSPORT_USB_CDC:
    if (active_transport == INTERFACE_TRANSPORT_UART)
    {
      uart_abort_rx();
    }
    active_transport = INTERFACE_TRANSPORT_USB_CDC;
    transport_locked = 1U;
    break;

  case INTERFACE_TRANSPORT_NONE:
  default:
    break;
  }
}

static void interface_process_command_byte(uint8_t byte)
{
  if (trx_state != TRX_TX)
  {
    rxb[rx_bc] = byte;

    if (rxb[1] == (uint8_t)(rx_bc + 1U))
    {
      HAL_TIM_Base_Stop_IT(&htim10);
      rx_bc = 0U;
      interface_comm = COMM_RDY;
      return;
    }

    if ((rx_bc + 1U) < sizeof(rxb))
    {
      rx_bc++;
      interface_start_timeout();
    }
    else
    {
      interface_comm = COMM_OVF;
    }
  }
}

static void interface_process_baseband_byte(uint8_t byte)
{
  if (trx_state != TRX_TX)
  {
    return;
  }

  tx_bsb_buff[tx_bsb_total_cnt % BSB_BUFLEN] = (int8_t)byte;
  tx_bsb_total_cnt++;
}

static void interface_queue_usb_pending(const uint8_t *buf, uint32_t len)
{
  if (len == 0U)
  {
    return;
  }

  uint32_t capacity = sizeof(usb_pending_buffer) - usb_pending_len;
  if (capacity == 0U)
  {
    return;
  }

  if (len > capacity)
  {
    len = capacity;
  }

  memcpy(&usb_pending_buffer[usb_pending_len], buf, len);
  usb_pending_len += len;
}

static void interface_consume_usb_pending(void)
{
  uint32_t idx = 0U;

  while (idx < usb_pending_len)
  {
    interface_process_command_byte(usb_pending_buffer[idx]);
    idx++;

    if (trx_state != TRX_TX && interface_comm != COMM_IDLE)
    {
      if (idx < usb_pending_len)
      {
        uint32_t remaining = usb_pending_len - idx;
        memmove(usb_pending_buffer, &usb_pending_buffer[idx], remaining);
        usb_pending_len = remaining;
      }
      else
      {
        usb_pending_len = 0U;
      }
      return;
    }
  }

  usb_pending_len = 0U;
}

void interface_poll_pending(void)
{
  if (active_transport == INTERFACE_TRANSPORT_USB_CDC && usb_pending_len > 0U && interface_comm == COMM_IDLE)
  {
    interface_consume_usb_pending();
  }
}

/* -------------------------------------------------------------------------- */
/* HAL callbacks                                                              */
/* -------------------------------------------------------------------------- */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  interface_activate_transport(INTERFACE_TRANSPORT_UART);

  if (active_transport != INTERFACE_TRANSPORT_UART)
  {
    return;
  }

  if (trx_state != TRX_TX)
  {
    interface_process_command_byte(rxb[rx_bc]);
    if (interface_comm == COMM_IDLE)
    {
      uart_rx_it((uint8_t *)&rxb[rx_bc], 1U);
    }
  }
  else
  {
    interface_process_baseband_byte(rxb[0]);
    uart_rx_it((uint8_t *)rxb, 1U);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  interface_handle_gpio_exti(GPIO_Pin);
}
