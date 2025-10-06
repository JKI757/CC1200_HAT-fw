#ifndef __INTERFACE_H
#define __INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "enums.h"
#include "stm32f4xx_hal.h"

#define BSB_BUFLEN         (6U*960U)           //total tx/rx buffer size in samples (6 frames = 240ms, 6*960 samples at fs=24kHz)
#define BSB_RUNUP          (2U*960U)           //runup length for transmissions (2 frames = 80ms, 2*960 samples at fs=24kHz)

#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))

typedef enum
{
  INTERFACE_TRANSPORT_NONE = 0,
  INTERFACE_TRANSPORT_UART,
  INTERFACE_TRANSPORT_USB_CDC
} interface_transport_t;

#ifndef INTERFACE_DEFAULT_TRANSPORT
#define INTERFACE_DEFAULT_TRANSPORT INTERFACE_TRANSPORT_NONE
#endif

extern volatile uint8_t rxb[100];
extern volatile uint8_t rx_bc;
extern volatile int8_t tx_bsb_buff[BSB_BUFLEN];
extern volatile uint32_t tx_bsb_total_cnt;
extern uint32_t tx_bsb_cnt;
extern int8_t tx_bsb_sample;
extern int8_t rx_bsb_sample;
extern volatile uint8_t bsb_tx_pend;
extern volatile uint8_t bsb_rx_pend;
extern volatile enum interface_comm_t interface_comm;

void interface_init(interface_transport_t transport);
HAL_StatusTypeDef interface_receive_it(uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef interface_transmit_command(uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef interface_transmit_baseband(uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef interface_abort_receive_it(void);
void interface_handle_period_elapsed(TIM_HandleTypeDef *htim);
void interface_handle_gpio_exti(uint16_t pin);
void interface_usb_receive_callback(uint8_t port, uint8_t *buf, uint32_t len);
void interface_usb_line_state_changed(uint8_t port, uint16_t state);
void interface_poll_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* __INTERFACE_H */
