#ifndef __CC1200_H
#define __CC1200_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "enums.h"
#include "stm32f4xx_hal.h"

#define IDENT_STR          "CC1200-HAT 420-450 MHz\nFW v1.1 by Wojciech SP5WWP"
#define CC1200_REG_NUM     51U                 //number of regs used to initialize CC1200s
struct trx_data_t
{
    uint8_t name[20];        //chip's name (CC1200, CC1201, unknown ID)
    uint32_t rx_frequency;   //frequency in hertz
    uint32_t tx_frequency;   //frequency in hertz
    uint8_t pwr;             //power setting (3..63)
    int16_t fcorr;           //frequency correction
    uint8_t pll_locked;      //PLL locked flag
};

extern uint32_t dev_err;
extern struct trx_data_t trx_data;
extern float tx_dbm;
extern enum trx_state_t trx_state;

void cc1200_init(void);
void set_nRST(uint8_t state);
void set_CS(uint8_t state);
uint8_t trx_readreg(uint16_t addr);
void trx_writereg(uint16_t addr, uint8_t val);
void trx_writecmd(uint8_t addr);
uint8_t read_pn(void);
uint8_t read_status(void);
void detect_ic(uint8_t *out);
void config_ic(uint8_t *settings);
void config_rf(enum mode_t mode, struct trx_data_t trx_data);
#ifdef __cplusplus
}
#endif

#endif /* __CC1200_H */
