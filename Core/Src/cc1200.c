#include "cc1200.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"
#include "tim.h"
#include "gpio.h"

uint32_t dev_err = (uint32_t)ERR_OK; //default state - no error
struct trx_data_t trx_data;
float tx_dbm = 0.0f;                  //RF power setpoint, dBm
enum trx_state_t trx_state = TRX_IDLE;

static void cc1200_set_defaults(void);

void set_nRST(uint8_t state)
{
    if(state)
    {
        TRX_nRST_GPIO_Port->BSRR = (uint32_t)TRX_nRST_Pin;
    }
    else
    {
        TRX_nRST_GPIO_Port->BSRR = (uint32_t)TRX_nRST_Pin << 16U;
    }
}

void set_CS(uint8_t state)
{
    if(state)
    {
        TRX_nCS_GPIO_Port->BSRR = (uint32_t)TRX_nCS_Pin;
    }
    else
    {
        TRX_nCS_GPIO_Port->BSRR = (uint32_t)TRX_nCS_Pin << 16U;
    }
}

uint8_t trx_readreg(uint16_t addr)
{
    uint8_t txd[3] = {0, 0, 0};
    uint8_t rxd[3] = {0, 0, 0};

    set_CS(0);
    if((addr >> 8U) == 0U)
    {
        txd[0] = (addr & 0xFFU) | 0x80U;
        txd[1] = 0;
        HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 2, 10);
        set_CS(1);
        return rxd[1];
    }

    txd[0] = ((addr >> 8U) & 0xFFU) | 0x80U;
    txd[1] = addr & 0xFFU;
    txd[2] = 0;
    HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 3, 10);
    set_CS(1);
    return rxd[2];
}

void trx_writereg(uint16_t addr, uint8_t val)
{
    uint8_t txd[3] = { (uint8_t)(addr >> 8U), (uint8_t)(addr & 0xFFU), val };

    set_CS(0);
    if((addr >> 8U) == 0U)
    {
        txd[0] = addr & 0xFFU;
        txd[1] = val;
        HAL_SPI_Transmit(&hspi1, txd, 2, 10);
    }
    else
    {
        txd[0] = (addr >> 8U) & 0xFFU;
        txd[1] = addr & 0xFFU;
        txd[2] = val;
        HAL_SPI_Transmit(&hspi1, txd, 3, 10);
    }
    set_CS(1);
}

void trx_writecmd(uint8_t addr)
{
    uint8_t txd = addr;

    set_CS(0);
    HAL_SPI_Transmit(&hspi1, &txd, 1, 10);
    set_CS(1);
}

uint8_t read_pn(void)
{
    return trx_readreg(0x2F8FU);
}

uint8_t read_status(void)
{
    uint8_t txd = STR_SNOP; //no operation strobe
    uint8_t rxd = 0;

    set_CS(0);
    HAL_SPI_TransmitReceive(&hspi1, &txd, &rxd, 1, 10);
    set_CS(1);

    return rxd;
}

void detect_ic(uint8_t *out)
{
    uint8_t trxid = read_pn();

    if(trxid == 0x20U)
    {
        sprintf((char*)out, "CC1200");
    }
    else if(trxid == 0x21U)
    {
        sprintf((char*)out, "CC1201");
    }
    else
    {
        sprintf((char*)out, "unknown ID (0x%02X)", trxid);
    }
}

void config_ic(uint8_t *settings)
{
    for(uint8_t i = 0; i < CC1200_REG_NUM; i++)
    {
        set_CS(0);
        if(settings[i * 3U] != 0U)
        {
            HAL_SPI_Transmit(&hspi1, &settings[i * 3U], 3, 10);
        }
        else
        {
            HAL_SPI_Transmit(&hspi1, &settings[i * 3U + 1U], 2, 10);
        }
        set_CS(1);
    }
}

void config_rf(enum mode_t mode, struct trx_data_t trx)
{
    static uint8_t cc1200_rx_settings[CC1200_REG_NUM * 3U] =
    {
        0x00, 0x01, 0x08,
        0x00, 0x03, 0x09,
        0x00, 0x08, 0x1F,
        0x00, 0x0A, 0xAD,
        0x00, 0x0B, 0x00,
        0x00, 0x0C, 0x5D,
        0x00, 0x0D, 0x00,
        0x00, 0x0E, 0x8A,
        0x00, 0x0F, 0xCB,
        0x00, 0x10, 0xAC,
        0x00, 0x11, 0x00,
        0x00, 0x12, 0x45,
        0x00, 0x13, 0x43,
        0x00, 0x14, 0xA9,
        0x00, 0x15, 0x2A,
        0x00, 0x16, 0x37,
        0x00, 0x17, 0xEC,
        0x00, 0x19, 0x11,
        0x00, 0x1B, 0x51,
        0x00, 0x1C, 0x87,
        0x00, 0x1D, 0x00,
        0x00, 0x20, 0x14,
        0x00, 0x26, 0x03,
        0x00, 0x27, 0x00,
        0x00, 0x28, 0x20,
        0x00, 0x2B, 0x03,
        0x00, 0x2E, 0xFF,
        0x2F, 0x00, 0x1C,
        0x2F, 0x01, 0x02,
        0x2F, 0x04, 0x0C,
        0x2F, 0x05, 0x09,
        0x2F, 0x0C, 0x57,
        0x2F, 0x0D, 0x00,
        0x2F, 0x0E, 0x00,
        0x2F, 0x10, 0xEE,
        0x2F, 0x11, 0x10,
        0x2F, 0x12, 0x07,
        0x2F, 0x13, 0xAF,
        0x2F, 0x16, 0x40,
        0x2F, 0x17, 0x0E,
        0x2F, 0x19, 0x03,
        0x2F, 0x1B, 0x33,
        0x2F, 0x1D, 0x17,
        0x2F, 0x1F, 0x00,
        0x2F, 0x20, 0x6E,
        0x2F, 0x21, 0x1C,
        0x2F, 0x22, 0xAC,
        0x2F, 0x27, 0xB5,
        0x2F, 0x32, 0x0E,
        0x2F, 0x36, 0x03,
        0x2F, 0x91, 0x08
    };

    static uint8_t cc1200_tx_settings[CC1200_REG_NUM * 3U] =
    {
        0x00, 0x01, 0x08,
        0x00, 0x03, 0x09,
        0x00, 0x08, 0x1F,
        0x00, 0x0A, 0xAD,
        0x00, 0x0B, 0x00,
        0x00, 0x0C, 0x5D,
        0x00, 0x0D, 0x00,
        0x00, 0x0E, 0x8A,
        0x00, 0x0F, 0xCB,
        0x00, 0x10, 0xAC,
        0x00, 0x11, 0x00,
        0x00, 0x12, 0x45,
        0x00, 0x13, 0x43,
        0x00, 0x14, 0xA9,
        0x00, 0x15, 0x2A,
        0x00, 0x16, 0x37,
        0x00, 0x17, 0xEC,
        0x00, 0x19, 0x11,
        0x00, 0x1B, 0x51,
        0x00, 0x1C, 0x87,
        0x00, 0x1D, 0x00,
        0x00, 0x20, 0x14,
        0x00, 0x26, 0x03,
        0x00, 0x27, 0x00,
        0x00, 0x28, 0x20,
        0x00, 0x2B, 0x03,
        0x00, 0x2E, 0xFF,
        0x2F, 0x00, 0x1C,
        0x2F, 0x01, 0x22,
        0x2F, 0x04, 0x0C,
        0x2F, 0x05, 0x09,
        0x2F, 0x0C, 0x57,
        0x2F, 0x0D, 0x00,
        0x2F, 0x0E, 0x00,
        0x2F, 0x10, 0xEE,
        0x2F, 0x11, 0x10,
        0x2F, 0x12, 0x07,
        0x2F, 0x13, 0xAF,
        0x2F, 0x16, 0x40,
        0x2F, 0x17, 0x0E,
        0x2F, 0x19, 0x03,
        0x2F, 0x1B, 0x33,
        0x2F, 0x1D, 0x17,
        0x2F, 0x1F, 0x00,
        0x2F, 0x20, 0x6E,
        0x2F, 0x21, 0x1C,
        0x2F, 0x22, 0xAC,
        0x2F, 0x27, 0xB5,
        0x2F, 0x32, 0x0E,
        0x2F, 0x36, 0x03,
        0x2F, 0x91, 0x08
    };

    uint32_t freq_word = 0;
    if(mode == MODE_RX)
    {
        freq_word = (uint32_t)roundf((float)trx.rx_frequency / 5000000.0f * (1UL << 16));
        cc1200_rx_settings[32U * 3U - 1U] = (uint8_t)((freq_word >> 16U) & 0xFFU);
        cc1200_rx_settings[33U * 3U - 1U] = (uint8_t)((freq_word >> 8U) & 0xFFU);
        cc1200_rx_settings[34U * 3U - 1U] = (uint8_t)(freq_word & 0xFFU);
        config_ic(cc1200_rx_settings);
        trx_writereg(0x0001, 29);
        trx_writereg(0x0000, 17);
        trx_writereg(0x0018, (uint8_t)(256 - 97));
        trx_writereg(0x0017, (uint8_t)(256 - 70));
    }
    else if(mode == MODE_TX)
    {
        uint8_t tx_pwr = trx.pwr;
        if(tx_pwr > 0x3FU)
        {
            tx_pwr = 0x3FU;
        }
        if(tx_pwr < 0x03U)
        {
            tx_pwr = 0x03U;
        }
        freq_word = (uint32_t)roundf((float)trx.tx_frequency / 5000000.0f * (1UL << 16));
        cc1200_tx_settings[26U * 3U - 1U] = tx_pwr;
        cc1200_tx_settings[32U * 3U - 1U] = (uint8_t)((freq_word >> 16U) & 0xFFU);
        cc1200_tx_settings[33U * 3U - 1U] = (uint8_t)((freq_word >> 8U) & 0xFFU);
        cc1200_tx_settings[34U * 3U - 1U] = (uint8_t)(freq_word & 0xFFU);
        config_ic(cc1200_tx_settings);
        trx_writereg(0x0001, 30);
    }

    trx_writereg(0x2F0A, (uint8_t)(((uint16_t)trx.fcorr >> 8U) & 0xFFU));
    trx_writereg(0x2F0B, (uint8_t)((uint16_t)trx.fcorr & 0xFFU));
    trx_writereg(0x2F06, 0);
}

void cc1200_init(void)
{
    set_nRST(1);
    HAL_Delay(50);
    set_CS(1);
    HAL_Delay(100);
    trx_writecmd(STR_SRES);
    HAL_Delay(50);
    detect_ic(trx_data.name);

    cc1200_set_defaults();

    config_rf(MODE_RX, trx_data);
    HAL_Delay(10);
    trx_writecmd(STR_SRX);
    HAL_Delay(50);

    trx_data.pll_locked = ((trx_readreg(0x2F8DU) ^ 0x80U) & 0x81U) == 0x81U;

    if(!trx_data.pll_locked)
    {
        dev_err |= (1UL << ERR_TRX_PLL);
    }
    if(strstr((char*)trx_data.name, "unknown") != NULL)
    {
        dev_err |= (1UL << ERR_TRX_SPI);
    }
}

static void cc1200_set_defaults(void)
{
    trx_data.rx_frequency = 433475000U;
    trx_data.tx_frequency = 433475000U;
    trx_data.fcorr = 0;
    trx_data.pwr = 3U;
    tx_dbm = 10.00f;
}
