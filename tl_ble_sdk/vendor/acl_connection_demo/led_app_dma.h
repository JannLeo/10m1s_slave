/********************************************************************************************************
 * @file    led_app_dma.h
 *
 * @brief   DMA driver header file for LED control module
 *          Provides RGBW LED data transmission and reset signal generation functions
 *
 * @author  BLE GROUP
 * @date    08,2025
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#ifndef LED_APP_DMA_H_
#define LED_APP_DMA_H_

#include "tl_common.h"
#include "app_config.h"

#define COLOR_PERIOD_US 100000
extern volatile unsigned char end_irq_flag;
extern volatile unsigned char rx_dma_flag;
/* ========================================
 * Hardware Configuration Definitions
 * ======================================== */

/** @brief SPI buffer structure */
typedef struct {
    uint8_t data[128] __attribute__((aligned(4)));  // SPI data buffer (128 bytes, 4-byte aligned)
    uint16_t bit_offset;  // Current data fill offset (in bits)
} spi_tx_buff_t;

/* ========================================
 * Global Variable Declarations
 * ======================================== */

/** @brief SPI transmission buffer (externally defined) */
extern spi_tx_buff_t spi_tx_buff;

/* ========================================
 * Function Declarations
 * ======================================== */

/**
 * @brief Send LED reset signal
 * @note  Used for LED initialization or resetting LED state
 */
void send_reset(void);

/**
 * @brief Send RGBW LED data
 * @param rg  Red grayscale value (0-3)
 * @param gg  Green grayscale value (0-3)
 * @param bg  Blue grayscale value (0-3)
 * @param w1g White channel 1 grayscale value (0-3)
 * @param w2g White channel 2 grayscale value (0-3)
 * @param r   Red actual value (0-255)
 * @param g   Green actual value (0-255)
 * @param b   Blue actual value (0-255)
 * @param w1  White channel 1 actual value (0-255)
 * @param w2  White channel 2 actual value (0-255)
 */
void send_rgbw_data(uint16_t rg, uint16_t gg, uint16_t bg, uint16_t w1g, 
                    uint16_t w2g, uint16_t r, uint16_t g, uint16_t b, 
                    uint16_t w1, uint16_t w2);

/**
 * @brief Get the number of bytes used in current SPI buffer
 * @return Number of bytes used (bit_offset rounded up to bytes)
 */
static inline uint16_t spi_used_bytes(void) {
    return (spi_tx_buff.bit_offset + 7) >> 3;  // bit→byte round up
}

/**
 * @brief Send reset signal with specified duration
 * @param us_low Reset signal low-level duration (microseconds)
 */
void spi_send_reset_us(uint16_t us_low);
void user_init_led_app(void);
#endif /* LED_APP_DMA_H_ */
