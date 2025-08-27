/********************************************************************************************************
 * @file    pwm_test.c
 *
 * @brief   This is the source file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2025
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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "app.h"
#include "app_config.h"
#include "led_app_dma.h"


#define MAC_ADDR_FLASH_SECTOR  0x1FF000  // 自定义用于存储 MAC 地址的地址
static uint8_t s_color_idx = 0;
static uint8_t s_send_count = 0;

// 发送 reset 码所用的 0 填充缓冲（预留到 10MHz, 500us 也够）
// 10 MHz * 500us / 8 bit/Byte = 625 Bytes，留 800 更安全
static uint8_t g_reset_zero_pad[800] __attribute__((aligned(4)));
unsigned char mac_addr[6] = { DEVICE_INDEX, 0x22, 0x33, 0x44, 0x55, 0x66 };  // MAC地址示例
static uint32_t t_color = 0;
void write_mac_to_flash(void) {
    // 1. 擦除所在扇区，确保可写
    flash_erase_sector(MAC_ADDR_FLASH_SECTOR);

    // 2. 写入 MAC 地址（只写6字节，也可以写一整页）
    flash_write_page(MAC_ADDR_FLASH_SECTOR, sizeof(mac_addr), mac_addr);
}
extern bool conn_stat;
void pwm_test(u8 key_code){
    send_reset();
     // 2) 选择本次颜色（每 5 次才切换一次）
    uint16_t r = 0, g = 0, b = 0;
    switch (key_code) {
        case 0: r = 0xFFFF; g = 0x0000; b = 0x0000; break;   // 红
        case 1: r = 0x0000; g = 0xFFFF; b = 0x0000; break;   // 绿
        case 2:r = 0x0000; g = 0x0000; b = 0xFFFF; break;   // 蓝
    }

    // 增益位按你原先示例
    uint16_t rg = 0x00, gg = 0x00, bg = 0x00;
    uint16_t w1g = 0x00, w2g = 0x00;
    uint16_t w1 = 0x0000, w2 = 0x0000;   // 白关

    // 发送RGBW数据，包括增益调节位
    send_rgbw_data(rg, gg, bg, w1g,w2g, r, g, b, w1, w2);
    uint16_t n = spi_used_bytes();
    uint16_t n_aligned = (n + 3) & ~3;
    for(int i=0;i<5;i++){
        end_irq_flag = 0;
        spi_master_write_dma_plus(GSPI_MODULE, 0, (unsigned int)NULL, (unsigned char *)&spi_tx_buff.data, n_aligned, SPI_MODE_WR_WRITE_ONLY);
        while (!end_irq_flag)
            ; //Wait for spi transmission end interrupt.
        end_irq_flag = 0;
    }
    
     // 4) 计数 + 条件 reset + 颜色轮换
    s_send_count++;
    if (s_send_count >= 3) {
        // reset 码：MOSI 低电平 ≥ 500us
        spi_send_reset_us(700);

        s_send_count = 0;
    }
}
