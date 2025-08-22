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

// PWM clock enums
enum pwm_clock
{
    // 每秒的时钟周期数
    CLOCK_PWM_CLOCK_1S  = PWM_PCLK_SPEED,
    // 每毫秒的时钟周期数
    CLOCK_PWM_CLOCK_1MS = (CLOCK_PWM_CLOCK_1S / 1000),
    // 每微秒的时钟周期数
    CLOCK_PWM_CLOCK_1US = (CLOCK_PWM_CLOCK_1S / 1000000),
};
#define MAC_ADDR_FLASH_SECTOR  0x1FF000  // 自定义用于存储 MAC 地址的地址

unsigned char mac_addr[6] = { DEVICE_INDEX, 0x22, 0x33, 0x44, 0x55, 0x66 };  // MAC地址示例

void write_mac_to_flash(void) {
    // 1. 擦除所在扇区，确保可写
    flash_erase_sector(MAC_ADDR_FLASH_SECTOR);

    // 2. 写入 MAC 地址（只写6字节，也可以写一整页）
    flash_write_page(MAC_ADDR_FLASH_SECTOR, sizeof(mac_addr), mac_addr);
}
extern bool conn_stat;
void pwm_test(u8 key_code){
    if (key_code > 4) return;        // 只接受 0~4

    u16 duty_ticks = (key_code * PWM_PERIOD_TICKS) / 4; // 0, 250, 500, 750, 1000

    pwm_set_tmax(PWM_ID, PWM_PERIOD_TICKS); // 周期固定
    pwm_set_tcmp(PWM_ID, duty_ticks);       // 占空比可变
    pwm_start(PWM_PIN);                     // 若已启动则再次设置即可

}