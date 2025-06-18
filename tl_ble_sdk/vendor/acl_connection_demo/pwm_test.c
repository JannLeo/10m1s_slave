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

unsigned char mac_addr[6] = { 0x09, 0x22, 0x33, 0x44, 0x55, 0x66 };  // MAC地址示例

void write_mac_to_flash(void) {
    // 1. 擦除所在扇区，确保可写
    flash_erase_sector(MAC_ADDR_FLASH_SECTOR);

    // 2. 写入 MAC 地址（只写6字节，也可以写一整页）
    flash_write_page(MAC_ADDR_FLASH_SECTOR, sizeof(mac_addr), mac_addr);
}

void pwm_test(u8 key_code){
    gpio_function_en(TEST_GPIO);
    gpio_output_en(TEST_GPIO);
    gpio_input_dis(TEST_GPIO);       // 禁用输入
    printf("key_code == %d\r\n",key_code);
    if (key_code == 1) {
        gpio_set_high_level(TEST_GPIO);
        printf( "[APP][PWM] PWM Enter\r\n");
        // 使能GPIO功能
        gpio_function_en(PWM_PIN);     // 设置为 PWM 功能
        gpio_output_en(PWM_PIN);       // 设置为输出
        gpio_input_dis(PWM_PIN);       // 禁用输入

        // 配置 PWM 相关设置
        pwm_set_pin(PWM_PIN,PWM0);          // 配置 PWM 引脚
        // 设置pwm0为正常模式
        pwm_set_pwm0_mode(PWM_NORMAL_MODE);
       // 配置PWM时钟，根据PCLK计算PWM时钟频率
        // 1000*1000 是转换成微赫兹
        pwm_set_clk((unsigned char)(sys_clk.pclk * 1000 * 1000 / PWM_PCLK_SPEED - 1));

        // 高电平时间值
        pwm_set_tcmp(PWM_ID, 100 * CLOCK_PWM_CLOCK_1US);

        // 信号周期值设置
        pwm_set_tmax(PWM_ID, 100 * CLOCK_PWM_CLOCK_1US);
        pwm_start(PWM_PIN);  // 启动PWM
        
    }
    else{
        pwm_stop(PWM_PIN);  // 关闭PWM
        gpio_set_low_level(TEST_GPIO);
    }
}