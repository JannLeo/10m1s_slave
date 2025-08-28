/********************************************************************************************************
 * @file    app_dma.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
 *
 * @author  Driver Group
 * @date    2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "app_config.h"
#include "led_app_dma.h"
    /**********************************************************************************************************************
 *                                         SPI device  selection                                                     *
 *********************************************************************************************************************/
    #define SPI_MASTER_DEVICE 1
    #define SPI_SLAVE_DEVICE  2

    #define SPI_DEVICE        SPI_MASTER_DEVICE

    /**********************************************************************************************************************
 *                                         SPI protocol demo selection                                                *
 *********************************************************************************************************************/
    #define B85M_SLAVE_PROTOCOL     1 // B85m as slave
    #define B91M_SLAVE_PROTOCOL     2 // Telink RISC-V MCU hspi/pspi/gspi/lspi slave mode as slave
    #define B91M_SPI_SLAVE_PROTOCOL 3 // Telink RISC-V MCU spi slave as slave

    #define SPI_PROTOCOL            B91M_SLAVE_PROTOCOL

    /**********************************************************************************************************************
 *                                         SPI data mode selection                                                *
 *********************************************************************************************************************/
    #define ONLY_DATA_MODE 1 // Only data is available for spi communication
    #define NORMAL_MODE    2 // Spi communication format: cmd+dummy+data


    #define DATA_MODE      ONLY_DATA_MODE

    /**********************************************************************************************************************
 *                                         SPI module selection                                                       *
 *********************************************************************************************************************/
    /* Note:TL321X only supports gspi!!!*/
    #define LSPI_MODULE 0
    #define GSPI_MODULE 1

    #define SPI_MODULE_SEL GSPI_MODULE
    /**********************************************************************************************************************
 *                                         SPI multiple slave
 *********************************************************************************************************************/
    #define ONE_SLAVE     0
    #define MULTI_SLAVE   1

    #define SPI_SLAVE_NUM ONE_SLAVE
    /**********************************************************************************************************************
 *                                         3line SPI  enable                                                      *
 *********************************************************************************************************************/
    #define SPI_NORMAL_SLAVE 0
    #define SPI_3LINE_SLAVE  1

    #define SPI_SLAVE_MODE   SPI_NORMAL_SLAVE
    /**********************************************************************************************************************
 *                                          SPI slave ready test
 *When the slave_ready bit of the slave is 1, the master sends read status cmd, and the slave will reply 0x5a.
 *When the slave_ready bit of the slave is 0, the master sends read status cmd, and the slave will reply 0x00.
 *********************************************************************************************************************/
    #define SPI_SLAVE_READY_TEST 0
/**********************************************************************************************************************
*                                         TL7518(256 bit dummy)SPI clock set
* * When sclk and hclk are set to 24MHz, the SPI_CLK[use DMA] range are as follows
* B85M_SLAVE_PROTOCOL           LSPI_MODULE             max:depend on spi slave  min:48000
*                               GSPI_MODULE             max:depend on spi slave  min:48000
* B91M_SLAVE_PROTOCOL           LSPI_MODULE  single io  max:6000000   min:48000
*                                            dual io    max:6000000   min:48000
*                                            quad io    max:6000000   min:48000
*                                            octal io   max:6000000   min:48000
*                               GSPI_MODULE  single io  max:6000000   min:48000
*                                            3_line io  max:6000000   min:48000
*                                            dual io    max:6000000   min:48000
*                                            quad io    max:6000000   min:48000
*                                            octal io   max:6000000   min:48000
* B91M_SPI_SLAVE_PROTOCOL       LSPI_MODULE single io   max:12000000  min:48000
*                                            dual io    max:12000000  min:48000
*                               GSPI_MODULE single io   max:10000000   min:48000
*                                            dual io    max:10000000   min:48000                                                      *
*********************************************************************************************************************/
#define SPI_CLK 8000000

// 码元定义（3位）
#define CODE_0  0xc0  
#define CODE_1  0xf8  
// 每个码元的长度（以比特为单位）
#define CODE_LENGTH 8  
// 颜色循环：0=R,1=G,2=B
static uint8_t s_color_idx = 0;
static uint8_t s_send_count = 0;

// 发送 reset 码所用的 0 填充缓冲（预留到 10MHz, 500us 也够）
// 10 MHz * 500us / 8 bit/Byte = 625 Bytes，留 800 更安全
static uint8_t g_reset_zero_pad[800] __attribute__((aligned(4)));


spi_tx_buff_t spi_tx_buff;

static uint32_t t_color = 0;
    

/**********************************************************************************************************************
 *                                        SPI  pin  define                                                        *
 *********************************************************************************************************************/

gspi_pin_config_t gspi_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};

    /**********************************************************************************************************************
 *                                          dma  channel                                                      *
 *********************************************************************************************************************/
    #define SPI_TX_DMA_CHN DMA1
    #define SPI_RX_DMA_CHN DMA0
    /**********************************************************************************************************************
*                                         SPI master setting                                                        *
*********************************************************************************************************************/
/**********************************************************************************************************************
 *                                          global variable                                                   *
 *********************************************************************************************************************/
volatile unsigned char end_irq_flag = 0;
volatile unsigned char rx_dma_flag  = 0;
        #define DATA_BYTE_LEN 16
unsigned char spi_rx_buff[DATA_BYTE_LEN] __attribute__((aligned(4))) = {0x00};
spi_wr_rd_config_t spi_b91m_slave_protocol_config = {
    .spi_io_mode   = SPI_SINGLE_MODE, /*IO mode set to SPI_3_LINE_MODE when SPI_3LINE_SLAVE.*/
    .spi_dummy_cnt = 0,               //B92 supports up to 32 clk cycle dummy, and TL751X,TL7518,TL721X,TL321X,TL322X supports up to 256 clk cycle dummy.
    .spi_cmd_en      = 0,
    .spi_addr_en     = 0,
    .spi_addr_len    = 0, //when spi_addr_en = 0,invalid set.
    .spi_cmd_fmt_en  = 0, //when spi_cmd_en = 0,invalid set.
    .spi_addr_fmt_en = 0, //when spi_addr_en = 0,invalid set.
};

void user_init_led_app(void)
{
    spi_master_init(SPI_MODULE_SEL, sys_clk.pll_clk * 1000000 / SPI_CLK, SPI_MODE0);
    spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT);  //clr
    // 启用SPI结束中断
    spi_set_irq_mask(SPI_MODULE_SEL, SPI_END_INT_EN); //endint_en
    // 将 SPI 发送配置为通过 DMA 传输数据
    spi_set_tx_dma_config(SPI_MODULE_SEL, SPI_TX_DMA_CHN);
    // 将 SPI 发送配置为通过 DMA 传输数据
    spi_set_master_rx_dma_config(SPI_MODULE_SEL, SPI_RX_DMA_CHN);

    	// 检查当前选择的 SPI 模块是否为 GSPI 模块。
    // 启用 GSPI 中断
    gspi_set_pin(&gspi_pin_config);
    plic_interrupt_enable(IRQ_GSPI);
    spi_master_config_plus(SPI_MODULE_SEL, &spi_b91m_slave_protocol_config);

    dma_set_irq_mask(SPI_RX_DMA_CHN, TC_MASK);
    plic_interrupt_enable(IRQ_DMA);
    core_interrupt_enable();
}
void spi_send_reset_us(uint16_t us_low)
{
    // 需要的 Byte 数：ceil(us * SPI_CLK / 8 / 1e6)
    uint32_t bytes = (uint32_t)((((uint64_t)us_low) * SPI_CLK + (8*1000000 - 1)) / (8*1000000));
    if (bytes < 1) bytes = 1;
    if (bytes > sizeof(g_reset_zero_pad)) bytes = sizeof(g_reset_zero_pad);

    // 4 字节对齐
    uint32_t len_aligned = (bytes + 3) & ~3;

    end_irq_flag = 0;
    spi_master_write_dma_plus(SPI_MODULE_SEL, 0, (unsigned int)NULL,
                              (unsigned char *)g_reset_zero_pad, len_aligned, SPI_MODE_WR_WRITE_ONLY);
    while(!end_irq_flag);
    end_irq_flag = 0;
}

void send_bit(uint32_t bit_value) {
    // 获取当前需要写入的字节和比特位置
    uint16_t byte_index = spi_tx_buff.bit_offset / 8; // 当前写入字节的索引

    // 将值写入到相应字节的适当位置
    uint8_t value_to_set = bit_value;  // 当前要写入的值
    spi_tx_buff.data[byte_index] = value_to_set;
    spi_tx_buff.bit_offset += 8;
}

void send_reset(void) {
    // 清空 SPI 发送数据缓存
    memset(spi_tx_buff.data, 0x00, sizeof(spi_tx_buff.data));
    // 重置位偏移量
    spi_tx_buff.bit_offset = 0; // 清零偏移量
}
// 增益调节位 + 16位数据转换为码元
void convert_rgbw_to_bits(uint16_t gain_data, uint16_t color_data, uint8_t *output_data) {
    for(int i=0;i<2;i++){
        if((gain_data >> (1 - i)) & 0x1){
            output_data[i] = CODE_1;  // 1对应CODE_1
        }else{
            output_data[i] = CODE_0;
        }
    }
    for (int i = 0; i < 16; i++) {
        // 判断是1还是0，根据高低电平
        if ((color_data >> (15 - i)) & 0x1) {
            output_data[i+2] = CODE_1;  // 1对应CODE_1
        } else {
            output_data[i+2] = CODE_0;  // 0对应CODE_0
        }
    }
}

// 发送RGBW数据并包括增益调节位
void send_rgbw_data(uint16_t rg, uint16_t gg, uint16_t bg, uint16_t w1g,uint16_t w2g, uint16_t r, uint16_t g, uint16_t b, uint16_t w1, uint16_t w2) {
    uint8_t r_bits[18], g_bits[18], b_bits[18], w1_bits[18], w2_bits[18];

    // 将增益调节位与RGBW数据合并后转换成码元
    convert_rgbw_to_bits(rg, r, r_bits);  // RG + R
    convert_rgbw_to_bits(gg, g, g_bits);  // GG + G
    convert_rgbw_to_bits(bg, b, b_bits);  // BG + B
    convert_rgbw_to_bits(w1g, w1, w1_bits);  // W1G + W1
    convert_rgbw_to_bits(w2g, w2, w2_bits);  // W2G + W2

    // 逐个发送每个颜色通道的码元数据
    for (int i = 0; i < 18; i++) {
        send_bit(g_bits[i]); // 发送G通道的码元
    }
    for (int i = 0; i < 18; i++) {
        send_bit(r_bits[i]); // 发送R通道的码元
    }
    for (int i = 0; i < 18; i++) {
        send_bit(b_bits[i]); // 发送B通道的码元
    }
    for (int i = 0; i < 18; i++) {
        send_bit(w1_bits[i]); // 发送W1通道的码元
    }
    for (int i = 0; i < 18; i++) {
        send_bit(w2_bits[i]); // 发送W2通道的码元
    }

}




_attribute_ram_code_sec_noinline_ void gspi_irq_handler(void)
{
    if (spi_get_irq_status(SPI_MODULE_SEL, SPI_END_INT)) {
        spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT); //clr
        end_irq_flag = 1;
    }
}


PLIC_ISR_REGISTER(gspi_irq_handler, IRQ_GSPI)

_attribute_ram_code_sec_noinline_ void dma_irq_handler(void)
{
    if (dma_get_tc_irq_status(BIT(SPI_RX_DMA_CHN))) {
    	// 清除指定DMA通道的传输完成中断状态
        dma_clr_tc_irq_status(BIT(SPI_RX_DMA_CHN));
        // 设置DMA接收完成的标志
        rx_dma_flag = 1;
    }
}
PLIC_ISR_REGISTER(dma_irq_handler, IRQ_DMA)