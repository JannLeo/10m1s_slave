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
#if (SPI_MODE == SPI_DMA_MODE)
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
    #ifndef MCU_CORE_TL321X
        #define LSPI_MODULE 0
    #endif
    #define GSPI_MODULE 1
    #if defined(MCU_CORE_TL322X)
        #define GSPI1_MODULE 2
        #define GSPI2_MODULE 3
        #define GSPI3_MODULE 4
        #define GSPI4_MODULE 5
    #endif

    #if defined(MCU_CORE_TL321X)
        #define SPI_MODULE_SEL GSPI_MODULE
    #else
        #define SPI_MODULE_SEL GSPI_MODULE
    #endif
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

    #if (SPI_MODULE_SEL == GSPI_MODULE)
        #if defined(MCU_CORE_B92) || defined(MCU_CORE_TL721X) || defined(MCU_CORE_TL321X) || defined(MCU_CORE_TL322X)
gspi_pin_config_t gspi_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
        #endif

        #if defined(MCU_CORE_TL751X)
gspi_pin_config_t gspi_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0,   //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PE2,   //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PE3,   //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io4_pin      = GPIO_FC_PE4,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io5_pin      = GPIO_FC_PE5,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io6_pin      = GPIO_FC_PE6,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io7_pin      = GPIO_FC_PE7,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_dm_pin       = GPIO_NONE_PIN, //opi psram is required,otherwise it is NONE_PIN.
};
        #endif

        #if defined(MCU_CORE_TL7518)
gspi_pin_config_t gspi_pin_config = {
    .spi_csn_pin      = GSPI_CSN0_PA0_PIN,
    .spi_clk_pin      = GSPI_CLK_PA1_PIN,
    .spi_mosi_io0_pin = GSPI_MOSI_PA2_PIN,
    .spi_miso_io1_pin = GSPI_MISO_PA3_PIN, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GSPI_IO2_PA4_PIN,  //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GSPI_IO3_PA5_PIN,  //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io4_pin      = GSPI_IO4_PA6_PIN,  //octal mode is required, otherwise it is NONE_PIN.
    .spi_io5_pin      = GSPI_IO5_PB0_PIN,  //octal mode is required, otherwise it is NONE_PIN.
    .spi_io6_pin      = GSPI_IO6_PF3_PIN,  //octal mode is required, otherwise it is NONE_PIN.
    .spi_io7_pin      = GSPI_IO7_PF4_PIN,  //octal mode is required, otherwise it is NONE_PIN.
    .spi_dm_pin       = GSPI_NONE_PIN,     //opi psram is required,otherwise it is NONE_PIN.

};
        #endif
    #endif

    #if defined(MCU_CORE_TL322X)
        #if (SPI_MODULE_SEL == GSPI1_MODULE)
gspi_pin_config_t gspi1_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
        #endif
        #if (SPI_MODULE_SEL == GSPI2_MODULE)
gspi_pin_config_t gspi2_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
        #endif
        #if (SPI_MODULE_SEL == GSPI3_MODULE)
gspi_pin_config_t gspi3_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
        #endif
        #if (SPI_MODULE_SEL == GSPI4_MODULE)
gspi_pin_config_t gspi4_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
        #endif
    #endif

    #if (SPI_SLAVE_NUM == MULTI_SLAVE)
        #if defined(MCU_CORE_B92)
            //39 GPIOs can be multiplexed as PIN for GSPI_CSN0_IO function.Only a part of IO is listed here.
            #define SLAVE_CSN_PIN_NUM 8
gpio_func_pin_e slave_csn_pin[SLAVE_CSN_PIN_NUM] = {GPIO_FC_PA0, GPIO_FC_PA4, GPIO_FC_PB2, GPIO_FC_PB3, GPIO_FC_PB4, GPIO_FC_PB5, GPIO_FC_PB6, GPIO_FC_PB7};
        #endif
        #if defined(MCU_CORE_TL7518)
            //4 GPIOs can be multiplexed as PIN for GSPI_CSN0_IO function.Only a part of IO is listed here.
            #define SLAVE_CSN_PIN_NUM 4
gspi_pin_def_e slave_csn_pin[SLAVE_CSN_PIN_NUM] = {GSPI_CSN0_PA0_PIN, GSPI_CSN0_PB7_PIN, GSPI_CSN0_PG1_PIN, GSPI_CSN0_PJ2_PIN};
        #endif
        #if defined(MCU_CORE_TL321X) || defined(MCU_CORE_TL322X) || defined(MCU_CORE_TL721X)
            // MCU_CORE_TL321X have 37 GPIOs,MCU_CORE_TL322X have 60 GPIOs and MCU_CORE_TL721X have 42 GPIOs can be multiplexed as PIN for GSPI_CSN0_IO function.Only a part of IO is listed here.
            #define SLAVE_CSN_PIN_NUM 8
gpio_func_pin_e slave_csn_pin[SLAVE_CSN_PIN_NUM] = {
    GPIO_FC_PA0,
    GPIO_FC_PA1,
    GPIO_FC_PA2,
    GPIO_FC_PA3,
    GPIO_FC_PA4,
    GPIO_FC_PB0,
    GPIO_FC_PB1,
    GPIO_FC_PB2,
};
        #endif
        #if defined(MCU_CORE_TL751X)
            //55 GPIOs can be multiplexed as PIN for GSPI_CSN0_IO function.Only a part of IO is listed here.
            #define SLAVE_CSN_PIN_NUM 8
gpio_func_pin_e slave_csn_pin[SLAVE_CSN_PIN_NUM] = {GPIO_FC_PA0, GPIO_FC_PA1, GPIO_FC_PA2, GPIO_FC_PA3, GPIO_FC_PA4, GPIO_FC_PA5, GPIO_FC_PA6, GPIO_FC_PB0};
        #endif
    #endif
    #ifndef MCU_CORE_TL321X
        #if (SPI_MODULE_SEL == LSPI_MODULE)
            #if defined(MCU_CORE_B92) || defined(MCU_CORE_TL721X)
lspi_pin_config_t lspi_pin_config = {
    .spi_csn_pin      = LSPI_CSN_PE0_PIN,
    .spi_clk_pin      = LSPI_CLK_PE1_PIN,
    .spi_mosi_io0_pin = LSPI_MOSI_IO0_PE2_PIN,
    .spi_miso_io1_pin = LSPI_MISO_IO1_PE3_PIN, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = LSPI_IO2_PE4_PIN,      //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = LSPI_IO3_PE5_PIN,      //quad  mode is required, otherwise it is NONE_PIN.
};
            #endif

            #if defined(MCU_CORE_TL322X)
lspi_pin_config_t lspi_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PA3, //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PA4, //quad  mode is required, otherwise it is NONE_PIN.
};
            #endif
            #if defined(MCU_CORE_TL751X)
lspi_pin_config_t lspi_pin_config = {
    .spi_csn_pin      = GPIO_FC_PA0,
    .spi_clk_pin      = GPIO_FC_PA1,
    .spi_mosi_io0_pin = GPIO_FC_PA2,
    .spi_miso_io1_pin = GPIO_FC_PB0,   //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = GPIO_FC_PE2,   //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = GPIO_FC_PE3,   //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io4_pin      = GPIO_FC_PE4,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io5_pin      = GPIO_FC_PE5,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io6_pin      = GPIO_FC_PE6,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_io7_pin      = GPIO_FC_PE7,   //octal mode is required, otherwise it is NONE_PIN.
    .spi_dm_pin       = GPIO_NONE_PIN, //opi psram is required,otherwise it is NONE_PIN.
};
            #endif

            #if defined(MCU_CORE_TL7518)
lspi_pin_config_t lspi_pin_config = {
    .spi_csn_pin      = LSPI_CSN_PB0_PIN,
    .spi_clk_pin      = LSPI_CLK_PE0_PIN,
    .spi_mosi_io0_pin = LSPI_MOSI_IO0_PE1_PIN,
    .spi_miso_io1_pin = LSPI_MISO_IO1_PE2_PIN, //3line mode is required, otherwise it is NONE_PIN.
    .spi_io2_pin      = LSPI_IO2_PE3_PIN,      //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io3_pin      = LSPI_IO3_PE4_PIN,      //quad  mode is required, otherwise it is NONE_PIN.
    .spi_io4_pin      = LSPI_IO4_PE5_PIN,      //octal mode is required, otherwise it is NONE_PIN.
    .spi_io5_pin      = LSPI_IO5_PE6_PIN,      //octal mode is required, otherwise it is NONE_PIN.
    .spi_io6_pin      = LSPI_IO6_PE7_PIN,      //octal mode is required, otherwise it is NONE_PIN.
    .spi_io7_pin      = LSPI_IO7_PF0_PIN,      //octal mode is required, otherwise it is NONE_PIN.
    .spi_dm_pin       = LSPI_NONE_PIN,         //opi psram is required,otherwise it is NONE_PIN.
};
            #endif
        #endif
    #endif
    /**********************************************************************************************************************
 *                                          dma  channel                                                      *
 *********************************************************************************************************************/
    #define SPI_TX_DMA_CHN DMA1
    #define SPI_RX_DMA_CHN DMA0
    /**********************************************************************************************************************
*                                         SPI master setting                                                        *
*********************************************************************************************************************/
    #if (SPI_DEVICE == SPI_MASTER_DEVICE)
/**********************************************************************************************************************
 *                                          global variable                                                   *
 *********************************************************************************************************************/
volatile unsigned char end_irq_flag = 0;
volatile unsigned char rx_dma_flag  = 0;
        #define DATA_BYTE_LEN 16
unsigned char spi_rx_buff[DATA_BYTE_LEN] __attribute__((aligned(4))) = {0x00};
        #if (SPI_PROTOCOL == B85M_SLAVE_PROTOCOL)
            #define SPI_B85M_READ_CMD  0x80
            #define SPI_B85M_WRITE_CMD 0x00

typedef struct
{
    unsigned char address[3];
    unsigned char cmd;
    unsigned char data[DATA_BYTE_LEN];
} spi_b85m_slave_protocol_t;

spi_b85m_slave_protocol_t __attribute__((aligned(4))) spi_tx_buff = {
    .address = {0x04, 0x40, 0x00},
    .cmd     = SPI_B85M_WRITE_CMD,
    .data    = {0xAA, 0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xBB},
};

        #elif (SPI_PROTOCOL == B91M_SLAVE_PROTOCOL)
            #if (SPI_SLAVE_READY_TEST == 1)
volatile unsigned char spi_b91m_slave_io_mode;
            #endif
            #if (DATA_MODE == NORMAL_MODE)
spi_wr_rd_config_t spi_b91m_slave_protocol_config = {
    .spi_io_mode   = SPI_SINGLE_MODE, /*IO mode set to SPI_3_LINE_MODE when SPI_3LINE_SLAVE.*/
    .spi_dummy_cnt = 32,              //B92 supports up to 32 clk cycle dummy, and TL751X,TL7518,TL721X,TL321X,TL322X supports up to 256 clk cycle dummy.
                #if defined(MCU_CORE_TL322X)
    .spi_dummy_hold = 0,
                #endif
    .spi_cmd_en      = 1,
    .spi_addr_en     = 0,
    .spi_addr_len    = 0, //when spi_addr_en = 0,invalid set.
    .spi_cmd_fmt_en  = 0, //when spi_cmd_en = 0,invalid set.
    .spi_addr_fmt_en = 0, //when spi_addr_en = 0,invalid set.
};
            #elif (DATA_MODE == ONLY_DATA_MODE)
spi_wr_rd_config_t spi_b91m_slave_protocol_config = {
    .spi_io_mode   = SPI_SINGLE_MODE, /*IO mode set to SPI_3_LINE_MODE when SPI_3LINE_SLAVE.*/
    .spi_dummy_cnt = 0,               //B92 supports up to 32 clk cycle dummy, and TL751X,TL7518,TL721X,TL321X,TL322X supports up to 256 clk cycle dummy.
                #if defined(MCU_CORE_TL322X)
    .spi_dummy_hold = 0,
                #endif
    .spi_cmd_en      = 0,
    .spi_addr_en     = 0,
    .spi_addr_len    = 0, //when spi_addr_en = 0,invalid set.
    .spi_cmd_fmt_en  = 0, //when spi_cmd_en = 0,invalid set.
    .spi_addr_fmt_en = 0, //when spi_addr_en = 0,invalid set.
};
            #endif

typedef struct
{
    // unsigned char address[4];
    unsigned char data_len;
    unsigned char data[DATA_BYTE_LEN   ];
} spi_b91m_slave_protocol_t;

// // 用于存储SPI传输的数据
// spi_b91m_slave_protocol_t __attribute__((aligned(4))) spi_tx_buff =
//     {
//         // .address  = {0xc0, 0x20, 0x04, 0x00},
//         .data_len = DATA_BYTE_LEN   ,
//         .data     = {},
// };
        #elif (SPI_PROTOCOL == B91M_SPI_SLAVE_PROTOCOL)
            #define spi_slave_address 0xc0200400 //When master and slave spi communicate, first compile the slave demo to get the value of spi_sspi_slave_rx_buff as the address of spi_slave_address.
spi_wr_rd_config_t spi_b91m_spi_slave_protocol_config = {
    .spi_io_mode     = SPI_SINGLE_MODE,
    .spi_dummy_cnt   = 8,
    .spi_cmd_en      = 1,
    .spi_addr_en     = 1,
    .spi_addr_len    = 4, //when hspi_addr_en = 0,invalid set.
    .spi_cmd_fmt_en  = 0, //when hspi_cmd_en = 0,invalid set.
    .spi_addr_fmt_en = 0, //when hspi_addr_en=0,invalid set.
};

typedef struct
{
    unsigned char data[DATA_BYTE_LEN];
} spi_b91m_spi_slave_protocol_t;

spi_b91m_spi_slave_protocol_t __attribute__((aligned(4))) spi_tx_buff =
    {
        .data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
};
        #endif

void user_init_led_app(void)
{

        #if defined(MCU_CORE_TL7518) || defined(MCU_CORE_TL751X)
    spi_master_init(SPI_MODULE_SEL, SRC_CLK_XTAL_48M, SRC_CLK_XTAL_48M / SPI_CLK, SPI_MODE0);
        #else
    spi_master_init(SPI_MODULE_SEL, sys_clk.pll_clk * 1000000 / SPI_CLK, SPI_MODE0);
        #endif
    spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT);  //clr
    // 启用SPI结束中断
    spi_set_irq_mask(SPI_MODULE_SEL, SPI_END_INT_EN); //endint_en
    // 将 SPI 发送配置为通过 DMA 传输数据
    spi_set_tx_dma_config(SPI_MODULE_SEL, SPI_TX_DMA_CHN);
    // 将 SPI 发送配置为通过 DMA 传输数据
    spi_set_master_rx_dma_config(SPI_MODULE_SEL, SPI_RX_DMA_CHN);

    	// 检查当前选择的 SPI 模块是否为 GSPI 模块。
        #if (SPI_MODULE_SEL == GSPI_MODULE)
    // 启用 GSPI 中断
    gspi_set_pin(&gspi_pin_config);
    plic_interrupt_enable(IRQ_GSPI);
        #endif
        #if defined(MCU_CORE_TL322X)
            #if (SPI_MODULE_SEL == GSPI1_MODULE)
    gspi1_set_pin(&gspi1_pin_config);
    plic_interrupt_enable(IRQ_GSPI1);
            #endif
            #if (SPI_MODULE_SEL == GSPI2_MODULE)
    gspi2_set_pin(&gspi2_pin_config);
    plic_interrupt_enable(IRQ_GSPI2);
            #endif
            #if (SPI_MODULE_SEL == GSPI3_MODULE)
    gspi3_set_pin(&gspi3_pin_config);
    plic_interrupt_enable(IRQ_GSPI3);
            #endif
            #if (SPI_MODULE_SEL == GSPI4_MODULE)
    gspi4_set_pin(&gspi4_pin_config);
    plic_interrupt_enable(IRQ_GSPI4);
            #endif
        #endif
        #ifndef MCU_CORE_TL321X
            #if (SPI_MODULE_SEL == LSPI_MODULE)
    lspi_set_pin(&lspi_pin_config);
    plic_interrupt_enable(IRQ_LSPI);
            #endif
        #endif
        #if (SPI_PROTOCOL == B85M_SLAVE_PROTOCOL)
    spi_master_config(SPI_MODULE_SEL, SPI_NORMAL);
        #elif (SPI_PROTOCOL == B91M_SLAVE_PROTOCOL)
    spi_master_config_plus(SPI_MODULE_SEL, &spi_b91m_slave_protocol_config);
            #if (SPI_SLAVE_READY_TEST == 1)
    spi_b91m_slave_io_mode = spi_b91m_slave_protocol_config.spi_io_mode;
            #endif

        #elif (SPI_PROTOCOL == B91M_SPI_SLAVE_PROTOCOL)
    spi_master_config_plus(SPI_MODULE_SEL, &spi_b91m_spi_slave_protocol_config);
        #endif

    dma_set_irq_mask(SPI_RX_DMA_CHN, TC_MASK);
    plic_interrupt_enable(IRQ_DMA);
    core_interrupt_enable();
}
        #if (SPI_SLAVE_READY_TEST == 1)
/**
 * @brief       This function servers to get lspi/gspi slave ready status. When slave is ready, slave ready reply a byte data:0x5a.
 * slave reply a byte data: 0x00.indicating that slave is not ready for data transmission.
 * @param[in]   spi_sel     - the spi module.
 * @param[in]   mode        - the spi master io mode.
 * @return      1:Indicates that the slave is ready. others:Indicates that slave is not ready.
 */
drv_api_status_e spi_master_get_slave_ready(spi_sel_e spi_sel, spi_io_mode_e mode)
{
    unsigned char slave_ready_flag = 0;

    switch (mode) {
    case SPI_SINGLE_MODE:
        spi_master_read_plus(spi_sel, SPI_READ_STATUS_SINGLE_CMD, (unsigned int)NULL, (unsigned char *)(&slave_ready_flag), 1, SPI_MODE_RD_DUMMY_READ);
        break;
    case SPI_DUAL_MODE:
        spi_master_read_plus(spi_sel, SPI_READ_STATUS_DUAL_CMD, (unsigned int)NULL, (unsigned char *)(&slave_ready_flag), 1, SPI_MODE_RD_DUMMY_READ);
        break;
    case SPI_QUAD_MODE:
        spi_master_read_plus(spi_sel, SPI_READ_STATUS_QUAD_CMD, (unsigned int)NULL, (unsigned char *)(&slave_ready_flag), 1, SPI_MODE_RD_DUMMY_READ);
        break;
            #if defined(MCU_CORE_TL7518) || defined(MCU_CORE_TL751X)
    case SPI_OCTAL_MODE:
        spi_master_read_plus(spi_sel, SPI_READ_STATUS_OCTAL_CMD, (unsigned int)NULL, (unsigned char *)(&slave_ready_flag), 1, SPI_MODE_RD_DUMMY_READ);
        break;
            #endif
    case SPI_3_LINE_MODE:
        spi_master_read_plus(spi_sel, SPI_READ_STATUS_SINGLE_CMD, (unsigned int)NULL, (unsigned char *)(&slave_ready_flag), 1, SPI_MODE_RD_DUMMY_READ);
        break;
    }
    if (SPI_WAIT(spi_is_busy, spi_sel, g_spi_timeout_error[spi_sel].g_spi_error_timeout_us, g_spi_timeout_error[spi_sel].spi_timeout_handler, SPI_API_ERROR_TIMEOUT_BUS_BUSY)) {
        return DRV_API_TIMEOUT;
    }
    if (slave_ready_flag == 0x5a) {
        return 1;
    } else {
        return 0;
    }
}
        #endif
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




        #if (SPI_MODULE_SEL == GSPI_MODULE)
_attribute_ram_code_sec_noinline_ void gspi_irq_handler(void)
        #endif
        #if defined(MCU_CORE_TL322X)
            #if (SPI_MODULE_SEL == GSPI1_MODULE)
    _attribute_ram_code_sec_noinline_ void gspi1_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI2_MODULE)
        _attribute_ram_code_sec_noinline_ void gspi2_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI3_MODULE)
            _attribute_ram_code_sec_noinline_ void gspi3_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI4_MODULE)
                _attribute_ram_code_sec_noinline_ void gspi4_irq_handler(void)
            #endif
        #endif
        #ifndef MCU_CORE_TL321X
            #if (SPI_MODULE_SEL == LSPI_MODULE)
                    _attribute_ram_code_sec_noinline_ void lspi_irq_handler(void)
            #endif
        #endif
{
    if (spi_get_irq_status(SPI_MODULE_SEL, SPI_END_INT)) {
        spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT); //clr
        end_irq_flag = 1;
    }
}


        #if (SPI_MODULE_SEL == GSPI_MODULE)
PLIC_ISR_REGISTER(gspi_irq_handler, IRQ_GSPI)
        #endif
        #if defined(MCU_CORE_TL322X)
            #if (SPI_MODULE_SEL == GSPI1_MODULE)
PLIC_ISR_REGISTER(gspi1_irq_handler, IRQ_GSPI1)
            #endif
            #if (SPI_MODULE_SEL == GSPI2_MODULE)
PLIC_ISR_REGISTER(gspi2_irq_handler, IRQ_GSPI2)
            #endif
            #if (SPI_MODULE_SEL == GSPI3_MODULE)
PLIC_ISR_REGISTER(gspi3_irq_handler, IRQ_GSPI3)
            #endif
            #if (SPI_MODULE_SEL == GSPI4_MODULE)
PLIC_ISR_REGISTER(gspi4_irq_handler, IRQ_GSPI4)
            #endif
        #endif
        #ifndef MCU_CORE_TL321X
            #if (SPI_MODULE_SEL == LSPI_MODULE)
PLIC_ISR_REGISTER(lspi_irq_handler, IRQ_LSPI)
            #endif
        #endif
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
    /**********************************************************************************************************************
    *                                         SPI slave setting                                                           *
    *********************************************************************************************************************/
    #elif (SPI_DEVICE == SPI_SLAVE_DEVICE)
        #if (SPI_PROTOCOL == B91M_SLAVE_PROTOCOL)
            /**********************************************************************************************************************
 *                                          global variable                                                         *
 *********************************************************************************************************************/
            #define SPI_RX_BUFF_LEN  24
            #define DATA_BYTE_LEN    16
            #define DATA_BYTE_OFFSET 8 //must be a multiple 4
unsigned char spi_slave_rx_buff[SPI_RX_BUFF_LEN + 4] __attribute__((aligned(4))) = {0x00};

        #endif
        #if (SPI_PROTOCOL == B91M_SPI_SLAVE_PROTOCOL)
            #define DATA_BYTE_LEN 16
volatile unsigned char spi_sspi_slave_rx_buff[DATA_BYTE_LEN + 4] __attribute__((aligned(4))) = {0x00};
        #endif
void user_init(void)
{
    gpio_output_en(LED1);     //enable output
    gpio_input_dis(LED1);     //disable input
    gpio_set_low_level(LED1); //LED Off
    gpio_function_en(LED1);   //enable gpio
    gpio_output_en(LED2);     //enable output
    gpio_input_dis(LED2);     //disable input
    gpio_set_low_level(LED2); //LED Off
    gpio_function_en(LED2);   //enable gpio

        #if (SPI_PROTOCOL == B91M_SLAVE_PROTOCOL)

            #if defined(MCU_CORE_TL751X)
    spi_slave_init(SPI_MODULE_SEL, SRC_CLK_XTAL_48M, SPI_MODE0);
            #else
    spi_slave_init(SPI_MODULE_SEL, SPI_MODE0);
            #endif

            //B92 supports up to 32 clk cycle dummy, and TL751X,TL7518,TL721X,TL321X,TL322X supports up to 256 clk cycle dummy.
            #if (DATA_MODE == NORMAL_MODE)
    spi_set_dummy_cnt(SPI_MODULE_SEL, 32);
    spi_clr_irq_status(SPI_MODULE_SEL, SPI_SLV_CMD_INT | SPI_END_INT);
    spi_set_irq_mask(SPI_MODULE_SEL, SPI_SLV_CMD_EN | SPI_END_INT_EN); //endint_en txfifoint_en rxfifoint_en
            #elif (DATA_MODE == ONLY_DATA_MODE)
    spi_cmd_dis(SPI_MODULE_SEL);
    spi_addr_dis(SPI_MODULE_SEL);
                #if defined(MCU_CORE_TL7518) || defined(MCU_CORE_TL751X) || defined(MCU_CORE_TL721X) || defined(MCU_CORE_TL321X) || defined(MCU_CORE_TL322X)
    spi_txdma_req_after_cmd_dis(SPI_MODULE_SEL);
                #endif
    spi_set_io_mode(SPI_MODULE_SEL, SPI_SINGLE_MODE);
    spi_set_dummy_cnt(SPI_MODULE_SEL, 0);
    spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT);
    spi_set_irq_mask(SPI_MODULE_SEL, SPI_END_INT_EN); //endint_en txfifoint_en rxfifoint_en
    spi_set_transmode(SPI_MODULE_SEL, SPI_MODE_READ_ONLY);
            #endif
    dma_set_irq_mask(SPI_RX_DMA_CHN, TC_MASK);
    plic_interrupt_enable(IRQ_DMA);
            #if (SPI_MODULE_SEL == GSPI_MODULE)
    gspi_set_pin(&gspi_pin_config);
    plic_interrupt_enable(IRQ_GSPI);
            #endif
            #if defined(MCU_CORE_TL322X)
                #if (SPI_MODULE_SEL == GSPI1_MODULE)
    gspi1_set_pin(&gspi1_pin_config);
    plic_interrupt_enable(IRQ_GSPI1);
                #endif
                #if (SPI_MODULE_SEL == GSPI2_MODULE)
    gspi2_set_pin(&gspi2_pin_config);
    plic_interrupt_enable(IRQ_GSPI2);
                #endif
                #if (SPI_MODULE_SEL == GSPI3_MODULE)
    gspi3_set_pin(&gspi3_pin_config);
    plic_interrupt_enable(IRQ_GSPI3);
                #endif
                #if (SPI_MODULE_SEL == GSPI4_MODULE)
    gspi4_set_pin(&gspi4_pin_config);
    plic_interrupt_enable(IRQ_GSPI4);
                #endif
            #endif
            #ifndef MCU_CORE_TL321X
                #if (SPI_MODULE_SEL == LSPI_MODULE)
    lspi_set_pin(&lspi_pin_config);
    plic_interrupt_enable(IRQ_LSPI);
                #endif
            #endif
    spi_set_tx_dma_config(SPI_MODULE_SEL, SPI_TX_DMA_CHN);
    spi_set_slave_rx_dma_config(SPI_MODULE_SEL, SPI_RX_DMA_CHN);
            #if (DATA_MODE == ONLY_DATA_MODE)
    spi_set_rx_dma(SPI_MODULE_SEL, (unsigned char *)(spi_slave_rx_buff + 4), SPI_RX_BUFF_LEN);
            #endif
            #if (SPI_SLAVE_MODE == SPI_3LINE_SLAVE)
    spi_set_io_mode(SPI_MODULE_SEL, SPI_3_LINE_MODE);
            #endif

    core_interrupt_enable();
        #elif (SPI_PROTOCOL == B91M_SPI_SLAVE_PROTOCOL)
            #if defined(MCU_CORE_B92)
    spi_slave_set_pin(); //spi slave only need set pin.
            #else
    sspi_pin_config_t sspi_pin_config =
        {
            .spi_clk_pin      = GPIO_FC_PA0,
            .spi_csn_pin      = GPIO_FC_PA1,
            .spi_mosi_io0_pin = GPIO_FC_PA2,
            .spi_miso_io1_pin = GPIO_FC_PA3,
        };
    spi_slave_set_pin(&sspi_pin_config); //spi slave only need set pin.
            #endif
        #endif
}

void main_loop(void)
{
        #if (SPI_SLAVE_READY_TEST == 1)
    spi_slave_ready_en(SPI_MODULE_SEL);
        #endif
}


        #if (SPI_MODULE_SEL == GSPI_MODULE)
_attribute_ram_code_sec_noinline_ void gspi_irq_handler(void)
        #endif
        #if defined(MCU_CORE_TL322X)
            #if (SPI_MODULE_SEL == GSPI1_MODULE)
    _attribute_ram_code_sec_noinline_ void gspi1_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI2_MODULE)
        _attribute_ram_code_sec_noinline_ void gspi2_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI3_MODULE)
            _attribute_ram_code_sec_noinline_ void gspi3_irq_handler(void)
            #endif
            #if (SPI_MODULE_SEL == GSPI4_MODULE)
                _attribute_ram_code_sec_noinline_ void gspi4_irq_handler(void)
            #endif
        #endif
        #ifndef MCU_CORE_TL321X
            #if (SPI_MODULE_SEL == LSPI_MODULE)
                    _attribute_ram_code_sec_noinline_ void lspi_irq_handler(void)
            #endif
        #endif
{
    if (spi_get_irq_status(SPI_MODULE_SEL, SPI_SLV_CMD_INT)) {
        switch (spi_slave_get_cmd(SPI_MODULE_SEL)) {
        case SPI_WRITE_DATA_SINGLE_CMD:
        case SPI_WRITE_DATA_DUAL_CMD:
        case SPI_WRITE_DATA_QUAD_CMD:
        #if defined(MCU_CORE_TL7518) || defined(MCU_CORE_TL751X)
        case SPI_WRITE_DATA_OCTAL_CMD:
        #endif
        #if defined(MCU_CORE_B92)
            spi_set_rx_dma(SPI_MODULE_SEL, (unsigned char *)spi_slave_rx_buff + 4);
        #else
            spi_set_rx_dma(SPI_MODULE_SEL, (unsigned char *)(spi_slave_rx_buff + 4), SPI_RX_BUFF_LEN);
        #endif
            break;

        case SPI_READ_DATA_SINGLE_CMD:
        case SPI_READ_DATA_DUAL_CMD:
        case SPI_READ_DATA_QUAD_CMD:
        #if defined(MCU_CORE_TL7518) || defined(MCU_CORE_TL751X)
        case SPI_READ_DATA_OCTAL_CMD:
        #endif
            spi_set_tx_dma(SPI_MODULE_SEL, (unsigned char *)(spi_slave_rx_buff + 4 + DATA_BYTE_OFFSET), DATA_BYTE_LEN);
            break;
        }
        spi_clr_irq_status(SPI_MODULE_SEL, SPI_SLV_CMD_INT); //clr
    }
    if (spi_get_irq_status(SPI_MODULE_SEL, SPI_END_INT)) {
        #if (SPI_SLAVE_READY_TEST == 1)
        if ((0 == spi_get_txfifo_num(SPI_MODULE_SEL)) && (0 == spi_get_rxfifo_num(SPI_MODULE_SEL))) {
            spi_slave_ready_en(SPI_MODULE_SEL);
        }
        #endif
        #if defined(MCU_CORE_TL322X)
        if (spi_get_transmode(SPI_MODULE_SEL) == SPI_MODE_WRITE_ONLY) {
            spi_set_transmode(SPI_MODULE_SEL, SPI_MODE_READ_ONLY);
            spi_set_rx_dma(SPI_MODULE_SEL, (unsigned char *)(spi_slave_rx_buff + 4), SPI_RX_BUFF_LEN);
        }
        #endif
        spi_clr_irq_status(SPI_MODULE_SEL, SPI_END_INT); //clr
    }
}

        #if (SPI_MODULE_SEL == GSPI_MODULE)
PLIC_ISR_REGISTER(gspi_irq_handler, IRQ_GSPI)
        #endif
        #if defined(MCU_CORE_TL322X)
            #if (SPI_MODULE_SEL == GSPI1_MODULE)
PLIC_ISR_REGISTER(gspi1_irq_handler, IRQ_GSPI1)
            #endif
            #if (SPI_MODULE_SEL == GSPI2_MODULE)
PLIC_ISR_REGISTER(gspi2_irq_handler, IRQ_GSPI2)
            #endif
            #if (SPI_MODULE_SEL == GSPI3_MODULE)
PLIC_ISR_REGISTER(gspi3_irq_handler, IRQ_GSPI3)
            #endif
            #if (SPI_MODULE_SEL == GSPI4_MODULE)
PLIC_ISR_REGISTER(gspi4_irq_handler, IRQ_GSPI4)
            #endif
        #endif
        #ifndef MCU_CORE_TL321X
            #if (SPI_MODULE_SEL == LSPI_MODULE)
PLIC_ISR_REGISTER(lspi_irq_handler, IRQ_LSPI)
            #endif
        #endif
/**
 * In order to solve the logic bug of GSPI rx DMA (LSPI tx/rx dma,GSPI tx DMA does not affect),
 * spi_hw_fsm_reset(GSPI_MODULE); must be called every time the DMA transfer is complete when using GSPI rx DMA.
 * Bugfix: reset GSPI RXDMA in DMA transfer completion interrupt(this is a hardware bug already confirmed with jianzhi)
 * changed by pengxiang.hong 20230328.
 */
_attribute_ram_code_sec_noinline_ void dma_irq_handler(void)
{
    if (dma_get_tc_irq_status(BIT(SPI_RX_DMA_CHN))) {
        dma_clr_tc_irq_status(BIT(SPI_RX_DMA_CHN));
        #if defined(MCU_CORE_B92)
        if (SPI_MODULE_SEL == GSPI_MODULE) {
        	// 重置 GSPI 模块的硬件有限状态机（FSM）。
            spi_hw_fsm_reset(GSPI_MODULE);
        }
        #endif
        // 设置 SPI 的传输模式为 仅写模式。
        spi_set_transmode(SPI_MODULE_SEL, SPI_MODE_WRITE_ONLY);
        // 使用 DMA 向 SPI 模块发送数据。
        spi_set_tx_dma(SPI_MODULE_SEL, (unsigned char *)(spi_slave_rx_buff + 4 + DATA_BYTE_OFFSET), DATA_BYTE_LEN);
    }
}
PLIC_ISR_REGISTER(dma_irq_handler, IRQ_DMA)

    #endif
#endif
