/**
 * MIT License
 * 
 * Copyright (c) 2023 My Techno Talent
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/**
 * @file nrf24l01_driver.c
 *
 * @brief Structs and functions for the NRF24L01 driver.
 * 
 * This contains the structs and functions for the NRF24L01 wireless 
 * transceiver module for use with the Raspberry Pi Pico microcontroller board.
 *
 * @author Kevin Thomas
 * @date   03/25/2023
 */

#include "stdio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "nrf24l01.h"
#include "nrf24l01_pins.h"

static nrf24l01_config_t nrf24l01_config = {
    .port = PORT,
    .ce = CE,
    .csn = CSN,
    .clk = CLK,
    .tx = TX,
    .rx = RX,
};

void nrf24l01_init()
{
    spi_init(nrf24l01_config.port, 8000000);
    gpio_set_function(nrf24l01_config.clk, GPIO_FUNC_SPI);
    gpio_set_function(nrf24l01_config.tx, GPIO_FUNC_SPI);
    gpio_set_function(nrf24l01_config.rx, GPIO_FUNC_SPI);
    gpio_init(nrf24l01_config.csn);
    gpio_init(nrf24l01_config.ce);  
    gpio_set_dir(nrf24l01_config.ce, 1);
    gpio_set_dir(nrf24l01_config.csn, 1);
    nrf24l01_ce_low();
    nrf24l01_csn_high();
    nrf24l01_reset();
    sleep_ms(11); // p20 Power on reset 10.3 ms
    nrf24l01_write_reg(NRF24L01_CONFIG, 0b0000010); // p53
    sleep_us(1500); // p20 Start up 1.5 ms
    nrf24l01_write_reg(NRF24L01_CONFIG, 0b0001110); // p53
    nrf24l01_write_reg(NRF24L01_EN_AA, 0b00000000); // p53
    nrf24l01_write_reg(NRF24L01_SETUP_AW , 0b00000011); // p54
    nrf24l01_set_channel(0b00000011); // p54
    nrf24l01_write_reg(NRF24L01_RF_SETUP, 0b00001111);  // p54
    nrf24l01_write_reg(NRF24L01_RX_PW_P0, 0x0a); // p56
    nrf24l01_write_data(NRF24L01_RX_ADDR_P0, "kevin", 0x05); // p55
    nrf24l01_write_data(NRF24L01_TX_ADDR, "kevin", 0x05); // p55
}

static void nrf24l01_csn_low() 
{ 
    gpio_put(nrf24l01_config.csn, 0);
}

static void nrf24l01_csn_high() 
{ 
    gpio_put(nrf24l01_config.csn, 1);
}

static void nrf24l01_ce_low()
{ 
    gpio_put(nrf24l01_config.ce, 0);
}

static void nrf24l01_ce_high()
{ 
    gpio_put(nrf24l01_config.ce, 1);
}

static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    reg = NRF24L01_R_REGISTER & reg; // p46
    uint8_t result = 0;
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &reg, 1);
    spi_read_blocking(nrf24l01_config.port, 0xff, &result, 1);
    nrf24l01_csn_high();
    return result;    
}

static void nrf24l01_write_cmd(uint8_t cmd)
{
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &cmd, 1);
    nrf24l01_csn_high();
}

static void nrf24l01_flush_rx()
{
    nrf24l01_write_cmd(NRF24L01_FLUSH_RX); // p46
}

static void nrf24l01_flush_tx()
{
    nrf24l01_write_cmd(NRF24L01_FLUSH_TX); // p46
}

static void nrf24l01_write_reg(uint8_t reg, uint8_t byte)
{
    reg  = NRF24L01_W_REGISTER | (NRF24L01_R_REGISTER & reg); // p46
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &reg, 1);
    spi_write_blocking(nrf24l01_config.port, (uint8_t *)&byte, 1);
    nrf24l01_csn_high();
}

static void nrf24l01_write_data(uint8_t reg, uint8_t *data, uint8_t size)
{
    reg  = NRF24L01_W_REGISTER | (NRF24L01_R_REGISTER & reg); // p46
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &reg, 1);
    spi_write_blocking(nrf24l01_config.port, (uint8_t*)data, size);
    nrf24l01_csn_high();
}

static void nrf24l01_reset()
{
    nrf24l01_csn_high();
    nrf24l01_ce_low();
    nrf24l01_write_reg(NRF24L01_CONFIG, 0b00000000); // p53 
    nrf24l01_write_reg(NRF24L01_SETUP_AW, 0b00000011); // p54
    nrf24l01_write_reg(NRF24L01_SETUP_RETR, 0b00000000); // p54
    nrf24l01_write_reg(NRF24L01_RF_SETUP, 0b000000); // p54
    nrf24l01_write_reg(NRF24L01_STATUS, 0b01110000); // p55
    nrf24l01_flush_rx();
    nrf24l01_flush_tx();
}

static void nrf24l01_set_channel(uint8_t channel)
{
    nrf24l01_write_reg(NRF24L01_RF_CH, channel); // p54
}

void nrf24l01_set_mode_tx()
{
    uint8_t nrf24l01_init = nrf24l01_read_reg(NRF24L01_CONFIG); // p53 
    nrf24l01_init &= ~(1);
    nrf24l01_write_reg(NRF24L01_CONFIG, nrf24l01_init); // p53
    nrf24l01_ce_low();
    sleep_us(130); // p20 TX Setting 130 us
}

void nrf24l01_set_mode_rx()
{
    uint8_t nrf24l01_init = nrf24l01_read_reg(NRF24L01_CONFIG); // p53
    nrf24l01_init |= 1;
    nrf24l01_write_reg(NRF24L01_CONFIG, nrf24l01_init); // p53
    nrf24l01_ce_high();
    sleep_us(130); // p20 RX Setting 130 us
}

void nrf24l01_send_msg(uint8_t *data, uint8_t size)
{
    size =  size > 32 ? 32 : size;
    uint8_t tx_payload = NRF24L01_W_TX_PAYLOAD; // p46
    uint8_t status = nrf24l01_read_reg(NRF24L01_STATUS); // p55
    if(status & 1)
    {
        nrf24l01_flush_tx();
    }
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &tx_payload, 1);
    spi_write_blocking(nrf24l01_config.port, data, size);
    nrf24l01_csn_high();
    nrf24l01_ce_high();
    while((nrf24l01_read_reg(NRF24L01_STATUS) & 0b00110000) == 0) {}; // p55
    nrf24l01_ce_low();
    nrf24l01_write_reg(NRF24L01_STATUS, 0b00110000); // p55 
}

uint8_t nrf24l01_send_msg_int(uint8_t *data, uint8_t size)
{
    if(nrf_tx_ready == 0) return 0;
    nrf_tx_ready = 0;
    size = size > 32 ? 32 : size;
    uint8_t tx_payload = NRF24L01_W_TX_PAYLOAD; // p46
    uint8_t status = nrf24l01_read_reg(NRF24L01_STATUS); // p55
    if(status & 1)
    {
        nrf24l01_flush_tx();
    }
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &tx_payload, 1);
    spi_write_blocking(nrf24l01_config.port, data, size);
    nrf24l01_csn_high();
    nrf24l01_ce_high();
    return 1;
}

void nrf24l01_recv_msg(uint8_t *data, uint8_t size)
{
    size = size > 32 ? 32 : size;
    uint8_t rx_payload = NRF24L01_R_RX_PAYLOAD; // p46
    nrf24l01_write_reg(NRF24L01_STATUS , 0b01000000); // p55
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &rx_payload, 1);
    spi_read_blocking(nrf24l01_config.port, 0xff,(uint8_t*)data, size);
    nrf24l01_csn_high();
}

uint8_t nrf24l01_recv_msg_int(uint8_t *data, uint8_t size)
{
    if( nrf_rx_ready == 0) return 0;
    size =  size > 32 ? 32 : size;
    uint8_t rx_payload = NRF24L01_R_RX_PAYLOAD; // p46
    nrf24l01_csn_low();
    spi_write_blocking(nrf24l01_config.port, &rx_payload, 1);
    spi_read_blocking(nrf24l01_config.port, 0xff, (uint8_t*)data, size);
    nrf24l01_csn_high();
    nrf_rx_ready = 0;
    nrf24l01_write_reg(NRF24L01_STATUS, 0b01000000); // p55
    nrf24l01_flush_rx();
    return 1;
}

void nrf24l01_end_of_transmission()
{
    nrf24l01_ce_low();
    nrf_tx_ready = 1;
}

void nrf24l01_int(uint gpio, uint32_t events)
{
    uint8_t status = nrf24l01_read_reg(NRF24L01_STATUS); // p55
    if(status & 0b00100000)
    {
        nrf24l01_write_reg(NRF24L01_STATUS, 0b00100000); // p55  
        nrf24l01_end_of_transmission();
    }
    if(status & 0b01000000)
    {
        nrf_rx_ready = 1;
        nrf24l01_write_reg(NRF24L01_STATUS, 0b01000000); // p55
    }
}

uint8_t nrf24l01_new_msg()
{
    uint8_t status = nrf24l01_read_reg(NRF24L01_STATUS);
    // status = 0b00100110;
    // status >> 1 = 0b00010011;
    // (status >> 1 & 0b00000111) = 0b00010011 & 0b00000111 = 0b00000011 = 3
    // return 3 < 6 = true;
    // status = 0b00101110;
    // status >> 1 = 0b00010111;
    // (status >> 1 & 0b00000111) = 0b00010111 & 0b00000111 = 0b00000111 = 7
    // return 7 < 6 = false;
    return (status >> 1 & 0b00000111) < 6; // p55
}
