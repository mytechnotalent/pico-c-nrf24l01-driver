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
 * @file nrf24l01_pins.h
 *
 * @brief Config defines for the NRF24L01 driver.
 * 
 * This contains the config defines for the NRF24L01 wireless transceiver
 * module for use with the Raspberry Pi Pico microcontroller board.
 *
 * @author Kevin Thomas
 * @date   03/25/2023
 */

#ifndef _NRF24L01_PINS_H_
#define _NRF24L01_PINS_H_

/** 
 * @brief Defines NRF24L01 config values.
 * 
 * These config defines are used to target NRF24L01 registers.
*/
#define PORT    spi1
#define CE      15
#define CSN     9
#define CLK     10
#define TX      11
#define RX      12

#endif /* _NRF24L01_PINS_H_ */