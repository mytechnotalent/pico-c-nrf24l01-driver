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
 * @file nrf24l01_driver.h
 *
 * @brief Struct and function prototypes for the NRF24L01 driver.
 * 
 * This contains the prototypes for the NRF24L01 wireless transceiver module
 * for use with the Raspberry Pi Pico microcontroller board.
 *
 * @author Kevin Thomas
 * @date   03/25/2023
 */

#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "hardware/spi.h"

/** 
 * @brief Defines NRF24L01 register values.
 * 
 * These defines are used to target NRF24L01 registers.
*/
#define NRF24L01_CONFIG         0x00
#define NRF24L01_EN_AA          0x01
#define NRF24L01_SETUP_AW       0x03
#define NRF24L01_SETUP_RETR     0x04
#define NRF24L01_RF_CH          0x05
#define NRF24L01_RF_SETUP       0x06
#define NRF24L01_STATUS         0x07
#define NRF24L01_RX_ADDR_P0     0x0a
#define NRF24L01_TX_ADDR        0x10
#define NRF24L01_RX_PW_P0       0x11
#define NRF24L01_R_REGISTER     0b00011111
#define NRF24L01_W_REGISTER     0b00100000
#define NRF24L01_R_RX_PAYLOAD   0b01100001
#define NRF24L01_W_TX_PAYLOAD   0b10100000
#define NRF24L01_FLUSH_TX       0b11100001
#define NRF24L01_FLUSH_RX       0b11100010

/**
 * @brief Struct containing the data for the NRF24L01 module and SPI communication.
 *
 * This struct contains configuration data for the NRF24L01 module and SPI
 * communication interface. It includes pointers to the SPI port used for
 * communication, as well as pin assignments for the CE, CSN, CLK, TX [MOSI],
 * and RX [MISO] pins on the Raspberry Pi Pico board.
 */
typedef struct {
    spi_inst_t *port;   /**< Pointer to the SPI port used for communication. */
    uint16_t ce;        /**< Pin assignment for the CE pin on the Raspberry Pi Pico board. */
    uint16_t csn;       /**< Pin assignment for the CSN pin on the Raspberry Pi Pico board. */
    uint16_t clk;       /**< Pin assignment for the CLK pin on the Raspberry Pi Pico board. */
    uint16_t tx;        /**< Pin assignment for the TX [MOSI] pin on the Raspberry Pi Pico board. */
    uint16_t rx;        /**< Pin assignment for the RX [MISO] pin on the Raspberry Pi Pico board. */
} nrf24l01_config_t;

/** @brief Two variables to indicate the readiness of NRF24L01 module for transmission
 *         and reception.
 * 
 * These two variables are used to indicate the readiness of the NRF24L01 module for transmission 
 * and reception respectively. The default value for both variables is 1, indicating that the 
 * module is ready to transmit or receive data.
*/
static uint32_t nrf_tx_ready = 1;
static uint32_t nrf_rx_ready = 1;

/**
 * @brief Function to set the CSN pin low.
 *
 * This function sets the CSN pin low, allowing the NRF24L01 module to receive
 * SPI data from the Raspberry Pi Pico board.
 */
static void nrf24l01_csn_low();

/**
 * @brief Function to set the CSN pin high.
 *
 * This function sets the CSN pin high, preventing the NRF24L01 module from
 * receiving SPI data from the Raspberry Pi Pico board.
 */
static void nrf24l01_csn_high();

/**
 * @brief Function to set the CE pin low.
 *
 * This function sets the CE pin low, disabling the NRF24L01 module's radio
 * transmission and reception capabilities.
 */
static void nrf24l01_ce_low();

/**
 * @brief Function to set the CE pin high.
 *
 * This function sets the CE pin high, enabling the NRF24L01 module's radio
 * transmission and reception capabilities.
 */
static void nrf24l01_ce_high();

/** 
 * @brief Function to read a register of the NRF24L01 module.
 * 
 * This function sends a read command to the NRF24L01 module to read the value
 * of the specified register. It uses SPI to communicate with the module.
 * 
 * @param reg   The register address to read.
 * 
 * @return      The value read from the register.
*/
static uint8_t nrf24l01_read_reg(uint8_t reg);

/**
 * @brief Function to write a command to the NRF24L01 module.
 * 
 * This function sends a command to the NRF24L01 module. It uses SPI to
 * communicate with the module.
 * 
 * @param cmd   The command to write.
*/
static void nrf24l01_write_cmd(uint8_t cmd);

/**
 * @brief Function to flush the receive buffer of the NRF24L01 module.
 * 
 * This function sends a command to the NRF24L01 module to flush the receive
 * buffer. It uses SPI to communicate with the module.
*/
static void nrf24l01_flush_rx();

/**
 * @brief Function to flush the transmit buffer of the NRF24L01 module.
 * 
 * This function sends a command to the NRF24L01 module to flush the transmit
 * buffer. It uses SPI to communicate with the module.
*/
static void nrf24l01_flush_tx();

/**
 * @brief Function to write a single byte to a register on the NRF24L01 module.
 * 
 * This function writes a single byte to a register on the NRF24L01 module.
 * The register address is provided as the first parameter and the value to write
 * to the register is provided as the second parameter. The function uses SPI
 * communication to write the data to the module.  The function calculates the correct 
 * command byte to send to the module based on the provided register address. The 
 * command byte is formed by setting the high bit to 1 (NRF24L01_W_REGISTER) and 
 * masking the low 5 bits with the register address.
 * 
 * @note This function assumes that the SPI interface and CSN (chip select) pin
 *       have been initialized and configured correctly before calling this function.
 *       This function also assumes that the module is in a ready state to receive data.
 * 
 * @param reg   The register address to write to.
 * @param byte  The value to write to the specified register.
*/
static void nrf24l01_write_reg(uint8_t reg, uint8_t byte);

/**
 * @brief Function to write an array of bytes to a register on the NRF24L01 module.
 * 
 * This function writes an array of bytes to a register on the NRF24L01 module.
 * The register address is provided as the first parameter, the pointer to the
 * data array is provided as the second parameter, and the number of bytes to write
 * is provided as the third parameter. The function uses SPI communication to
 * write the data to the module.  The function calculates the correct command byte 
 * to send to the module based on the provided register address. The command byte 
 * is formed by setting the high bit to 1 (NRF24L01_W_REGISTER) and masking the 
 * low 5 bits with the register address.
 * 
 * @note This function assumes that the SPI interface and CSN (chip select) pin
 *       have been initialized and configured correctly before calling this function.
 *       This function also assumes that the module is in a ready state to receive data.
 * 
 * @param reg   The register address to write to.
 * @param data  Pointer to the array of bytes to write to the register.
 * @param size  The number of bytes to write.
*/
static void nrf24l01_write_data(uint8_t reg, uint8_t *data, uint8_t size);

/**
 * @brief Function to reset the NRF24L01 module to default configuration.
 * 
 * This function resets the NRF24L01 module to its default configuration by
 * writing specific values to its registers. The function first sets the chip select
 * pin and CE pin to their inactive states. Then, it writes the default values to
 * several registers using the nrf24l01_write_reg function. Finally, it flushes
 * the RX and TX FIFOs using the nrf24l01_flush_rx and nrf24l01_flush_tx functions.
 * 
 * @note This function assumes that the SPI interface, CSN (chip select) pin, and
 *       CE (chip enable) pin have been initialized and configured correctly before
 *       calling this function. This function also assumes that the module is in a
 *       ready state to receive data.
*/
static void nrf24l01_reset();

/**
 * @brief Function to set the operating channel for the NRF24L01 module.
 * 
 * This function sets the operating channel for the NRF24L01 module using
 * the nrf24l01_write_reg function. The channel number is provided as the
 * first parameter. The function writes the channel number to the RF_CH register
 * of the module.
 * 
 * @note This function assumes that the SPI interface and CSN (chip select) pin
 * have been initialized and configured correctly before calling this function.
 * This function also assumes that the module is in a ready state to receive data.
 * 
 * @param channel   The channel number to set.
*/
static void nrf24l01_set_channel(uint8_t channel);

/**
 * @brief Function to initialize the NRF24L01 module with default settings.
 * 
 * This function initializes the NRF24L01 module with default settings using the
 * various nrf24l01_write_reg and nrf24l01_write_data functions. It sets the
 * powerup configuration, enables CRC encoding, disables acknowledgement, sets
 * the address width to 5 bytes, sets the frequency channel to 3, sets the air
 * data rate to 2 Mbps, sets the payload size to 10 bytes, and writes data to
 * registers 0x0a and 0x10.
 * 
 * @note This function assumes that the SPI interface and CSN [chip select] pin
 *       have been initialized and configured correctly before calling this function.
 *       This function also assumes that the module is in a ready state to receive data.
*/
void nrf24l01_init();

/**
 * @brief Function to set the NRF24L01 module to transmit mode.
 * 
 * This function sets the NRF24L01 module to transmit mode by modifying the configuration
 * register and setting the chip enable pin to low after a short delay.
*/
void nrf24l01_set_mode_tx();

/** @brief Function to set the NRF24L01 module to receive mode.
 * 
 * This function sets the NRF24L01+ module to receive mode by setting the 
 * appropriate bit in the CONFIG register and enabling the chip enable (CE)
 * pin. The function also introduces a delay of 130 microseconds to allow the
 * module to stabilize before data transmission or reception can begin.
*/
void nrf24l01_set_mode_rx();

/**
 * @brief Function to send a message using the NRF24L01 module.
 * 
 * This function sends a message of a given size using the NRF24L01 module. 
 * The maximum size of the message is 32 bytes.  It first checks the status register
 * of the NRF24L01 module to see if the transmission buffer is empty. If it is not 
 * empty, it flushes the buffer.  The message is then transmitted over the air using 
 * the SPI interface of the module. The function waits for the transmission to complete 
 * or for the maximum number of retransmissions to be reached.  Finally, it clears 
 * the transmission complete and maximum number of retransmissions flags in the status register.
 * 
 * @param data  Pointer to the data buffer containing the message to be sent.
 * @param size  The size of the message to be sent, in bytes.
*/
void nrf24l01_send_msg(uint8_t *data, uint8_t size);

/** @brief Function to send a message using the NRF24L01 module using the interrupt-based approach.
 * 
 * This function sends a message of the given size via the NRF24L01 module using the 
 * interrupt-based approach.  The message is sent using the NRF24L01's built-in hardware 
 * SPI interface.  This function blocks until the entire message has been sent.
 * If successful, the function returns 0, otherwise it returns a non-zero error code.
 * 
 * @param[in] data Pointer to the buffer containing the message to be sent.
 * @param[in] size The size of the message to be sent.
 * 
 * @return Returns 0 on success, otherwise a non-zero error code.
*/
uint8_t nrf24l01_send_msg_int(uint8_t *data, uint8_t size);

/** @brief Function to receive a message from the NRF24L01 transceiver.
 * 
 * This function receives a message from the NRF24L01 transceiver and stores it in the provided 
 * data buffer.  The maximum size of the data buffer is 32 bytes, and the size parameter 
 * is used to determine the actual size of the message received.
 * 
 * @param data  Pointer to the buffer where the received message will be stored.
 * @param size  The expected size of the message to be received. The actual size of 
 *              the message may be smaller or equal to this value.
*/
void nrf24l01_recv_msg(uint8_t *data, uint8_t size);

/** @brief Function to receive a message using the NRF24L01 module using the interrupt-based
 *         approach.
 * 
 * This function receives a message of up to size bytes from the NRF24L01 module and stores
 * it in the buffer data.  If there is no message available to be received, this function 
 * returns 0. Otherwise, it returns the number of bytes received. If the size of the received
 * message is greater than size, the excess bytes will be discarded.
 * 
 * @param data Pointer to a buffer to store the received message.
 * @param size The maximum size of the message to receive.
 * 
 * @return The number of bytes received, or 0 if there is no message available.
*/
uint8_t nrf24l01_recv_msg_int(uint8_t *data, uint8_t size);

/** @brief Function to end the transmission and set the TX ready flag to 1.
 * 
 * This function lowers the CE pin to end the transmission and sets the nrf_tx_ready 
 * flag to 1, indicating that the NRF24L01 module is ready to transmit again.
*/
void nrf24l01_end_of_transmission();

/** @brief Function to handle interrupts from the NRF24L01 module.
 * 
 * This function is called when an interrupt event occurs on the specified GPIO pin.
 * It handles the interrupt by reading the status register of the NRF24L01 module and
 * determining the cause of the interrupt. The appropriate action is then taken based
 * on the interrupt cause.
 * 
 * @param gpio The GPIO pin number that is associated with the interrupt.
 * @param events The interrupt event that has occurred on the GPIO pin.
*/
void nrf24l01_int(uint gpio, uint32_t events);

/**
 * @brief Function to check if a new message has been received by the NRF24L01 module.
 * 
 * This function reads the status register of the NRF24L01 module and checks if a new 
 * message has been received.  It returns a boolean value indicating whether a new message 
 * has been received or not.
 * 
 * @return A boolean value indicating whether a new message has been received or not. 
 *         1 if a new message is available, 0 otherwise.
*/
uint8_t nrf24l01_new_msg();

#endif /* _NRF24L01_H_ */