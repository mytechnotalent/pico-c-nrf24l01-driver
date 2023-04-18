![image](https://github.com/mytechnotalent/pico-c-nrf24l01-driver/blob/main/Pico%20C%20NRF24L01%20Driver.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Pico C NRF24L01 Driver
A Raspberry Pi Pico NRF24L01 driver written in C.

# Usage
#### If the `SEND_FLAG 0` configuration within `main.c` exists the demo will be set for receive mode and if `SEND_FLAG 1` exists the demo will be set for transmit mode.  The configuration exists within `nrf24l01_pins.h`.

# `main.c`
```c
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

#include "stdio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "nrf24l01.h"

#define SEND_FLAG 0 // change to 1 to run the program in send mode
#define SEND_FLAG_INT 1 // change to 1 to run the program in send int mode
#define RECV_FLAG 0 // change to 1 to run the program in recv mode
#define RECV_FLAG_INT 0 // change to 1 to run the program in recv int mode
#define LED 25

int main()
{
    char msg_sent[10] = {"8675309"}; // test data to send
    char msg_recv[10]; // test data received

    stdio_init_all();
    nrf24l01_init();
    gpio_init(LED);
    gpio_set_dir(LED, 1);

    if(SEND_FLAG)
    {
        nrf24l01_set_mode_tx();

        while(1)
        { 
            gpio_put(LED, 1);
            nrf24l01_send_msg((uint8_t*)msg_sent, 10);
            sleep_ms(100);
            gpio_put(LED, 0);
        }
    }
    else if(SEND_FLAG_INT)
    {
        gpio_set_irq_enabled_with_callback(3, GPIO_IRQ_EDGE_FALL, true, &nrf24l01_int);
        nrf24l01_set_mode_tx();

        while(1)
        { 
            gpio_put(LED, 1);
            nrf24l01_send_msg_int((uint8_t *)msg_sent, 10);
            sleep_ms(100);
            gpio_put(LED, 0);
        }
    }
    else if(RECV_FLAG)
    {
        nrf24l01_set_mode_rx();

        while(1)
        {
            if(nrf24l01_new_msg() )
            {
                gpio_put(LED, 1);
                nrf24l01_recv_msg((uint8_t *)&msg_recv, 10);
                printf("%s\n\r", msg_recv);
                sleep_ms(100);
                gpio_put(LED, 0);
            }
        }
    }
    else
    {
        gpio_set_irq_enabled_with_callback(3, GPIO_IRQ_EDGE_FALL, true, &nrf24l01_int);
        nrf24l01_set_mode_rx();

        while(1)
        {
            if(nrf24l01_recv_msg_int((uint8_t *)&msg_recv, 10))
            {
                gpio_put(LED, 1);
                printf("%s\n\r", msg_recv);
                sleep_ms(100);
                gpio_put(LED, 0);
            }
        }
    }

    return 0;
}
```

# `nrf24l01_pins.h`
```c
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
```

## License
[MIT](https://raw.githubusercontent.com/mytechnotalent/pico-c-nrf24l01-driver/main/LICENSE)
