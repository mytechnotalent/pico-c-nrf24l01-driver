![image](https://github.com/mytechnotalent/pico-c-nrf24l01-driver/blob/main/Pico%20C%20NRF24L01%20Driver.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Pico C NRF24L01 Driver
A Raspberry Pi Pico NRF24L01 driver written in C.

# Usage
#### If the `SEND_FLAG 0` configuration within `main.c` exists the demo will be set for receive mode and if `SEND_FLAG 1`
exists the demo will be set for transmit mode.  The configuration exists within `nrf24l01_pins.h`.

# `main.c`
```c
#include "stdio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "nrf24l01.h"

#define SEND_FLAG 0 // change to 0 to run the program in recv mode

int main()
{
    stdio_init_all();

    char msg_sent[10] = {"8675309"}; // test data to send
    char msg_recv[10]; // test data received

    nrf24l01_init();

    if(SEND_FLAG)
    {
        nrf24l01_set_mode_tx();

        while(1)
        { 
            nrf24l01_send_msg((uint8_t*)msg_sent, 10);
            sleep_ms(100);
        }
    }
    else
    {
        nrf24l01_set_mode_rx();

        while(1)
        {
            if(nrf24l01_new_msg() )
            {
                nrf24l01_recv_msg((uint8_t*)&msg_recv, 10);
                printf("%s\n\r", msg_recv);
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
