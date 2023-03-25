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