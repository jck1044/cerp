//
//  uart.c
//
// Somewhat generic code for buffered UART output. No checks for buffer overflow.
// Packet length is always 8 bytes and baud rate guarantees adequate throughput

#ifndef STM32F0 // helps Xcode resolve headers
#define STM32F0
#endif

#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#include "uart.h"

#include <stdio.h>
#include <string.h>

// output buffer
int uartbuf[64];
long qnext, qlast;

// **** ISR
// Used for transmit only
void usart1_isr(void)
{
    if (USART_ISR(USART1) & USART_ISR_TXE) // (should be the only bit enabled)
    {
        if (qnext != qlast)
        {
            gpio_set(GPIOC, GPIO9);
            USART_TDR(USART1) = uartbuf[qnext++];
        }
        else
        {
            usart_disable_tx_interrupt(USART1);
            gpio_clear(GPIOC, GPIO9);
        }
        qnext &= 0x3f;
    }
}
// ****

// Buffered single char output. Put in mem buf and return. ISR sends it
void putch(int ch)
{
    usart_disable_tx_interrupt(USART1); // prevent collision ISR access to qlast
    uartbuf[qlast++] = ch;
    qlast &= 0x3f;
    usart_enable_tx_interrupt(USART1);
}

void putwd(long wd) // Write 16-bit value to UART
{
    putch((wd & 0xFF00) >> 8); // msb
    putch(wd & 0xFF);          // lsb
}

void putswab(long wd) // Write 16-bit byte swappped to UART
{
    putch(wd & 0xFF);          // lsb
    putch((wd & 0xFF00) >> 8); // msb
}

void SetupUART(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);

    // UART at 57600baud
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    qnext = 0;
    qlast = 0;

    usart_enable(USART1);

    putch('!'); // send a char to show it was reset
}

// void printWord(long toPrint)
// {
//     for (unsigned int i = 0; i <= sizeof(toPrint); i++)
//     {
//         putch(toPrint);
//     }
//     putch('\r');
//     putch('\n');

    // char buffer[12];

    // snprintf(buffer, sizeof(buffer), "%li", raw);

    // char msg[] = "raw: ";

    // int sizeMSG = strlen(msg);
    // for (int i = 0; i < sizeMSG; i++)
    // {
    //     usart_send_blocking(USART1, msg[i]);
    //     }
    //     int sizeBUF = strlen(buffer);
    //     for (int i = 0; i < sizeBUF; i++)
    //     {
    //         usart_send_blocking(USART1, buffer[i]);
    //     }
    //     usart_send_blocking(USART1, '\r');
    //     usart_send_blocking(USART1, '\n');
// }
