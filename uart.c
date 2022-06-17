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

// Buffered single char output. Put in mem buf and return. ISR sends it
void putch(int ch)
{
    gpio_toggle(GPIOC, GPIO8);
    usart_send_blocking(USART1, ch);
}

void putswab(long wd) // Write 16-bit byte swappped to UART
{
    putch(wd & 0xFF);          // lsb
    putch((wd & 0xFF00) >> 8); // msb
}

void SetupUART(void)
{
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_enable(USART1);

    putch('!'); // send a char to show it was reset
}
