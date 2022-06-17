/*********************************************************************

 Y3 CERPA Software

  UART sends 8bytes every 2ms. 57.6K baud

  Output Format:  SYNC,SEQ,RPAm,RPAl,SWPm,SWPl,HKm,HKl

HK vals mux by SEQ 3 lsbs
 0 BUS_Vmon	PA1 AIN 1
 1 BUS_Imon	PA2 AIN 2
 2 V2V5_mon	PA3 AIN 3
 3 ENDMON	PA0 AIN 4
 4 V15V_mon	PA7 AIN 7
 5 TEMP		    AIN 16
 6 VREF		    AIN 17
 7 packet count/8

Main does all setup and then goes into an idle loop.

TIM1 sample interrupt routine sends packets.

*********************************************************************/

#ifndef STM32F0 // helps Xcode resolve headers
#define STM32F0
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>

#include "samp.h"
#include "uart.h"
#include "HK.h"

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable clocks to subsystems
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1); // Data output to TM
    rcc_periph_clock_enable(RCC_TIM1);   // CNV pulse to external ADC
    rcc_periph_clock_enable(RCC_SPI1);   // SPI interface to 16-bit ADC
    rcc_periph_clock_enable(RCC_ADC);    // HK ADC
    rcc_periph_clock_enable(RCC_DAC1);   // sweep
}

static void gpio_setup(void)
{
    uint16_t pins;

    // ------ PORT A
    // CNV  PA8  AF2 TIM1_CH1
    pins = GPIO8;
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
    gpio_set_af(GPIOA, GPIO_AF2, pins);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);

    // SPI_1: AF0  SCK-PA5 MISO-PA6
    pins = GPIO5 | GPIO6;
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
    gpio_set_af(GPIOA, GPIO_AF0, pins);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, pins);

    // USART_1: AF1  TX-PA9 RX-PA10 (PUPD_NONE required for RX 5V tolerance)
    pins = GPIO9 | GPIO10;
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
    gpio_set_af(GPIOA, GPIO_AF1, pins);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

    // DEBUG pins (only on Discovery EVAL board)
    // USER push button PA0
    // LED: Blue PC8 Green PC9
}

int main(void)
{
    clock_setup();
    gpio_setup();

    StartSAMP(); // Start SAMP interrupt

    // TIM1 sample interrupt routine does everything.
    // Main loop handles cmd for bench testing. Never sees input in flight
}
