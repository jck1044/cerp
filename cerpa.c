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

// **** GLOBALS used by sample ISR ***
long pakcount;
long sweeptable[32];
long step;
long flip;
uint8_t ropt; // sweep chan (or debug readback option)

// sweep options
long rampstart; // starting DAC value
long nsteps;    // total number of steps. Max 32
long nsteps1;   // number of steps at 1st slope
long rampbump1; // 1st slope DAC bump
long rampbump2; // 2nd slope DAC bump

uint32_t period; // step period

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

    // HK Analog in (PA4 DAC_OUT also uses analog pin mode)
    pins = GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO7;
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pins);

    // ------ PORT B
    // HK Analog in PB0
    pins = GPIO0;
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pins);

    // Set unused pins to input with pulldown
    pins = GPIO11 | GPIO12 | GPIO15;
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, pins);
    pins = GPIO1 | GPIO4 | GPIO5 | GPIO6 | GPIO7;
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, pins);

    // DEBUG pins (only on Discovery EVAL board)
    // USER push button PA0
    // LED: Blue PC8 Green PC9
}

static void SetupDAC(void)
{
    dac_disable(DAC1, DAC_CHANNEL1);
    dac_disable_waveform_generation(DAC1, DAC_CHANNEL1);
    dac_enable(DAC1, DAC_CHANNEL1);
    dac_buffer_enable(DAC1, DAC_CHANNEL1);
    dac_trigger_disable(DAC1, DAC_CHANNEL1);
}

// Fill in sweeptable[] with 2 slope ramp
// sweep parms are globals that can be changed by cmds
// no checks for valid parm value
static void SetupSweep(void)
{
    long ramp;

    step = 0;
    ramp = rampstart;
    while (step < nsteps1)
    {
        sweeptable[step] = ramp;
        ramp += rampbump1;
        step++;
    }

    while (step < nsteps)
    {
        sweeptable[step] = ramp;
        ramp += rampbump2;
        step++;
    }

    flip = -1;
    step = 0;
}

// Bench test commands
// Commands are only for bench testing and are not used in flight
// Disabled at startup. Must be initially enabled by a '*'
// The enable is for extra insurance against recieving a garbage char
// Stays enabled unless an invalid character is seen
// Typical sequence sends value and then cmd
// 8-bit  param: v <lsb> <cmd>
// 16-bit param: v <msb> m v <lsb> l <cmd>
// no checking is done here on values. Bad parms can crash
static long isval;
static unsigned char val8;
static long setval;
static long cmdenable;

static void DoCmd(long c)
{
    if (c == '*')
        cmdenable = 1;
    if (!cmdenable)
        return;

    if (isval) // prev cmd was 'v' this char is binary value
    {
        val8 = c;
        isval = 0;
    }
    else if (c == 'v')
        isval = 1; // next char is binary value
    else if (c == 'm')
        setval = val8 << 8; // load msb of setval
    else if (c == 'l')
        setval |= val8; // load lsb of setval

    else if (c == 'p') // set  period
    {
        period = (unsigned)setval;
        timer_set_oc_value(TIM1, TIM_OC1, period >> 1);
        timer_set_period(TIM1, period - 1);
    }
    // ramp readback option: 8-SWPMON,4-DACMON
    // 0 to 18 ADC, 19-SWPTAB,20-REFCAL,21-CAL30C,22-CAL110C
    else if (c == 'o')
    {
        ropt = val8;
    }
    else if (c == 's')
    {
        rampstart = setval;
        SetupSweep();
    }
    else if (c == 'n')
    {
        nsteps1 = val8;
        if (nsteps1 > nsteps)
            nsteps = nsteps1;
        SetupSweep();
    }
    else if (c == 'N')
    {
        nsteps = val8;
        if (nsteps1 > nsteps)
            nsteps1 = nsteps;
        SetupSweep();
    }
    else if (c == 'b')
    {
        rampbump1 = setval;
        SetupSweep();
    }
    else if (c == 'B')
    {
        rampbump2 = setval;
        SetupSweep();
    }
    else if (c != '*')
        cmdenable = 0;
}

int main(void)
{
    period = 12000; // 2ms

    clock_setup();
    gpio_setup();

    SetupUART();
    SetupHK();
    SetupDAC();

    // sweep ramp options
    nsteps = 24;
    nsteps1 = 21;
    rampstart = 124;
    rampbump1 = 62;
    rampbump2 = 1300;

    cmdenable = 0; // require initial enable cmd

    ropt = 8; // set SWP readout option to SWPMON

    SetupSweep();

    StartSAMP(); // Start SAMP interrupt

    // TIM1 sample interrupt routine does everything.
    // Main loop handles cmd for bench testing. Never sees input in flight
    while (1)
    {
        // for (int j = 0; j < 800000; j++)
        //     __asm__("nop");
        // if (USART_ISR(USART1) & USART_ISR_RXNE) // if UART input char avail, process cmd
        // {
        //     DoCmd(USART_RDR(USART1));
        // }
    }
}
