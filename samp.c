
// ---------- Sampling Interrupt --------------
// Sample and send one data packet

// TIM1 generates hardware CNV pulse at 2ms period
// RPA is sampled at CNV edge and held by external ADC
//  Sample SWPMON internal ADC
//  Output next sweep value to DAC
//  Read external ADC via SPI
//  Sample HK internal ADC

#ifndef STM32F0 // helps Xcode resolve headers
#define STM32F0
#endif

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

#include "uart.h"
#include "samp.h"
#include "HK.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define SYNC 0xBB

// **** GLOBALS used by sample ISR ***
extern long pakcount;
extern long sweeptable[];
extern long step;
extern long nsteps;
extern long flip;
int counter = 15;
extern uint32_t period; // step period (for setup)
unsigned binary = 0;
// *****  TIM1 ISR interrupts at rising edge of CNV *******************
void tim1_cc_isr(void)
{
    SWP_convert(); // sample sweep before changing it
                   // while (!(ADC_ISR(ADC1) & ADC_ISR_EOS)); // wait for end of sampling

    // update sweep DAC
    dac_load_data_buffer_single(DAC1, sweeptable[step], DAC_ALIGN_RIGHT12, DAC_CHANNEL2);

    if ((step <= 0) || step >= (nsteps - 1))
        flip = -flip; // up-down sweep
    step += flip;     // next step

    // Packet format:  SYNC,SEQ, RPAm,RPAl, SWPm,SWPl, HKm,HKl
    // putch(SYNC);            // Send start packet
    // putch(pakcount & 0xFF); // Send SEQ byte
    // Assume external ADC conversion is complete by now (needs 900ns)
    SPI1_DR = 0x1; // read out external ADC via SPI
    while (!(SPI1_SR & SPI_SR_RXNE))
        ; // wait for SPI transfer complete

    ////////////////VVV NEW CODE VVV//////////////////////////////////

    // old raw: raw value that we have been using in the past, straight from SPI1_DR.
    ////////////////VVV prints old raw in binary VVV//////////////////////////////////
    // unsigned n = SPI1_DR;
    // unsigned i;
    // int zero = 0;
    // int one = 0;
    // // decimal --> binary function
    // for (i = 1 << 15; i > 0; i = i / 2)
    // {
    //     (n & i) ? one++ : zero++;
    // }
    // if (one > zero)
    // {
    //     putch('0');
    // }
    // else
    // {
    //     putch('1');
    // }
    // if (counter >= 15)
    // {
    //     putch('\r');
    //     putch('\n');
    //     putch('\n');
    //     counter = 0;
    // }
    // counter++;
    ////////////////^^^ prints old raw in binary ^^^//////////////////////////////////

    ////////////////VVV prints old raw in decimal VVV//////////////////////////////////
    // unsigned n = SPI1_DR;
    // char str[16];
    // sprintf(str, "%d", n);
    // for (int i = 0; i < 5; i++)
    // {
    //     putch(str[i]);
    // }
    // putch('\r');
    // putch('\n');
    // putch('\n');
    ////////////////^^^prints old raw in decimal ^^^//////////////////////////////////

    // new raw: I noticed that the 16 bits of the old raw were always all 1s or all 0s.
    //          So, I took those 16 bits and interpreted them as 1 bit.
    //          16 bits of new raw = 256 bits of old raw.
    ////////////////VVV prints new raw in binary VVV//////////////////////////////////
    // unsigned n = SPI1_DR;
    // if (n > 32766)
    // {
    //     putch('1');
    // }
    // else
    // {
    //     putch('0');
    // }
    // if (counter <= 0)
    // {
    //     putch('\r');
    //     putch('\n');
    //     putch('\n');
    //     counter = 15;
    //     binary = 0;
    // }
    // counter--;
    ////////////////^^^ prints new raw in binary ^^^//////////////////////////////////

    ////////////////VVV prints new raw in decimal VVV//////////////////////////////////
    // unsigned n = SPI1_DR;
    // if (n > 32766)
    // {
    //     // putch('1');
    //     int power = 1;
    //     for (int i = 0; i < counter; i++) {
    //         power = power * 2;
    //     }
    //     binary += power;
    // }
    // if (counter <= 0)
    // {
    //     char str[16];
    //     sprintf(str, "%d", binary);
    //     for (int i = 0; i < 16; i++)
    //     {
    //         putch(str[i]);
    //     }
    //     putch('\r');
    //     putch('\n');
    //     putch('\n');
    //     counter = 15;
    //     binary = 0;
    // }
    // counter--;
    ////////////////^^^ prints new raw in decimal ^^^//////////////////////////////////

    ////////////////^^^ NEW CODE ^^^//////////////////////////////////

    // putswab(SPI1_DR); // Send RPA sample (SPI readout has bytes swapped)

    // extern ADC readout completed. Force CNV low
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW; // (assumes all other bits are zero)
    TIM1_CCMR1 = TIM_CCMR1_OC1M_TOGGLE;

    // putwd(SWP_read()); // Send SWP sample from prev  SWP_convert()

    // putwd(HK_samp()); // Send one of 8 different HK vals muxed by lsbs of pakcount

    pakcount++;

    TIM1_SR = ~TIM_SR_CC1IF; // clear interrupt
}
// ********************************************************************

// Set up SPI and timer for sampling
// Assumes GPIO bits are already set up
void StartSAMP(void)
{
    // Set up SPI interface to external ADC and DAC
    spi_reset(SPI1);
    // 10MHz. CPOL=0 clock normally lo. CPHA=1 clock on falling edge.

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);

    spi_set_data_size(SPI1, 16);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_enable(SPI1);

    // Set up TIMER 1 to generate CNV pulse
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP); // FIXME THIS IS NEW
    // timer_reset(TIM1);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE); // TIM1_CH1 CNV
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, period >> 1);
    timer_set_prescaler(TIM1, 7); // need prescale TIM1 is 16-bit
    timer_set_period(TIM1, period - 1);

    // interrupt on CNV leading edge
    timer_generate_event(TIM1, TIM_EGR_CC1G | TIM_EGR_TG);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    TIM_CR1(TIM1) |= TIM_CR1_CEN; // Start timer
}
