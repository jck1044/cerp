
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

#define SYNC 0xBB

// **** GLOBALS used by sample ISR ***
extern long pakcount;
extern long sweeptable[];
extern long step;
extern long nsteps;
extern long flip;

extern uint32_t period;    // step period (for setup)

// *****  TIM1 ISR interrupts at rising edge of CNV *******************
void tim1_cc_isr(void)
{	
    SWP_convert(); // sample sweep before changing it
	//while (!(ADC_ISR(ADC1) & ADC_ISR_EOS)); // wait for end of sampling
    
    // update sweep DAC
    dac_load_data_buffer_single(sweeptable[step], RIGHT12, CHANNEL_1);
    
    if((step<=0) || step >= (nsteps-1)) flip = -flip;   // up-down sweep
    step += flip;   // next step
    
    // Packet format:  SYNC,SEQ, RPAm,RPAl, SWPm,SWPl, HKm,HKl
    putch(SYNC);            // Send start packet
    putch(pakcount&0xFF);   // Send SEQ byte
    
    // Assume external ADC conversion is complete by now (needs 900ns)
    SPI1_DR=0x1;                        // read out external ADC via SPI
    while (!(SPI1_SR & SPI_SR_RXNE));   // wait for SPI transfer complete
    
    putswab(SPI1_DR);       // Send RPA sample (SPI readout has bytes swapped)
    
    // extern ADC readout completed. Force CNV low
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW;      // (assumes all other bits are zero)
    TIM1_CCMR1 = TIM_CCMR1_OC1M_TOGGLE;
    
	putwd(SWP_read());      // Send SWP sample from prev  SWP_convert()
    
	putwd(HK_samp());       // Send one of 8 different HK vals muxed by lsbs of pakcount
    
    pakcount++;
    
    TIM1_SR = ~TIM_SR_CC1IF;   // clear interrupt
}
// ********************************************************************


// Set up SPI and timer for sampling
// Assumes GPIO bits are already set up
void StartSAMP(void)
{
    // Set up SPI interface to external ADC and DAC
    spi_reset(SPI1);
    // 10MHz. CPOL=0 clock normally lo. CPHA=1 clock on falling edge.
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, 0, SPI_CR1_MSBFIRST);
    spi_set_data_size(SPI1, 16);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_enable(SPI1);
    
    // Set up TIMER 1 to generate CNV pulse
    timer_reset(TIM1);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE);     // TIM1_CH1 CNV
    timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, period>>1);
    timer_set_prescaler(TIM1, 7);       // need prescale TIM1 is 16-bit
    timer_set_period(TIM1, period-1);
        
    // interrupt on CNV leading edge
    timer_generate_event(TIM1, TIM_EGR_CC1G | TIM_EGR_TG);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
            
    TIM_CR1(TIM1) |= TIM_CR1_CEN;     // Start timer
}


