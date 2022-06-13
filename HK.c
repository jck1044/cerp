
//	------- Housekeeping ADC sampling routines -----------

#ifndef STM32F0 // helps Xcode resolve headers
#define STM32F0
#endif

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

#include "HK.h"

// ST provided factory calibration values @ 3.3V
// Can be read by debug cmds
#define ST_VREFINT_CAL		MMIO16(0x1FFFF7BA)
#define ST_TSENSE_CAL1_30C	MMIO16(0x1FFFF7B8)
#define ST_TSENSE_CAL2_110C	MMIO16(0x1FFFF7C2)

// HK channels
#define BUSV_CHAN 1
#define BUSI_CHAN 2
#define V2V5_CHAN 3
#define DACMON_CHAN 4
#define ENDMON_CHAN 0
#define V15V_CHAN 7
#define SWPMON_CHAN 8
#define TEMPMON_CHAN 16
#define VREFMON_CHAN 17

extern long pakcount;
extern long sweeptable[];
extern long step;
extern uint8_t ropt;  // sweep readback debug option


// Single chan ADC conversion with wait
static long adc_samp(long chan)
{	
	ADC_CHSELR(ADC1) = 1<<chan;
    ADC_CR(ADC1) |= ADC_CR_ADSTART;;
	while (!(ADC_ISR(ADC1) & ADC_ISR_EOC)); // wait for conversion
        
	return(ADC_DR(ADC1));
}

// Start conversion for SWPMON
// No-wait allows overlap with other tasks
// Normal operation samples SWPMON ropt=8. Debug commands can change ropt
void SWP_convert(void)
{
	ADC_CHSELR(ADC1) = 1<<ropt;
    ADC_CR(ADC1) |= ADC_CR_ADSTART;;
}

// Read previously converted ADC value.
// Called about 10us after ADSTART so no need to wait.
// Debug options: ropt= 0 to 18 ADC, SWPTAB,REFCAL,CAL30C,CAL110C
long SWP_read(void)
{
    long val;
    
	//while (!adc_eoc(ADC1)); // wait for ADC.
        
    if(ropt <= 18) return(ADC_DR(ADC1));    // normal operation
    
    // debug options substitute other values for SWPMON
    if(ropt == 19) val = sweeptable[step];
    else if(ropt == 20) val = ST_VREFINT_CAL;
    else if(ropt == 21) val = ST_TSENSE_CAL1_30C;
    else if(ropt == 22) val = ST_TSENSE_CAL2_110C;
    else val = -1;
    
    return(val);
}

// 8 HK values multiplexed by lsbs of pakcount
static long hkchans[] = {BUSV_CHAN,BUSI_CHAN,V2V5_CHAN,ENDMON_CHAN,V15V_CHAN,TEMPMON_CHAN,VREFMON_CHAN};

long HK_samp(void)
{
    long mux,chan,val;
    
    mux = pakcount&0x7;
    if(mux == 7) return pakcount>>3;    // mux 7 returns msbs of pakcount
    
    // other mux: sample hkchans in sequence
    chan = hkchans[mux];
    adc_samp(chan);    // pre sample to  settle input mux.
    
    val = adc_samp(chan);   // use 2nd sample
        
    return val;
}

// Set up for single chan software trig conversion.
// Assumes GPIO bits are already set up
void SetupHK(void)
{    
    adc_power_off(ADC1);
    adc_calibrate(ADC1);    // requires adc disabled
    while(adc_is_calibrating(ADC1));
    
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_055DOT5);// works best for SWPMON
	adc_power_on(ADC1);
        
	adc_enable_temperature_sensor();
    adc_enable_vrefint();
    
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2); // use sync clock for no jitter
	adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
	adc_set_right_aligned(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_single_conversion_mode(ADC1);
}

