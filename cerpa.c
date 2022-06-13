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

#include "string.h"
#include "stdio.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>

#define SYNC 0xBB

// **** GLOBALS used by sample ISR ***
long pakcount;
long sweeptable[32];
long step;
long flip;
char uartbuf[64];
long qnext, qlast;
uint8_t ropt; // sweep chan (or debug readback option)

// sweep options
long rampstart; // starting DAC value
long nsteps;    // total number of steps. Max 32
long nsteps1;   // number of steps at 1st slope
long rampbump1; // 1st slope DAC bump
long rampbump2; // 2nd slope DAC bump

uint32_t period; // step period

// *****  TIM1 ISR interrupts at rising edge of CNV *******************
void tim1_cc_isr(void)
{
    char tim1_buf[20];
    int tim1_buf_len;
    snprintf(tim1_buf, sizeof(tim1_buf), "----TIM1----");
    tim1_buf_len = strlen(tim1_buf);
    for (int j = 0; j < tim1_buf_len; j++)
        usart_send_blocking(USART1, tim1_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    SWP_convert(); // sample sweep before changing it
                   // while (!(ADC_ISR(ADC1) & ADC_ISR_EOS)); // wait for end of sampling

    // update sweep DAC
    dac_load_data_buffer_single(DAC1, sweeptable[step], DAC_ALIGN_RIGHT12, DAC_CHANNEL1);

    if ((step <= 0) || step >= (nsteps - 1))
        flip = -flip; // up-down sweep
    step += flip;     // next step

    char sync_buf[20];
    int sync_buf_len;
    snprintf(sync_buf, sizeof(sync_buf), "SYNC: %i", SYNC);
    sync_buf_len = strlen(sync_buf);
    for (int j = 0; j < sync_buf_len; j++)
        usart_send_blocking(USART1, sync_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    // Packet format:  SYNC,SEQ, RPAm,RPAl, SWPm,SWPl, HKm,HKl
    putch(SYNC); // Send start packet

    char pakcount_buf[20];
    int pakcount_buf_len;
    snprintf(pakcount_buf, sizeof(pakcount_buf), "pakcount: %li", pakcount & 0xFF);
    pakcount_buf_len = strlen(pakcount_buf);
    for (int j = 0; j < pakcount_buf_len; j++)
        usart_send_blocking(USART1, pakcount_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    putch(pakcount & 0xFF); // Send SEQ byte
    // Assume external ADC conversion is complete by now (needs 900ns)
    SPI1_DR = 0x1; // read out external ADC via SPI
    while (!(SPI1_SR & SPI_SR_RXNE))
        ; // wait for SPI transfer complete

    // swab part 1
    char spi1_buf[20];
    int spi1_buf_len;
    snprintf(spi1_buf, sizeof(spi1_buf), "SPI_DR 1: %li", SPI1_DR & 0xFF);
    spi1_buf_len = strlen(spi1_buf);
    for (int j = 0; j < spi1_buf_len; j++)
        usart_send_blocking(USART1, spi1_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    // swab part 2
    char spi2_buf[20];
    int spi2_buf_len;
    snprintf(spi2_buf, sizeof(spi2_buf), "SPI_DR 2: %li", (SPI1_DR & 0xFF00) >> 8);
    spi2_buf_len = strlen(spi2_buf);
    for (int j = 0; j < spi2_buf_len; j++)
        usart_send_blocking(USART1, spi2_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    putswab(SPI1_DR); // Send RPA sample (SPI readout has bytes swapped)

    // extern ADC readout completed. Force CNV low
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW; // (assumes all other bits are zero)
    TIM1_CCMR1 = TIM_CCMR1_OC1M_TOGGLE;

    int swp_read = SWP_read();
    // wd part 1
    char swp1_buf[20];
    int swp1_buf_len;
    snprintf(swp1_buf, sizeof(swp1_buf), "SWP_read 1: %i", (swp_read & 0xFF00) >> 8);
    swp1_buf_len = strlen(swp1_buf);
    for (int j = 0; j < swp1_buf_len; j++)
        usart_send_blocking(USART1, swp1_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    // wd part 2
    char swp2_buf[20];
    int swp2_buf_len;
    snprintf(swp2_buf, sizeof(swp2_buf), "SWP_read 2: %i", swp_read & 0xFF);
    swp2_buf_len = strlen(swp2_buf);
    for (int j = 0; j < swp2_buf_len; j++)
        usart_send_blocking(USART1, swp2_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    putwd(swp_read); // Send SWP sample from prev  SWP_convert()

    int hk_read = HK_samp();
    // wd part 1
    char hk1_buf[20];
    int hk1_buf_len;
    snprintf(hk1_buf, sizeof(hk1_buf), "HK_samp 1: %i", (hk_read & 0xFF00) >> 8);
    hk1_buf_len = strlen(hk1_buf);
    for (int j = 0; j < hk1_buf_len; j++)
        usart_send_blocking(USART1, hk1_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    // wd part 2
    char hk2_buf[20];
    int hk2_buf_len;
    snprintf(hk2_buf, sizeof(hk2_buf), "HK_samp 2: %i", hk_read & 0xFF);
    hk2_buf_len = strlen(hk2_buf);
    for (int j = 0; j < hk2_buf_len; j++)
        usart_send_blocking(USART1, hk2_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    putwd(HK_samp()); // Send one of 8 different HK vals muxed by lsbs of pakcount

    pakcount++;

    TIM1_SR = ~TIM_SR_CC1IF; // clear interrupt
}
// ********************************************************************

void SWP_convert(void)
{
    ADC_CHSELR(ADC1) = 1 << ropt;
    ADC_CR(ADC1) |= ADC_CR_ADSTART;
    ;
}

// Read previously converted ADC value.
// Called about 10us after ADSTART so no need to wait.
// Debug options: ropt= 0 to 18 ADC, SWPTAB,REFCAL,CAL30C,CAL110C
long SWP_read(void)
{
    long val;

    // while (!adc_eoc(ADC1)); // wait for ADC.

    if (ropt <= 18)
        return (ADC_DR(ADC1)); // normal operation

    // debug options substitute other values for SWPMON
    if (ropt == 19)
        val = sweeptable[step];
    else if (ropt == 20)
        val = ST_VREFINT_CAL;
    else if (ropt == 21)
        val = ST_TSENSE_CAL1_30C;
    else if (ropt == 22)
        val = ST_TSENSE_CAL2_110C;
    else
        val = -1;

    return (val);
}

// Single chan ADC conversion with wait
static long adc_samp(long chan)
{
    ADC_CHSELR(ADC1) = 1 << chan;
    ADC_CR(ADC1) |= ADC_CR_ADSTART;
    ;
    while (!(ADC_ISR(ADC1) & ADC_ISR_EOC))
        ; // wait for conversion

    return (ADC_DR(ADC1));
}

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
// 8 HK values multiplexed by lsbs of pakcount
static long hkchans[] = {BUSV_CHAN, BUSI_CHAN, V2V5_CHAN, ENDMON_CHAN, V15V_CHAN, TEMPMON_CHAN, VREFMON_CHAN};

long HK_samp(void)
{
    long mux, chan, val;

    mux = pakcount & 0x7;
    if (mux == 7)
        return pakcount >> 3; // mux 7 returns msbs of pakcount

    // other mux: sample hkchans in sequence
    chan = hkchans[mux];
    adc_samp(chan); // pre sample to  settle input mux.

    val = adc_samp(chan); // use 2nd sample

    return val;
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

static void clock_setup(void)
{
    char clock_buf[20];
    int clock_buf_len;
    snprintf(clock_buf, sizeof(clock_buf), "SETTING UP CLOCK");
    clock_buf_len = strlen(clock_buf);
    for (int j = 0; j < clock_buf_len; j++)
        usart_send_blocking(USART1, clock_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable clocks to subsystems
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    // rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1); // Data output to TM
    rcc_periph_clock_enable(RCC_TIM1);   // CNV pulse to external ADC
    rcc_periph_clock_enable(RCC_SPI1);   // SPI interface to 16-bit ADC
    rcc_periph_clock_enable(RCC_ADC);    // HK ADC
    rcc_periph_clock_enable(RCC_DAC1);   // sweep
}

static void gpio_setup(void)
{

    char gpio_buf[20];
    int gpio_buf_len;
    snprintf(gpio_buf, sizeof(gpio_buf), "SETTING UP GPIO");
    gpio_buf_len = strlen(gpio_buf);
    for (int j = 0; j < gpio_buf_len; j++)
        usart_send_blocking(USART1, gpio_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

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
    char dac_buf[20];
    int dac_buf_len;
    snprintf(dac_buf, sizeof(dac_buf), "SETTING UP DAC");
    dac_buf_len = strlen(dac_buf);
    for (int j = 0; j < dac_buf_len; j++)
        usart_send_blocking(USART1, dac_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    dac_disable(DAC1, DAC_CHANNEL1);
    dac_disable_waveform_generation(DAC1, DAC_CHANNEL1);
    dac_enable(DAC1, DAC_CHANNEL1);
    dac_buffer_enable(DAC1, DAC_CHANNEL1);
    dac_trigger_disable(DAC1, DAC_CHANNEL1);
}

// Buffered single char output. Put in mem buf and return. ISR sends it
void putch(int ch)
{
    // char putch_buf[20];
    // int putch_buf_len;
    // snprintf(putch_buf, sizeof(putch_buf), "CHAR: %i", ch);
    // putch_buf_len = strlen(putch_buf);
    // for (int j = 0; j < putch_buf_len; j++)
    //     usart_send_blocking(USART1, putch_buf[j]);
    // usart_send_blocking(USART1, '\r');
    // usart_send_blocking(USART1, '\n');

    usart_disable_tx_interrupt(USART1); // prevent collision ISR access to qlast
    uartbuf[qlast++] = ch;
    qlast &= 0x3f;
    usart_enable_tx_interrupt(USART1);
}

// Set up SPI and timer for sampling
// Assumes GPIO bits are already set up
void StartSAMP(void)
{
    char samp_buf[20];
    int samp_buf_len;
    snprintf(samp_buf, sizeof(samp_buf), "STARTING SAMP");
    samp_buf_len = strlen(samp_buf);
    for (int j = 0; j < samp_buf_len; j++)
        usart_send_blocking(USART1, samp_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    // Set up SPI interface to external ADC and DAC
    spi_reset(SPI1);
    // 10MHz. CPOL=0 clock normally lo. CPHA=1 clock on falling edge.
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
    spi_set_data_size(SPI1, 16);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_enable(SPI1);

    // Set up TIMER 1 to generate CNV pulse
    // timer_reset(TIM1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE); // TIM1_CH1 CNV
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, period >> 1);
    timer_set_prescaler(TIM1, 47); // need prescale TIM1 is 16-bit
    timer_set_period(TIM1, period - 1);

    // interrupt on CNV leading edge
    timer_generate_event(TIM1, TIM_EGR_CC1G | TIM_EGR_TG);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    TIM_CR1(TIM1) |= TIM_CR1_CEN; // Start timer
}

// Fill in sweeptable[] with 2 slope ramp
// sweep parms are globals that can be changed by cmds
// no checks for valid parm value
static void SetupSweep(void)
{

    char sweep_buf[20];
    int sweep_buf_len;
    snprintf(sweep_buf, sizeof(sweep_buf), "SETTING UP SWEEP");
    sweep_buf_len = strlen(sweep_buf);
    for (int j = 0; j < sweep_buf_len; j++)
        usart_send_blocking(USART1, sweep_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

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

void SetupUART(void)
{

    char uart_buf[20];
    int uart_buf_len;
    snprintf(uart_buf, sizeof(uart_buf), "SETTING UP UART");
    uart_buf_len = strlen(uart_buf);
    for (int j = 0; j < uart_buf_len; j++)
        usart_send_blocking(USART1, uart_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    nvic_enable_irq(NVIC_USART1_IRQ);

    // UART at 57600baud
    usart_set_baudrate(USART1, 112500);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    qnext = 0;
    qlast = 0;

    usart_enable(USART1);

    putch('!'); // send a char to show it was reset
}

// Set up for single chan software trig conversion.
// Assumes GPIO bits are already set up
void SetupHK(void)
{
    char hk_buf[20];
    int hk_buf_len;
    snprintf(hk_buf, sizeof(hk_buf), "SETTING UP HK");
    hk_buf_len = strlen(hk_buf);
    for (int j = 0; j < hk_buf_len; j++)
        usart_send_blocking(USART1, hk_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    adc_power_off(ADC1);
    adc_calibrate(ADC1); // requires adc disabled
    while (adc_is_calibrating(ADC1))
        ;

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_055DOT5); // works best for SWPMON
    adc_power_on(ADC1);

    adc_enable_temperature_sensor();
    adc_enable_vrefint();

    adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2); // use sync clock for no jitter
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_right_aligned(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_single_conversion_mode(ADC1);
}

#define PORT_LED GPIOC
#define PIN_LED GPIO8

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
    char cmd_buf[20];
    int cmd_buf_len;
    snprintf(cmd_buf, sizeof(cmd_buf), "DOING CMD");
    cmd_buf_len = strlen(cmd_buf);
    for (int j = 0; j < cmd_buf_len; j++)
        usart_send_blocking(USART1, cmd_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    if (c == '*')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        cmdenable = 1;
    }
    if (!cmdenable)
    { // ENTERS
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        return;
    }

    if (isval) // prev cmd was 'v' this char is binary value
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        val8 = c;
        isval = 0;
    }
    else if (c == 'v')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        isval = 1; // next char is binary value
    }
    else if (c == 'm')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        setval = val8 << 8; // load msb of setval
    }
    else if (c == 'l')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        setval |= val8; // load lsb of setval
    }
    else if (c == 'p') // set  period
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        period = (unsigned)setval;
        timer_set_oc_value(TIM1, TIM_OC1, period >> 1);
        timer_set_period(TIM1, period - 1);
    }
    // ramp readback option: 8-SWPMON,4-DACMON
    // 0 to 18 ADC, 19-SWPTAB,20-REFCAL,21-CAL30C,22-CAL110C
    else if (c == 'o')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        ropt = val8;
    }
    else if (c == 's')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        rampstart = setval;
        SetupSweep();
    }
    else if (c == 'n')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        nsteps1 = val8;
        if (nsteps1 > nsteps)
        {
            gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

            nsteps = nsteps1;
        }
        SetupSweep();
    }
    else if (c == 'N')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        nsteps = val8;
        if (nsteps1 > nsteps)
        {
            gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

            nsteps1 = nsteps;
        }
        SetupSweep();
    }
    else if (c == 'b')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        rampbump1 = setval;
        SetupSweep();
    }
    else if (c == 'B')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        rampbump2 = setval;
        SetupSweep();
    }
    else if (c != '*')
    {
        // gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

        cmdenable = 0;
    }
}

int main(void)
{
    char main_buf[20];
    int main_buf_len;
    snprintf(main_buf, sizeof(main_buf), "STARTING MAIN");
    main_buf_len = strlen(main_buf);
    for (int j = 0; j < main_buf_len; j++)
        usart_send_blocking(USART1, main_buf[j]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');
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
    // float voltage;
    // char voltage_buf[20];
    // int voltage_buf_len;
    int k = 0;
    while (1)
    {
        tim1_cc_isr();
        char while_buf[20];
        int while_buf_len;
        snprintf(while_buf, sizeof(while_buf), "IN WHILE LOOP");
        while_buf_len = strlen(while_buf);
        for (int j = 0; j < while_buf_len; j++)
            usart_send_blocking(USART1, while_buf[j]);
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');

        /** Interrupt & status register (USART_ISR) */
        // USART_ISR_RXNE: Read data register not empty

        // if (USART_ISR(USART1) & USART_ISR_RXNE) // if UART input char avail, process cmd
        if (USART_ISR(USART1)) // if UART input char avail, process cmd
        {                      // never gets in here

            gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

            spi_send(SPI1, (uint8_t)k);
            k++;

            // uint16_t raw = spi_read(SPI1);
            // snprintf(voltage_buf, sizeof(voltage_buf), "Raw value: %f\r\n", raw);
            // voltage_buf_len = strlen(voltage_buf);

            // for (int j = 0; j < voltage_buf_len; j++)
            // {
            //     usart_send_blocking(USART1, voltage_buf[j]);
            // }

            /** Receive data register (USART_RDR) */
            // DoCmd(USART_RDR(USART1));
            DoCmd('*');
        }

        for (int i = 0; i < 8000000; i++)
        { /* Wait a bit. */
            __asm__("nop");
        }
    }
}
