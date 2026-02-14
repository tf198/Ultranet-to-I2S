/*
* Behringer Ultranet decoder
* Reads Ultranet stream, decoding 8 x audio channels
* outputs pairs of channels as i2S stereo streams
*
* Ultranet bit rate = 12.288MHz (8x32bit samples at 48khz)
* Ultranet biphase clock rate = 2 x 12.288MHz = 24.576MHz
* We need pio module to sample at 7 x incoming clock rate
* Ideal system clock therefore = 7 x 24.576 = 172.032MHz
* We set cpu clock frequency as close to 172032KHz as possible
* but can only set certain numbers of MHz, so 172000khz
*
* OR we need pio module to sample at 8 x incoming clock rate
* Ideal system clock therefore = 8 x 24.576 = 196.608MHz
* We set cpu clock frequency as close to 196608KHz as possible
* but can only set certain numbers of MHz, so 196500khz
*
* Ultranet audio sample depth is in fact 22 bits, not 24
* So we mask the 2 LSBs when reading words from Ultranet stream
*
* Use PICO LED to indicate if we have a framing error
* ie we get out of sync with the 8 subframes of the ultranet stream
* if we don't detect start of frame sync in the right place, 
* toggle the LED - so frequency of frame errors can be seen
*
* Use multicore to compensate for difference in speed between
* Ultranet input stream (clocked from source) and I2S output
* stream (clocked from this pico). Array of samples is filled
* from the Ulranet stream by core0, then read out to I2S using
* core1, asynchronously from core0.
*/

#include "ultranet.h"

static const uint32_t SYNC_B = 0b1111;
static const uint32_t SYNC_M = 0b1011;
static const uint32_t SYNC_W = 0b0111;

// array for raw ultranet samples
// need to align so we can use the DMA ring buffer
volatile int32_t samples[8] __attribute__((aligned(2*sizeof(int32_t))));   // array of samples read from Ultranet stream

uint32_t samples_c[8] = {0};
uint32_t samples_d[5] = {0};
uint64_t samples_ts = 0;
volatile uint32_t ultranet_samples_received = 0;
volatile float ultranet_samples_dropped = 0;
char ultranet_status[200];

void ultranet_gpio_init(void)
{
    int count;
    for(count=SELECTOR_SW_BASE; count < (SELECTOR_SW_BASE+3);count++)
    {
        gpio_init(count);
#ifdef SW_COMM_LOW                                          // switch common can be 0v or +3.3v
        gpio_pull_up(count);                                // for switch common to +3.3v
#else
        gpio_pull_down(count);                              // for switch common to +3.3v
#endif // SW_COMM_LOW
    }
}

// state machine init functions (used to be defined in <prog>.pio file)
void ultranet_pio_init(PIO pio, uint sm, uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);                               // set ultranet pin as input
    gpio_set_pulls(pin, true, false);                       // set pullup on ultranet pin
    uint offset = pio_add_program(pio, &ultranet_program);  // load code into pio mem
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure

    //const uint32_t div_int = 1; //SPDIF_RX_SYS_CLK_FREQ / SPDIF_RX_PIO_CLK_FREQ;
    //const uint8_t  div_frac8 = (uint8_t) (((uint64_t) SPDIF_RX_SYS_CLK_FREQ * 256) / SPDIF_RX_PIO_CLK_FREQ - 256);
    //printf("Set fractional scaling: %d/%d\n", div_int, div_frac8);
    //sm_config_set_clkdiv(&c, 150000000.0/147456000.0);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);          // configure 8 depth input fifo
    sm_config_set_in_pins (&c, pin);                        // input pin range base
    sm_config_set_in_shift(&c, true, false, 32);            // shift_right, no autopush, 32bit
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
    pio_gpio_init(pio, pin);  
    pio_sm_set_jmp_pin(pio, UNET_SM, pin);                  // specify pin for jmp instructions
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running
}

// Embedded binary information (for picotool interrogation of programmed device)
void set_binary_info(void)
{
    bi_decl(bi_program_description(DESCRIPTION));           // Description field for embedded identification 
    bi_decl(bi_program_version_string(VERSION));            // Version field for embedded identification
#ifdef UNETH_PIN
    bi_decl(bi_2pins_with_names(UNETL_PIN, "Ultranet Low (1-8) Stream Input", UNETH_PIN, "Ultranet High (9-16) Input"));
#else
    bi_decl(bi_1pin_with_name(UNET_PIN, "Ultranet Stream Input"));
#endif // UNETH_PIN
#ifdef MCLK
    bi_decl(bi_1pin_with_name(MCLK_PIN, "I2S MCLK Output"));
#endif // MCLK
    //set_core1_info();                                       // info for pins used by core1
}

/**
 * Count number of bits set in a 32bit integer.
 * https://stackoverflow.com/questions/109023/count-the-number-of-set-bits-in-a-32-bit-integer
 */
int popcount(uint32_t i)
{
     i = i - ((i >> 1) & 0x55555555);        // add pairs of bits
     i = (i & 0x33333333) + ((i >> 2) & 0x33333333);  // quads
     i = (i + (i >> 4)) & 0x0F0F0F0F;        // groups of 8
     i *= 0x01010101;                        // horizontal sum of bytes
     return  i >> 24;               // return just that top byte (after truncating to 32-bit even when int is wider than uint32_t)
}

/**
 * Check the given sample is a valid ultranet subframe and returns the channel number
 * https://blog.thestaticturtle.fr/ultranet-adventures-part-2/
 */
int8_t sample_channel(uint32_t sample) {
    int8_t channel = 0;

    // check if pio completely filled the buffer - subframes always have LSB set
    if ((sample & 1) == 0) return -1;

    // check parity
    if (popcount(sample & 0xFFFFFFF0) % 2 != 0) return -2;

    // check play bit - pretty sure this is always set
    if ((sample >> 28) & 1 == 0) return -3;

    // 8 channels L/R with identifier in bits 4 & 5
    channel = ((sample >> 4) & 0b11) * 2;
    switch (sample & 0b1111) {
        case SYNC_B:
            return channel; // Start of ultranet frame on channel A
        case SYNC_M: // left or A channel
            return channel;
        case SYNC_W: // right or B channel
            return channel+1;
        default:
            return -4;
    }
}

void reset_stats() {
    ultranet_samples_dropped = (float)samples_d[0]/ultranet_samples_received; // TODO - rolling average

    memset((void*)samples_c, 0, 8*sizeof(uint32_t));
    memset((void*)samples_d, 0, 5*sizeof(uint32_t));
    ultranet_samples_received = 0;
    samples_ts = time_us_64();
}

void ultranet_generate_stats() {
    for (int i=0; i<8; i++) {
        sprintf(&ultranet_status[i*10], "%2d: %-6lu ", i, samples_c[i]);
    }
    //sprintf(&ultranet_status[70], " | ");
    for (int i=1; i<5; i++) {
        sprintf(&ultranet_status[70+i*10], "%2d: %-6lu ", i*-1, samples_d[i]);
    }
    sprintf(&ultranet_status[121], "(%f%%) [%luns]", samples_d[0]*100.0/ultranet_samples_received, time_us_64()-samples_ts-1000000);
    reset_stats();
}

void print_sample(uint32_t sample) {

    int8_t status = sample_channel(sample);
    printf("%08x %2d ", sample, status);

    for (int i = sizeof(uint32_t) * 8 - 1; i >= 0; i--) {
        printf("%d", (sample >> i) & 1);
    }
    printf("\n");
}

void ultranet_dump_samples() {
    uint32_t test_samples[500];

    // dump some samples so we can see what we are dealing with
    for (int i=0; i<500; i++) {
        test_samples[i] = pio_sm_get_blocking(UNET_PIO, UNET_SM);
    }
    for (int i=0; i<500; i++) {
        print_sample(test_samples[i]);
    }
}

void ultranet_analyse_samples() {
    volatile uint32_t sample;
    int8_t c;

    reset_stats();
    while(true) {
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);
        c = sample_channel(sample);
        ultranet_samples_received++;
        if (c < 0) {
            samples_d[0]++;
            samples_d[c*-1]++;
            //print_sample(sample);
        } else {
            samples_c[c]++;
        }

        
        if (ultranet_samples_received == 384000) {
            ultranet_print_stats();
        }
    }
}

#ifdef TEST_SIGNAL
int32_t* generate_test_signal(uint16_t length, uint16_t bitrate) {

    int32_t* sinewave = (int32_t*)malloc(length * sizeof(int32_t));
    int32_t max_int = 0x0FFFFFFF;

    double m = (M_PI * 2) / (double)length;
    double v;
    for (int i=0; i<length; i++) {
        v = sin(i * m);
        sinewave[i] = (int)(v * max_int);
    }
    return sinewave;
}
#endif

// read selector switch and return uint with switch positions in the 3 LSBs
uint get_selector(void)
{
    static uint sw_mask = 0b111 << SELECTOR_SW_BASE;        // Mask for selecting only switch bits from all GPIOs

#ifdef SW_COMM_LOW                                          // sw pulls gpio pins low, so invert sw result
    return ((~gpio_get_all()) & sw_mask) >> SELECTOR_SW_BASE;
#else                                                       // sw pulls gpio pins high, so non-inverted result
    return (gpio_get_all() & sw_mask) >> SELECTOR_SW_BASE;
#endif // SW_COMM_LOW
}

void ultranet_decode_forever()
{
    volatile uint32_t sample;                               // temp store for sample read from Ultranet stream
    int8_t channel;

    reset_stats();

    while (true)
    {
        // get next ultranet frame
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);
        channel = sample_channel(sample);
        ultranet_samples_received++;
        if (channel < 0) {
            samples_d[0]++;
            samples_d[channel*-1]++;
        } else {
            // move 22 bits of audio into MSBs
            samples_c[channel]++;
            samples[channel] = (int32_t)((sample << 4) & 0xFFFFFC00);
        }

        if(ultranet_samples_received == 384000) {
            ultranet_generate_stats();
            //puts(ultranet_status);
        }
    }
}
