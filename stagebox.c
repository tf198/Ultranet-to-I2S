#include "ultranet.h"
#include "pico/multicore.h"


// strings for inclusion in binary info (for query by picotool)
#define DESCRIPTION "Single Ultranet stream input (sw selected), 4xI2S stereo, 8xPWM mono"
#define VERSION "1.2"

// conditional compilation switches for hardware options
//#define DEBUG                    // enable debug code DEBUG DEBUG DEBUG
//#define LOGGING

// Selector binary switch (3 pole)
// #define SW_COMM_HIGH             // switch common pin(s) are connected to 3.3v
#define SELECTOR_SW_BASE 12         // base pin (switch is 3-pin, base+2) switches to ground
#define SW_COMM_LOW                 // switch common pin(s) are connected to 0v

#define LED_ORANGE 14
#define LED_GREEN 15

// Ultranet input and MCLK state machines use pio0
#define UNETL_PIN 2                 // ultranet low stream (1-8) input pin
#define UNETH_PIN 3                 // ultranet high stream (9-16) input pin
#define UNET_PIN UNETH_PIN          // ultranet default input pin
#define UNET_PIO pio0               // PIO module to use for Ultranet input
#define UNET_SM 0                   // state machine to use for Ultranet input

#define I2S_PIO pio1                // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 4                 // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 7                // base for I2S output pins (3 pins starting point)
//#define I2S3_PINS 12                 // base for I2S output pins (3 pins starting point)
//#define I2S4_PINS 15                // base for I2S output pins (3 pins starting point)

volatile struct UltranetStream *stream; // __attribute__((aligned(2*sizeof(int32_t))));
volatile int32_t samples[16] __attribute__((aligned(2*sizeof(int32_t))));

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

void stagebox_gpio_init(void)
{
    for(int i=0; i<2; i++) {
        gpio_init(SELECTOR_SW_BASE+i);
        gpio_set_dir(SELECTOR_SW_BASE+i, false);
#ifdef SW_COMM_LOW                                          // switch common can be 0v or +3.3v
        gpio_pull_up(SELECTOR_SW_BASE+i);                                // for switch common to +3.3v
#else
        gpio_pull_down(SELECTOR_SW_BASE+i);                              // for switch common to +3.3v
#endif // SW_COMM_LOW
    }
}

// read selector switch and return uint with switch positions in the 3 LSBs
uint get_selector(void)
{
    static uint sw_mask = 0b11 << SELECTOR_SW_BASE;        // Mask for selecting only switch bits from all GPIOs

    printf("Switch: %d %d\n", gpio_get(SELECTOR_SW_BASE), gpio_get(SELECTOR_SW_BASE+1));
    return 0;

    printf("%x\n", gpio_get_all());
#ifdef SW_COMM_LOW                                          // sw pulls gpio pins low, so invert sw result
    return ((~gpio_get_all()) & sw_mask) >> SELECTOR_SW_BASE;
#else                                                       // sw pulls gpio pins high, so non-inverted result
    return (gpio_get_all() & sw_mask) >> SELECTOR_SW_BASE;
#endif // SW_COMM_LOW
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

void core1_entry() {
    ultranet_decode_forever(stream, samples);
}

int main()
{
    uint selector;

    set_binary_info();                                      // info for querying by picotool                                    // initialise SDK libraries and interfaces
    if (CLOCKSPEED != 150000) {
        if (!set_sys_clock_khz(CLOCKSPEED,true)) {
            printf("Failed to set clock\n");
            return 1;
        };
    }
    stdio_init_all();   
    sleep_ms(1000);                                          // allow time for clocks etc. to settle

    stagebox_gpio_init();                                   // initialise required GPIO pins

    selector = get_selector();                              // read selector switch once at boot time
    
    printf("Clock: %dkhz (%d,%d)\n", clock_get_hz(clk_sys)/1000, ultranet_cy, ultranet_mp);
    printf("Selector = %d\n", selector);

    gpio_init(LED_ORANGE);
    gpio_set_dir(LED_ORANGE, GPIO_OUT);
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);

    stream = ultranet_init(UNET_PIO);

    int base_ch = 1;
    if(!gpio_get(SELECTOR_SW_BASE)) {                                   // Most significant switch bit selects Ultranet input stream pin
        printf("Selected stream 9-16\n");
        ultranet_sm_init(stream, UNETH_PIN);
        base_ch = 9;
    } else {
        printf("Selected stream 1-8\n");
        ultranet_sm_init(stream, UNETL_PIN);
    }

    int ch_offset = (!gpio_get(SELECTOR_SW_BASE+1)) ? 4 : 0;
    printf("Channels: %d-%d\n", base_ch+ch_offset, base_ch+ch_offset+3);

    // start DMA transfer of memory address to i2s
    uint i2s_offset = i2s_pio_init(I2S_PIO);
    i2s_connect_channels(I2S_PIO, i2s_offset, 0, I2S1_PINS, &samples[ch_offset]);
    i2s_connect_channels(I2S_PIO, i2s_offset, 1, I2S2_PINS, &samples[ch_offset+2]);
#ifdef I2S3_PINS
    i2s_connect_channels(I2S_PIO, 1, I2S3_PINS, &(stream.samples[ch_offset+4]));
#endif
#ifdef I2S4_PINS
    i2s_connect_channels(I2S_PIO, 1, I2S4_PINS, &(stream.samples[ch_offset+6]));
#endif


#ifdef DEBUG
    ultranet_dump_samples(20, stream);
#endif

    puts("Starting decoder");
    multicore_launch_core1(core1_entry);
    sleep_ms(100);                                          // wait for core1 to start
    puts("FINISHED setting everything up\n");
    gpio_put(LED_GREEN, true);
    
    while(true) {
        sleep_ms(750);
        gpio_put(LED_ORANGE, true);
        sleep_ms(250);
        gpio_put(LED_ORANGE, false);
        #ifdef LOGGING
        puts((char*)stream->status);
        #endif
    }

}
