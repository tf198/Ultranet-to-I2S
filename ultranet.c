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
#include "led.h"

static const uint32_t SYNC_B = 0b1111;
static const uint32_t SYNC_M = 0b1011;
static const uint32_t SYNC_W = 0b0111;

volatile uint32_t samples[8];   // array of samples read from Ultranet stream

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
#ifdef PICO_LED
    pico_led_init();
#endif // PICO_LED
}

// state machine init functions (used to be defined in <prog>.pio file)
void ultranet_pio_init(PIO pio, uint sm, uint pin)
{
    printf("GPIO Pin: %d\n", pin);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);                               // set ultranet pin as input
    gpio_set_pulls(pin, true, false);                       // set pullup on ultranet pin
    uint offset = pio_add_program(pio, &ultranet_program);  // load code into pio mem
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);          // configure 8 depth input fifo
    sm_config_set_in_pins (&c, pin);                        // input pin range base
    sm_config_set_in_shift(&c, true, false, 32);            // shift_right, no autopush, 32bit
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
    pio_sm_set_jmp_pin(pio, UNET_SM, pin);                  // specify pin for jmp instructions
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running
}

volatile uint32_t led_state = 0xFFFFFFFF;   
#ifdef WS2812                           
void ws2812_pio_init(PIO pio, uint sm, uint pin)            // Set up PIO SM for ws2812 LED module
{
    uint offset = pio_add_program(pio, &ws2812_program);    // PIO program shares code space with UNET and MCLK
    pio_gpio_init(pio, pin);                                // Set up GPIO pin for PIO...
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // ...output

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);           // set shift direction RIGHT, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // use 8 deep TX FIFO

#define CYCLES_PER_BIT ((ws2812_T1)+(ws2812_T2)+(ws2812_T3)) // constants defined in .pio source file
    float div = clock_get_hz(clk_sys) / (800000 * CYCLES_PER_BIT);  // ws2812 needs precise 800KHz timing
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);                      // Set ws2812 PIO state machine running
    led_state = BLACK;                                      // initialise WS2812 LED to all off
    pio_sm_put(pio, sm, led_state);                         // Clear all LED colours to off
}
#endif // WS2812

/*
* Timer callback for periodically turning off Ultranet detected LED
*  Outputs current state (as turned on by Ultranet stream code) then clears flag
* If no Ultranet stream received, LED will turn off at next alarm tick
*/
int64_t alarm_callback(alarm_id_t id, __unused void *repeatptr)
{
#ifdef WS2812
    put_pixel(led_state);                                   // Output current sate of led_state flag to LED
#endif // WS2812
#ifdef PICO_LED
    pico_set_led((led_state & ~LED_STREAM_MASK) > 0);       // led_state has been set by Ultranet stream code
#endif // PICO_LED
    led_state = led_state & LED_STREAM_MASK;                // Zero out the stream LED colour bits
    return *(const uint32_t*)repeatptr;                     // return value is repeat time
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
#ifdef WS2812
    bi_decl(bi_1pin_with_name(WS2812_PIN, "WS2812 NeoPixel LED"));
#endif // WS2812
#ifdef PICO_LED
    bi_decl(bi_1pin_with_name(1, "PICO board normal LED enabled"));
#endif // PICO_LED
    set_core1_info();                                       // info for pins used by core1
}

int popcount(uint32_t i)
{
     i = i - ((i >> 1) & 0x55555555);        // add pairs of bits
     i = (i & 0x33333333) + ((i >> 2) & 0x33333333);  // quads
     i = (i + (i >> 4)) & 0x0F0F0F0F;        // groups of 8
     i *= 0x01010101;                        // horizontal sum of bytes
     return  i >> 24;               // return just that top byte (after truncating to 32-bit even when int is wider than uint32_t)
}

int8_t sample_channel(uint32_t sample) {
    int8_t channel = 0;

    // check if pio completely filled the buffer
    if (sample & 1 != 1) return -1;

    // check parity bit
    uint8_t bitcount = popcount(sample & 0x0ffffffe);
    uint8_t parity = (bitcount + 1) & 1;
    if (sample >> 31 != parity) return -2; 

    channel = ((sample >> 4) & 0b11) * 2;
    switch (sample & 0b1111) {
        case SYNC_B:
            return -1; // Ultranet doesn't use SYNC_B
        case SYNC_M: // left or A channel
            return channel;
        case SYNC_W: // right or B channel
            return channel+1;
        default:
            return -3;
    }
}

void print_sample(uint32_t sample) {

    int8_t status = sample_channel(sample);
    printf("%08x %d ", sample, status);

    for (int i = sizeof(uint32_t) * 8 - 1; i >= 0; i--) {
        printf("%d", (sample >> i) & 1);
    }
    printf("\n");
}

void analyse_samples() {
    volatile uint32_t sample;
    uint64_t ts;
    int8_t c;
    uint32_t channels[8];
    uint32_t invalid = 0;
    uint32_t total = 0;

    // dump some samples so we can see what we are dealing with
    for (int i=0; i<200; i++) {
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);
        print_sample(sample);
    }
    

    for (int i=0; i<8; i++) channels[i] = 0;

    ts = time_us_64();
    while(true) {
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);
        c = sample_channel(sample);
        total++;
        if (c < 0) {
            invalid++;
        } else {
            channels[c]++;
        }

        
        if (time_us_64() - ts > 1000000) {
            for (int i=0; i<8; i++) {
                printf("%d: %6lu ", i, channels[i]);
                channels[i] = 0;
            }
            printf("X: %6lu/%7lu (%lu%%)\n", invalid, total, invalid*100/total);
            invalid = total = 0;
            ts = time_us_64();
        }
    }
}


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

int main()
{
    volatile uint32_t sample;                               // temp store for sample read from Ultranet stream
    const uint64_t repeat_us = STREAM_LED_RESET;            // Repeat time period for alarm to clear Ultranet stream LED
    uint selector;                                          // Selector switch state
    
    int32_t dropped = 0;
    int32_t total = 0;
    int8_t channel;

    set_binary_info();                                      // info for querying by picotool                                    // initialise SDK libraries and interfaces
    if (!set_sys_clock_khz(CLOCKSPEED,true)) {
        printf("Failed to set clock\n");
        return 1;
    };                    // set cpu clock frequency
    stdio_init_all();   

#ifdef DEBUG
    sleep_ms(5000);                                         // allow time for USB serial to connect
#else
    sleep_ms(500);                                          // allow time for clocks etc. to settle
#endif // DEBUG

#ifdef WS2812
    ws2812_pio_init(WS2812_PIO, WS2812_SM, WS2812_PIN);     // ws2812 output pio state machine
#endif // WS2812

    printf("Clock: %dkhz (%d,%d)\n", clock_get_hz(clk_sys)/1000, ultranet_cy, ultranet_mp);
    ultranet_gpio_init();                                   // initialise required GPIO pins

    add_alarm_in_us(repeat_us, alarm_callback, (void*)&repeat_us, false);  // start timer for stream LED blanking

    selector = get_selector();                              // read selector switch once at boot time
    printf("Selector = %d\n", selector);

    if(selector & 0b100)                                    // Most significant switch bit selects Ultranet input stream pin
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETH_PIN);    // initialise and start ultranet state machine
    else
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETL_PIN);    // initialise and start ultranet state machine

    multicore_launch_core1(core1_entry);                    // start core 1

    sleep_ms(100);                                          // wait for core1 to start
    puts("FINISHED setting everything up\n");

    // discard first 200 samples
    for(int count=0; count<200; count++)  sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);    // get frame word from Ultranet FIFO

#ifdef DEBUG
    analyse_samples();
#endif

    while (true)
    {
        // get next ultranet frame
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);
        channel = sample_channel(sample);
        total++;
        if (channel < 0) {
            dropped++;
            led_state = 0x0;
        } else {
            // move 22 bits of audio into MSBs
            samples[channel] = (sample << 4) & 0xFFFFFC00;
            led_state = 0xFFFFFFFF;
        }
        
        if(total % 5000000 == 0) {
            printf("Dropped: %lu/%lu [%lu%%]\n", dropped, total, dropped*100/total);
            dropped = total = 0;
        }
    }
}
