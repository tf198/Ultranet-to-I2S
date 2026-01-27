
/*
* Header file for the Ultranet project
* contains all #includes and #defines for the 
* project C files
*
* Ultranet provides 16 separate audio channels (or 8 stereo pairs) via two twisted pairs in a
* CAT5/CAT6 cable. Each pair carries 8 channels, and is termed a "stream" in this project.
* This module decodes a single Ultranet stream, but can select between both available streams.
*
* The module can be used either as an all-channels decoder (8 simultaneous channels) or as a
* specific decoder for selected channels. An optional binary selector switch sets the offset for
* the order of output channels onto board pins, such that a single output pair (I2S or PWM) can
* be selected to be any of the 8 available stereo pairs from the two Ultranet input streams.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/binary_info.h"

#include "build/ultranet.pio.h"     // derived automatically from the "ultranet.pio" source file

// strings for inclusion in binary info (for query by picotool)
#define DESCRIPTION "Single Ultranet stream input (sw selected), 4xI2S stereo, 8xPWM mono"
#define VERSION "1.2"

// conditional compilation switches for hardware options
#define DEBUG                    // enable debug code DEBUG DEBUG DEBUG
#define LOGGING
#define MCLK                        // Enable MCLK clock for I2S devices

#define SAMPLERATE 48000
#define BITRATE 16
#define TEST_SIGNAL 440

#define CLOCKSPEED ultranet_cs     // Required clockspeed (from pio)
#define AUDIV ultranet_cy           // Audio divider for pio timing (from pio))
// Ultranet input and MCLK state machines use pio0
#define UNETL_PIN 2                 // ultranet low stream (1-8) input pin
#define UNETH_PIN 3                 // ultranet high stream (9-16) input pin
#define UNET_PIN UNETL_PIN          // ultranet default input pin
#define UNET_PIO pio0               // PIO module to use for Ultranet input
#define UNET_SM 0                   // state machine to use for Ultranet input
#define UNET_DISCARD 1000000        // discard this many samples to sync
#ifdef MCLK                         // if we want an I2S MCLK clock
    #define MCLK_PIN 4              // I2S Master Clock Pin (if used)
    #define MCLK_PIO pio0           // state machine for I2S master clock
    #define MCLK_SM 1               // state machine for I2S master clock
#endif // MCLK
// I2S outputs use second pio (pio1), four I2S outputs, 3 pins each
#define I2S_PIO pio1                // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 6                 // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 9                 // base for I2S output pins (3 pins starting point)
#define I2S3_PINS 12                 // base for I2S output pins (3 pins starting point)
#define I2S4_PINS 15                // base for I2S output pins (3 pins starting point)
// Selector binary switch (3 pole)
#define SELECTOR_SW_BASE 5         // base pin (switch is 3-pin, base+2) switches to ground
#define SW_COMM_LOW                 // switch common pin(s) are connected to 0v
// #define SW_COMM_HIGH             // switch common pin(s) are connected to 3.3v

// for PWM analog audio outputs
#define PIN_PWM_1A 18               // A channel of PWM slice (left audio)
#define PIN_PWM_1B 19               // B channel of PWM slice (right audio)
#define PIN_PWM_2A 20               // A channel of PWM slice (left audio)
#define PIN_PWM_2B 21               // B channel of PWM slice (right audio)
#define PIN_PWM_3A 26               // A channel of PWM slice (left audio)
#define PIN_PWM_3B 27               // B channel of PWM slice (right audio)
#define PIN_PWM_4A 22               // A channel of PWM slice (left audio)
#define PIN_PWM_4B 28               // B channel of PWM slice (right audio)

// these need to be "volatile" otherwise the compiler optimises them out!
extern volatile int32_t samples[8]; // array of samples read from Ultranet stream

extern void ultranet_gpio_init(void);           // setup gpio for selector
extern void ultranet_pio_init(PIO, uint, uint); // setup state machine to decode frames
extern void ultranet_decode_forever(void);      // starts decoding ultranet frames and writing them to samples array
extern void ultranet_print_stats(void);
extern void ultranet_analyse_samples(void);

extern volatile uint32_t ultranet_samples_received;
extern volatile float ultranet_samples_dropped;

extern void i2s_connect_channels(PIO, uint, uint, volatile uint32_t*);

extern uint get_selector(void);     // return selector switch state in low 3 bits

#ifdef TEST_SIGNAL
extern int32_t* generate_test_signal(uint16_t, uint16_t);
#endif // TEST_SIGNAL