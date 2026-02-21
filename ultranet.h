
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
#include "hardware/clocks.h"
#include "pico/binary_info.h"

#include "build/ultranet.pio.h"     // derived automatically from the "ultranet.pio" source file

#define SAMPLERATE 48000
#define BITRATE 16
#define TEST_SIGNAL 440

#define CLOCKSPEED ultranet_cs     // Required clockspeed (from pio)
#define AUDIV ultranet_cy           // Audio divider for pio timing (from pio))

struct UltranetStream {
    int32_t samples[16] __attribute__((aligned(2*sizeof(int32_t))));

    uint32_t received[16];
    uint32_t errors[5];
    uint32_t start_ts; 
    char status[200];
};

extern void ultranet_pio_init(PIO, uint, uint); // setup state machine to decode frames
extern void ultranet_decode_forever(struct UltranetStream*, PIO, uint);      // starts decoding ultranet frames and writing them to samples array
extern void ultranet_print_stats(void);
extern void ultranet_dump_samples(int, PIO, uint);

extern void i2s_connect_channels(PIO, uint, uint, volatile uint32_t*);
