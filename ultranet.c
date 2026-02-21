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

// state machine init functions (used to be defined in <prog>.pio file)
void ultranet_pio_init(PIO pio, uint sm, uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);                               // set ultranet pin as input
    gpio_set_pulls(pin, true, false);                       // set pullup on ultranet pin
    uint offset = pio_add_program(pio, &ultranet_program);  // load code into pio mem
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure

    //sm_config_set_clkdiv(&c, 150000000.0/147456000.0);    // no need to set fractional scaling if running at 172MHz
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);          // configure 8 depth input fifo
    sm_config_set_in_pins (&c, pin);                        // input pin range base
    sm_config_set_in_shift(&c, true, false, 32);            // shift_right, no autopush, 32bit
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
    pio_gpio_init(pio, pin);  
    pio_sm_set_jmp_pin(pio, sm, pin);                  // specify pin for jmp instructions
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running
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

void reset_stats(struct UltranetStream *stream) {
    memset((void*)stream->received, 0, 16*sizeof(uint32_t));
    memset((void*)stream->errors  , 0, 5*sizeof(uint32_t));
    stream->start_ts = time_us_64();
}

void ultranet_generate_stats(struct UltranetStream *stream) {
    char buffer[100];

    sprintf(stream->status, "|");
    for (int i=0; i<8; i++) {
        sprintf(buffer, " %2d: %-3lu", i, 48000-stream->received[i]);
        strcat(stream->status, buffer);
    }
    //sprintf(&ultranet_status[70], " | ");
    strcat(stream->status, " |");
    for (int i=1; i<5; i++) {
        sprintf(buffer, " %2d: %-3lu", i*-1, stream->errors[i]);
        strcat(stream->status, buffer);
    }
    sprintf(buffer, "| %5.3f%% | %4dns |", stream->errors[0]*100.0/384000, time_us_64()-stream->start_ts-1000000);
    strcat(stream->status, buffer);
    reset_stats(stream);
}

void print_sample(uint32_t sample) {

    int8_t status = sample_channel(sample);
    printf("%08x %2d ", sample, status);

    for (int i = sizeof(uint32_t) * 8 - 1; i >= 0; i--) {
        printf("%d", (sample >> i) & 1);
    }
    printf("\n");
}

void ultranet_dump_samples(int count, PIO pio, uint sm) {
    uint32_t *test_samples = (uint32_t *)malloc(count * sizeof(uint32_t));

    // dump some samples so we can see what we are dealing with
    for (int i=0; i<count; i++) {
        test_samples[i] = pio_sm_get_blocking(pio, sm);
    }
    for (int i=0; i<count; i++) {
        print_sample(test_samples[i]);
    }
    free(test_samples);
}



void ultranet_decode_forever(struct UltranetStream *stream, PIO pio, uint sm)
{
    volatile uint32_t sample;                               // temp store for sample read from Ultranet stream
    int8_t channel;

    uint32_t samples_received = 0;

    reset_stats(stream);

    while (true)
    {
        // get next ultranet frame
        sample = pio_sm_get_blocking(pio, sm);
        channel = sample_channel(sample);
        samples_received++;
        if (channel < 0) {
            stream->errors[0]++;
            stream->errors[channel*-1]++;
        } else {
            // move 22 bits of audio into MSBs
            stream->received[channel]++;
            stream->samples[channel] = (int32_t)((sample << 4) & 0xFFFFFC00);
        }

        if(samples_received == 384000) {
            ultranet_generate_stats(stream);
            samples_received = 0;
        }
    }
}
