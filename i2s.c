
#include "ultranet.h"
#include "hardware/dma.h"

uint i2s_pio_init(PIO pio) {
    /*
    uint32_t clk = clock_get_hz(clk_sys);
    float ratio = clk/24576000.0;
    uint div = (uint)ratio;
    ratio -= div;
    uint frac = ratio*256;
    printf("Clock: %d [%f] -> %d %d\n", clk, ratio, div, frac);
    */
    return pio_add_program(pio, &i2s_program);    // load i2c output code once for all state machines 
}

void i2s_sm_init(PIO pio, uint offset, uint sm, uint pin)
{

    pio_sm_config c = i2s_program_get_default_config(offset);  // get default structure
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);  // set base+3 pins to output
    sm_config_set_clkdiv(&c, AUDIV);
    sm_config_set_out_pins (&c, pin, 3);                    // out pin range base and count
    sm_config_set_sideset_pins (&c, pin+1);                 // sideset pin range base
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // configure 8 depth output fifo
    sm_config_set_out_shift(&c, false, false, 32);          // set shift left, no autpull for out FIFO
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
}

uint32_t dma_init(PIO pio, uint sm, volatile uint32_t* target) {
    printf("Initialising DMA for state machine %d\n", sm);
    uint32_t pio_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config pio_dma_chan_config = dma_channel_get_default_config(pio_dma_chan);
    // transfer 32 bits
    channel_config_set_transfer_data_size(&pio_dma_chan_config, DMA_SIZE_32);
    // increment read address
    channel_config_set_read_increment(&pio_dma_chan_config, true);
    channel_config_set_write_increment(&pio_dma_chan_config, false);
    channel_config_set_ring(&pio_dma_chan_config, false, 3);
    // Transfer when PIO SM TX FIFO has space
    channel_config_set_dreq(&pio_dma_chan_config, pio_get_dreq(pio, sm, true));
    
    dma_channel_configure(
        pio_dma_chan,
        &pio_dma_chan_config,
        &pio->txf[sm],
        target,
        dma_encode_endless_transfer_count(), // TODO: check if this is the right way to do this
        false
    );
    return pio_dma_chan;
}

void i2s_connect_channels(PIO pio, uint offset, uint sm, uint pins, volatile int32_t* target) {
    i2s_sm_init(pio, offset, sm, pins);        

    uint32_t pio_dma_chan = dma_init(pio, sm, (volatile uint32_t*)target);

    dma_channel_start(pio_dma_chan);
    pio_sm_set_enabled(pio, sm, true);
}
