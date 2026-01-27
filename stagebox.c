#include "ultranet.h"
#include "pico/multicore.h"

int main()
{
    uint selector;

    //set_binary_info();                                      // info for querying by picotool                                    // initialise SDK libraries and interfaces
    if (!set_sys_clock_khz(CLOCKSPEED,true)) {
        printf("Failed to set clock\n");
        return 1;
    };
    stdio_init_all();   

#ifdef DEBUG
    sleep_ms(5000);                                         // allow time for USB serial to connect
#else
    sleep_ms(1000);                                          // allow time for clocks etc. to settle
#endif // DEBUG

    ultranet_gpio_init();                                   // initialise required GPIO pins
    cyw43_arch_init(); // TODO: Figure out how to get rid of this

    selector = get_selector();                              // read selector switch once at boot time
#ifdef LOGGING
    printf("Clock: %dkhz (%d,%d)\n", clock_get_hz(clk_sys)/1000, ultranet_cy, ultranet_mp);
    printf("Selector = %d\n", selector);
#endif

    if(selector & 0b100)                                    // Most significant switch bit selects Ultranet input stream pin
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETH_PIN);    // initialise and start ultranet state machine
    else
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETL_PIN);    // initialise and start ultranet state machine

    // start DMA transfer of memory address to i2s
    i2s_connect_channels(I2S_PIO, 0, I2S1_PINS, &samples[0]);


    sleep_ms(100);                                          // wait for core1 to start
#ifdef LOGGING
    puts("FINISHED setting everything up\n");
#endif


#ifdef DEBUG
    analyse_samples();
#endif

    multicore_launch_core1(ultranet_decode_forever);
    
    while(true) {
        sleep_ms(1000);
        printf("Dropped: %0.2f%%\n", ultranet_samples_dropped*100);
    }

}
