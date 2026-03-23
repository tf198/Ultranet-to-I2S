#include "ultranet.h"
#include "pico/multicore.h"

// strings for inclusion in binary info (for query by picotool)
#define DESCRIPTION "16 Channel, 4 bus ultranet mixer with up to 4 I2S outputs"
#define VERSION "1.2"

// conditional compilation switches for hardware options
//#define DEBUG                    // enable debug code DEBUG DEBUG DEBUG
#define LOGGING
#define DEBUG_PIN 10

// Ultranet input and MCLK state machines use pio0
#define UNET_L_PIN 2                 // ultranet low stream (1-8) input pin
#define UNET_H_PIN 3                 // ultranet high stream (9-16) input pin
#define UNET_PIO pio0               // PIO module to use for Ultranet input
#define UNET_SM_L 0                   // state machine to use for Ultranet low stream input
#define UNET_SM_H 1                   // state machine to use for Ultranet high stream input

#define I2S_PIO pio1                // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 4                 // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 7                // base for I2S output pins (3 pins starting point)
//#define I2S3_PINS 16                 // base for I2S output pins (3 pins starting point)
//#define I2S4_PINS 19                // base for I2S output pins (3 pins starting point)

//volatile struct UltranetStream stream; // __attribute__((aligned(2*sizeof(int32_t))));
struct UltranetStream *stream;
volatile int32_t samples[16] __attribute__((aligned(2*sizeof(int32_t))));
volatile int32_t buses[4] __attribute__((aligned(2*sizeof(int32_t))));
volatile float mixes[4][16];

// Embedded binary information (for picotool interrogation of programmed device)
void set_binary_info(void)
{
    bi_decl(bi_program_description(DESCRIPTION));           // Description field for embedded identification 
    bi_decl(bi_program_version_string(VERSION));            // Version field for embedded identification
    bi_decl(bi_2pins_with_names(UNET_L_PIN, "Ultranet Low (1-8) Stream Input", UNET_H_PIN, "Ultranet High (9-16) Input"));
}

void core1_entry() {
    ultranet_decode_and_mix(stream, samples, mixes);
}

void read_line(char* buf, int len) {
  char c;
  int i = 0;

  //printf("read_line(): ");
  while ((c=getchar()) != '\r' && c != '\n' && c != EOF) {
    //printf("%02x ", c);
    printf("%c", c);
    buf[i++] = c;
    if (i>=len-1) break;
  }
  buf[i] = 0;
  //printf("%02x\n", c);
  printf("\n");
}

int handle_mix_fader(char* cmd, char* val, char* result) {
  int mix, channel;
  float level;

  if (sscanf(cmd, "ch/%d/mix/%d/fader", &channel, &mix) != 2) return -1;
  
  if (channel < 1 || channel > 16) return -2;
  if (mix < 1 || mix > 4) return -3;

  if (val) {
    if (sscanf(val, "%f", &level) != 1) {
      return -1;
    };
    mixes[mix-1][channel-1] = level;
  } else {
    sprintf(result, "%0.3f", mixes[mix-1][channel-1]);
  }
  return 0;
}

int handle_bus_level_read(char *cmd) {}

char* handle_osc_write(char* cmd, char* val) {
  char *result = (char*)malloc(sizeof(char)*20);
  if (handle_mix_fader(cmd, val, result) == 0) return "Updated bus level";
  free(result);
  return "Unhandled write command";
}

char* handle_osc_read(char* cmd) {
  char *result = (char*)malloc(sizeof(char)*20);
  if (strcmp(cmd, "xinfo") == 0) return "foo";
  if (strcmp(cmd, "status") == 0) return (char*) ultranet_stats.status;
  if (handle_mix_fader(cmd, NULL, result) == 0) return result; 
  free(result);
  return "Unhandled read command";
}

void parse_osc(char *buf) {
  char cmd[20];
  char val[20];
  char *result;
  switch (sscanf(buf, "/%s %s", cmd, val)) {
    case 1:
      result = handle_osc_read(cmd);
      break;
    case 2:
      result = handle_osc_write(cmd, val);
      break;
    default:
      printf("Error: bad osc command [%s]\n", buf);
      return;
  }
  printf("Result: %s\n", result);
}


int main()
{
    set_binary_info();                                      // info for querying by picotool                                    // initialise SDK libraries and interfaces
    if (CLOCKSPEED != 150000) {
        if (!set_sys_clock_khz(CLOCKSPEED,true)) {
            printf("Failed to set clock\n");
            return 1;
        };
    }
    stdio_init_all();   
    sleep_ms(1000);                                          // allow time for clocks etc. to settle

#ifdef LOGGING
    printf("Clock: %dkhz (%d,%d)\n", clock_get_hz(clk_sys)/1000, ultranet_cy, ultranet_mp);
#endif

#ifdef DEBUG_PIN
    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
#endif

    stream = ultranet_init(UNET_PIO);

    // initialise the mix levels 
    for (int bus=0; bus<4; bus++) {
      for(int ch=0; ch<16; ch++) {
        mixes[bus][ch] = 0.8;
      }
    }
    memset((void*)mixes, 0, 16*4*sizeof(int32_t));

    ultranet_sm_init(stream, UNET_L_PIN);    // initialise and start ultranet state machine 0
    ultranet_sm_init(stream, UNET_H_PIN);    // initialise and start ultranet state machine 1

    // start DMA transfer of memory address to i2s
    uint i2s_offset = i2s_pio_init(I2S_PIO);
    i2s_connect_channels(I2S_PIO, i2s_offset, 0, I2S1_PINS, &samples[0]);
    i2s_connect_channels(I2S_PIO, i2s_offset, 1, I2S2_PINS, &samples[2]);
#ifdef I2S3_PINS
    i2s_connect_channels(I2S_PIO, i2s_offset, 2, I2S3_PINS, &buses[4]);
#endif
#ifdef I2S4_PINS
    i2s_connect_channels(I2S_PIO, i2s_offset, 3, I2S4_PINS, &buses[6]);
#endif


#ifdef DEBUG
    ultranet_dump_samples(10, stream);
#endif

    multicore_launch_core1(core1_entry);
    sleep_ms(100);                                          // wait for core1 to start
#ifdef LOGGING
    puts("FINISHED setting everything up\n");
#endif
    

    char buf[1024];
    int mix, ch;
    float val;
    while(true) {
      printf("> ");
      read_line(buf, 1000);
      //parse_osc(buf);
      //printf("Buf: %s\n", buf);
      int result = sscanf(buf, "%d %d %f", &mix, &ch, &val);
      switch(result) {
        case 3:
          if (mix < 1 || mix > 4) break;
          if (ch < 1 || mix > 16) break;
          if (val < 0 || val > 1.2) break;
          mixes[mix-1][ch-1] = val;
          printf("Mix %d, channel %d set to %f\n", mix, ch, val);
          break;
        case 2:
          if (mix < 1 || mix > 4) break;
          if (ch < 1 || mix > 16) break;
          printf("Mix %d, channel %d: %f\n", mix, ch, mixes[mix-1][ch-1]);
          break;
        case 1:
          if (mix < 1 || mix > 4) break;
          printf("Mix %d\n", mix);
          for (int i=0; i<16; i++) {
            printf("%02x ", (int)(mixes[mix-1][i]*250));
          }
          printf("\n");
          break;
        default:
          puts((char*)ultranet_stats.status);

      }
      /*
      sleep_ms(1000);
      puts(stream->status);
      */
    }

}
