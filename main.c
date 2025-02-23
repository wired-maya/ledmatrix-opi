#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

// TODO: I'll do this like a normal lib at some point, headers and stuff

// Matrix Control Pins
#define OE 2
#define CLK 5
#define LAT 27

// Matrix Colour Pins
#define R1 19
#define R2 21
#define G1 22
#define G2 24
#define B1 20
#define B2 9

// Matrix Address Pins
#define A 8
#define B 25
#define C 7
#define D 26
#define E 10

// Display w/h
#define HEIGHT 64
#define WIDTH 64

// Information for achieving higher colour depth
#define COLOUR_DEPTH 8
#define BCM_DELAY 25 // us
#define GAMMA_CORRECTION 2.2

// Info for mmap
#define BLOCK_SIZE (4*1024)

// This library is for the ICN2037 LED driver chip
// https://olympianled.com/wp-content/uploads/2021/05/ICN2037_datasheet_EN_2017_V2.0.pdf

// Single packet of data to send
// 1 byte for faster memory access
typedef struct Packet_t {
  uint8_t r1: 1;
  uint8_t r2: 1;
  uint8_t g1: 1;
  uint8_t g2: 1;
  uint8_t b1: 1;
  uint8_t b2: 1;
} Packet;

// Colour storage struct
typedef struct Colour_t {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
} Colour;

// Framebuffer in regular format
// This is just a simplified type for the colour array
typedef Colour Framebuffer[WIDTH * HEIGHT];

// Framebuffer in format optimized for writing to panel
typedef Packet Framebuffer_Packet[WIDTH * COLOUR_DEPTH * (HEIGHT / 2)];

// Global vals to avoid passing stuff around, speeding up transfer
volatile uint32_t* gpio_map; // Always needs to be volatile

// Registers for writing each pin
// The current pinout only needs these 3
volatile uint32_t* pin_reg_1; // OE, LAT, R1, R2, G1, B1, B2, A, B, D
volatile uint32_t* pin_reg_2; // CLK, C, E
volatile uint32_t* pin_reg_3; // G2

// Store register changes here before committing
uint32_t pin_reg_1_store;
uint32_t pin_reg_2_store;
uint32_t pin_reg_3_store;

uint32_t oe_index;
uint32_t clk_index;
uint32_t lat_index;
uint32_t r1_index;
uint32_t r2_index;
uint32_t g1_index;
uint32_t g2_index;
uint32_t b1_index;
uint32_t b2_index;
uint32_t a_index;
uint32_t b_index;
uint32_t c_index;
uint32_t d_index;
uint32_t e_index;

Framebuffer_Packet current_frame;
double brightness;

// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.20 steps = 256 range = 0-255
const uint8_t gamma_lut[256] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
  1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,
  3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,
  6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,
  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
  20,  20,  21,  22,  22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,
  30,  30,  31,  32,  33,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,
  42,  43,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,
  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,
  73,  74,  75,  76,  77,  78,  79,  81,  82,  83,  84,  85,  87,  88,  89,  90,
  91,  93,  94,  95,  97,  98,  99, 100, 102, 103, 105, 106, 107, 109, 110, 111,
  113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 135,
  137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161,
  163, 165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190,
  192, 194, 196, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
  223, 225, 227, 229, 231, 234, 236, 238, 240, 242, 244, 246, 248, 251, 253, 255,
};

// Useful func for bit shifting
// Translates something like bit depth to its power
unsigned int find_binary_power(unsigned int num) {
  unsigned int power = 0;

  while (num) {
    num = num >> 1;
    power++;
  }

  return power - 1;
}

volatile uint32_t* get_pin_reg(int pin) {
  pin = wpiPinToGpio(pin);

  uint32_t mmap_seek = (((pin >> 5) * 36) + 0x10) >> 2;

  return gpio_map + mmap_seek;
}

uint32_t get_pin_index(int pin) {
  pin = wpiPinToGpio(pin);
  
  return pin & 0x1F;
}

// Initialize all necessary information for set_pin_fast
void init_fast_pinout() {
  // The address already mapped by wiringPi is not exposed
  // in the header file, so we have to map it again here
  int dev_mem = open("/dev/mem", O_RDWR | O_SYNC);

  if (dev_mem < 0) {
    printf("ERROR init_fast_pinout: Cannot open /dev/mem, please run as root\n");
    exit(1);
  }

  // Map is incorrect
  gpio_map = mmap(
    NULL, // Any address in our space will do
    BLOCK_SIZE, // Size of the map, value is from wiringPi
    PROT_READ | PROT_WRITE, // Enable r/w to /dev/mem
    MAP_SHARED, // Do not take away from wiringPi
    dev_mem, // File that is mapped
    H6_GPIO_BASE_ADDR // Base address of GPIO from wiringPi
  );

  close(dev_mem); // Not needed after map

  // Check if map is successful
  // MAP_FAILED value changes based on what the error is, so using it
  // as the error code here is valid
  if (gpio_map == MAP_FAILED) {
    printf("ERROR init_fast_pinout: mmap to /dev/mem failed\n");
    exit(1);
  }

  // Precalculate all reg addresses'
  // Needs to be updated if pinouts change, this is
  // VERY specific to the OPZ2W with the Adafruit bonnet
  pin_reg_1 = get_pin_reg(OE);
  pin_reg_2 = get_pin_reg(CLK);
  pin_reg_3 = get_pin_reg(G2);

  // Update store to be accurate to current values
  pin_reg_1_store = *pin_reg_1;
  pin_reg_2_store = *pin_reg_2;
  pin_reg_3_store = *pin_reg_3;

  // Precalculate all indices
  oe_index = get_pin_index(OE);
  clk_index = get_pin_index(CLK);
  lat_index = get_pin_index(LAT);
  r1_index = get_pin_index(R1);
  r2_index = get_pin_index(R2);
  g1_index = get_pin_index(G1);
  g2_index = get_pin_index(G2);
  b1_index = get_pin_index(B1);
  b2_index = get_pin_index(B2);
  a_index = get_pin_index(A);
  b_index = get_pin_index(B);
  c_index = get_pin_index(C);
  d_index = get_pin_index(D);
  e_index = get_pin_index(E);
}

// Sets pins faster than wiringpi can since displays require extremely fast io
// Pins still need to be initialized by wiringPi, this just shortcuts the
// digitalWrite function by taking out all the extra code
inline void set_pin_fast(volatile uint32_t* write_addr, uint32_t index, int val) {
  if (val == HIGH) *write_addr |= (1 << index);
  else *write_addr &= ~(1 << index);
}

// Update register store instead of the pins directly, requires update_pin_reg()
// be ran to see any changes
inline void set_pin_store(uint32_t* store, uint32_t index, int val) {
  if (val == HIGH) *store |= (1 << index);
  else *store &= ~(1 << index);
}

// Inline function to update pin register for ease of use
inline void update_pin_reg(volatile uint32_t* write_addr, uint32_t store) {
  *write_addr = store;
}

inline void init_pin(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); // Set initial val
}

// Uses set_pin_fast since it needs to be updated directly, and just setting
// the pins directly is faster in the case of pulsing
// Does not need to update stores since the pin goes LOW -> HIGH -> LOW during
// the pulse
inline void clk_pulse() {
  // Since you are always toggling clock pulse, you can just run this twice
  // To directly pulse the clock
  *pin_reg_2 ^= 0x1 << clk_index;
  *pin_reg_2 ^= 0x1 << clk_index;
}

inline void disable_display() {
  set_pin_store(&pin_reg_1_store, oe_index, HIGH);
}

inline void enable_display() {
  set_pin_store(&pin_reg_1_store, oe_index, LOW);
}

inline void close_latch() {
  set_pin_store(&pin_reg_1_store, lat_index, LOW);
}

inline void open_latch() {
  set_pin_store(&pin_reg_1_store, lat_index, HIGH);
}

inline void set_address(uint8_t addr) {
  // Write addr, LSB first
  set_pin_store(&pin_reg_1_store, a_index, addr & 0x1);
  set_pin_store(&pin_reg_1_store, b_index, (addr >> 1) & 0x1);
  set_pin_store(&pin_reg_2_store, c_index, (addr >> 2) & 0x1);
  set_pin_store(&pin_reg_1_store, d_index, (addr >> 3) & 0x1);
  set_pin_store(&pin_reg_2_store, e_index, (addr >> 4) & 0x1);
}

inline void set_leds(Packet packet) {
  // Updating stores which would be in cache is a lot faster
  // than always writing to the mmap directly
  set_pin_store(&pin_reg_1_store, r1_index, packet.r1);
  set_pin_store(&pin_reg_1_store, r2_index, packet.r2);
  set_pin_store(&pin_reg_1_store, g1_index, packet.g1);
  set_pin_store(&pin_reg_3_store, g2_index, packet.g2);
  set_pin_store(&pin_reg_1_store, b1_index, packet.b1);
  set_pin_store(&pin_reg_1_store, b2_index, packet.b2);
}

inline void refresh_row(uint8_t row_width, uint8_t addr, Packet* packets) {
  // Set address of row being refreshed
  set_address(addr);

  // Open latch to commit loaded data into display buffer
  // Data is from previous refresh, allowing for double buffering
  open_latch();

  // Update pins (ABCDE and LAT are just regs 1 and 2)
  update_pin_reg(pin_reg_1, pin_reg_1_store);
  update_pin_reg(pin_reg_2, pin_reg_2_store);

  // Close latch to load next data
  close_latch();

  // Update closing of latch
  update_pin_reg(pin_reg_1, pin_reg_1_store);

  for (int x = 0; x < row_width; x++) {
    set_leds(packets[x]);
    set_pin_store(&pin_reg_2_store, clk_index, HIGH);

    // Update set LEDs, most are on reg 1 but G2 is on reg 2
    update_pin_reg(pin_reg_1, pin_reg_1_store);
    update_pin_reg(pin_reg_3, pin_reg_3_store);
    update_pin_reg(pin_reg_2, pin_reg_2_store);

    set_pin_store(&pin_reg_2_store, clk_index, LOW);
    update_pin_reg(pin_reg_2, pin_reg_2_store);
  }
}

// PWM Cheat Sheet:
// uint16_t 0~65536
// CCR 0~65535 default 512 (pwmWrite)
// ARR (1~65536) - 1 default 1024 (pwmSetRange)
// DIV 1~256 default 1 (pwmSetClock)
// TONE (1~65536) - 1 def 23475 (pwmToneWrite)
// PWM duty cycle = CRR/ARR
// PWM freq = TONE/DIV
// PWM freq > 24 000 000 / (65536 * DIV)

// libgpiod does not support hardware PWM, so I'll just hack it for now

// Sets a pin to output PWM
// Panics if pin does not support PWM
void init_pwm(int pin) {
  system("gpio mode 2 pwm"); // For some reason the wiringpi thing doesn't work
}

// Set frequency, 367~65536
inline void set_pwm_freq(int pin, uint16_t freq) {
  if (freq < 367) {
    printf("WARNING set_pwm_freq: Frequency must be >366Hz, setting to 367Hz\n");
    freq = 367;
  }

  pwmToneWrite(pin, freq);
  pwmSetClock(pin, 1);
}

// Set frequency with a greater possible selection
inline void set_pwm_freq_div(int pin, uint16_t tone, uint16_t div) {
  int min_tone = 24000000 / (65536 * div);

  if (tone <= min_tone) {
    printf("ERROR set_pwm_freq_div: Tone must be >%uHz (> 24 000 000 / (65536 * DIV))\n", min_tone);
    exit(1);
  }

  pwmToneWrite(pin, tone);
  pwmSetClock(pin, div);
}

// Set duty, default range is 1024
inline void set_pwm_duty(int pin, double duty) {
  if (duty > 1.0) {
    printf("WARNING set_pwm_duty: Duty cannot be >1.0, clamping to 1.0\n");
    duty = 1.0;
  }

  pwmSetRange(pin, 1024);
  pwmWrite(pin, duty * 1024);

}

// Range is like "resolution", this allows you to change it
inline void set_pwm_duty_range(int pin, double duty, uint16_t range) {
  if (duty > 1.0) {
    printf("WARNING set_pwm_duty_range: Duty cannot be >1.0, clamping to 1.0\n");
    duty = 1.0;
  }

  pwmSetRange(pin, range);
  pwmWrite(pin, duty * range);
}

// Brightness Cheat Sheet:
// DIM 0~1 (brightness)
// D 0~1 (duty cycle)
// f (freq, usually (1~65536) - 1)
// D = DIM + f*6*10^(-4)

// Set up the given pin to be used for brightness control
// Setting frequency does not matter for PWMing LEDs, only duty
void init_brightness_control(int pin) {
  init_pwm(pin);
  set_pwm_duty(pin, 1.0); // Set initially to 0 brightness
}

double correct_gamma(double val) {
  return pow(val, GAMMA_CORRECTION);
}

// Double to set brightness since range can change
// No check for valid brightness since set_pwm_duty will just clamp it
inline void set_brightness(int pin, double brightness) {
  brightness = correct_gamma(brightness);

  set_pwm_duty(pin, 1.0 - brightness);
}

void init_display() {
  wiringPiSetup();

  // Ready all pins for output
  init_pin(OE);
  init_pin(CLK);
  init_pin(LAT);
  init_pin(R1);
  init_pin(R2);
  init_pin(G1);
  init_pin(G2);
  init_pin(B1);
  init_pin(B2);
  init_pin(A);
  init_pin(B);
  init_pin(C);
  init_pin(D);
  init_pin(E);

  init_brightness_control(OE); // Using 2 for now
  set_brightness(OE, 0.1);

  init_fast_pinout();
}

// Full screen refresh loop
void* display_loop(void* vargp) {
  // Precalculate necessary values
  const int row_count = HEIGHT / 2;
  const uint8_t colour_depth_power = find_binary_power(COLOUR_DEPTH);
  const uint8_t width_power = find_binary_power(WIDTH);
  const uint8_t y_offset_power = colour_depth_power + width_power;

  uint8_t y = 0;
  uint8_t bit_offset = 0;
  // unsigned int start_us_frame = micros();
  
  printf("\n"); // Do not erase term prompt
  
  while (1) {
    unsigned int start_us = micros();
    // int bcm_delay = BCM_DELAY << bit_offset; // BCM_DELAY*2^bit_offset

    refresh_row(WIDTH, y, current_frame + (y << y_offset_power) + (bit_offset << width_power));

    // Wait while showing row for BCM
    // Remove jitter by not delaying a hard amount
    unsigned int diff = micros() - start_us;
    printf("\x1b[1F"); // Move to beginning of previous line
    printf("\x1b[2K"); // Clear entire line
    printf("Line time: %u us\n", diff);
    // if (diff < bcm_delay) delayMicroseconds(bcm_delay - diff);

    // If statements are faster than MOD, so:
    bit_offset++;
    if (bit_offset == COLOUR_DEPTH) { bit_offset = 0; y++; }
    if (y == row_count) {
      y = 0;
      // printf("\x1b[1F"); // Move to beginning of previous line
      // printf("\x1b[2K"); // Clear entire line
      // printf("Frame time: %u us\n", micros() - start_us_frame);
      // start_us_frame = micros();// Update last frame timestamp
    }
  }
}

// Correct gamma using lookup table
void correct_gamma_colour(Colour* colour) {
  colour->r = gamma_lut[colour->r];
  colour->g = gamma_lut[colour->g];
  colour->b = gamma_lut[colour->b];
  colour->a = gamma_lut[colour->a];
}

// TODO: Just for initial debug, a more optimized method will be used in the display loop
// Framebuffer types are passed by ref since they are just arrays
void framebuffer_to_displayable(Framebuffer framebuffer, Framebuffer_Packet displayable) {
  for (int y = 0; y < (HEIGHT / 2); y++) { // Nested loops for clarity
    for (int bit = 0; bit < COLOUR_DEPTH; bit++) {
      for (int x = 0; x < WIDTH; x++) {
        // Get the two colours from framebuffer
        Colour colour1 = framebuffer[(WIDTH * y) + x];
        Colour colour2 = framebuffer[(WIDTH * (y + HEIGHT / 2)) + x];
        correct_gamma_colour(&colour1);
        correct_gamma_colour(&colour2);


        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].r1 = (colour1.r >> bit) & 0x1;
        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].r2 = (colour2.r >> bit) & 0x1;
        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].g1 = (colour1.g >> bit) & 0x1;
        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].g2 = (colour2.g >> bit) & 0x1;
        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].b1 = (colour1.b >> bit) & 0x1;
        displayable[(y * COLOUR_DEPTH * WIDTH) + (bit * WIDTH) + x].b2 = (colour2.b >> bit) & 0x1;
      }
    }
  }
}

int main(void) {
  init_display();

  // Create rainbow square
  Framebuffer frame;
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 0; x < WIDTH; x++) {
      Colour colour;
      colour.r = 255 - x;
      colour.g = 255 - y;
      colour.b = x;
      colour.a = 0xFF;
  
      frame[(WIDTH * y) + x] = colour;
    }
  }

  // Data structure is organized in this way:
  // N/A N/A r1 r2 g1 g2 b1 b2 -> Packet (uint_8)
  // Packet*64 -> Row of sequential data
  // (Packet*64)*8 -> 8 bit colour depth for row
  // ((Packet*64)*8)*32 -> Frame
  // A frame therefore is sizeof(Packet) * WIDTH * COLOUR_DEPTH * (HEIGHT / 2)
  // in length
  framebuffer_to_displayable(frame, current_frame);

  pthread_t thread;
  pthread_create(&thread, NULL, display_loop, NULL);
  pthread_join(thread, NULL);

  // display_loop(NULL);

  // deinit_display(display);

  exit(0);
}