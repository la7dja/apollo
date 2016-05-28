#include <stdint.h>
#include <avr/eeprom.h>

#ifndef USB_DEBUG
#undef DEBUG
#endif
#include <kernel/kernel.h>
#include <kernel/ports.h>

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_MICRO 0

#define ATU_STATUS_RESET 0
#define ATU_STATUS_IN_PROGRESS 1
#define ATU_STATUS_FAILED 2
#define ATU_STATUS_SUCCEEDED 3
#define ATU_STATUS_ABORTED 4

#define ATU_MAX_VSWR 2.0 // Max VSWR considered a success. TBD

#define RELAY_LATCH_TIME 10000 // µs (10ms)
#define RELAY_LATCH_DELAY 500 // µs
#define RELAY_SETTLE_TIME 50000 // µs (50ms)
#define PTT_DELAY 100000 // µs (100ms)


/* Port definitions */
#define LED1 F, 3
#define LED2 F, 2
#define STATUS D, 0 
#define PTT_OUT D, 1
#define PTT_LED LED1

#define RCK F, 4
#define SRCK F, 5
#define SER_IN F, 6


extern uint32_t frequency;


struct eeprom {
  uint8_t lpf_current_band;
  uint8_t lpf_enabled;
	uint8_t atu_inductors;
	uint16_t atu_capacitors;
  uint8_t atu_enabled;
};

extern struct eeprom EEMEM settings;
extern bool ptt_enabled;


void init_lpf ();
void update_lpf ();
void enable_lpf ();
void disable_lpf ();
uint8_t lpf_get_status ();
void enable_ptt (uint16_t timeout);
void disable_ptt ();
void init_atu();
void enable_atu();
void disable_atu();
void reset_atu();
void atu_tune(uint32_t frequency);
uint32_t atu_get_status ();

//static inline int min (int a, int b) { return a < b ? a : b; };
#define min(a,b) ((a) < (b) ? (a) : (b))
