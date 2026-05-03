/**
 * Low-level serial transport for ARDev1161/rp2040_swim programmer.
 */
#ifndef __LIBRP2040SWIM_H
#define __LIBRP2040SWIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct _rp2040swim_t {
  int fd;
  uint16_t sequence;
} rp2040swim_t;

typedef struct _rp2040swim_error_t {
  int code;
  char message[256];
  int device_status;
} rp2040swim_error_t;

#define RP2040SWIM_ERROR_IO       1
#define RP2040SWIM_ERROR_FRAME    2
#define RP2040SWIM_ERROR_STATUS   3
#define RP2040SWIM_ERROR_VERSION  4

rp2040swim_error_t *rp2040swim_get_last_error(void);

rp2040swim_t *rp2040swim_open(const char *device);
void rp2040swim_close(rp2040swim_t *pgm);

bool rp2040swim_fetch_version(rp2040swim_t *pgm);
bool rp2040swim_set_pins(rp2040swim_t *pgm, uint8_t swim_pin, uint8_t nrst_pin, bool pullup);
bool rp2040swim_set_speed(rp2040swim_t *pgm, bool high_speed);
bool rp2040swim_enter_swim(rp2040swim_t *pgm);
bool rp2040swim_reset_target(rp2040swim_t *pgm);

bool rp2040swim_read(rp2040swim_t *pgm, uint8_t *buffer, unsigned int addr, size_t size);
bool rp2040swim_write(rp2040swim_t *pgm, const uint8_t *buffer, unsigned int addr, size_t size);
bool rp2040swim_flash_erase(rp2040swim_t *pgm, unsigned int addr, unsigned int length);
bool rp2040swim_flash_write_block(rp2040swim_t *pgm, unsigned int addr,
                                  const uint8_t *buffer, size_t size);
#endif
