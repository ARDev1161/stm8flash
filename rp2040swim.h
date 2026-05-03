#ifndef __RP2040SWIM_H
#define __RP2040SWIM_H

#include <stdbool.h>
#include "pgm.h"

bool rp2040swim_pgm_open(programmer_t *pgm);
void rp2040swim_pgm_close(programmer_t *pgm);
void rp2040swim_srst(programmer_t *pgm);
int rp2040swim_swim_read_range(programmer_t *pgm, const stm8_device_t *device,
                               unsigned char *buffer, unsigned int start,
                               unsigned int length);
int rp2040swim_swim_write_range(programmer_t *pgm, const stm8_device_t *device,
                                unsigned char *buffer, unsigned int start,
                                unsigned int length, const memtype_t memtype);

#endif
