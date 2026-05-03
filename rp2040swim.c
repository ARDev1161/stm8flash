#include "rp2040swim.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "librp2040swim.h"

#define DM_CSR2 0x7F99u
#define STM8_IAPSR_WR_PG_DIS 0x01u
#define STM8_IAPSR_PUL       0x02u
#define STM8_IAPSR_EOP       0x04u
#define STM8_IAPSR_DUL       0x08u

static int rp2040swim_read_byte(programmer_t *pgm, unsigned int addr) {
  uint8_t byte = 0;
  if (!rp2040swim_read(pgm->rp2040swim, &byte, addr, 1)) return -1;
  return byte;
}

static bool rp2040swim_write_byte(programmer_t *pgm, uint8_t byte, unsigned int addr) {
  return rp2040swim_write(pgm->rp2040swim, &byte, addr, 1);
}

static bool rp2040swim_stall(programmer_t *pgm, bool stall) {
  int csr = rp2040swim_read_byte(pgm, DM_CSR2);
  if (csr == -1) return false;
  return rp2040swim_write_byte(pgm, stall ? (uint8_t)(csr | 8) : (uint8_t)(csr & ~8), DM_CSR2);
}

static bool rp2040swim_reconnect(programmer_t *pgm) {
  if (!rp2040swim_enter_swim(pgm->rp2040swim)) return false;
  return rp2040swim_stall(pgm, true);
}

static bool rp2040swim_wait_iapsr_mask(programmer_t *pgm,
                                       const stm8_device_t *device,
                                       uint8_t mask,
                                       unsigned int retries) {
  while (retries-- > 0) {
    int iapsr = rp2040swim_read_byte(pgm, device->regs.FLASH_IAPSR);
    if (iapsr < 0) {
      usleep(10000);
      continue;
    }
    if ((iapsr & mask) == mask) {
      return true;
    }
    usleep(10000);
  }

  fprintf(stderr, "timeout waiting for FLASH_IAPSR mask 0x%02x\n", mask);
  return false;
}

static bool rp2040swim_prepare_for_flash(programmer_t *pgm,
                                         const stm8_device_t *device,
                                         const memtype_t memtype) {
  if (!rp2040swim_stall(pgm, true)) return false;

  if (memtype == FLASH) {
    if (!rp2040swim_write_byte(pgm, 0x56, device->regs.FLASH_PUKR)) return false;
    if (!rp2040swim_write_byte(pgm, 0xae, device->regs.FLASH_PUKR)) return false;
    if (!rp2040swim_wait_iapsr_mask(pgm, device, STM8_IAPSR_PUL, 20)) return false;
  }
  if (memtype == EEPROM || memtype == OPT) {
    if (!rp2040swim_write_byte(pgm, 0xae, device->regs.FLASH_DUKR)) return false;
    if (!rp2040swim_write_byte(pgm, 0x56, device->regs.FLASH_DUKR)) return false;
    if (!rp2040swim_wait_iapsr_mask(pgm, device, STM8_IAPSR_DUL, 20)) return false;
  }

  return true;
}

static bool rp2040swim_wait_eop(programmer_t *pgm, const stm8_device_t *device, unsigned int retries) {
  while (retries-- > 0) {
    int iapsr = rp2040swim_read_byte(pgm, device->regs.FLASH_IAPSR);
    if (iapsr < 0) {
      usleep(10000);
      continue;
    }
    if (iapsr & STM8_IAPSR_WR_PG_DIS) {
      fprintf(stderr, "target page is write protected (UBC) or read-out protection is enabled\n");
      return false;
    }
    if (iapsr & STM8_IAPSR_EOP) {
      return true;
    }
    usleep(10000);
  }
  return false;
}

int rp2040swim_swim_read_range(programmer_t *pgm, const stm8_device_t *device,
                               unsigned char *buffer, unsigned int start,
                               unsigned int length) {
  (void)device;
  unsigned int i = 0;
  while (i < length) {
    size_t current_size = length - i;
    if (current_size > 1024) current_size = 1024;
    if (!rp2040swim_read(pgm->rp2040swim, buffer + i, start + i, current_size)) {
      return i;
    }
    i += (unsigned int)current_size;
  }
  return i;
}

static bool rp2040swim_block_needs_erase(const unsigned char *current,
                                         const unsigned char *desired,
                                         unsigned int len) {
  /*
   * STM8S flash erased value is 0x00. Programming can set bits 0 -> 1.
   * Erase is only needed if any bit must transition 1 -> 0.
   */
  for (unsigned int i = 0; i < len; i++) {
    if ((current[i] & (uint8_t)~desired[i]) != 0u) {
      return true;
    }
  }
  return false;
}

int rp2040swim_swim_write_range(programmer_t *pgm, const stm8_device_t *device,
                                unsigned char *buffer, unsigned int start,
                                unsigned int length, const memtype_t memtype) {
  unsigned int i = 0;

  /*
   * For FLASH we use firmware-side FLASH_ERASE/FLASH_WRITE_BLOCK below.
   * Do not unlock program flash before the pre-read; do it after comparing
   * current/desired data, right before the firmware flash operation.
   */
  if (memtype != FLASH && !rp2040swim_prepare_for_flash(pgm, device, memtype)) {
      return 0;
  }

  /*
   * Use the programmer firmware's validated flash operations instead of the
   * stm8flash/espstlink-style direct CR2 block-programming sequence. The RP2040
   * firmware already has target-specific handling for erase/program timing,
   * post-trigger SWIM resync, and conservative byte programming.
   */
  if (memtype == FLASH) {
    unsigned int block_size = device->flash_block_size;
    unsigned int rounded_size = ((length - 1) / block_size + 1) * block_size;
    unsigned char *current = malloc(rounded_size);
    unsigned char *desired = malloc(rounded_size);
    unsigned int written = 0;

    if (!current || !desired) {
      free(current);
      free(desired);
      return 0;
    }

    if (rp2040swim_swim_read_range(pgm, device, current, start, rounded_size) != (int)rounded_size) {
      free(current);
      free(desired);
      return 0;
    }

    memcpy(desired, buffer, length);
    if (rounded_size > length) {
      memcpy(desired + length, current + length, rounded_size - length);
    }

    if (!rp2040swim_prepare_for_flash(pgm, device, memtype)) {
      free(current);
      free(desired);
      return 0;
    }

    for (unsigned int off = 0; off < rounded_size; off += block_size) {
      if (memcmp(current + off, desired + off, block_size) == 0) {
        if (off < length) {
          written = off + block_size;
          if (written > length) written = length;
        }
        continue;
      }

      if (rp2040swim_block_needs_erase(current + off, desired + off, block_size)) {
        if (!rp2040swim_flash_erase(pgm->rp2040swim, start + off, block_size)) {
          break;
        }
      }
      if (!rp2040swim_flash_write_block(pgm->rp2040swim, start + off, desired + off, block_size)) {
        break;
      }

      if (off < length) {
        written = off + block_size;
        if (written > length) written = length;
      }
    }

    free(current);
    free(desired);

    return written;
  }

  if (memtype == OPT) {
    if (!rp2040swim_write_byte(pgm, 0x80, device->regs.FLASH_CR2)) return 0;
    if (device->regs.FLASH_NCR2 != 0 && !rp2040swim_write_byte(pgm, 0x7f, device->regs.FLASH_NCR2)) return 0;

    for (i = 0; i < length; i++) {
      if (!rp2040swim_write_byte(pgm, buffer[i], start + i)) return i;
      usleep(6000);
      if (!rp2040swim_wait_eop(pgm, device, 5)) return i;
    }
  } else {
    unsigned int rounded_size = ((length - 1) / device->flash_block_size + 1) * device->flash_block_size;
    unsigned char *current = malloc(rounded_size);
    unsigned char *desired = malloc(rounded_size);
    if (!current || !desired) {
      free(current);
      free(desired);
      return 0;
    }

    if (rp2040swim_swim_read_range(pgm, device, current, start, rounded_size) != (int)rounded_size) {
      free(current);
      free(desired);
      return 0;
    }

    memcpy(desired, buffer, length);
    if (rounded_size > length) {
      memcpy(desired + length, current + length, rounded_size - length);
    }

    for (i = 0; i < length; i += device->flash_block_size) {
      if (memcmp(current + i, desired + i, device->flash_block_size) == 0) {
        continue;
      }

      uint8_t prgmode = 0x10;
      if (memtype == FLASH || memtype == EEPROM) {
        for (unsigned int j = 0; j < device->flash_block_size; j++) {
          if (current[i + j]) {
            prgmode = 0x01;
            break;
          }
        }

        if (!rp2040swim_write_byte(pgm, prgmode, device->regs.FLASH_CR2)) break;
        if (device->regs.FLASH_NCR2 != 0 && !rp2040swim_write_byte(pgm, (uint8_t)~prgmode, device->regs.FLASH_NCR2)) break;
      }

      if (!rp2040swim_write(pgm->rp2040swim, desired + i, start + i, device->flash_block_size)) break;

      if (memtype == FLASH || memtype == EEPROM) {
        usleep(prgmode == 0x10 ? 3000 : 6000);
        if (!rp2040swim_wait_eop(pgm, device, 10)) break;
      }
    }

    free(current);
    free(desired);
  }

  if (memtype == FLASH || memtype == EEPROM || memtype == OPT) {
    int iapsr = rp2040swim_read_byte(pgm, device->regs.FLASH_IAPSR);
    if (iapsr != -1) {
      (void)rp2040swim_write_byte(pgm, (uint8_t)(iapsr & ~0x0a), device->regs.FLASH_IAPSR);
    }
  }

  return i;
}

void rp2040swim_srst(programmer_t *pgm) {
  if (!pgm || !pgm->rp2040swim) return;
  (void)rp2040swim_reset_target(pgm->rp2040swim);
}

bool rp2040swim_pgm_open(programmer_t *pgm) {
  pgm->rp2040swim = rp2040swim_open(pgm->port);
  if (pgm->rp2040swim == NULL) return false;

  /*
   * Defaults match current rp2040_swim wiring:
   *   SWIM GPIO2, NRST GPIO3, external pull-up, low-speed.
   */
  return rp2040swim_fetch_version(pgm->rp2040swim) &&
         rp2040swim_set_pins(pgm->rp2040swim, 2, 3, false) &&
         rp2040swim_set_speed(pgm->rp2040swim, false) &&
         rp2040swim_reconnect(pgm);
}

void rp2040swim_pgm_close(programmer_t *pgm) {
  if (!pgm) return;
  rp2040swim_close(pgm->rp2040swim);
  pgm->rp2040swim = NULL;
}
