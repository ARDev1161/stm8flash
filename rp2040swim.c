#include "rp2040swim.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "librp2040swim.h"

#define DM_CSR2 0x7F99u
#define SWIM_CSR_ADDR      0x7F80u
#define SWIM_CSR_FINISH_VALUE 0x80u

#define STM8_IAPSR_WR_PG_DIS 0x01u
#define STM8_IAPSR_PUL       0x02u
#define STM8_IAPSR_EOP       0x04u
#define STM8_IAPSR_DUL       0x08u

#define RECONNECT_ATTEMPTS 7

static bool rp2040swim_use_high_speed(void) {
  const char *hs = getenv("RP2040SWIM_HS");
  return (hs != NULL) && (hs[0] != '\0') && (strcmp(hs, "0") != 0);
}

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

  if (rp2040swim_use_high_speed()) {
    fprintf(stderr, "rp2040swim: experimental high-speed SWIM enabled\n");
    if (!rp2040swim_set_speed(pgm->rp2040swim, true)) {
      return false;
    }
  }
  return true;
}


static bool rp2040swim_ensure_connected(programmer_t *pgm) {
  if (pgm->rp2040swim_connected) {
    return true;
  }

  if (!rp2040swim_reconnect(pgm)) {
    pgm->rp2040swim_connected = false;
    return false;
  }

  pgm->rp2040swim_connected = true;
  return true;
}

static bool rp2040swim_ensure_connected_with_retries(programmer_t *pgm) {
  for (unsigned int attempt = 0; attempt < RECONNECT_ATTEMPTS; attempt++) {
    if (attempt != 0) {
      fprintf(stderr, "rp2040swim: ENTER_SWIM retry %u/%u\n",
              attempt, RECONNECT_ATTEMPTS);
      usleep(50000);
    }

    if (rp2040swim_ensure_connected(pgm)) {
      return true;
    }

    pgm->rp2040swim_connected = false;
    (void)rp2040swim_reset_target(pgm->rp2040swim);
    usleep(1000);
    (void)rp2040swim_release_target(pgm->rp2040swim);
  }

  return false;
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
  if (!rp2040swim_ensure_connected(pgm)) return false;

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
    if (current_size > 1024) {
      current_size = 1024;
    }

    bool ok = false;

    for (unsigned int attempt = 0; attempt < RECONNECT_ATTEMPTS; attempt++) {
      if (attempt != 0) {
        fprintf(stderr,
                "rp2040swim: MEMORY_READ retry %u/%u at 0x%06x len=%zu\n",
                attempt, RECONNECT_ATTEMPTS, start + i, current_size);
      }

      if (!rp2040swim_ensure_connected(pgm)) {
        pgm->rp2040swim_connected = false;
        usleep(20000);
        continue;
      }

      if (rp2040swim_read(pgm->rp2040swim, buffer + i, start + i, current_size)) {
        ok = true;
        break;
      }

      pgm->rp2040swim_connected = false;
      (void)rp2040swim_release_target(pgm->rp2040swim);
      usleep(20000);
    }

    if (!ok) {
      return i;
    }

    i += (unsigned int)current_size;
  }

  return i;
}

static bool rp2040swim_block_needs_erase(const unsigned char *current,
                                         const unsigned char *desired,
                                         unsigned int len) {
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
   * With lazy-enter, open() no longer enters SWIM.
   * write_range() must explicitly enter SWIM before any DM_CSR2 / memory access.
   */
  if (!rp2040swim_ensure_connected_with_retries(pgm)) {
    return 0;
  }

  bool use_firmware_flash_path = false;
  const char *fw_flash = getenv("RP2040SWIM_FW_FLASH");
  if (fw_flash != NULL && fw_flash[0] != '\0' && strcmp(fw_flash, "0") != 0) {
    use_firmware_flash_path = true;
  }

  /*
   * Optional experimental firmware-side flash path. The default path below stays
   * close to upstream stm8flash/espstlink: CR2/NCR2 + MEMORY_WRITE.
   */
  if (memtype == FLASH && use_firmware_flash_path) {
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
    if (!rp2040swim_prepare_for_flash(pgm, device, memtype)) {
      return 0;
    }
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

    if ((memtype == FLASH || memtype == EEPROM) &&
        !rp2040swim_prepare_for_flash(pgm, device, memtype)) {
      free(current);
      free(desired);
      return 0;
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

  if (i > length) {
    i = length;
  }
  return i;
}

static bool rp2040swim_clear_stall(programmer_t *pgm) {
  for (unsigned attempt = 0; attempt < 3; attempt++) {
    if (attempt != 0) {
      usleep(5000);
    }

    if (rp2040swim_stall(pgm, false)) {
      return true;
    }
  }

  fprintf(stderr, "rp2040swim: warning: failed to clear STALL\n");
  return false;
}
void rp2040swim_srst(programmer_t *pgm) {
  if (!pgm || !pgm->rp2040swim) return;

  /*
   * Reset/run policy:
   *
   * Do not try to continue the old SWIM session after reset.
   * Instead:
   *   - release old pins/session,
   *   - hardware reset the target,
   *   - enter SWIM again,
   *   - clear STALL,
   *   - release pins so user firmware can run.
   *
   * This is intentionally the old working flow. It is needed after -w,
   * because stm8flash calls pgm->reset() after successful write.
   */
  pgm->rp2040swim_connected = false;

  (void)rp2040swim_release_target(pgm->rp2040swim);
  usleep(1000);

  (void)rp2040swim_reset_target(pgm->rp2040swim);
  usleep(2000);

  if (!rp2040swim_ensure_connected_with_retries(pgm)) {
    fprintf(stderr,
            "rp2040swim: warning: failed to enter SWIM after reset; releasing lines only\n");
    (void)rp2040swim_release_target(pgm->rp2040swim);
    return;
  }

  (void)rp2040swim_clear_stall(pgm);
  usleep(1000);

  pgm->rp2040swim_connected = false;
  (void)rp2040swim_release_target(pgm->rp2040swim);
}

bool rp2040swim_pgm_open(programmer_t *pgm) {
  pgm->rp2040swim = rp2040swim_open(pgm->port);
  if (pgm->rp2040swim == NULL) return false;

  pgm->rp2040swim_connected = false;

  if (!rp2040swim_fetch_version(pgm->rp2040swim) ||
      !rp2040swim_set_pins(pgm->rp2040swim, 2, 3, false))
    {
      rp2040swim_close(pgm->rp2040swim);
      pgm->rp2040swim = NULL;
      pgm->rp2040swim_connected = false;
      return false;
    }

    (void)rp2040swim_release_target(pgm->rp2040swim);

    return true;
}

void rp2040swim_pgm_close(programmer_t *pgm) {
  if (!pgm) return;

  if (pgm->rp2040swim) {
    if (pgm->rp2040swim_connected) {
      (void)rp2040swim_clear_stall(pgm);
      usleep(1000);
      pgm->rp2040swim_connected = false;
    }

    (void)rp2040swim_release_target(pgm->rp2040swim);

    rp2040swim_close(pgm->rp2040swim);
    pgm->rp2040swim = NULL;
    pgm->rp2040swim_connected = false;
  }
}
