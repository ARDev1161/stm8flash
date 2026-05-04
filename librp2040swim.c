/**
 * Low-level serial transport for ARDev1161/rp2040_swim programmer.
 */
#include "librp2040swim.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define RPSW_MAGIC 0x53575052u
#define RPSW_VERSION 1u
#define RPSW_MAX_PAYLOAD 1024u

#define RPSW_CMD_GET_VERSION   0x01u
#define RPSW_CMD_SET_PINS      0x02u
#define RPSW_CMD_SET_SPEED     0x03u
#define RPSW_CMD_ENTER_SWIM    0x04u
#define RPSW_CMD_RESET_TARGET  0x05u
#define RPSW_CMD_MEMORY_READ   0x08u
#define RPSW_CMD_MEMORY_WRITE  0x09u
#define RPSW_CMD_FLASH_ERASE    0x0au
#define RPSW_CMD_FLASH_WRITE_BLOCK 0x0bu
#define RPSW_CMD_GET_LAST_ERROR 0x0du
#define RPSW_CMD_RELEASE_TARGET 0x12u

#define RPSW_STATUS_OK 0u

static rp2040swim_error_t error;

rp2040swim_error_t *rp2040swim_get_last_error(void) {
  return &error;
}

static void set_error(int code, int device_status, const char *fmt, ...) {
  va_list ap;
  memset(&error, 0, sizeof(error));
  error.code = code;
  error.device_status = device_status;
  va_start(ap, fmt);
  vsnprintf(error.message, sizeof(error.message), fmt, ap);
  va_end(ap);
  fprintf(stderr, "%s\n", error.message);
}

static uint16_t get_u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u16le(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xffu);
  p[1] = (uint8_t)((v >> 8) & 0xffu);
}

static void put_u32le(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xffu);
  p[1] = (uint8_t)((v >> 8) & 0xffu);
  p[2] = (uint8_t)((v >> 16) & 0xffu);
  p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
  }
  return ~crc;
}

static bool read_exact_timeout(int fd, uint8_t *buf, size_t len, int timeout_ms) {
  size_t done = 0;
  while (done < len) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int r = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (r <= 0) {
      return false;
    }

    ssize_t n = read(fd, buf + done, len - done);
    if (n <= 0) {
      return false;
    }
    done += (size_t)n;
  }
  return true;
}

static bool write_all(int fd, const uint8_t *buf, size_t len) {
  size_t done = 0;
  while (done < len) {
    ssize_t n = write(fd, buf + done, len - done);
    if (n <= 0) {
      return false;
    }
    done += (size_t)n;
  }
  return true;
}

rp2040swim_t *rp2040swim_open(const char *device) {
  const char *path = device == NULL ? "/dev/ttyACM0" : device;
  int fd = open(path, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror("Couldn't open tty");
    return NULL;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr failed");
    close(fd);
    return NULL;
  }

  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);
  cfmakeraw(&tty);
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10; /* 1 second kernel read timeout; select below is stricter per read. */

  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr failed");
    close(fd);
    return NULL;
  }

  rp2040swim_t *pgm = calloc(1, sizeof(*pgm));
  if (!pgm) {
    close(fd);
    return NULL;
  }
  pgm->fd = fd;
  pgm->sequence = 1;

  /* Firmware reboots USB CDC sometimes; allow it to settle and discard stale bytes. */
  usleep(250000);
  tcflush(fd, TCIOFLUSH);
  return pgm;
}

void rp2040swim_close(rp2040swim_t *pgm) {
  if (!pgm) return;
  close(pgm->fd);
  free(pgm);
}

static bool command(rp2040swim_t *pgm, uint8_t command_id,
                    const uint8_t *payload, uint16_t payload_len,
                    uint8_t *response, uint16_t *response_len) {
  if (payload_len > RPSW_MAX_PAYLOAD) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "payload too large");
    return false;
  }

  uint8_t frame[10 + RPSW_MAX_PAYLOAD + 4];
  put_u32le(&frame[0], RPSW_MAGIC);
  frame[4] = RPSW_VERSION;
  frame[5] = command_id;
  uint16_t seq = pgm->sequence++;
  put_u16le(&frame[6], seq);
  put_u16le(&frame[8], payload_len);
  if (payload_len != 0 && payload != NULL) {
    memcpy(&frame[10], payload, payload_len);
  }
  uint32_t crc = crc32_update(0, frame, 10u + payload_len);
  put_u32le(&frame[10u + payload_len], crc);

  if (!write_all(pgm->fd, frame, 10u + payload_len + 4u)) {
    set_error(RP2040SWIM_ERROR_IO, errno, "write failed: %s", strerror(errno));
    return false;
  }

  uint8_t hdr[10];
  if (!read_exact_timeout(pgm->fd, hdr, sizeof(hdr), 15000)) {
    set_error(RP2040SWIM_ERROR_IO, errno, "timeout waiting for response header");
    return false;
  }

  if (get_u32le(&hdr[0]) != RPSW_MAGIC || hdr[4] != RPSW_VERSION) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "bad response header");
    return false;
  }

  uint8_t resp_cmd = hdr[5];
  uint16_t resp_seq = get_u16le(&hdr[6]);
  uint16_t len = get_u16le(&hdr[8]);
  if (resp_cmd != (uint8_t)(command_id | 0x80u) || resp_seq != seq || len < 2u || len > (RPSW_MAX_PAYLOAD + 2u)) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "unexpected response cmd=0x%02x seq=%u len=%u", resp_cmd, resp_seq, len);
    return false;
  }

  uint8_t body_crc[RPSW_MAX_PAYLOAD + 2u + 4u];
  if (!read_exact_timeout(pgm->fd, body_crc, (size_t)len + 4u, 15000)) {
    set_error(RP2040SWIM_ERROR_IO, errno, "timeout waiting for response payload");
    return false;
  }

  uint32_t expected_crc = get_u32le(&body_crc[len]);
  uint8_t tmp[10 + RPSW_MAX_PAYLOAD + 2u];
  memcpy(tmp, hdr, 10);
  memcpy(&tmp[10], body_crc, len);
  uint32_t actual_crc = crc32_update(0, tmp, 10u + len);
  if (actual_crc != expected_crc) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "bad response crc");
    return false;
  }

  uint16_t status = get_u16le(&body_crc[0]);
  uint16_t out_len = (uint16_t)(len - 2u);
  if (status != RPSW_STATUS_OK) {
    char detail[192] = {0};
    if (command_id != RPSW_CMD_GET_LAST_ERROR) {
      uint16_t detail_len = sizeof(detail) - 1u;
      if (command(pgm, RPSW_CMD_GET_LAST_ERROR, NULL, 0, (uint8_t *)detail, &detail_len)) {
        detail[detail_len < sizeof(detail) ? detail_len : sizeof(detail) - 1u] = '\0';
      }
    }
    set_error(RP2040SWIM_ERROR_STATUS, status, "command 0x%02x failed: status=%u %s", command_id, status, detail);
    return false;
  }

  if (response_len != NULL) {
    if (*response_len < out_len) {
      set_error(RP2040SWIM_ERROR_FRAME, -1, "response buffer too small: have=%u need=%u", *response_len, out_len);
      return false;
    }
    *response_len = out_len;
  }
  if (response != NULL && out_len != 0u) {
    memcpy(response, &body_crc[2], out_len);
  }
  return true;
}

bool rp2040swim_fetch_version(rp2040swim_t *pgm) {
  uint8_t resp[64];
  uint16_t len = sizeof(resp);
  if (!command(pgm, RPSW_CMD_GET_VERSION, NULL, 0, resp, &len)) {
    return false;
  }
  if (len < 5u || resp[3] != RPSW_VERSION) {
    set_error(RP2040SWIM_ERROR_VERSION, -1, "unsupported rp2040_swim protocol version");
    return false;
  }
  return true;
}

bool rp2040swim_set_pins(rp2040swim_t *pgm, uint8_t swim_pin, uint8_t nrst_pin, bool pullup) {
  uint8_t payload[3] = {swim_pin, nrst_pin, pullup ? 1u : 0u};
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_SET_PINS, payload, sizeof(payload), NULL, &len);
}

bool rp2040swim_set_speed(rp2040swim_t *pgm, bool high_speed) {
  uint8_t payload[1] = {high_speed ? 1u : 0u};
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_SET_SPEED, payload, sizeof(payload), NULL, &len);
}

bool rp2040swim_enter_swim(rp2040swim_t *pgm) {
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_ENTER_SWIM, NULL, 0, NULL, &len);
}

bool rp2040swim_reset_target(rp2040swim_t *pgm) {
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_RESET_TARGET, NULL, 0, NULL, &len);
}

bool rp2040swim_release_target(rp2040swim_t *pgm) {
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_RELEASE_TARGET, NULL, 0, NULL, &len);
}

bool rp2040swim_read(rp2040swim_t *pgm, uint8_t *buffer, unsigned int addr, size_t size) {
  uint8_t payload[6];
  put_u32le(&payload[0], addr);
  put_u16le(&payload[4], (uint16_t)size);
  uint16_t len = (uint16_t)size;
  return command(pgm, RPSW_CMD_MEMORY_READ, payload, sizeof(payload), buffer, &len) && len == size;
}

bool rp2040swim_write(rp2040swim_t *pgm, const uint8_t *buffer, unsigned int addr, size_t size) {
  if (size > (RPSW_MAX_PAYLOAD - 6u)) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "write too large");
    return false;
  }
  uint8_t payload[RPSW_MAX_PAYLOAD];
  put_u32le(&payload[0], addr);
  put_u16le(&payload[4], (uint16_t)size);
  if (size != 0u) {
    memcpy(&payload[6], buffer, size);
  }
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_MEMORY_WRITE, payload, (uint16_t)(6u + size), NULL, &len);
}

bool rp2040swim_flash_erase(rp2040swim_t *pgm, unsigned int addr, unsigned int length) {
  uint8_t payload[8];
  put_u32le(&payload[0], addr);
  put_u32le(&payload[4], length);
  uint16_t len = 0;
  return command(pgm, RPSW_CMD_FLASH_ERASE, payload, sizeof(payload), NULL, &len);
}

bool rp2040swim_flash_write_block(rp2040swim_t *pgm, unsigned int addr,
                                  const uint8_t *buffer, size_t size) {
  if (size > (RPSW_MAX_PAYLOAD - 6u)) {
    set_error(RP2040SWIM_ERROR_FRAME, -1, "flash write block too large");
    return false;
  }

  uint8_t payload[RPSW_MAX_PAYLOAD];
  put_u32le(&payload[0], addr);
  put_u16le(&payload[4], (uint16_t)size);
  if (size != 0u) {
    memcpy(&payload[6], buffer, size);
  }

  uint16_t len = 0;
  return command(pgm, RPSW_CMD_FLASH_WRITE_BLOCK, payload, (uint16_t)(6u + size), NULL, &len);
}
