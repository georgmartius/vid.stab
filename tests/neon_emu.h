/* neon_emu.h -- scalar emulation of the few NEON intrinsics used by
 * src/motiondetect_neon.c, so that kernel can be compiled and its results
 * checked against the C reference on a non-ARM development machine.
 *
 * This is a TEST ONLY facility and is never compiled into the library.  It
 * makes no attempt at being a general NEON shim (use sse2neon.h or SIMDe for
 * that): it defines exactly the operations motiondetect_neon.c uses, with the
 * semantics given in the ARM C Language Extensions, and nothing else.
 *
 * Passing these tests says the *logic* of the NEON kernel is right.  It says
 * nothing about how it performs, or about intrinsics-to-instruction issues on
 * real silicon -- that still needs a run on an actual ARM machine.
 */

#ifndef VS_NEON_EMU_H
#define VS_NEON_EMU_H

#include <stdint.h>
#include <string.h>

typedef struct { uint8_t  v[16]; } uint8x16_t;
typedef struct { uint16_t v[8];  } uint16x8_t;
typedef struct { uint32_t v[4];  } uint32x4_t;

static inline uint8x16_t vld1q_u8(const uint8_t* p) {
  uint8x16_t r;
  memcpy(r.v, p, 16);
  return r;
}

static inline uint8x16_t vdupq_n_u8(uint8_t x) {
  uint8x16_t r; int i;
  for (i = 0; i < 16; i++) r.v[i] = x;
  return r;
}

static inline uint16x8_t vdupq_n_u16(uint16_t x) {
  uint16x8_t r; int i;
  for (i = 0; i < 8; i++) r.v[i] = x;
  return r;
}

static inline uint32x4_t vdupq_n_u32(uint32_t x) {
  uint32x4_t r; int i;
  for (i = 0; i < 4; i++) r.v[i] = x;
  return r;
}

/* absolute difference, lanewise */
static inline uint8x16_t vabdq_u8(uint8x16_t a, uint8x16_t b) {
  uint8x16_t r; int i;
  for (i = 0; i < 16; i++)
    r.v[i] = (uint8_t)(a.v[i] > b.v[i] ? a.v[i] - b.v[i] : b.v[i] - a.v[i]);
  return r;
}

/* pairwise add of b's 16 lanes into 8, accumulated into a */
static inline uint16x8_t vpadalq_u8(uint16x8_t a, uint8x16_t b) {
  uint16x8_t r; int i;
  for (i = 0; i < 8; i++)
    r.v[i] = (uint16_t)(a.v[i] + (uint16_t)b.v[2*i] + (uint16_t)b.v[2*i + 1]);
  return r;
}

/* pairwise add of b's 8 lanes into 4, accumulated into a */
static inline uint32x4_t vpadalq_u16(uint32x4_t a, uint16x8_t b) {
  uint32x4_t r; int i;
  for (i = 0; i < 4; i++)
    r.v[i] = a.v[i] + (uint32_t)b.v[2*i] + (uint32_t)b.v[2*i + 1];
  return r;
}

static inline uint8x16_t vminq_u8(uint8x16_t a, uint8x16_t b) {
  uint8x16_t r; int i;
  for (i = 0; i < 16; i++) r.v[i] = a.v[i] < b.v[i] ? a.v[i] : b.v[i];
  return r;
}

static inline uint8x16_t vmaxq_u8(uint8x16_t a, uint8x16_t b) {
  uint8x16_t r; int i;
  for (i = 0; i < 16; i++) r.v[i] = a.v[i] > b.v[i] ? a.v[i] : b.v[i];
  return r;
}

/* across-vector reductions (AArch64) */
static inline uint32_t vaddvq_u32(uint32x4_t a) {
  return a.v[0] + a.v[1] + a.v[2] + a.v[3];
}

static inline uint8_t vminvq_u8(uint8x16_t a) {
  uint8_t m = a.v[0]; int i;
  for (i = 1; i < 16; i++) if (a.v[i] < m) m = a.v[i];
  return m;
}

static inline uint8_t vmaxvq_u8(uint8x16_t a) {
  uint8_t m = a.v[0]; int i;
  for (i = 1; i < 16; i++) if (a.v[i] > m) m = a.v[i];
  return m;
}

#endif /* VS_NEON_EMU_H */
