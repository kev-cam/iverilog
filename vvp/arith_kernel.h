/*
 * arith_kernel.h — pure C model fragments carved from arith.cc.
 *
 * Each kernel is a leaf function that operates on word-packed 4-state
 * bit slices (one uint32_t holds the a-polarity of 32 bits; a second
 * holds the b-polarity). The same source compiles cleanly on the
 * x86_64 host (linked into libvvp) and on the RV32I softcore worker.
 */
#ifndef IVL_arith_kernel_H
#define IVL_arith_kernel_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* vvp_arith_sum_kernel — combinational adder net.
 *
 * Inputs:
 *   wid       — output width in bits
 *   sizes     — packed: low 16 bits = a_size, high 16 bits = b_size.
 *               Operands beyond their declared size are zero-padded
 *               (BIT4_0), matching vvp's pad behaviour.
 *   a_abits   — a operand, a-polarity word array (ceil(a_size/32) words)
 *   a_bbits   — a operand, b-polarity
 *   b_abits   — b operand, a-polarity (ceil(b_size/32) words)
 *   b_bbits   — b operand, b-polarity
 *
 * Outputs:
 *   out_abits — wid-bit result, a-polarity (ceil(wid/32) words)
 *   out_bbits — wid-bit result, b-polarity (always zero unless X-flag)
 *
 * Returns 0 on success. Returns 1 if any input bit was X or Z (or the
 * carry ever became X/Z), in which case out_*bits are zeroed and the
 * caller must propagate its precomputed x_val_ instead.
 */
int vvp_arith_sum_kernel(uint32_t wid, uint32_t sizes,
                         const uint32_t *a_abits, const uint32_t *a_bbits,
                         const uint32_t *b_abits, const uint32_t *b_bbits,
                         uint32_t *out_abits, uint32_t *out_bbits);

#ifdef __cplusplus
}
#endif

#endif
