/*
 * arith_kernel.c — pure C model fragments carved from arith.cc.
 *
 * Designed to compile identically on x86_64 (host vvp) and RV32I
 * (softcore worker). No C++ features, no globals, no heap, no I/O.
 */
#include "arith_kernel.h"

int vvp_arith_sum_kernel(uint32_t wid, uint32_t sizes,
                         const uint32_t *a_abits, const uint32_t *a_bbits,
                         const uint32_t *b_abits, const uint32_t *b_bbits,
                         uint32_t *out_abits, uint32_t *out_bbits)
{
    uint32_t a_size = sizes & 0xFFFFu;
    uint32_t b_size = sizes >> 16;
    uint32_t nw_out = (wid + 31u) / 32u;
    uint32_t w;
    for (w = 0; w < nw_out; w++) { out_abits[w] = 0; out_bbits[w] = 0; }

    /* 4-state carry. (carry_a, carry_b) follows the same encoding as
     * a single vvp_bit4_t: (0,0)=0, (1,0)=1, (*,1)=X. We never produce
     * a Z carry; an X/Z input short-circuits the whole add. */
    uint32_t carry_a = 0;
    uint32_t i;
    for (i = 0; i < wid; i++) {
        uint32_t aw = i >> 5, ab = i & 31u;
        uint32_t ai_a = (i < a_size) ? ((a_abits[aw] >> ab) & 1u) : 0u;
        uint32_t ai_b = (i < a_size) ? ((a_bbits[aw] >> ab) & 1u) : 0u;
        uint32_t bi_a = (i < b_size) ? ((b_abits[aw] >> ab) & 1u) : 0u;
        uint32_t bi_b = (i < b_size) ? ((b_bbits[aw] >> ab) & 1u) : 0u;

        /* Any 4-state X or Z bit (b-polarity = 1) on inputs or carry
         * makes the entire sum X, matching add_with_carry()'s
         * bit4_is_xz short-circuit. */
        if ((ai_b | bi_b) != 0u) {
            for (w = 0; w < nw_out; w++) { out_abits[w] = 0; out_bbits[w] = 0; }
            return 1;
        }

        uint32_t sum = ai_a + bi_a + carry_a;
        out_abits[aw] |= (sum & 1u) << ab;
        carry_a = (sum >> 1) & 1u;
    }
    return 0;
}
