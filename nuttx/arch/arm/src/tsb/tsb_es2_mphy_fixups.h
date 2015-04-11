#ifndef _ARA_COMMON_ES2_MPHY_FIXUPS_H_
#define _ARA_COMMON_ES2_MPHY_FIXUPS_H_

#include <stdint.h>

/*
 * This specifies an M-PHY "fixup"; i.e., a value that must be set to
 * a DME attribute while the link is still in PWM-G1, before
 * transitioning to HS link power modes.
 *
 * Use tsb_mphy_r1_fixup_is_magic() to test if a register 1 map fixup
 * must have its value drawn from M-PHY trim values or magic debug
 * registers (switch ports and bridges handle this case differently).
 */
struct tsb_mphy_fixup {
    uint16_t attrid;
    uint16_t select_index;
    uint32_t value;

#define TSB_MPHY_FIXUP_LAST     0x1
#define TSB_MPHY_FIXUP_MAGIC_R1 0x2
    uint32_t flags;
};

/*
 * Use tsb_mphy_fixup_is_last() to test if a fixup is the last one in
 * each of these array.
 */
extern const struct tsb_mphy_fixup tsb_register_1_map_mphy_fixups[];
extern const struct tsb_mphy_fixup tsb_register_2_map_mphy_fixups[];

static inline int tsb_mphy_fixup_is_last(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_LAST);
}

static inline int tsb_mphy_r1_fixup_is_magic(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_MAGIC_R1);
}

#endif
