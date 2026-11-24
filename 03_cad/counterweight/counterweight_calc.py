"""Size the swing counterweight and check that the arm cannot tip the foot over.

Replace the ESTIMATED masses/distances below with values the team weighs and
measures (kitchen scale to 0.1 g, ruler from the yaw axis). Units: g and mm.
Distance = horizontal distance from the base yaw axis, + toward the arm.

Run:  python3 counterweight_calc.py
"""
import math

# --- Arm, fully stretched out horizontally (worst case for tipping) -----------
# ESTIMATES from STL volumes (PLA, ~65 % effective fill) and 9 g SG90 servos.
ARM_EXTENDED = [  # (name, mass_g, distance_mm)
    ("link 1 plastic", 6.0, 25),
    ("elbow servo", 9.0, 45),
    ("link 2 plastic", 6.0, 75),
    ("wrist servo", 9.0, 95),
    ("front link + plate", 5.0, 120),
    ("gripper servo", 9.0, 135),
    ("gripper jaws/racks", 7.0, 160),
    ("payload", 5.0, 180),
]
# Arm folded up (servos near vertical) -> its CoM ends up close to the axis.
ARM_TUCKED_COM_MM = 10.0

STATIC_MASS_G = 34.0      # foot + base servo + swing + shoulder servo (all ~on axis)
FOOT_ARM_MM = 55.0        # foot is a + of 110 mm; support polygon is the square
EDGE_WORST_MM = FOOT_ARM_MM / math.sqrt(2)   # 38.9 mm when the arm points diagonally

# --- Counterweight as designed (Swing_v2_counterweight) ------------------------
CW_RADIUS_MM = 52.0       # pocket centre, on the side opposite the arm
COIN_G = 7.12             # UK 2p (post-1992). 1p = 3.56 g
COINS = 7
CW_PLASTIC_G = 7.0        # extra printed plastic + lid, CoM ~ 45 mm
CW_PLASTIC_R = 45.0


def main():
    m_arm = sum(m for _, m, _ in ARM_EXTENDED)
    mom_ext = sum(m * d for _, m, d in ARM_EXTENDED)
    mom_tuck = m_arm * ARM_TUCKED_COM_MM
    cw_mass = COINS * COIN_G + CW_PLASTIC_G
    cw_mom = COINS * COIN_G * CW_RADIUS_MM + CW_PLASTIC_G * CW_PLASTIC_R
    e = EDGE_WORST_MM

    print(f"arm mass {m_arm:.0f} g, moment extended {mom_ext:.0f} g.mm, tucked {mom_tuck:.0f} g.mm")
    print(f"counterweight {cw_mass:.1f} g, moment {cw_mom:.0f} g.mm ({COINS} coins)\n")

    def tip_margin(arm_mom, cw_on):
        """Resisting minus overturning moment about the foot edge under the arm (g.mm).
        Positive = stable."""
        over = arm_mom
        resist = (STATIC_MASS_G + m_arm) * e
        if cw_on:
            resist += cw_mass * e + cw_mom
        return resist - over

    def back_margin(arm_mom):
        """Same, about the foot edge under the counterweight (arm tucked)."""
        return (STATIC_MASS_G + m_arm) * e + arm_mom + cw_mass * e - cw_mom

    for label, cw in (("WITHOUT counterweight", False), ("WITH counterweight", True)):
        mg = tip_margin(mom_ext, cw)
        print(f"{label:22s} forward tip margin (arm out, diagonal): {mg:+6.0f} g.mm "
              f"-> {'STABLE' if mg > 0 else 'TIPS OVER'}")
    mb = back_margin(mom_tuck)
    print(f"{'WITH counterweight':22s} backward tip margin (arm tucked):      {mb:+6.0f} g.mm "
          f"-> {'STABLE' if mb > 0 else 'TIPS BACKWARD'}")

    need = (mom_ext - (STATIC_MASS_G + m_arm) * e - CW_PLASTIC_G * (CW_PLASTIC_R + e)) / (CW_RADIUS_MM + e)
    best = ((mom_ext + mom_tuck) / 2 - CW_PLASTIC_G * CW_PLASTIC_R) / CW_RADIUS_MM
    print(f"\nminimum coin mass to stop forward tipping : {max(need, 0):5.1f} g ({math.ceil(max(need, 0) / COIN_G)} x 2p)")
    print(f"mass that balances the servo shaft best   : {best:5.1f} g  "
          f"(equal +/- moment between arm out and tucked)")
    print(f"net side moment on base servo, arm out    : {mom_ext - cw_mom:+6.0f} g.mm (was {mom_ext:.0f})")
    print(f"net side moment on base servo, arm tucked : {mom_tuck - cw_mom:+6.0f} g.mm (was {mom_tuck:.0f})")
    i_arm = sum(m * d * d for _, m, d in ARM_EXTENDED)
    i_cw = COINS * COIN_G * CW_RADIUS_MM ** 2 + CW_PLASTIC_G * CW_PLASTIC_R ** 2
    print(f"yaw inertia increase                      : +{100 * i_cw / i_arm:.0f} % (slower base accel)")


if __name__ == "__main__":
    main()
