/*
 * test_zheight.c
 *
 * Tests for the integer-only Z-height controller.
 *
 * Expected values are derived by hand from the documented contract in
 * zheight.h / zheight.c (Q16.16 PD + hysteresis + slew limit + clamp),
 * not read back out of the implementation. A test that fails is therefore
 * a real disagreement between the code and its stated design.
 *
 * Sign convention under test: zctl_update() returns a step whose sign
 * drives the sensor reading back toward the target, matching the demo's
 * plant model (freq -= step). A positive error yields a positive step.
 */

#include "test_harness.h"
#include "zheight.h"

/* Config used by the demo, and by most tests below. */
#define T_TARGET   10000
#define T_DB_IN    3
#define T_DB_OUT   8
#define T_MAX_STEP 20

static void init_default(ZHeightController *ctl)
{
    int rc = zctl_init(ctl, T_TARGET, T_DB_IN, T_DB_OUT,
                       to_q(0, 3, 10),    /* kp = 0.3  */
                       to_q(0, 8, 10),    /* kd = 0.8  */
                       T_MAX_STEP,
                       to_q(0, 4, 10));   /* slew = 0.4 */
    CHECK_EQ(rc, 0);
}

/* Init with an explicit slew limit, holding the rest of the demo config. */
static void init_with_slew(ZHeightController *ctl, int32_t slew_q)
{
    int rc = zctl_init(ctl, T_TARGET, T_DB_IN, T_DB_OUT,
                       to_q(0, 3, 10), to_q(0, 8, 10), T_MAX_STEP, slew_q);
    CHECK_EQ(rc, 0);
}

/* ---------------------------------------------------------------------
 * Fixed-point helpers
 * ------------------------------------------------------------------- */

TEST(to_q_builds_expected_fixed_point_values)
{
    CHECK_EQ(to_q(1, 0, 1), Q_ONE);           /* 1.0 */
    CHECK_EQ(to_q(0, 1, 2), Q_ONE / 2);       /* 0.5 */
    CHECK_EQ(to_q(2, 1, 4), 2 * Q_ONE + Q_ONE / 4);
}

TEST(to_q_handles_negative_whole_and_fraction)
{
    CHECK_EQ(to_q(-1, 0, 1), -Q_ONE);
    CHECK_EQ(to_q(0, -1, 2), -(Q_ONE / 2));
}

TEST(q_mul_multiplies_in_fixed_point)
{
    CHECK_EQ(q_mul(Q_ONE, Q_ONE), Q_ONE);                 /* 1 * 1 = 1 */
    CHECK_EQ(q_mul(Q_ONE / 2, 4 * Q_ONE), 2 * Q_ONE);     /* 0.5 * 4 = 2 */
    CHECK_EQ(q_mul(0, 5 * Q_ONE), 0);
}

TEST(q_mul_is_sign_correct)
{
    CHECK_EQ(q_mul(-(Q_ONE / 2), 4 * Q_ONE), -(2 * Q_ONE));
    CHECK_EQ(q_mul(-(Q_ONE / 2), -(4 * Q_ONE)), 2 * Q_ONE);
}

TEST(q_mul_does_not_overflow_on_large_operands)
{
    /* 200.0 * 100.0 = 20000.0. The raw int32 product of the two Q16.16
     * operands overflows (200<<16 * 100<<16 needs 48 bits), so this only
     * works because q_mul widens to int64 before shifting. The result,
     * 20000.0, still fits in Q16.16 (whole part max ~32767). */
    CHECK_EQ(q_mul(to_q(200, 0, 1), to_q(100, 0, 1)), to_q(20000, 0, 1));
}

/* ---------------------------------------------------------------------
 * Initialization and config validation
 * ------------------------------------------------------------------- */

TEST(init_stores_config_and_zeroes_state)
{
    ZHeightController ctl;
    int rc = zctl_init(&ctl, 10000, 3, 8, to_q(0, 3, 10), to_q(0, 8, 10),
                       20, to_q(0, 4, 10));

    CHECK_EQ(rc, 0);
    CHECK_EQ(ctl.freq_target, 10000);
    CHECK_EQ(ctl.deadband_in, 3);
    CHECK_EQ(ctl.deadband_out, 8);
    CHECK_EQ(ctl.kp_q, to_q(0, 3, 10));
    CHECK_EQ(ctl.kd_q, to_q(0, 8, 10));
    CHECK_EQ(ctl.max_step, 20);
    CHECK_EQ(ctl.slew_q, to_q(0, 4, 10));

    CHECK_EQ(ctl.prev_error, 0);
    CHECK_EQ(ctl.last_output_q, 0);
    CHECK_EQ(ctl.correcting, 0);
}

TEST(init_rejects_equal_deadbands)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, 10000, 5, 5, Q_ONE, Q_ONE, 20, Q_ONE), -1);
}

TEST(init_rejects_inverted_deadbands)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, 10000, 9, 4, Q_ONE, Q_ONE, 20, Q_ONE), -1);
}

TEST(init_accepts_minimum_valid_deadband_gap)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, 10000, 3, 4, Q_ONE, Q_ONE, 20, Q_ONE), 0);
}

/* ---------------------------------------------------------------------
 * Hysteresis deadband state machine
 * ------------------------------------------------------------------- */

TEST(holds_when_error_is_zero)
{
    ZHeightController ctl;
    init_default(&ctl);

    CHECK_EQ(zctl_update(&ctl, T_TARGET), 0);
    CHECK_EQ(ctl.correcting, 0);
}

TEST(holds_while_error_is_within_outer_deadband)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Outer band is 8; entry requires abs_err > 8, so 8 itself still holds. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + T_DB_OUT), 0);
    CHECK_EQ(ctl.correcting, 0);
}

TEST(enters_correcting_only_beyond_outer_deadband)
{
    ZHeightController ctl;
    init_default(&ctl);

    CHECK_EQ(zctl_update(&ctl, T_TARGET + T_DB_OUT + 1), 0); /* slew-limited to 0 */
    CHECK_EQ(ctl.correcting, 1);
}

TEST(enters_correcting_for_negative_error_too)
{
    ZHeightController ctl;
    init_default(&ctl);

    zctl_update(&ctl, T_TARGET - (T_DB_OUT + 1));
    CHECK_EQ(ctl.correcting, 1);
}

TEST(keeps_correcting_between_inner_and_outer_deadbands)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Enter correcting well outside the band... */
    zctl_update(&ctl, T_TARGET + 40);
    CHECK_EQ(ctl.correcting, 1);

    /* ...then sit between inner (3) and outer (8). Hysteresis means we stay
     * correcting rather than flipping back to hold. */
    zctl_update(&ctl, T_TARGET + 5);
    CHECK_EQ(ctl.correcting, 1);
}

TEST(exits_correcting_only_inside_inner_deadband)
{
    ZHeightController ctl;
    init_default(&ctl);

    zctl_update(&ctl, T_TARGET + 40);
    CHECK_EQ(ctl.correcting, 1);

    /* Inner band is 3; exit requires abs_err < 3, so 3 itself keeps going. */
    zctl_update(&ctl, T_TARGET + T_DB_IN);
    CHECK_EQ(ctl.correcting, 1);

    zctl_update(&ctl, T_TARGET + T_DB_IN - 1);
    CHECK_EQ(ctl.correcting, 0);
}

TEST(hold_returns_zero_step_and_clears_output_history)
{
    ZHeightController ctl;
    init_default(&ctl);

    zctl_update(&ctl, T_TARGET + 40);
    zctl_update(&ctl, T_TARGET + 30);   /* build up some output history */

    CHECK_EQ(zctl_update(&ctl, T_TARGET), 0);
    CHECK_EQ(ctl.last_output_q, 0);
    CHECK_EQ(ctl.prev_error, 0);
}

TEST(hold_tracks_prev_error_so_derivative_does_not_jump_on_entry)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* A holding sample must still record its error; otherwise the first
     * correcting sample sees a fabricated derivative spike. */
    zctl_update(&ctl, T_TARGET + 4);
    CHECK_EQ(ctl.prev_error, 4);
}

/* ---------------------------------------------------------------------
 * Slew-rate limiting
 * ------------------------------------------------------------------- */

TEST(slew_limit_caps_first_step_from_standstill)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Hold at +40 sets prev_error = 40 without correcting... no: +40 is
     * outside the outer band, so this sample enters correcting with
     * prev_error still 0 from init. d_error = 40 - 0 = 40.
     * raw output = 0.3*40 + 0.8*40 = 44.0, but slew caps the change from
     * last_output_q (0) to 0.4 -> step = floor(0.4) = 0. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), 0);
}

TEST(slew_limit_ramp_accumulates_across_samples_and_reaches_a_real_step)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* The fractional ramp must persist between samples. At slew = 0.4 with
     * the PD term demanding far more, the internal output climbs
     * 0.4 -> 0.8 -> 1.2 -> ..., so the emitted integer step is 0, 0, then 1.
     * If the accumulator were discarded each tick the ramp would restart
     * from zero and the controller would stall at 0 forever. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), 0);
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), 0);
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), 1);
}

TEST(controller_eventually_commands_motion_for_a_persistent_error)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Regression guard for the stall bug: a steady out-of-band error must
     * produce a nonzero step within a handful of samples, no matter how
     * the ramp is implemented internally. */
    int moved = 0;
    for (int i = 0; i < 10; i++) {
        if (zctl_update(&ctl, T_TARGET + 40) != 0) {
            moved = 1;
            break;
        }
    }
    CHECK_TRUE(moved);
}

TEST(larger_slew_allows_immediate_nonzero_step)
{
    ZHeightController ctl;
    init_with_slew(&ctl, to_q(5, 0, 1));   /* slew = 5.0 steps/sample */

    /* error = 40, prev_error = 0 -> raw = 0.3*40 + 0.8*40 = 44.0.
     * Slew caps the delta from 0 to 5.0 -> step = 5. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), 5);
}

TEST(slew_limit_bounds_step_change_between_consecutive_samples)
{
    ZHeightController ctl;
    init_with_slew(&ctl, to_q(2, 0, 1));   /* slew = 2.0 steps/sample */

    int32_t prev = 0;
    int32_t freqs[] = { T_TARGET + 40, T_TARGET + 35, T_TARGET - 30,
                        T_TARGET + 25, T_TARGET - 20 };

    for (int i = 0; i < 5; i++) {
        int32_t step = zctl_update(&ctl, freqs[i]);
        int32_t change = step - prev;
        if (change < 0) change = -change;
        /* Allow 2 (the limit) plus 1 for the truncation of the fractional
         * part when converting Q16.16 -> integer steps. */
        CHECK_LE(change, 3);
        prev = step;
    }
}

TEST(slew_limit_permits_unlimited_output_when_very_large)
{
    ZHeightController ctl;
    init_with_slew(&ctl, to_q(1000, 0, 1));

    /* Slew no longer binds: raw = 0.3*40 + 0.8*40 = 44.0, clamped by
     * max_step = 20. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40), T_MAX_STEP);
}

/* ---------------------------------------------------------------------
 * PD arithmetic (with slew effectively disabled so the term shows through)
 * ------------------------------------------------------------------- */

TEST(proportional_term_scales_with_error)
{
    ZHeightController ctl;
    /* kp = 0.5, kd = 0, slew huge, max_step large: step = 0.5 * error. */
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(0, 5, 10), 0,
                       10000, to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET + 100), 50);
}

TEST(proportional_term_is_signed)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(0, 5, 10), 0,
                       10000, to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET - 100), -50);
}

TEST(derivative_term_responds_to_error_change)
{
    ZHeightController ctl;
    /* kp = 0, kd = 1.0: step = 1.0 * (error - prev_error). */
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, 0, to_q(1, 0, 1),
                       10000, to_q(30000, 0, 1)), 0);

    /* First correcting sample: error 100, prev_error 0 -> d = 100. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 100), 100);

    /* Error shrinks to 60: d = 60 - 100 = -40. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 60), -40);
}

TEST(derivative_term_is_zero_when_error_is_steady)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, 0, to_q(1, 0, 1),
                       10000, to_q(30000, 0, 1)), 0);

    zctl_update(&ctl, T_TARGET + 100);
    /* Same error twice -> derivative contributes nothing. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 100), 0);
}

TEST(pd_terms_sum)
{
    ZHeightController ctl;
    /* kp = 0.5, kd = 2.0. First sample: error 100, prev 0.
     * step = 0.5*100 + 2.0*100 = 250. */
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(0, 5, 10), to_q(2, 0, 1),
                       10000, to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET + 100), 250);
}

/* ---------------------------------------------------------------------
 * Hard step clamp
 * ------------------------------------------------------------------- */

TEST(clamps_positive_step_to_max_step)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(2, 0, 1), 0,
                       15, to_q(30000, 0, 1)), 0);

    /* raw = 2.0 * 1000 = 2000, clamped to 15. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 1000), 15);
}

TEST(clamps_negative_step_to_negative_max_step)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(2, 0, 1), 0,
                       15, to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET - 1000), -15);
}

TEST(zero_max_step_pins_output_to_zero)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, to_q(2, 0, 1), 0,
                       0, to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET + 1000), 0);
}

TEST(never_exceeds_max_step_over_a_long_noisy_run)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Deterministic pseudo-noise; no rand() so the run is reproducible.
     * Unsigned math: signed overflow would be UB and trips the sanitizer. */
    uint32_t x = 12345u;
    for (int i = 0; i < 500; i++) {
        x = (x * 1103515245u + 12345u) & 0x7fffffffu;
        int32_t offset = (int32_t)(x % 401u) - 200;   /* -200 .. +200 */
        int32_t step = zctl_update(&ctl, T_TARGET + offset);
        CHECK_EQ_AT(step > T_MAX_STEP || step < -T_MAX_STEP, 0, "i", i);
    }
}

/* ---------------------------------------------------------------------
 * Closed-loop behavior: the actual anti-oscillation requirement
 * ------------------------------------------------------------------- */

/* Run the demo plant model and report the settled state. */
typedef struct {
    int32_t final_freq;
    int     direction_reversals;
    int     samples_to_settle;   /* -1 if never settled */
} SimResult;

static SimResult simulate(ZHeightController *ctl, int32_t start_freq, int n)
{
    SimResult r;
    r.final_freq = start_freq;
    r.direction_reversals = 0;
    r.samples_to_settle = -1;

    int32_t freq = start_freq;
    int32_t prev_step = 0;
    int settled_streak = 0;

    for (int i = 0; i < n; i++) {
        int32_t step = zctl_update(ctl, freq);

        if ((step > 0 && prev_step < 0) || (step < 0 && prev_step > 0)) {
            r.direction_reversals++;
        }
        if (step != 0) prev_step = step;

        /* crude plant model: motor step reduces frequency error ~1.2x */
        freq -= (int32_t)(((int64_t)step * 6) / 5);

        int32_t err = freq - ctl->freq_target;
        if (err < 0) err = -err;
        if (err <= ctl->deadband_out) {
            settled_streak++;
            if (settled_streak >= 5 && r.samples_to_settle < 0) {
                r.samples_to_settle = i;
            }
        } else {
            settled_streak = 0;
        }
    }

    r.final_freq = freq;
    return r;
}

TEST(converges_into_the_target_window_from_above)
{
    ZHeightController ctl;
    init_default(&ctl);

    SimResult r = simulate(&ctl, T_TARGET + 40, 200);

    int32_t err = r.final_freq - T_TARGET;
    if (err < 0) err = -err;
    CHECK_LE(err, T_DB_OUT);
}

TEST(converges_into_the_target_window_from_below)
{
    ZHeightController ctl;
    init_default(&ctl);

    SimResult r = simulate(&ctl, T_TARGET - 40, 200);

    int32_t err = r.final_freq - T_TARGET;
    if (err < 0) err = -err;
    CHECK_LE(err, T_DB_OUT);
}

TEST(does_not_rubber_band_once_settled)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* The whole point of the rewrite: no sustained direction flipping.
     * A handful of reversals during the approach is fine; dozens means
     * the limit cycle is back. */
    SimResult r = simulate(&ctl, T_TARGET + 40, 200);
    CHECK_LE(r.direction_reversals, 4);
}

TEST(settles_within_a_reasonable_number_of_samples)
{
    ZHeightController ctl;
    init_default(&ctl);

    SimResult r = simulate(&ctl, T_TARGET + 40, 200);

    /* At 256 Hz, 128 samples is half a second to close a 40-unit error. */
    CHECK_TRUE(r.samples_to_settle >= 0);
    CHECK_LE(r.samples_to_settle, 128);
}

TEST(stays_quiet_under_small_sensor_noise_inside_the_window)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Noise strictly inside the outer deadband must never command motion.
     * This is the sensor-jitter case that made the old controller chatter. */
    int32_t offsets[] = { 0, 2, -2, 5, -5, 8, -8, 1, -1, 7, -7, 0 };
    for (int i = 0; i < 12; i++) {
        CHECK_EQ_AT(zctl_update(&ctl, T_TARGET + offsets[i]), 0, "i", i);
    }
}

TEST(recovers_after_a_disturbance_pushes_it_out_of_band)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* Settle, then kick it hard and confirm it comes back. */
    simulate(&ctl, T_TARGET + 40, 200);
    SimResult r = simulate(&ctl, T_TARGET + 60, 200);

    int32_t err = r.final_freq - T_TARGET;
    if (err < 0) err = -err;
    CHECK_LE(err, T_DB_OUT);
}

/* ---------------------------------------------------------------------
 * Numeric edge cases
 * ------------------------------------------------------------------- */

TEST(handles_large_error_without_undefined_shift_overflow)
{
    ZHeightController ctl;
    init_default(&ctl);

    /* error << Q_SHIFT overflows int32 once |error| >= 32768. The
     * controller must still produce a bounded, correctly-signed step for a
     * sensor reading this far off target. */
    int32_t step = zctl_update(&ctl, T_TARGET + 100000);
    CHECK_LE(step, T_MAX_STEP);
    CHECK_TRUE(step >= 0);
}

/* Saturation at the fixed-point boundary. An error beyond the Q16.16 range
 * must saturate to full authority in the correct direction, not wrap around
 * to the opposite sign -- a sign inversion here would drive the head into
 * the workpiece. Slew is set wide so the PD term reaches the clamp on the
 * first sample. */
TEST(saturates_positively_for_error_beyond_fixed_point_range)
{
    ZHeightController ctl;
    init_with_slew(&ctl, to_q(30000, 0, 1));

    /* |error| = 40000 > 32767, so error << 16 cannot be represented. */
    CHECK_EQ(zctl_update(&ctl, T_TARGET + 40000), T_MAX_STEP);
}

TEST(saturates_negatively_for_error_beyond_fixed_point_range)
{
    ZHeightController ctl;
    init_with_slew(&ctl, to_q(30000, 0, 1));

    CHECK_EQ(zctl_update(&ctl, T_TARGET - 40000), -T_MAX_STEP);
}

TEST(step_sign_is_never_inverted_across_a_wide_error_sweep)
{
    /* Sweep errors through and well past the Q16.16 boundary. The commanded
     * step must always share the sign of the error (or be zero); it must
     * never point the wrong way. */
    int32_t errors[] = { 20, 100, 1000, 30000, 32767, 32768, 50000,
                         100000, 1000000 };

    for (int i = 0; i < 9; i++) {
        ZHeightController up, down;
        init_with_slew(&up, to_q(30000, 0, 1));
        init_with_slew(&down, to_q(30000, 0, 1));

        CHECK_EQ_AT(zctl_update(&up, T_TARGET + errors[i]) > 0, 1,
                    "err", errors[i]);
        CHECK_EQ_AT(zctl_update(&down, T_TARGET - errors[i]) < 0, 1,
                    "err", errors[i]);
    }
}

TEST(handles_large_negative_error_without_overflow)
{
    ZHeightController ctl;
    init_default(&ctl);

    int32_t step = zctl_update(&ctl, T_TARGET - 100000);
    CHECK_TRUE(step >= -T_MAX_STEP);
    CHECK_TRUE(step <= 0);
}

TEST(works_with_zero_target)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, 0, 3, 8, to_q(0, 5, 10), 0, 20,
                       to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, 40), 20);   /* 0.5*40 = 20, at the clamp */
    CHECK_EQ(ctl.correcting, 1);
}

TEST(works_with_negative_sensor_readings)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, 0, 3, 8, to_q(0, 5, 10), 0, 20,
                       to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, -40), -20);
}

TEST(zero_gains_command_no_motion)
{
    ZHeightController ctl;
    CHECK_EQ(zctl_init(&ctl, T_TARGET, 3, 8, 0, 0, 20,
                       to_q(30000, 0, 1)), 0);

    CHECK_EQ(zctl_update(&ctl, T_TARGET + 1000), 0);
    /* Still tracks that it *should* be correcting, even with no authority. */
    CHECK_EQ(ctl.correcting, 1);
}

/* ------------------------------------------------------------------- */

int main(void)
{
    RUN(to_q_builds_expected_fixed_point_values);
    RUN(to_q_handles_negative_whole_and_fraction);
    RUN(q_mul_multiplies_in_fixed_point);
    RUN(q_mul_is_sign_correct);
    RUN(q_mul_does_not_overflow_on_large_operands);

    RUN(init_stores_config_and_zeroes_state);
    RUN(init_rejects_equal_deadbands);
    RUN(init_rejects_inverted_deadbands);
    RUN(init_accepts_minimum_valid_deadband_gap);

    RUN(holds_when_error_is_zero);
    RUN(holds_while_error_is_within_outer_deadband);
    RUN(enters_correcting_only_beyond_outer_deadband);
    RUN(enters_correcting_for_negative_error_too);
    RUN(keeps_correcting_between_inner_and_outer_deadbands);
    RUN(exits_correcting_only_inside_inner_deadband);
    RUN(hold_returns_zero_step_and_clears_output_history);
    RUN(hold_tracks_prev_error_so_derivative_does_not_jump_on_entry);

    RUN(slew_limit_caps_first_step_from_standstill);
    RUN(slew_limit_ramp_accumulates_across_samples_and_reaches_a_real_step);
    RUN(controller_eventually_commands_motion_for_a_persistent_error);
    RUN(larger_slew_allows_immediate_nonzero_step);
    RUN(slew_limit_bounds_step_change_between_consecutive_samples);
    RUN(slew_limit_permits_unlimited_output_when_very_large);

    RUN(proportional_term_scales_with_error);
    RUN(proportional_term_is_signed);
    RUN(derivative_term_responds_to_error_change);
    RUN(derivative_term_is_zero_when_error_is_steady);
    RUN(pd_terms_sum);

    RUN(clamps_positive_step_to_max_step);
    RUN(clamps_negative_step_to_negative_max_step);
    RUN(zero_max_step_pins_output_to_zero);
    RUN(never_exceeds_max_step_over_a_long_noisy_run);

    RUN(converges_into_the_target_window_from_above);
    RUN(converges_into_the_target_window_from_below);
    RUN(does_not_rubber_band_once_settled);
    RUN(settles_within_a_reasonable_number_of_samples);
    RUN(stays_quiet_under_small_sensor_noise_inside_the_window);
    RUN(recovers_after_a_disturbance_pushes_it_out_of_band);

    RUN(handles_large_error_without_undefined_shift_overflow);
    RUN(saturates_positively_for_error_beyond_fixed_point_range);
    RUN(saturates_negatively_for_error_beyond_fixed_point_range);
    RUN(step_sign_is_never_inverted_across_a_wide_error_sweep);
    RUN(handles_large_negative_error_without_overflow);
    RUN(works_with_zero_target);
    RUN(works_with_negative_sensor_readings);
    RUN(zero_gains_command_no_motion);

    return test_summary();
}
