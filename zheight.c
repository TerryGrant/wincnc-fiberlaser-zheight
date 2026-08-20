/*
 * zheight.c
 *
 * Integer-only Z-axis (laser height) controller.
 *
 * Solves the "rubber-banding" / limit-cycle oscillation that occurs when a
 * bang-bang (on/off, full-correction) controller is used to track a sensor
 * frequency reading against a target window.
 *
 * Root causes of the oscillation:
 *   1. No hysteresis      -> noise or tiny residual error right at a single
 *                            threshold flips direction every sample.
 *   2. No proportional scaling -> small errors get the same size correction
 *                            as large ones, guaranteeing overshoot near the
 *                            setpoint.
 *   3. No derivative damping   -> nothing anticipates/cancels overshoot
 *                            caused by sensor + motor lag, so it "rings."
 *   4. No slew-rate limit      -> commanded motion can change abruptly
 *                            between samples, causing violent direction
 *                            reversals.
 *
 * Fix: fixed-point (Q16.16) PD control + hysteresis deadband + slew-rate
 * limiting + hard step clamp. No floating point anywhere -- only shifts,
 * multiplies, adds.
 *
 * Sample rate assumed: 256 Hz (call zctl_update() once per sensor read).
 *
 * Designed for embedded targets: no dynamic allocation, no libm, no
 * floating point. Uses int32_t / int64_t from <stdint.h> for portability.
 */

#include "zheight.h"

int zctl_init(ZHeightController *ctl,
              int32_t freq_target,
              int32_t deadband_in,
              int32_t deadband_out,
              int32_t kp_q,
              int32_t kd_q,
              int32_t max_step,
              int32_t slew_q)
{
    if (deadband_out <= deadband_in) {
        /* Hysteresis requires a real gap between the two thresholds,
         * otherwise you're back to single-line chatter. */
        return -1;
    }

    ctl->freq_target   = freq_target;
    ctl->deadband_in   = deadband_in;
    ctl->deadband_out  = deadband_out;
    ctl->kp_q          = kp_q;
    ctl->kd_q          = kd_q;
    ctl->max_step      = max_step;
    ctl->slew_q        = slew_q;

    ctl->prev_error    = 0;
    ctl->last_output_q = 0;
    ctl->correcting    = 0;

    return 0;
}

int32_t zctl_update(ZHeightController *ctl, int32_t freq_sample)
{
    int32_t error = freq_sample - ctl->freq_target;
    int32_t abs_err = (error >= 0) ? error : -error;

    /* --- Hysteresis deadband state machine --- */
    if (!ctl->correcting) {
        if (abs_err > ctl->deadband_out) {
            ctl->correcting = 1;
        } else {
            ctl->prev_error    = error;
            ctl->last_output_q = 0;
            return 0;
        }
    } else {
        if (abs_err < ctl->deadband_in) {
            ctl->correcting    = 0;
            ctl->prev_error    = error;
            ctl->last_output_q = 0;
            return 0;
        }
    }

    /* --- PD term in fixed point ---
     * q_from_int multiplies (never left-shifts) and saturates, so negative
     * errors stay well-defined and an error beyond the Q16.16 range pins to
     * full magnitude with its sign intact instead of wrapping. The error
     * difference is computed in 64-bit for the same reason. */
    int32_t error_q   = q_from_int(error);
    int32_t d_error_q = q_sat(((int64_t)error - (int64_t)ctl->prev_error)
                              * Q_ONE);

    int32_t p_term = q_mul(ctl->kp_q, error_q);
    int32_t d_term = q_mul(ctl->kd_q, d_error_q);
    /* Sum in 64-bit: two saturated terms can still overflow int32 together. */
    int32_t output_q = q_sat((int64_t)p_term + (int64_t)d_term);

    /* --- Slew-rate limit: cap how fast the commanded step itself can
     * change. This is what stops a hard direction reversal in a single
     * tick -- the output can only ramp toward a new direction. --- */
    int64_t delta = (int64_t)output_q - (int64_t)ctl->last_output_q;
    if (delta > (int64_t)ctl->slew_q) {
        output_q = q_sat((int64_t)ctl->last_output_q + (int64_t)ctl->slew_q);
    } else if (delta < -(int64_t)ctl->slew_q) {
        output_q = q_sat((int64_t)ctl->last_output_q - (int64_t)ctl->slew_q);
    }

    /* --- Convert back to integer motor steps, clamp to max_step --- */
    int32_t step = output_q >> Q_SHIFT;
    if (step > ctl->max_step) {
        step = ctl->max_step;
    } else if (step < -ctl->max_step) {
        step = -ctl->max_step;
    }

    ctl->prev_error = error;
    /* Store the pre-quantization output, NOT step << Q_SHIFT. The slew
     * limiter ramps last_output_q by at most slew_q per sample; if we wrote
     * back the truncated integer step instead, any ramp that had not yet
     * reached 1.0 would be rounded to 0 and restart from zero every sample,
     * so the output could never climb past the first fractional step and
     * the controller would stall at 0 forever. */
    ctl->last_output_q = output_q;
    return step;
}
