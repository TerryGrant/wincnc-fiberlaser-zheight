/*
 * zheight.h
 *
 * Integer-only Z-axis (laser height) controller: public interface.
 *
 * Fixed-point (Q16.16) PD control + hysteresis deadband + slew-rate limiting
 * + hard step clamp. No dynamic allocation, no libm, no floating point.
 *
 * Sample rate assumed: 256 Hz (call zctl_update() once per sensor read).
 */

#ifndef ZHEIGHT_H
#define ZHEIGHT_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 * Fixed-point Q16.16 helpers
 * ------------------------------------------------------------------- */

#define Q_SHIFT 16
#define Q_ONE   (1L << Q_SHIFT)

/* Largest / smallest values representable in Q16.16. */
#define Q_MAX   ((int32_t)0x7fffffff)
#define Q_MIN   ((int32_t)(-0x7fffffff - 1))

/* Saturate a 64-bit fixed-point intermediate down to int32 Q16.16.
 * Clamping (rather than wrapping) matters here: a wrapped value flips sign,
 * which on a height controller means commanding full travel in the wrong
 * direction. Saturating keeps the sign and just pins the magnitude. */
static inline int32_t q_sat(int64_t v) {
    if (v > (int64_t)Q_MAX) return Q_MAX;
    if (v < (int64_t)Q_MIN) return Q_MIN;
    return (int32_t)v;
}

/* Convert a whole number of sensor units to Q16.16, saturating.
 * Multiplies rather than shifts: left-shifting a negative value is
 * undefined behavior in C99, and error values are routinely negative. */
static inline int32_t q_from_int(int32_t v) {
    return q_sat((int64_t)v * Q_ONE);
}

/* Build a Q16.16 constant from whole + num/den at compile/init time.
 * Uses int64_t internally (and multiplication, not shifts, so negative
 * inputs stay well-defined) to avoid overflow before the divide. */
static inline int32_t to_q(int32_t whole, int32_t num, int32_t den) {
    int64_t whole_part = (int64_t)whole * Q_ONE;
    int64_t frac_part  = ((int64_t)num * Q_ONE) / den;
    return q_sat(whole_part + frac_part);
}

/* Multiply two Q16.16 numbers -> Q16.16 result.
 * Widen to int64_t before the shift to avoid intermediate overflow, then
 * saturate rather than wrap.
 *
 * The right shift is kept (rather than a divide) to preserve the original
 * floor-toward-negative-infinity rounding; `/ Q_ONE` truncates toward zero
 * and would silently change controller output for negative errors. A
 * right shift of a negative value is implementation-defined, not undefined,
 * and is arithmetic on every supported target. */
static inline int32_t q_mul(int32_t a, int32_t b) {
    int64_t product = (int64_t)a * (int64_t)b;
    return q_sat(product >> Q_SHIFT);
}

/* ---------------------------------------------------------------------
 * Controller state and configuration
 * ------------------------------------------------------------------- */

typedef struct {
    /* --- configuration (set once at init) --- */
    int32_t freq_target;   /* target sensor frequency, raw sensor units   */
    int32_t deadband_in;   /* inner half-width: stop correcting inside    */
    int32_t deadband_out;  /* outer half-width: start correcting outside  */
    int32_t kp_q;          /* proportional gain, Q16.16                   */
    int32_t kd_q;          /* derivative gain, Q16.16                     */
    int32_t max_step;      /* hard clamp on motor steps per sample        */
    int32_t slew_q;        /* max change in commanded step per sample, Q16.16 */

    /* --- runtime state (updated every call) --- */
    int32_t prev_error;
    int32_t last_output_q;
    uint8_t correcting;    /* hysteresis state: 0 = holding, 1 = correcting */
} ZHeightController;

/* Initialize the controller. Returns 0 on success, -1 on invalid config. */
int zctl_init(ZHeightController *ctl,
              int32_t freq_target,
              int32_t deadband_in,
              int32_t deadband_out,
              int32_t kp_q,
              int32_t kd_q,
              int32_t max_step,
              int32_t slew_q);

/* Call once per sensor sample (256 Hz).
 * Returns an integer motor step whose sign drives the sensor reading back
 * toward freq_target; 0 = hold position (inside the safety window). */
int32_t zctl_update(ZHeightController *ctl, int32_t freq_sample);

#endif /* ZHEIGHT_H */
