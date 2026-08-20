/*
 * zheight_demo.c
 *
 * Example usage / smoke test for the Z-height controller start
 * 40 sensor units above target, apply the crude plant model
 * freq -= step * 1.2 (approximated in integer math as (step*6)/5), and
 * show the controller settle without oscillating.
 */

#include <stdio.h>

#include "zheight.h"

int main(void)
{
    ZHeightController ctl;

    int rc = zctl_init(&ctl,
                        10000,            /* freq_target  */
                        3,                /* deadband_in  */
                        8,                /* deadband_out */
                        to_q(0, 3, 10),   /* kp = 0.3     */
                        to_q(0, 8, 10),   /* kd = 0.8     */
                        20,               /* max_step     */
                        to_q(0, 4, 10));  /* slew = 0.4   */

    if (rc != 0) {
        fprintf(stderr, "zctl_init failed: invalid deadband configuration\n");
        return 1;
    }

    int32_t freq = 10040;

    printf("sample  freq   step  correcting\n");
    for (int i = 0; i < 40; i++) {
        int32_t step = zctl_update(&ctl, freq);
        /* crude plant model: motor step reduces frequency error ~1.2x */
        freq -= (int32_t)(((int64_t)step * 6) / 5);
        printf("%6d  %5d  %4d  %d\n", i, freq, step, ctl.correcting);
    }

    return 0;
}
