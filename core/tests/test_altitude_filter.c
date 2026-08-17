#include "fc/altitude_filter.h"
#include "test_harness.h"

FC_TEST_MAIN_BEGIN()

/* Filter should settle near a constant baro reading when there's no
 * acceleration (sitting on the pad). */
{
    fc_altitude_filter_t f;
    fc_altitude_filter_init(&f, 0.0f);

    for (int i = 0; i < 200; i++) {
        fc_altitude_filter_predict(&f, 0.0f, 0.01f);
        fc_altitude_filter_update_baro(&f, 100.0f);
    }

    FC_CHECK_NEAR(f.x[0], 100.0f, 0.5f);
    FC_CHECK_NEAR(f.x[1], 0.0f, 0.5f);
}

/* Constant upward acceleration should show up as increasing altitude and
 * velocity even without baro corrections during the burst. */
{
    fc_altitude_filter_t f;
    fc_altitude_filter_init(&f, 0.0f);

    float dt = 0.01f;
    for (int i = 0; i < 100; i++) { /* 1s of 20 m/s^2 upward */
        fc_altitude_filter_predict(&f, 20.0f, dt);
    }

    /* analytic: v = a*t = 20, alt = 0.5*a*t^2 = 10 */
    FC_CHECK_NEAR(f.x[1], 20.0f, 2.0f);
    FC_CHECK_NEAR(f.x[0], 10.0f, 2.0f);
}

FC_TEST_MAIN_END()
