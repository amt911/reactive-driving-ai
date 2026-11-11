/* Links against target/release/librdai.a and drives one agent for a second. */
#include <math.h>
#include <stdio.h>
#include "rdai.h"

int main(void) {
    RdaiTrackPoint pts[24];
    for (int k = 0; k < 24; k++) {
        float a = (float)k / 24.0f * 6.2831853f;
        pts[k] = (RdaiTrackPoint){150.0f * cosf(a), 100.0f * sinf(a), 6.0f, 6.0f};
    }
    RdaiVehicleLimits lim = {12.0f, 7.0f, 12.0f, 330.0f, 0.00035f, 80.0f};
    RdaiTrack *t = rdai_track_new(pts, 24, 2.0f, 2.0f, &lim);
    if (!t || rdai_abi_version() != RDAI_ABI_VERSION) return 1;
    RdaiVehicleGeometry g = {2.7f, 1.35f, 0.45f, 4.6f, 1.9f, 0.0f};
    RdaiDriverProfile p = {0.7f, 1.0f, 0.2f};
    RdaiAgent *a = rdai_agent_new(&g, &p, 1);
    RdaiObservation o = {{150.0f, 0.0f}, 1.5707963f, 10.0f};
    RdaiObstacle stopped = {{140.0f, 40.0f}, 1.9f, 0.0f, 0.0f, 4.6f, 1.9f};
    RdaiControls c;
    for (int i = 0; i < 60; i++)
        if (rdai_agent_tick(a, t, &o, &stopped, 1, 1.0f / 60.0f, &c) != 0) return 2;
    printf("ok: lap %.2f s, steer %.3f throttle %.2f brake %.2f mode %u\n", rdai_track_ideal_lap(t), c.steer,
           c.throttle, c.brake, c.mode);
    rdai_agent_free(a);
    rdai_track_free(t);
    return 0;
}
