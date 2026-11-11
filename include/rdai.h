/* rdai.h — C ABI of reactive-driving-ai (crates/rdai-ffi). Keep in sync with src/lib.rs.
 * Units: SI (metres, seconds, radians), right-handed 2D, angles CCW from +X.
 * Unreal (cm, left-handed, Z-up): x_m = X/100, y_m = -Y/100, heading = -Yaw (rad). */
#ifndef RDAI_H
#define RDAI_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define RDAI_ABI_VERSION 1

typedef struct { float x, y; } RdaiVec2;
typedef struct { float x, y, width_right, width_left; } RdaiTrackPoint;
typedef struct {
    float max_lat_accel, max_accel, max_brake, power_to_mass, drag_per_mass, max_speed;
} RdaiVehicleLimits;
typedef struct {
    float wheelbase, rear_axle_offset, max_steer, length, width, understeer_gradient;
} RdaiVehicleGeometry;
typedef struct { float experience, stamina, aggression; } RdaiDriverProfile;
typedef struct { RdaiVec2 position; float heading, speed; } RdaiObservation;
typedef struct { RdaiVec2 position; float heading, speed, accel, length, width; } RdaiObstacle;
/* mode: 0 cruise, 1 follow, 2 avoid, 3 emergency */
typedef struct { float steer, throttle, brake; uint8_t reverse, mode; } RdaiControls;

typedef struct RdaiTrack RdaiTrack;
typedef struct RdaiAgent RdaiAgent;

uint32_t rdai_abi_version(void);
RdaiTrack *rdai_track_new(const RdaiTrackPoint *points, size_t n, float spacing, float edge_margin,
                          const RdaiVehicleLimits *limits);
void rdai_track_free(RdaiTrack *t);
float rdai_track_length(const RdaiTrack *t);
float rdai_track_ideal_lap(const RdaiTrack *t);
size_t rdai_track_racing_line(const RdaiTrack *t, RdaiVec2 *out, size_t cap);
RdaiAgent *rdai_agent_new(const RdaiVehicleGeometry *g, const RdaiDriverProfile *p, uint64_t seed);
void rdai_agent_free(RdaiAgent *a);
int rdai_agent_tick(RdaiAgent *a, const RdaiTrack *t, const RdaiObservation *obs,
                    const RdaiObstacle *obstacles, size_t n_obstacles, float dt, RdaiControls *out);
void rdai_agent_get_profile(const RdaiAgent *a, RdaiDriverProfile *out);
void rdai_agent_set_profile(RdaiAgent *a, const RdaiDriverProfile *p);

#ifdef __cplusplus
}
#endif
#endif
