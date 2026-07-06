// pmove.h - ported ReGameDLL player-move primitives used by the `sim` detector.
#pragma once
#include "trace.h"

namespace pw {

#define MOVETYPE_WALK   3
#define MAX_CLIP_PLANES 5
#define STOP_EPSILON    0.1f

// PM_ClipVelocity - pm_shared.cpp:562. out = in - normal*(dot(in,normal)*ob);
// components within +/-STOP_EPSILON snapped to 0. Returns blocked flags.
int  PM_ClipVelocity(const vec3_t in, const vec3_t normal, vec3_t out, float overbounce);

// PM_FlyMove - pm_shared.cpp:816. Slides pm->origin/velocity for one frame
// (pm->frametime) via up to 4 swept traces. Returns blocked flags.
int  PM_FlyMove(PmContext* pm);

// Half-split gravity around the move (pm_shared.cpp:606/628).
void PM_AddCorrectGravity(PmContext* pm);
void PM_FixupGravityVelocity(PmContext* pm);

// PM_CategorizePosition - pm_shared.cpp:1620. Sets pm->onground via a 2u down
// trace (onground iff ground plane normal.z >= 0.7) and snaps origin down.
void PM_CategorizePosition(PmContext* pm);

// clamp NaN + sv_maxvelocity (pm_shared.cpp:524).
void PM_CheckVelocity(PmContext* pm);

// Ground/air acceleration toward wishdir (pm_shared.cpp:998 / :1336).
void PM_Accelerate(PmContext* pm, const vec3_t wishdir, float wishspeed, float accel);
void PM_AirAccelerate(PmContext* pm, const vec3_t wishdir, float wishspeed, float accel);

// PM_Friction - pm_shared.cpp:1251 (incl. the edgefriction down-trace).
void PM_Friction(PmContext* pm);

// PM_WalkMove (:1039, with stair-step) / PM_AirMove (:1490).
void PM_WalkMove(PmContext* pm);
void PM_AirMove(PmContext* pm);

// One full MOVETYPE_WALK player frame (pm_shared.cpp:3258 sequence, non-water):
// AngleVectors -> gravity -> friction/accel -> walk|air move -> categorize ->
// fixup gravity. Reads pm->angles + pm->cmd_forwardmove/sidemove + movevars.
void PM_PlayerMoveFrame(PmContext* pm);
} // namespace pw
