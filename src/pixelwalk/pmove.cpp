#include "pmove.h"
#include <cmath>

namespace pw {

static const vec3_t vec3_origin = { 0.0f, 0.0f, 0.0f };

// PM_CheckVelocity - clamp NaN + sv_maxvelocity.
void PM_CheckVelocity(PmContext* pm) {
    float mx = pm->mv_maxvelocity;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(pm->velocity[i])) pm->velocity[i] = 0.0f;
        if (std::isnan(pm->origin[i]))   pm->origin[i]   = 0.0f;
        if      (pm->velocity[i] >  mx) pm->velocity[i] =  mx;
        else if (pm->velocity[i] < -mx) pm->velocity[i] = -mx;
    }
}

// PM_ClipVelocity - pm_shared.cpp:562.
int PM_ClipVelocity(const vec3_t in, const vec3_t normal, vec3_t out, float overbounce) {
    float angle = normal[2];
    int blocked = 0x00;
    if (angle > 0)  blocked |= 0x01;   // floor
    if (angle == 0) blocked |= 0x02;   // wall/step

    float backoff = DotProduct(in, normal) * overbounce;
    for (int i = 0; i < 3; ++i) {
        float change = in[i] - normal[i] * backoff;
        out[i] = change;
        if (out[i] > -STOP_EPSILON && out[i] < STOP_EPSILON)
            out[i] = 0.0f;
    }
    return blocked;
}

void PM_AddCorrectGravity(PmContext* pm) {
    if (pm->waterjumptime) return;
    float ent_gravity = (pm->gravity != 0.0f) ? pm->gravity : 1.0f;
    pm->velocity[2] -= (ent_gravity * pm->mv_gravity * 0.5f * pm->frametime);
    pm->velocity[2] += pm->basevelocity[2] * pm->frametime;
    pm->basevelocity[2] = 0.0f;
    PM_CheckVelocity(pm);
}

void PM_FixupGravityVelocity(PmContext* pm) {
    if (pm->waterjumptime) return;
    float ent_gravity = (pm->gravity != 0.0f) ? pm->gravity : 1.0f;
    pm->velocity[2] -= (pm->mv_gravity * pm->frametime * ent_gravity * 0.5f);
    PM_CheckVelocity(pm);
}

// PM_FlyMove - pm_shared.cpp:816. PM_AddToTouched is a no-op for the finder.
int PM_FlyMove(PmContext* pm) {
    int   numbumps = 4;
    int   blocked = 0x00;
    int   numplanes = 0;
    vec3_t planes[MAX_CLIP_PLANES];
    vec3_t original_velocity, primal_velocity, new_velocity;
    vec3_t end, dir;
    int   i, j;

    VectorCopy(pm->velocity, original_velocity);
    VectorCopy(pm->velocity, primal_velocity);
    float allFraction = 0.0f;
    float time_left = pm->frametime;

    for (int bumpcount = 0; bumpcount < numbumps; ++bumpcount) {
        if (pm->velocity[0] == 0.0f && pm->velocity[1] == 0.0f && pm->velocity[2] == 0.0f)
            break;

        for (i = 0; i < 3; ++i)
            end[i] = pm->origin[i] + time_left * pm->velocity[i];

        pmtrace_t trace = PM_PlayerTrace(pm, pm->origin, end, PM_NORMAL, -1);
        allFraction += trace.fraction;

        if (trace.allsolid) {
            VectorCopy(vec3_origin, pm->velocity);
            return 4;
        }

        if (trace.fraction > 0.0f) {
            VectorCopy(trace.endpos, pm->origin);
            VectorCopy(pm->velocity, original_velocity);
            numplanes = 0;
        }

        if (trace.fraction == 1.0f)
            break;

        if (trace.plane.normal[2] > 0.7f)  blocked |= 0x01;   // floor
        if (trace.plane.normal[2] == 0.0f) blocked |= 0x02;   // step/wall

        time_left -= time_left * trace.fraction;

        if (numplanes >= MAX_CLIP_PLANES) {
            VectorCopy(vec3_origin, pm->velocity);
            break;
        }

        VectorCopy(trace.plane.normal, planes[numplanes]);
        numplanes++;

        if (numplanes == 1 && pm->movetype == MOVETYPE_WALK &&
            (pm->onground == -1 || pm->friction != 1.0f)) {
            for (i = 0; i < numplanes; ++i) {
                if (planes[i][2] > 0.7f) {
                    PM_ClipVelocity(original_velocity, planes[i], new_velocity, 1.0f);
                    VectorCopy(new_velocity, original_velocity);
                } else {
                    PM_ClipVelocity(original_velocity, planes[i], new_velocity,
                                    1.0f + pm->mv_bounce * (1.0f - pm->friction));
                }
            }
            VectorCopy(new_velocity, pm->velocity);
            VectorCopy(new_velocity, original_velocity);
        } else {
            for (i = 0; i < numplanes; ++i) {
                PM_ClipVelocity(original_velocity, planes[i], pm->velocity, 1.0f);
                for (j = 0; j < numplanes; ++j) {
                    if (j != i) {
                        if (DotProduct(pm->velocity, planes[j]) < 0.0f)
                            break;
                    }
                }
                if (j == numplanes)
                    break;
            }

            if (i == numplanes) {
                if (numplanes != 2) {
                    VectorCopy(vec3_origin, pm->velocity);
                    break;
                }
                CrossProduct(planes[0], planes[1], dir);
                float d = DotProduct(dir, pm->velocity);
                VectorScale(dir, d, pm->velocity);
            }

            if (DotProduct(pm->velocity, primal_velocity) <= 0.0f) {
                VectorCopy(vec3_origin, pm->velocity);
                break;
            }
        }
    }

    if (allFraction == 0.0f)
        VectorCopy(vec3_origin, pm->velocity);

    return blocked;
}

// PM_CategorizePosition - pm_shared.cpp:1620. Water handling omitted.
void PM_CategorizePosition(PmContext* pm) {
    vec3_t point;
    point[0] = pm->origin[0];
    point[1] = pm->origin[1];
    point[2] = pm->origin[2] - 2.0f;

    if (pm->velocity[2] > 180.0f) {
        pm->onground = -1;
        return;
    }

    pmtrace_t tr = PM_PlayerTrace(pm, pm->origin, point, PM_NORMAL, -1);

    if (tr.plane.normal[2] < 0.7f)
        pm->onground = -1;
    else
        pm->onground = tr.ent;

    if (pm->onground != -1) {
        pm->waterjumptime = 0.0f;
        if (pm->waterlevel < 2 && !tr.startsolid && !tr.allsolid)
            VectorCopy(tr.endpos, pm->origin);
    }
}

// PM_Accelerate - pm_shared.cpp:998.
void PM_Accelerate(PmContext* pm, const vec3_t wishdir, float wishspeed, float accel) {
    if (pm->dead || pm->waterjumptime) return;
    float currentspeed = DotProduct(pm->velocity, wishdir);
    float addspeed = wishspeed - currentspeed;
    if (addspeed <= 0) return;
    float accelspeed = accel * pm->frametime * wishspeed * pm->friction;
    if (accelspeed > addspeed) accelspeed = addspeed;
    for (int i = 0; i < 3; ++i) pm->velocity[i] += accelspeed * wishdir[i];
}

// PM_AirAccelerate - pm_shared.cpp:1336 (wishspeed capped at 30 in the air).
void PM_AirAccelerate(PmContext* pm, const vec3_t wishdir, float wishspeed, float accel) {
    if (pm->dead || pm->waterjumptime) return;
    float wishspd = wishspeed;
    if (wishspd > 30.0f) wishspd = 30.0f;
    float currentspeed = DotProduct(pm->velocity, wishdir);
    float addspeed = wishspd - currentspeed;
    if (addspeed <= 0) return;
    float accelspeed = accel * wishspeed * pm->frametime * pm->friction;
    if (accelspeed > addspeed) accelspeed = addspeed;
    for (int i = 0; i < 3; ++i) pm->velocity[i] += accelspeed * wishdir[i];
}

// PM_Friction - pm_shared.cpp:1251 (with the edgefriction ledge trace).
void PM_Friction(PmContext* pm) {
    if (pm->waterjumptime) return;
    float* vel = pm->velocity;
    float speed = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
    if (speed < 0.1f) return;
    float drop = 0.0f;
    if (pm->onground != -1) {
        vec3_t start, stop;
        start[0] = stop[0] = pm->origin[0] + vel[0] / speed * 16.0f;
        start[1] = stop[1] = pm->origin[1] + vel[1] / speed * 16.0f;
        start[2] = pm->origin[2] + PLAYER_MINS[pm->usehull][2];
        stop[2]  = start[2] - 34.0f;
        pmtrace_t tr = PM_PlayerTrace(pm, start, stop, PM_NORMAL, -1);
        float friction = (tr.fraction == 1.0f) ? pm->mv_friction * pm->mv_edgefriction
                                               : pm->mv_friction;
        friction *= pm->friction;
        float control = (speed < pm->mv_stopspeed) ? pm->mv_stopspeed : speed;
        drop += friction * control * pm->frametime;
    }
    float newspeed = speed - drop;
    if (newspeed < 0) newspeed = 0;
    newspeed /= speed;
    vel[0] *= newspeed; vel[1] *= newspeed; vel[2] *= newspeed;
}

// PM_AirMove - pm_shared.cpp:1490 (PM_AirMove_internal).
void PM_AirMove(PmContext* pm) {
    float fmove = pm->cmd_forwardmove, smove = pm->cmd_sidemove;
    pm->forward[2] = 0; pm->right[2] = 0;
    VectorNormalize(pm->forward); VectorNormalize(pm->right);
    vec3_t wishvel;
    for (int i = 0; i < 2; ++i) wishvel[i] = pm->forward[i]*fmove + pm->right[i]*smove;
    wishvel[2] = 0;
    vec3_t wishdir; VectorCopy(wishvel, wishdir);
    float wishspeed = VectorNormalize(wishdir);
    if (wishspeed > pm->maxspeed) { VectorScale(wishvel, pm->maxspeed/wishspeed, wishvel); wishspeed = pm->maxspeed; }
    PM_AirAccelerate(pm, wishdir, wishspeed, pm->mv_airaccelerate);
    VectorAdd(pm->velocity, pm->basevelocity, pm->velocity);
    PM_FlyMove(pm);
}

// PM_WalkMove - pm_shared.cpp:1039 (ground move with stair-stepping).
void PM_WalkMove(PmContext* pm) {
    float fmove = pm->cmd_forwardmove, smove = pm->cmd_sidemove, maxspeed = pm->maxspeed;
    pm->forward[2] = 0; pm->right[2] = 0;
    VectorNormalize(pm->forward); VectorNormalize(pm->right);
    vec3_t wishvel;
    for (int i = 0; i < 2; ++i) wishvel[i] = pm->forward[i]*fmove + pm->right[i]*smove;
    wishvel[2] = 0;
    vec3_t wishdir; VectorCopy(wishvel, wishdir);
    float wishspeed = VectorNormalize(wishdir);
    if (wishspeed > maxspeed) { VectorScale(wishvel, maxspeed/wishspeed, wishvel); wishspeed = maxspeed; }

    pm->velocity[2] = 0;
    PM_Accelerate(pm, wishdir, wishspeed, pm->mv_accelerate);
    pm->velocity[2] = 0;
    VectorAdd(pm->velocity, pm->basevelocity, pm->velocity);

    if (Length(pm->velocity) < 1.0f) { VectorClear(pm->velocity); return; }

    int oldonground = pm->onground;
    vec3_t dest = { pm->origin[0] + pm->velocity[0]*pm->frametime,
                    pm->origin[1] + pm->velocity[1]*pm->frametime,
                    pm->origin[2] };
    pmtrace_t trace = PM_PlayerTrace(pm, pm->origin, dest, PM_NORMAL, -1);
    if (trace.fraction == 1.0f) { VectorCopy(trace.endpos, pm->origin); return; }
    if (oldonground == -1 && pm->waterlevel == 0) return;
    if (pm->waterjumptime) return;

    vec3_t original, originalvel, down, downvel;
    VectorCopy(pm->origin, original);
    VectorCopy(pm->velocity, originalvel);
    PM_FlyMove(pm);
    VectorCopy(pm->origin, down);
    VectorCopy(pm->velocity, downvel);
    VectorCopy(original, pm->origin);
    VectorCopy(originalvel, pm->velocity);

    VectorCopy(pm->origin, dest);
    dest[2] += pm->mv_stepsize;
    trace = PM_PlayerTrace(pm, pm->origin, dest, PM_NORMAL, -1);
    if (!trace.startsolid && !trace.allsolid) VectorCopy(trace.endpos, pm->origin);
    PM_FlyMove(pm);

    VectorCopy(pm->origin, dest);
    dest[2] -= pm->mv_stepsize;
    trace = PM_PlayerTrace(pm, pm->origin, dest, PM_NORMAL, -1);

    bool usedown = (trace.plane.normal[2] < 0.7f);
    if (!usedown) {
        if (!trace.startsolid && !trace.allsolid) VectorCopy(trace.endpos, pm->origin);
        VectorCopy(pm->origin, pm->up);
        float downdist = (down[0]-original[0])*(down[0]-original[0]) + (down[1]-original[1])*(down[1]-original[1]);
        float updist   = (pm->up[0]-original[0])*(pm->up[0]-original[0]) + (pm->up[1]-original[1])*(pm->up[1]-original[1]);
        if (downdist > updist) usedown = true;
        else pm->velocity[2] = downvel[2];
    }
    if (usedown) { VectorCopy(down, pm->origin); VectorCopy(downvel, pm->velocity); }
}

// One full MOVETYPE_WALK frame, non-water (pm_shared.cpp:3258 sequence). No jump/
// duck/ladder/water — the finder drives a plain +forward walk.
void PM_PlayerMoveFrame(PmContext* pm) {
    AngleVectors(pm->angles, pm->forward, pm->right, pm->up);
    PM_AddCorrectGravity(pm);
    if (pm->onground != -1) { pm->velocity[2] = 0; PM_Friction(pm); }
    PM_CheckVelocity(pm);
    if (pm->onground != -1) PM_WalkMove(pm);
    else                    PM_AirMove(pm);
    PM_CategorizePosition(pm);
    PM_CheckVelocity(pm);
    PM_FixupGravityVelocity(pm);
    if (pm->onground != -1) pm->velocity[2] = 0;
}
} // namespace pw
