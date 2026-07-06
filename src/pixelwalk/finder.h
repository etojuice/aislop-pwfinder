// finder.h - pixelwalk detection over candidate seams.
#pragma once
#include <array>
#include <vector>
#include "bsp.h"
#include "candidates.h"
#include "world.h"

namespace pw {

struct Find {
    std::array<float,3> pos{};        // resting hull-center world position
    int  usehull = 0;                 // 0 standing, 1 duck
    bool by_slope = false;            // caught on a tilted (ramp) floor pixel
    std::array<float,3> floor_normal{}; float floor_dist = 0;
    std::array<float,3> approach{};    // unit dir to walk (into the wall)
    int  floor_model = -1;
    int  wall_model  = -1;
    int  catch_frames = 0;            // frames the hull stayed caught on the pixel (strength)
    int  cluster_size = 1;            // raw sub-pixel hits merged into this find (reported as `samples`)
    // --zones: this find is a contiguous walkable SPAN. `pos` is one endpoint
    // (from), `to` the other (end); `length` = span length in units. For point
    // output (default) `to == pos` and `length == 0`.
    std::array<float,3> to{};
    float length = 0.0f;
};

struct FinderConfig {
    bool  standing = true;
    bool  duck     = true;
    float grid       = 1.0f / 64.0f;  // cross-seam sub-pixel step
    float along_step = 1.0f;          // step along the seam
    int   max_finds  = 0;             // 0 = unlimited
    int   min_samples = 6;            // drop finds with fewer than this many sub-pixel hits
    int   threads    = 0;             // 0 = auto
    bool  verbose    = false;
    bool  skip_categorize = false;    // --skip-categorize-pos: don't skip on-ground starts
    bool  zones      = false;         // --zones: group contiguous collinear finds into
                                      //   from->to spans instead of 64u point clusters
    float zone_gap   = 8.0f;          // --zone-gap: max along-wall gap (units) within one zone
};

std::vector<Find> RunFinder(const Map& map, const WorldModels& wm,
                            const std::vector<Seam>& seams, const FinderConfig& cfg);

// Debug: run the full PM_PlayerMove sim at a given origin, holding +forward along
// `yaw` (level pitch), and print per-frame origin, velocity, and onground.
void TraceAt(const WorldModels& wm, const float origin[3], float yaw, int usehull,
             int frames, float dist_epsilon, float init_vz = 0.0f);
} // namespace pw
