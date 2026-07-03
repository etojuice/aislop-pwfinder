#include "world.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <unordered_map>

namespace pw {

namespace {

// Minimal entity-lump parser: a sequence of { "key" "value" ... } blocks.
using Ent = std::unordered_map<std::string, std::string>;

std::vector<Ent> ParseEntities(const std::string& s) {
    std::vector<Ent> ents;
    size_t i = 0, n = s.size();
    while (i < n) {
        // find next '{'
        while (i < n && s[i] != '{') ++i;
        if (i >= n) break;
        ++i;
        Ent e;
        while (i < n && s[i] != '}') {
            // read a quoted key
            while (i < n && s[i] != '"' && s[i] != '}') ++i;
            if (i >= n || s[i] == '}') break;
            ++i;
            size_t ks = i;
            while (i < n && s[i] != '"') ++i;
            std::string key = s.substr(ks, i - ks);
            if (i < n) ++i;                    // closing quote
            // read a quoted value
            while (i < n && s[i] != '"' && s[i] != '}') ++i;
            if (i >= n || s[i] == '}') break;
            ++i;
            size_t vs = i;
            while (i < n && s[i] != '"') ++i;
            std::string val = s.substr(vs, i - vs);
            if (i < n) ++i;
            e[key] = val;
        }
        if (i < n && s[i] == '}') ++i;
        ents.push_back(std::move(e));
    }
    return ents;
}

std::string lower(std::string v) {
    for (char& c : v) c = static_cast<char>(std::tolower((unsigned char)c));
    return v;
}

// Parse "x y z"; missing components stay 0.
std::array<float,3> parse_vec3(const std::string& v) {
    std::array<float,3> out{0, 0, 0};
    if (v.empty()) return out;
    std::sscanf(v.c_str(), "%f %f %f", &out[0], &out[1], &out[2]);
    return out;
}

// Solidity policy: a brush entity ("model" "*N") is traced as SOLID_BSP by
// DEFAULT (so func_door / func_breakable / func_train / func_rotating /
// func_pushable / func_button / func_conveyor / chargers / etc. are all included)
// EXCEPT the pass-through / non-solid classes below. Default-solid is the safe
// bias for a finder: better to over-include a solid than miss a real pixelwalk.
// (Caveat: movers like doors are traced in their spawn state.)
bool classname_is_solid(const std::string& cn) {
    if (cn.rfind("trigger", 0) == 0) return false;      // all trigger_* (SOLID_TRIGGER)
    static const char* nonsolid[] = {
        "func_illusionary", "func_ladder", "func_water", "func_friction",
        "func_buyzone", "func_bomb_target", "func_hostage_rescue",
        "func_vip_safetyzone", "func_escapezone", "func_grenade_catch",
        "func_weaponcheck", "func_mortar_field",
        "env_bubbles", "func_dustcloud", "func_dustmotes", "func_snow", "func_rain",
    };
    for (const char* ns : nonsolid)
        if (cn == ns) return false;
    return true;
}

} // namespace

WorldModels BuildWorld(const Map& map, const std::vector<model_t>& models) {
    WorldModels wm;
    const int nmodels = static_cast<int>(map.models.size());
    wm.model_solid.assign(nmodels, 0);
    wm.model_classname.assign(nmodels, "");
    wm.model_origin.assign(nmodels, {0, 0, 0});
    wm.model_angles.assign(nmodels, {0, 0, 0});

    // World (model 0).
    wm.world.model = models.empty() ? nullptr : &models[0];
    VectorClear(wm.world.origin);
    VectorClear(wm.world.angles);
    wm.world.solid = SOLID_BSP;
    wm.world.skin = 0;
    wm.world.rendermode = 0;
    wm.world.info = 0;
    if (nmodels > 0) {
        wm.model_solid[0] = 1;
        wm.model_classname[0] = "worldspawn";
    }

    // Brush entities.
    std::vector<Ent> ents = ParseEntities(map.entities);
    for (const Ent& e : ents) {
        auto it = e.find("model");
        if (it == e.end() || it->second.empty() || it->second[0] != '*') continue;
        int idx = std::atoi(it->second.c_str() + 1);
        if (idx <= 0 || idx >= nmodels) continue;

        std::string cn;
        auto cit = e.find("classname");
        if (cit != e.end()) cn = lower(cit->second);

        std::array<float,3> origin{0, 0, 0}, angles{0, 0, 0};
        auto oit = e.find("origin");
        if (oit != e.end()) origin = parse_vec3(oit->second);
        else { origin = { map.models[idx].origin[0], map.models[idx].origin[1], map.models[idx].origin[2] }; }
        auto ait = e.find("angles");
        if (ait != e.end()) angles = parse_vec3(ait->second);

        int rendermode = 0;
        auto rit = e.find("rendermode");
        if (rit != e.end()) rendermode = std::atoi(rit->second.c_str());

        wm.model_classname[idx] = cn;
        wm.model_origin[idx] = origin;
        wm.model_angles[idx] = angles;

        if (!classname_is_solid(cn)) continue;
        wm.model_solid[idx] = 1;

        SolidBrush sb;
        sb.pe.model = &models[idx];
        for (int k = 0; k < 3; ++k) sb.pe.origin[k] = origin[k];
        for (int k = 0; k < 3; ++k) sb.pe.angles[k] = angles[k];
        sb.pe.solid = SOLID_BSP;
        sb.pe.skin = 0;
        sb.pe.rendermode = rendermode;
        sb.pe.info = idx;
        for (int k = 0; k < 3; ++k) {
            sb.wmins[k] = map.models[idx].mins[k] + origin[k];
            sb.wmaxs[k] = map.models[idx].maxs[k] + origin[k];
        }
        wm.brushes.push_back(sb);
    }

    // Sort brushes by model index so the trace visits lower indices first
    // (the "lower model index wins on a fraction tie" rule).
    std::sort(wm.brushes.begin(), wm.brushes.end(),
              [](const SolidBrush& a, const SolidBrush& b) { return a.pe.info < b.pe.info; });

    return wm;
}

void SelectPhysents(const WorldModels& wm, const float qmins[3], const float qmaxs[3],
                    std::vector<physent_t>& out) {
    // Expand each brush bbox by this margin (> hull half-width) so a hull near a
    // wall is still tested against it.
    constexpr float kMargin = 40.0f;
    out.clear();
    out.push_back(wm.world);   // index 0
    for (const SolidBrush& b : wm.brushes) {
        bool overlap = true;
        for (int k = 0; k < 3; ++k) {
            if (b.wmaxs[k] + kMargin < qmins[k] || b.wmins[k] - kMargin > qmaxs[k]) {
                overlap = false;
                break;
            }
        }
        if (overlap) out.push_back(b.pe);
    }
}
} // namespace pw
