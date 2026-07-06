#include "clipdecomp.h"
#include <cmath>

namespace pw {

namespace {
using V3 = std::array<float,3>;
struct Plane { V3 n; float d; };
constexpr float ON_EPS = 0.01f;

// Large convex quad lying on plane (n,d), centered near sphere(c,radius) so it
// covers the model bounds. Sized to bounds (NOT 10e6) to keep float32 precise.
std::vector<V3> BaseWinding(const V3& n, float d, const V3& c, float radius) {
    int ax = 0; float m = std::fabs(n[0]);
    if (std::fabs(n[1]) > m) { ax = 1; m = std::fabs(n[1]); }
    if (std::fabs(n[2]) > m) { ax = 2; }
    V3 up = {0,0,0}; up[ax == 2 ? 0 : 2] = 1.0f;          // an axis != major
    float dn = up[0]*n[0] + up[1]*n[1] + up[2]*n[2];
    for (int i = 0; i < 3; ++i) up[i] -= n[i]*dn;         // orthogonalize to n
    float ul = std::sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
    if (ul < 1e-6f) return {};
    for (int i = 0; i < 3; ++i) up[i] /= ul;
    V3 rt = { n[1]*up[2]-n[2]*up[1], n[2]*up[0]-n[0]*up[2], n[0]*up[1]-n[1]*up[0] };
    float cd = c[0]*n[0] + c[1]*n[1] + c[2]*n[2] - d;
    V3 proj = { c[0]-n[0]*cd, c[1]-n[1]*cd, c[2]-n[2]*cd }; // bounds center -> plane
    std::vector<V3> w(4);
    for (int i = 0; i < 3; ++i) {
        w[0][i] = proj[i] - rt[i]*radius - up[i]*radius;
        w[1][i] = proj[i] - rt[i]*radius + up[i]*radius;
        w[2][i] = proj[i] + rt[i]*radius + up[i]*radius;
        w[3][i] = proj[i] + rt[i]*radius - up[i]*radius;
    }
    return w;
}

// Sutherland-Hodgman: keep the part of w with dot(p,n) <= d (behind outward plane).
std::vector<V3> ClipWinding(const std::vector<V3>& w, const V3& n, float d) {
    size_t N = w.size();
    if (N < 3) return {};
    std::vector<float> dist(N);
    std::vector<int> side(N);       // 0 outside, 1 inside, 2 on
    int cnt[3] = {0,0,0};
    for (size_t i = 0; i < N; ++i) {
        float dd = w[i][0]*n[0] + w[i][1]*n[1] + w[i][2]*n[2] - d;
        dist[i] = dd;
        side[i] = dd > ON_EPS ? 0 : (dd < -ON_EPS ? 1 : 2);
        cnt[side[i]]++;
    }
    if (cnt[0] == 0) return w;      // nothing outside -> unchanged
    if (cnt[1] == 0) return {};     // nothing strictly inside -> gone
    std::vector<V3> out;
    for (size_t i = 0; i < N; ++i) {
        if (side[i] != 0) out.push_back(w[i]);           // keep inside + on
        size_t j = (i + 1) % N;
        if (side[i] == 2 || side[j] == 2 || side[i] == side[j]) continue;
        float t = dist[i] / (dist[i] - dist[j]);         // FRONT<->BACK crossing
        V3 mid;
        for (int k = 0; k < 3; ++k) mid[k] = w[i][k] + t*(w[j][k]-w[i][k]);
        out.push_back(mid);
    }
    return out;
}

void RemoveColinear(std::vector<V3>& w) {
    size_t N = w.size();
    if (N < 3) return;
    std::vector<V3> out;
    for (size_t i = 0; i < N; ++i) {
        const V3& a = w[(i+N-1)%N]; const V3& b = w[i]; const V3& c = w[(i+1)%N];
        V3 e1 = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
        V3 e2 = { c[0]-b[0], c[1]-b[1], c[2]-b[2] };
        float l1 = std::sqrt(e1[0]*e1[0]+e1[1]*e1[1]+e1[2]*e1[2]);
        float l2 = std::sqrt(e2[0]*e2[0]+e2[1]*e2[1]+e2[2]*e2[2]);
        if (l1 < 1e-5f) continue;                         // duplicate vertex -> drop
        if (l2 < 1e-5f) { out.push_back(b); continue; }
        for (int k = 0; k < 3; ++k) { e1[k]/=l1; e2[k]/=l2; }
        float dot = e1[0]*e2[0]+e1[1]*e2[1]+e1[2]*e2[2];
        if (dot < 0.9999f) out.push_back(b);              // keep only real corners
    }
    if (out.size() >= 3) w.swap(out);
}

void EmitBrush(const std::vector<Plane>& stack, int contents,
               const V3& center, float radius, std::vector<DecompBrush>& out) {
    DecompBrush b; b.contents = contents;
    for (size_t i = 0; i < stack.size(); ++i) {
        std::vector<V3> w = BaseWinding(stack[i].n, stack[i].d, center, radius);
        for (size_t j = 0; j < stack.size() && !w.empty(); ++j) {
            if (i == j) continue;
            w = ClipWinding(w, stack[j].n, stack[j].d);
        }
        if (w.size() < 3) continue;
        RemoveColinear(w);
        if (w.size() < 3) continue;
        DecompFace f; f.n = stack[i].n; f.d = stack[i].d; f.w = std::move(w);
        b.faces.push_back(std::move(f));
    }
    if (b.faces.size() >= 4) out.push_back(std::move(b));  // a solid needs >= 4 faces
}

void Recurse(const dclipnode_t* cn, int numcn, const mplane_t* pl, int num,
             std::vector<Plane>& stack, const V3& center, float radius,
             std::vector<DecompBrush>& out) {
    if (num < 0) {
        if (num != CONTENTS_EMPTY) EmitBrush(stack, num, center, radius, out);
        return;
    }
    if (num >= numcn) return;                              // defensive
    const dclipnode_t& node = cn[num];
    const mplane_t& P = pl[node.planenum];
    V3 pn = { P.normal[0], P.normal[1], P.normal[2] };
    // front child (dot>=dist region): outward plane (-n,-d)
    stack.push_back({ {-pn[0], -pn[1], -pn[2]}, -P.dist });
    Recurse(cn, numcn, pl, node.children[0], stack, center, radius, out);
    stack.pop_back();
    // back child (dot<=dist region): outward plane (n,d)
    stack.push_back({ pn, P.dist });
    Recurse(cn, numcn, pl, node.children[1], stack, center, radius, out);
    stack.pop_back();
}
} // namespace

std::vector<DecompBrush> DecompileClipHull(
    const dclipnode_t* clipnodes, int numclipnodes, const mplane_t* planes,
    int root, const float mins[3], const float maxs[3]) {
    std::vector<DecompBrush> out;
    if (root < 0) return out;                             // empty model (leaf root)
    const float PAD = 40.0f;                              // cover the expanded hull past model bounds
    V3 lo = { mins[0]-PAD, mins[1]-PAD, mins[2]-PAD };
    V3 hi = { maxs[0]+PAD, maxs[1]+PAD, maxs[2]+PAD };
    V3 center = { (lo[0]+hi[0])*0.5f, (lo[1]+hi[1])*0.5f, (lo[2]+hi[2])*0.5f };
    float dx = hi[0]-lo[0], dy = hi[1]-lo[1], dz = hi[2]-lo[2];
    float radius = 0.5f*std::sqrt(dx*dx + dy*dy + dz*dz) + 16.0f;
    std::vector<Plane> stack;
    stack.push_back({ {1,0,0},   hi[0] });   stack.push_back({ {-1,0,0}, -lo[0] });
    stack.push_back({ {0,1,0},   hi[1] });   stack.push_back({ {0,-1,0}, -lo[1] });
    stack.push_back({ {0,0,1},   hi[2] });   stack.push_back({ {0,0,-1}, -lo[2] });
    Recurse(clipnodes, numclipnodes, planes, root, stack, center, radius, out);
    return out;
}

bool PointInBrush(const DecompBrush& b, const float p[3], float eps) {
    for (const auto& f : b.faces)
        if (p[0]*f.n[0] + p[1]*f.n[1] + p[2]*f.n[2] - f.d > eps) return false;
    return true;
}
} // namespace pw
