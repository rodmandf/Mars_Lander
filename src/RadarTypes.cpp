#include "RadarTypes.h"
#include <cmath>
#include <algorithm>

static Vec2 operator+(Vec2 a, Vec2 b){ return {a.x + b.x, a.y + b.y}; }
static Vec2 operator-(Vec2 a, Vec2 b){ return {a.x - b.x, a.y - b.y}; }
static Vec2 operator*(Vec2 a, float k){ return {a.x * k, a.y * k}; }

static float dot(Vec2 a, Vec2 b){ return a.x*b.x + a.y*b.y; }
static float cross(Vec2 a, Vec2 b){ return a.x*b.y - a.y*b.x; }

static float len(Vec2 v){ return std::sqrt(dot(v,v)); }
static Vec2 norm(Vec2 v){
    float L = len(v);
    if (L < 1e-6f) return {0.f, 0.f};
    return {v.x / L, v.y / L};
}

static Vec2 rotate(Vec2 v, float ang){
    float c = std::cos(ang), s = std::sin(ang);
    return {v.x*c - v.y*s, v.x*s + v.y*c};
}

// пересечение луча с отрезком
static bool raySegmentIntersect(Vec2 O, Vec2 D, Vec2 A, Vec2 B, float& outT, float& outU) {
    Vec2 v = B - A;
    float den = cross(D, v);
    if (std::abs(den) < 1e-8f) return false; // параллельно

    Vec2 w = O - A;
    float t = cross(v, w) / den;
    float u = cross(D, w) / den;

    if (t >= 0.f && u >= 0.f && u <= 1.f) {
        outT = t;
        outU = u;
        return true;
    }
    return false;
}

std::vector<RayHit> scanRadar(const std::vector<float>& terrain,
                              Vec2 origin,
                              float shipAngleRad,
                              const RadarConfig& cfg)
{
    std::vector<RayHit> hits;
    if (terrain.size() < 2 || cfg.rays <= 0) return hits;

    // вниз в системе корабля
    Vec2 downWorld = rotate({0.f, 1.f}, shipAngleRad);

    int xMin = std::max(0, (int)std::floor(origin.x - cfg.maxXSpan));
    int xMax = std::min((int)terrain.size() - 2, (int)std::ceil(origin.x + cfg.maxXSpan));

    hits.reserve((size_t)cfg.rays);

    for (int i = 0; i < cfg.rays; ++i) {
        float a = (cfg.rays == 1) ? 0.f : (float)i / (float)(cfg.rays - 1);
        float ang = (-0.5f * cfg.fovRad) + a * cfg.fovRad;

        Vec2 D = rotate(downWorld, ang);
        D = norm(D);

        RayHit best;
        best.origin = origin;
        best.dir = D;
        best.hit = false;
        best.t = cfg.maxRange;
        best.point = origin + D * best.t; // точка в конце луча, если не было попадания

        for (int x = xMin; x <= xMax; ++x) {
            Vec2 A{(float)x, terrain[x]};
            Vec2 B{(float)(x + 1), terrain[x + 1]};

            float t, u;
            if (!raySegmentIntersect(origin, D, A, B, t, u)) continue;
            if (t > cfg.maxRange) continue;

            if (!best.hit || t < best.t) {
                best.hit = true;
                best.t = t;
                best.point = origin + D * t;
                best.segIndex = x;
            }
        }

        hits.push_back(best);
    }

    return hits;
}
