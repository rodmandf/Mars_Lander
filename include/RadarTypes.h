#pragma once
#include <vector>

struct Vec2 {
    float x = 0.f;
    float y = 0.f;
};

struct RayHit {
    Vec2 origin;
    Vec2 dir;
    bool hit = false;
    Vec2 point;
    float t = 0.f;
    int segIndex = -1;
};

struct RadarConfig {
    int rays = 157;
    float fovRad = 2.0f;
    float maxRange = 1200.f;
    float maxXSpan = 1200.f;
};

std::vector<RayHit> scanRadar(const std::vector<float>& terrain,
                              Vec2 origin,
                              float shipAngleRad,
                              const RadarConfig& cfg);
